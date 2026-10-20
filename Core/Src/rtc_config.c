/**
  ******************************************************************************
  * @file    rtc_config.c
  * @brief   系统时间服务实现 —— 外部 DS3231 时基 + RAM 镜像（中断安全）
  *
  * 详见 rtc_config.h 顶部的设计说明。
  ******************************************************************************
  */

#include "rtc_config.h"
#include "ds3231.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

/* ============================== 内部状态 ============================== */

/* --- 时间镜像（seqlock 保护，可被中断并发读取） ---
 *   s_seq 为偶数且前后两次读到相同值 => 本次读取期间镜像未被修改，数据有效。
 *   s_seq 为奇数                      => 主循环正在更新，需重试。
 */
static volatile uint8_t        s_seq;       /* 序号锁计数                      */
/* 以下三个成员被主循环写、中断读，必须 volatile，防止 -O1 下被优化成寄存器缓存
   或读写被重排到 seqlock 之外 */
static volatile RTC_DateTime_t s_dt;        /* 同步时刻的日历                  */
static volatile uint64_t  s_base_ms;        /* 同步时刻的线性毫秒（= 秒 * 1000）*/
static volatile uint32_t  s_tick_at_sync;   /* 同步时刻的 HAL_GetTick() 值      */

static uint8_t  s_valid;            /* 1 = DS3231 有效（掉电不丢失）            */
static uint8_t  s_present;          /* 1 = DS3231 硬件在线                      */
static uint8_t  s_time_was_reset;   /* 1 = 本次上电曾用编译时间初始化（OSF=1）  */
static uint64_t s_boot_ms;          /* 开机基准：U= 运行时间的零点              */
static uint32_t s_sync_fail_cnt;    /* I2C 同步失败累计次数（诊断用）           */

/* ========================== 内部工具函数 ========================== */

/**
  * @brief 判断是否为闰年
  */
static uint8_t is_leap(uint16_t y)
{
    return (uint8_t)((((y % 4U) == 0U) && ((y % 100U) != 0U)) || ((y % 400U) == 0U));
}

/**
  * @brief 日历 -> 线性秒（自 2000-01-01 00:00:00）
  */
static uint64_t ymdhms_to_sec(const RTC_DateTime_t *dt)
{
    static const uint16_t mdays[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    uint16_t y = (uint16_t)(2000U + dt->year);
    uint32_t days = 0U;
    uint8_t  mm;

    for (uint16_t yy = 2000U; yy < y; yy++)
    {
        days += is_leap(yy) ? 366U : 365U;
    }
    for (mm = 1U; mm < dt->month; mm++)
    {
        days += mdays[mm - 1U];
        if ((mm == 2U) && is_leap(y))
        {
            days += 1U;
        }
    }
    days += (uint32_t)dt->day - 1U;

    return ((uint64_t)days * 86400ULL)
         + ((uint64_t)dt->hour * 3600ULL)
         + ((uint64_t)dt->min  * 60ULL)
         + ((uint64_t)dt->sec);
}

/**
  * @brief 线性秒 -> 日历（自 2000-01-01 00:00:00）
  * @note  年份以 4 年为周期快速推进，再用逐月减法精确定位，最坏约 30 次循环。
  */
static void sec_to_ymdhms(uint64_t sec, RTC_DateTime_t *dt)
{
    static const uint16_t mdays[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

    uint32_t days = (uint32_t)(sec / 86400ULL);
    uint32_t rem  = (uint32_t)(sec % 86400ULL);

    dt->hour = (uint8_t)(rem / 3600U);
    dt->min  = (uint8_t)((rem % 3600U) / 60U);
    dt->sec  = (uint8_t)(rem % 60U);

    uint16_t y = 2000U;
    while (days >= 365U)
    {
        uint32_t ylen = is_leap(y) ? 366U : 365U;
        if (days < ylen)
        {
            break;
        }
        days -= ylen;
        y++;
    }

    uint8_t m = 1U;
    while (m < 12U)
    {
        uint32_t mlen = mdays[m - 1U];
        if ((m == 2U) && is_leap(y))
        {
            mlen = 29U;
        }
        if (days < mlen)
        {
            break;
        }
        days -= mlen;
        m++;
    }

    dt->year  = (uint8_t)(y - 2000U);
    dt->month = m;
    dt->day   = (uint8_t)(days + 1U);
}

/* ------------------------- seqlock：发布 / 读取 ------------------------- */

static void snapshot_publish(const RTC_DateTime_t *dt, uint64_t base_ms, uint32_t tick)
{
    s_seq++;                    /* -> 奇数：更新开始 */
    __DMB();
    s_dt           = *dt;
    s_base_ms      = base_ms;
    s_tick_at_sync = tick;
    __DMB();
    s_seq++;                    /* -> 偶数：更新完成 */
}

static void snapshot_read(RTC_DateTime_t *dt, uint64_t *base_ms, uint32_t *tick)
{
    uint8_t a, b;
    do
    {
        a = s_seq;
        __DMB();
        if (dt      != NULL) { *dt      = s_dt; }
        if (base_ms != NULL) { *base_ms = s_base_ms; }
        if (tick    != NULL) { *tick    = s_tick_at_sync; }
        __DMB();
        b = s_seq;
    } while ((a != b) || ((a & 1U) != 0U));
}

/* ========================= 初始化 / 周期同步 ========================= */

int8_t RTC_Config_Init(void)
{
    RTC_DateTime_t dt;
    uint8_t status = 0U;

    s_valid          = 0U;
    s_present        = 0U;
    s_time_was_reset = 0U;
    s_sync_fail_cnt  = 0U;

    /* --- 1) 探测 DS3231 --- */
    if (DS3231_Probe() != 0)
    {
        /* --- 1b) 首次探测失败：尝试释放被卡住的 I2C 总线后再探一次 --- */
        (void)DS3231_BusRecover();
    }

    if (DS3231_Probe() == 0)
    {
        s_present = 1U;

        /* --- 2) 确保电池供电时振荡器继续运行（掉电不丢失的前提） --- */
        (void)DS3231_EnableBatteryOscillator();

        /* --- 3) 判定 OSF：时间是否可信 --- */
        if (DS3231_ReadStatus(&status) == 0)
        {
            if ((status & DS3231_STATUS_OSF) != 0U)
            {
                /* 振荡器曾停止（首次上电 / 纽扣电池耗尽 / 电池被取下）：
                 * 用编译时间初始化，给系统一个可用的起点；随后清除 OSF。
                 * 只要电池正常，以后每次上电 OSF 都应为 0，不会再进这里。 */
                if (RTC_ParseCompileTime(&dt) == 0)
                {
                    (void)DS3231_SetDateTime(dt.year, dt.month, dt.day,
                                             dt.hour, dt.min,   dt.sec);
                    s_time_was_reset = 1U;
                }
            }
        }
        else
        {
            status = 0U;
        }

        /* --- 4) 读芯片，建立第一份镜像 --- */
        RTC_Sync();
        s_valid = 1U;
    }
    else
    {
        /* --- 降级：DS3231 不可用，改用"编译时间 + SysTick"软件时钟 --- */
        if (RTC_ParseCompileTime(&dt) != 0)
        {
            dt.year = 0U; dt.month = 1U; dt.day = 1U;
            dt.hour = 0U; dt.min   = 0U; dt.sec = 0U;
        }
        snapshot_publish(&dt, ymdhms_to_sec(&dt) * 1000ULL, HAL_GetTick());
        s_valid = 0U;
    }

    /* --- 5) 抓取开机基准，作为 U= 运行时间的零点 --- */
    RTC_CaptureBoot();

    return s_valid ? 0 : -1;
}

void RTC_Sync(void)
{
    RTC_DateTime_t dt;
    uint32_t tick_before;

    if (s_present == 0U)
    {
        return;                     /* 降级模式：软件时钟无需同步 */
    }

    tick_before = HAL_GetTick();

    if (DS3231_GetDateTime(&dt.year, &dt.month, &dt.day,
                           &dt.hour, &dt.min,   &dt.sec) == 0)
    {
        /* I2C 传输耗时 < 1 ms，用传输开始时刻作为镜像基准即可 */
        snapshot_publish(&dt, ymdhms_to_sec(&dt) * 1000ULL, tick_before);
    }
    else
    {
        s_sync_fail_cnt++;          /* 保留旧镜像，由 SysTick 继续外推 */
    }
}

/* =========================== 时间读取（中断安全） =========================== */

uint64_t RTC_NowMs(void)
{
    uint64_t base;
    uint32_t tick;
    uint32_t delta_ms;
    uint32_t max_extrap;

    snapshot_read(NULL, &base, &tick);

    /* 无符号相减天然处理 HAL_GetTick() 的 49.7 天回绕 */
    delta_ms = (uint32_t)(HAL_GetTick() - tick);

    /* DS3231 模式下限制外推跨度，避免主循环长时间失控造成时间暴涨；
     * 降级模式没有同步源，必须允许无上限外推（否则软件时钟会停住）。 */
    max_extrap = (s_valid != 0U) ? (uint32_t)RTC_MAX_EXTRAP_MS : 0xFFFFFFFFU;
    if (delta_ms > max_extrap)
    {
        delta_ms = max_extrap;
    }

    return base + (uint64_t)delta_ms;
}

void RTC_Now(RTC_DateTime_t *dt)
{
    if (dt == NULL)
    {
        return;
    }
    sec_to_ymdhms(RTC_NowMs() / 1000ULL, dt);
}

uint64_t RTC_GetUptimeMs(void)
{
    uint64_t now = RTC_NowMs();
    /* 校时把时间往回调时，避免输出负值（uint64 下会变成一个天文数字） */
    return (now >= s_boot_ms) ? (now - s_boot_ms) : 0ULL;
}

void RTC_CaptureBoot(void)
{
    s_boot_ms = RTC_NowMs();
}

uint64_t RTC_ToTotalSec(const RTC_DateTime_t *dt)
{
    return (dt == NULL) ? 0ULL : ymdhms_to_sec(dt);
}

/* ============================ 校时 / 诊断 ============================ */

int8_t RTC_SetDateTime(uint8_t year, uint8_t month, uint8_t day,
                       uint8_t hour, uint8_t min, uint8_t sec)
{
    RTC_DateTime_t dt;
    int8_t ret;

    /* 入参合法性粗筛，避免把非法 BCD 写进芯片导致读出乱码 */
    if ((month < 1U) || (month > 12U) || (day < 1U) || (day > 31U) ||
        (hour > 23U) || (min  > 59U)  || (sec  > 59U))
    {
        return -1;
    }

    if (s_present != 0U)
    {
        ret = DS3231_SetDateTime(year, month, day, hour, min, sec);
        /* 立即刷新镜像，使 T= 字段马上反映新时间（不等下个 1 s 同步点） */
        RTC_Sync();
    }
    else
    {
        /* 降级模式：直接改写软件时钟基准 */
        dt.year = year; dt.month = month; dt.day = day;
        dt.hour = hour; dt.min   = min;   dt.sec = sec;
        snapshot_publish(&dt, ymdhms_to_sec(&dt) * 1000ULL, HAL_GetTick());
        ret = -1;
    }
    return ret;
}

int8_t RTC_ParseString(const char *s, RTC_DateTime_t *dt)
{
    int y, m, d, hh, mm, ss;
    const char *p = s;

    if ((s == NULL) || (dt == NULL))
    {
        return -1;
    }
    while (*p && ((*p < '0') || (*p > '9')))
    {
        p++;                                    /* 跳过 "set " 等非数字前缀 */
    }
    if (sscanf(p, "%d-%d-%d %d:%d:%d", &y, &m, &d, &hh, &mm, &ss) != 6)
    {
        return -1;
    }
    if ((y < 2000) || (y > 2099) || (m < 1) || (m > 12) ||
        (d < 1) || (d > 31) || (hh < 0) || (hh > 23) ||
        (mm < 0) || (mm > 59) || (ss < 0) || (ss > 59))
    {
        return -1;
    }

    dt->year  = (uint8_t)(y - 2000);
    dt->month = (uint8_t)m;
    dt->day   = (uint8_t)d;
    dt->hour  = (uint8_t)hh;
    dt->min   = (uint8_t)mm;
    dt->sec   = (uint8_t)ss;
    return 0;
}

int8_t RTC_ParseCompileTime(RTC_DateTime_t *dt)
{
    int year, day, hh, mm, ss;
    char mon[4];
    static const char *mons[12] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                                   "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    int m = 0;

    if (dt == NULL)
    {
        return -1;
    }
    if (sscanf(__DATE__, "%3s %d %d", mon, &day, &year) != 3)
    {
        return -1;
    }
    if (sscanf(__TIME__, "%d:%d:%d", &hh, &mm, &ss) != 3)
    {
        return -1;
    }
    for (int i = 0; i < 12; i++)
    {
        if (strcmp(mon, mons[i]) == 0)
        {
            m = i + 1;
            break;
        }
    }
    if (m == 0)
    {
        return -1;
    }
    if ((year < 2000) || (year > 2099))
    {
        return -1;
    }

    dt->year  = (uint8_t)(year - 2000);
    dt->month = (uint8_t)m;
    dt->day   = (uint8_t)day;
    dt->hour  = (uint8_t)hh;
    dt->min   = (uint8_t)mm;
    dt->sec   = (uint8_t)ss;
    return 0;
}

uint8_t RTC_IsValid(void)
{
    return s_valid;
}

uint8_t RTC_IsPresent(void)
{
    return s_present;
}

float RTC_GetTemp(void)
{
    return (s_present != 0U) ? DS3231_ReadTemp() : -99.0f;
}

const char *RTC_GetSourceName(void)
{
    return (s_valid != 0U) ? "DS3231" : "SOFT";
}

/**
  * @brief  供调试输出：本次上电是否曾用编译时间重置（OSF = 1）
  * @note   在 rtc_config.h 中未导出，如需在 main.c 使用请自行声明 extern。
  */
uint8_t RTC_WasTimeReset(void)
{
    return s_time_was_reset;
}

/**
  * @brief  供调试输出：I2C 同步失败累计次数
  */
uint32_t RTC_GetSyncFailCount(void)
{
    return s_sync_fail_cnt;
}
