/**
  ******************************************************************************
  * @file    rtc_config.h
  * @brief   系统时间服务（掉电不丢失）—— 对外统一时基接口
  *
  * 设计要点
  * --------
  * 1) 时间源 = 外部 DS3231（自带 TCXO + 纽扣电池），**不再使用 STM32 内部 RTC**。
  *    原因：本板 STM32 的 VBAT 直接接 3.3 V，无备份电池，VDD 一断内部 RTC
  *    立即停走并复位，无法满足"掉电后时间不丢失"。
  *
  * 2) 中断安全（本工程最关键的一点）
  *    UART_SendRecord() 在 HAL_ADC_ConvCpltCallback（**中断上下文**）中被调用，
  *    需要读取当前时间。而读 DS3231 必须通过阻塞式 I2C（约 0.5~1 ms），
  *    若直接在中断里跑 I2C，会把每次脉冲的处理时间拖长近千倍，彻底破坏
  *    微秒级计时精度，且 I2C 传输还可能被更高优先级中断打断而失败。
  *    因此本模块在 RAM 中维护一份**时间镜像**：
  *        - 主循环每 1 s 调用一次 RTC_Sync()：走 I2C 读 DS3231，刷新镜像；
  *        - 中断侧调用 RTC_NowMs() / RTC_Now()：只读镜像并借助 SysTick
  *          (HAL_GetTick) 做亚秒外推，完全不触碰 I2C。
  *    镜像更新采用 seqlock（序号锁）保护，杜绝中断打断写过程造成的数据撕裂。
  *
  * 3) 掉电恢复策略
  *    DS3231 状态寄存器的 OSF 位反映振荡器是否曾停止：
  *      - OSF = 0：备份电池一直供电，走时未中断 -> 直接沿用芯片当前时间；
  *      - OSF = 1：首次上电 / 纽扣电池耗尽 -> 用编译时间 __DATE__/__TIME__
  *                写入芯片作为初值，并清除 OSF（之后上电将不再进入此分支）。
  *    同时初始化时会清零控制寄存器的 EOSC 位，确保电池供电期间不主动停振。
  *
  * 4) 降级策略
  *    若 DS3231 未应答（未焊接 / 断线 / 上拉缺失），模块自动降级为
  *    "编译时间 + SysTick 运行时间"的软件时钟，保证 U= 运行时间字段仍然
  *    可用，T= 字段显示软件推算时间，并通过调试串口给出明确告警。
  ******************************************************************************
  */

#ifndef __RTC_CONFIG_H
#define __RTC_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * 镜像外推的最大允许跨度（ms）
 *   正常情况主循环每 1 s 同步一次，外推量恒 < 1000 ms。
 *   若因故长时间未同步（例如主循环被卡住），钳位在此值，避免时间大幅跳变。
 * ------------------------------------------------------------------------- */
#define RTC_MAX_EXTRAP_MS   5000U

/**
  * @brief  日历时间结构
  * @note   year 为相对 2000 年的偏移量（0~99，如 26 表示 2026 年），
  *         与 DS3231 的 YEAR 寄存器格式一致，方便直接透传。
  */
typedef struct
{
    uint8_t year;    /* 0~99  -> 20xx */
    uint8_t month;   /* 1~12          */
    uint8_t day;     /* 1~31          */
    uint8_t hour;    /* 0~23          */
    uint8_t min;     /* 0~59          */
    uint8_t sec;     /* 0~59          */
} RTC_DateTime_t;

/* ======================= 初始化 / 周期同步（仅线程） ======================= */

/**
  * @brief  初始化系统时间（必须在 MX_I2C2_Init() 之后调用）
  * @retval  0 = DS3231 在线，时基为外部 RTC（掉电不丢失）
  *         -1 = DS3231 无应答，已降级为"编译时间 + SysTick"软件时钟
  * @note   内部流程：探测 -> 尝试总线恢复 -> 使能电池振荡器(清 EOSC)
  *         -> 判 OSF（必要时写编译时间）-> 读芯片刷新镜像 -> 抓取开机基准。
  *         本函数会执行阻塞式 I2C，**只能在主循环/初始化阶段调用**。
  */
int8_t RTC_Config_Init(void);

/**
  * @brief  用 DS3231 当前时间刷新 RAM 镜像（主循环以 1 Hz 周期调用）
  * @note   阻塞式 I2C，**只能在线程上下文调用**。
  *         降级模式下本函数为空操作（软件时钟由 SysTick 自行推进）。
  */
void   RTC_Sync(void);

/* ==================== 时间读取（线程 / 中断均可调用） ==================== */

/**
  * @brief  取当前时间的线性毫秒数（自 2000-01-01 00:00:00 起算）
  * @retval 线性毫秒（uint64，约可表示到公元 5849 年）
  * @note   读 RAM 镜像 + SysTick 外推，**不访问 I2C，可在中断中安全调用**。
  */
uint64_t RTC_NowMs(void);

/**
  * @brief  取当前日历时间
  * @param  dt 输出：年(偏移)/月/日/时/分/秒
  * @note   同上，中断安全。
  */
void   RTC_Now(RTC_DateTime_t *dt);

/**
  * @brief  取"自本次上电以来的运行时间"（ms），即 U= 字段的数据源
  * @retval 运行时间（ms）；若时间被回拨返回 0，不会出现负数
  * @note   开机基准由 RTC_CaptureBoot() 抓取。中断安全。
  */
uint64_t RTC_GetUptimeMs(void);

/**
  * @brief  抓取开机时刻基准（RTC_Config_Init() 内已自动调用一次）
  * @note   校时命令导致时间大幅回拨后，可再次调用以重新对齐 U= 零点。
  */
void   RTC_CaptureBoot(void);

/**
  * @brief  将日历时间转换为线性秒数（自 2000-01-01 00:00:00）
  */
uint64_t RTC_ToTotalSec(const RTC_DateTime_t *dt);

/* ============================== 校时 / 诊断 ============================== */

/**
  * @brief  设置系统时间（写 DS3231，掉电保持）
  * @retval  0 = 成功写入 DS3231
  *         -1 = I2C 失败（降级模式下仅更新软件时钟基准）
  * @note   阻塞式 I2C，**只能在线程上下文调用**。
  *         成功写入后会自动清除 OSF 标志。
  */
int8_t RTC_SetDateTime(uint8_t year, uint8_t month, uint8_t day,
                       uint8_t hour, uint8_t min, uint8_t sec);

/**
  * @brief  解析 "YYYY-MM-DD HH:MM:SS"（允许带 "set "/"T " 等非数字前缀）
  * @retval 0 = 解析成功；-1 = 格式错误或数值越界
  */
int8_t RTC_ParseString(const char *s, RTC_DateTime_t *dt);

/**
  * @brief  解析编译期宏 __DATE__ / __TIME__ 为日历时间
  * @retval 0 = 成功；-1 = 解析失败
  */
int8_t RTC_ParseCompileTime(RTC_DateTime_t *dt);

/**
  * @brief  查询时基是否为外部 DS3231
  * @retval 1 = DS3231 有效（掉电不丢失）；0 = 降级为软件时钟
  */
uint8_t RTC_IsValid(void);

/**
  * @brief  查询 DS3231 硬件是否在位
  * @retval 1 = 在线；0 = 无应答
  */
uint8_t RTC_IsPresent(void);

/**
  * @brief  读取 DS3231 内部温度（°C，0.25 °C 分辨率）
  * @retval 温度值；不使用 DS3231 时返回 -99.0f
  * @note   阻塞式 I2C，仅线程上下文；可用来判断芯片是否工作正常。
  */
float  RTC_GetTemp(void);

/**
  * @brief  返回时基来源名称字符串（供调试输出）
  * @retval "DS3231" 或 "SOFT"
  */
const char *RTC_GetSourceName(void);

/**
  * @brief  本次上电是否曾用编译时间重置时钟（即 DS3231 的 OSF 标志为 1）
  * @retval 1 = 时钟曾停振（首次上电 / 纽扣电池耗尽），时间来自编译时间
  *         0 = 备份电池一直供电，走时未中断
  * @note   连续多次上电都返回 1，说明纽扣电池没电或没装好，需要检查硬件。
  */
uint8_t RTC_WasTimeReset(void);

/**
  * @brief  I2C 同步失败累计次数（供诊断；持续上涨说明 I2C 总线不稳定）
  */
uint32_t RTC_GetSyncFailCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __RTC_CONFIG_H */
