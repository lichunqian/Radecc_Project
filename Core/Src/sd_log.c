/**
  ******************************************************************************
  * @file    sd_log.c
  * @brief   UART2 调试输出 -> SD 卡文本日志（FATFS + SPI2 SD 驱动）
  *
  * 数据通路：
  *   main.c 各输出点 -> SDLOG_UART2()
  *     ├─ HAL_UART_Transmit(&huart2)      PA2/PA3，115200，行为与原先一致
  *     └─ f_write(LOG_xxx.TXT) + f_sync   SD 卡追加写入，掉电前已落盘
  *
  * 容错策略：
  *   - 上电挂载失败：不影响串口输出，每 SDLOG_RETRY_MS 静默重试一次；
  *   - 运行中写失败（卡拔出/写满）：关闭文件，转回重试逻辑；
  *   - 所有 FATFS/SPI 调用都带超时，最坏情况一次调用阻塞数百毫秒，
  *     因此本模块只能在主循环上下文使用（与现有 UART2 输出点一致）。
  ******************************************************************************
  */
#include "sd_log.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include "ff.h"
#include "diskio.h"
#include "bsp_sd_spi.h"
#include "rtc_config.h"

/* huart2 在 main.c 中定义（PA2/PA3 调试串口） */
extern UART_HandleTypeDef huart2;

/* 每写入多少行强制 f_sync 刷盘一次（1 = 每行都刷，掉电最多丢最后一行） */
#define SDLOG_SYNC_EVERY_N    1U
/* 卡不可用时的静默重试间隔（ms） */
#define SDLOG_RETRY_MS        5000U

static FATFS    sd_fs;               /* 文件系统对象 */
static FIL      sd_file;             /* 日志文件对象 */
static uint8_t  sd_mounted = 0U;     /* 1 = f_mount 成功 */
static uint8_t  sd_log_open = 0U;    /* 1 = 日志文件已打开 */
static uint8_t  sd_init_tried = 0U;  /* 1 = 至少执行过一次初始化 */
static uint32_t sd_retry_tick = 0U;  /* 上次重试时刻 */
static uint16_t sd_line_cnt = 0U;    /* 距上次 f_sync 的行数 */
static char     sd_fname[16] = "LOG.txt";   /* 当前日志文件名（8.3） */

/* ------------------------------ 内部函数 ------------------------------ */

/**
  * @brief  根据当前 RTC 生成日志文件名
  * @note   DS3231 在位 -> LOG_YYYYMMDD.txt；缺席 -> LOG.txt
  */
static void SDLOG_MakeFileName(void)
{
  RTC_DateTime_t dt;
  RTC_Now(&dt);

  if (RTC_IsPresent())
  {
    (void)snprintf(sd_fname, sizeof(sd_fname), "LOG_%04u%02u%02u.txt",
                   (unsigned)(2000U + dt.year),
                   (unsigned)dt.month,
                   (unsigned)dt.day);
  }
  else
  {
    (void)snprintf(sd_fname, sizeof(sd_fname), "LOG.txt");
  }
}

/**
  * @brief  尝试挂载文件系统并打开日志文件（追加模式）
  * @retval 1 = 文件已就绪
  */
static uint8_t SDLOG_TryOpen(void)
{
  FRESULT fr;

  /* 1) 挂载卷 "0:"（SD 卡） */
  if (!sd_mounted)
  {
    fr = f_mount(&sd_fs, "0:", 1);      /* 强制立即挂载 */
    if (fr != FR_OK)
      return 0U;
    sd_mounted = 1U;
  }

  /* 2) 打开（不存在则创建）并以追加方式定位到末尾 */
  if (!sd_log_open)
  {
    SDLOG_MakeFileName();
    fr = f_open(&sd_file, sd_fname, FA_WRITE | FA_OPEN_APPEND);
    if (fr != FR_OK)
      return 0U;
    sd_log_open = 1U;
    sd_line_cnt = 0U;
  }
  return 1U;
}

/**
  * @brief  写失败后的降级处理：关文件、保留挂载状态，等待重试
  */
static void SDLOG_FailClose(void)
{
  if (sd_log_open)
  {
    (void)f_close(&sd_file);
    sd_log_open = 0U;
  }
  sd_line_cnt = 0U;
}

/* ------------------------------ 对外接口 ------------------------------ */

uint8_t SDLOG_Init(void)
{
  sd_init_tried = 1U;
  sd_retry_tick = HAL_GetTick();

  if (SDLOG_TryOpen())
  {
    /* 会话起始标记：卡在电脑上打开文件即可分辨每次上电 */
    char hdr[64];
    RTC_DateTime_t dt;
    RTC_Now(&dt);
    int n = snprintf(hdr, sizeof(hdr),
                     "---- session start %04u-%02u-%02u %02u:%02u:%02u ----\r\n",
                     (unsigned)(2000U + dt.year), (unsigned)dt.month, (unsigned)dt.day,
                     (unsigned)dt.hour, (unsigned)dt.min, (unsigned)dt.sec);
    if (n > 0 && n <= (int)sizeof(hdr))
    {
      UINT bw = 0U;
      if (f_write(&sd_file, hdr, (UINT)n, &bw) == FR_OK && bw == (UINT)n
          && f_sync(&sd_file) == FR_OK)
      {
        return 1U;
      }
      SDLOG_FailClose();
      return 0U;
    }
    return 1U;
  }
  return 0U;
}

void SDLOG_UART2(const uint8_t *data, uint16_t len)
{
  /* 1) 串口输出永远执行（与改动前行为一致） */
  (void)HAL_UART_Transmit(&huart2, (uint8_t *)data, len, 100U);

  /* 2) 未初始化过（不可能发生，防御式）或卡不可用：按节流重试 */
  if (!sd_init_tried)
    return;

  if (!sd_log_open)
  {
    if ((HAL_GetTick() - sd_retry_tick) < SDLOG_RETRY_MS)
      return;                       /* 重试窗口未到，静默跳过 */
    sd_retry_tick = HAL_GetTick();
    if (!SDLOG_TryOpen())
      return;                       /* 卡仍不可用 */
  }

  /* 3) 追加写一行 */
  {
    UINT bw = 0U;
    FRESULT fr = f_write(&sd_file, data, (UINT)len, &bw);
    if (fr != FR_OK || bw != (UINT)len)
    {
      SDLOG_FailClose();            /* 卡拔出 / 写满 / 接触不良 */
      return;
    }

    sd_line_cnt++;
    if (sd_line_cnt >= SDLOG_SYNC_EVERY_N)
    {
      sd_line_cnt = 0U;
      if (f_sync(&sd_file) != FR_OK)
      {
        SDLOG_FailClose();
      }
    }
  }
}

uint8_t SDLOG_IsReady(void)
{
  return sd_log_open;
}

const char *SDLOG_GetFileName(void)
{
  return sd_fname;
}
