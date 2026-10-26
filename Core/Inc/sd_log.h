/**
  ******************************************************************************
  * @file    sd_log.h
  * @brief   UART2 调试输出同步落盘（SD 卡文本文件）
  *
  * 功能：把原本只从 USART2(PA2/PA3) 发出的每一条调试信息，同时以纯文本
  *       形式追加写入 SD 卡中的日志文件，实现"串口可见即卡里也有"。
  *
  * 日志文件名（8.3 大写，卡在电脑上可直接打开）：
  *   - DS3231 在位：LOG_YYYYMMDD.TXT（按开机当天的日期命名）
  *   - DS3231 缺席：LOG.TXT
  *
  * 使用约定：
  *   1) SDLOG_Init() 必须在 RTC_Config_Init() 之后、第一条 UART2 输出之前
  *      调用（文件名要用到 RTC）；只在主循环初始化阶段调用一次。
  *   2) SDLOG_UART2() 仅供主循环上下文调用（现工程所有 UART2 输出点均满足）；
  *      禁止在中断里调用（SPI 阻塞传输会破坏微秒级计时）。
  *   3) 卡未插 / 挂载失败时自动跳过写卡，串口输出不受影响；每 5 s 静默重试。
  ******************************************************************************
  */
#ifndef __SD_LOG_H
#define __SD_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
  * @brief  初始化 SD 卡并打开日志文件（追加模式）
  * @retval 1 = 卡就绪且日志文件已打开
  *         0 = 卡不可用（未插/初始化失败），后续调用会自动重试
  */
uint8_t  SDLOG_Init(void);

/**
  * @brief  UART2 输出 + SD 卡追加写入（替代原 HAL_UART_Transmit(&huart2,...)）
  * @param  data : 待发送缓冲（含 "\r\n" 结尾的一行文本）
  * @param  len  : 字节数
  * @note   串口发送始终执行；写卡为尽力而为，失败静默降级。
  */
void     SDLOG_UART2(const uint8_t *data, uint16_t len);

/**
  * @brief  查询日志文件是否处于打开状态
  */
uint8_t  SDLOG_IsReady(void);

/**
  * @brief  返回当前日志文件名（如 "LOG_20261020.TXT"），供诊断输出
  */
const char *SDLOG_GetFileName(void);

#ifdef __cplusplus
}
#endif

#endif /* __SD_LOG_H */
