/**
  ******************************************************************************
  * @file    ds3231.h
  * @brief   DS3231 高精度实时时钟底层驱动（I2C 接口）
  *
  * @note    硬件连接（与本板原理图一致）
  *            STM32G431CBT6            DS3231
  *            PA8  I2C2_SDA   <----->  SDA
  *            PA9  I2C2_SCL   <----->  SCL
  *            7 位从机地址 = 0x68（硬件固定，不可更改）
  *
  *          DS3231 内部集成温补晶振（TCXO，±2ppm，约 ±1 分钟/年），并且
  *          具备独立的 VBAT 引脚接纽扣电池（CR1220 / CR2032）。主电源 VCC
  *          掉电后由电池维持振荡器与计数，因此**掉电后时间不丢失**。
  *
  *          STM32 内部 RTC 不可用于本需求：本板 STM32 的 VBAT 引脚直接接到
  *          3.3 V（未接电池），VDD 一断内部 RTC 立即停走并复位。故整个工程的
  *          日历时基统一由 DS3231 提供，内部 RTC 外设不再使用。
  *
  *  @warning 本驱动的全部 API 均为**阻塞式 I2C 传输**（超时 DS3231_TIMEOUT_MS），
  *           只允许在线程（主循环）上下文调用，**严禁在中断服务函数 / HAL
  *           回调中调用**。中断侧需要读时间请使用 rtc_config.h 提供的
  *           RTC_NowMs() / RTC_Now()（读内存镜像，不触碰 I2C）。
  ******************************************************************************
  */

#ifndef __DS3231_H
#define __DS3231_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ============================ 器件地址与寄存器 ============================ */

#define DS3231_I2C_ADDR        0x68U                            /* 7 位从机地址           */
#define DS3231_I2C_WRITE       ((DS3231_I2C_ADDR << 1) | 0x00U) /* HAL 要求左移后的 8 位地址 = 0xD0 */

#define DS3231_REG_SECONDS     0x00U   /* 秒 (BCD, 00~59)                        */
#define DS3231_REG_MINUTES     0x01U   /* 分 (BCD, 00~59)                        */
#define DS3231_REG_HOURS       0x02U   /* 时 (BCD, bit6=0 表示 24 小时制)         */
#define DS3231_REG_DAY         0x03U   /* 星期 (1~7, 1=周日, 驱动自动推算)        */
#define DS3231_REG_DATE        0x04U   /* 日 (BCD, 01~31)                         */
#define DS3231_REG_MONTH       0x05U   /* 月 (BCD, bit7=世纪位)                   */
#define DS3231_REG_YEAR        0x06U   /* 年 (BCD, 00~99 -> 20xx)                 */
#define DS3231_REG_AL1_SEC     0x07U
#define DS3231_REG_CONTROL     0x0EU   /* 控制寄存器                              */
#define DS3231_REG_STATUS      0x0FU   /* 状态寄存器, bit7 = OSF 振荡器停止标志   */
#define DS3231_REG_AGING       0x10U   /* 老化微调（有符号，步进约 0.1 ppm）      */
#define DS3231_REG_TEMP_MSB    0x11U   /* 温度高位（有符号整数部分）              */
#define DS3231_REG_TEMP_LSB    0x12U   /* 温度低位（bit7:6 = 小数，0.25 °C/位）   */

/* 状态寄存器 OSF（Oscillator Stop Flag） */
#define DS3231_STATUS_OSF      0x80U   /* 1 = 振荡器曾停止，时间不可信            */
#define DS3231_STATUS_EN32KHZ  0x08U   /* 1 = 使能 32 kHz 输出引脚                */
#define DS3231_STATUS_BSY      0x04U   /* 1 = 正在执行温度转换，忙                */

/* 控制寄存器 */
#define DS3231_CTRL_EOSC       0x80U   /* 1 = 电池供电时停振（必须清零！）        */
#define DS3231_CTRL_BBSQW      0x40U   /* 1 = 电池供电时输出方波（耗电，建议 0）  */
#define DS3231_CTRL_CONV       0x20U   /* 写 1 启动一次温度转换                   */
#define DS3231_CTRL_INTCN      0x04U   /* 1 = 中断/方波引脚工作于中断模式         */

/* ================================ 公开 API ================================ */

/**
  * @brief  探测 DS3231 是否在 I2C 总线上应答
  * @retval 0 = 在线；-1 = 无应答（未焊接 / 焊反 / 断线 / 上拉缺失）
  * @note   仅做地址握手，**不清除 OSF 标志**（OSF 的判定与清除统一由
  *         rtc_config.c 的 RTC_Config_Init() 负责，否则"首次上电写默认时间"
  *         逻辑会永久失效）。
  */
int8_t DS3231_Probe(void);

/**
  * @brief  查询 DS3231 是否在位（供诊断/状态输出使用）
  * @retval 1 = 在线；0 = 无应答
  */
uint8_t DS3231_IsPresent(void);

/**
  * @brief  读取 DS3231 状态寄存器 (0x0F)
  * @param  status 输出：状态字节（bit7 = OSF）
  * @retval 0 = 成功；-1 = I2C 读失败
  */
int8_t DS3231_ReadStatus(uint8_t *status);

/**
  * @brief  清除 OSF（振荡器停止）标志
  * @retval 0 = 成功；-1 = I2C 失败
  * @note   写时间后调用，向系统声明"此后时间可信"。
  *         只要备份电池正常，下一次上电 OSF 应保持为 0，说明走时未中断。
  */
int8_t DS3231_ClearOSF(void);

/**
  * @brief  确保电池供电时振荡器继续运行
  * @retval 0 = 成功；-1 = I2C 失败
  * @note   清零控制寄存器的 EOSC 位（=1 时会**在电池供电期间主动停振**，
  *         这是掉电丢时间最常见的软件原因），同时清 BBSQW 以降低电池耗电。
  *         初始化阶段调用一次即可，设置保存在 DS3231 内部寄存器中。
  */
int8_t DS3231_EnableBatteryOscillator(void);

/**
  * @brief  写 DS3231 日期时间（内部自动完成 十进制 -> BCD）
  * @param  year  年偏移（0~99，表示 20xx 年，如 26 -> 2026）
  * @param  month 月（1~12）
  * @param  day   日（1~31）
  * @param  hour  时（0~23，强制 24 小时制）
  * @param  min   分（0~59）
  * @param  sec   秒（0~59）
  * @retval 0 = 成功；-1 = I2C 写失败
  * @note   星期由 Zeller 公式从年月日推算后自动写入，调用方无需关心。
  *         写完后自动清除 OSF 标志。
  */
int8_t DS3231_SetDateTime(uint8_t year, uint8_t month, uint8_t day,
                          uint8_t hour, uint8_t min, uint8_t sec);

/**
  * @brief  读 DS3231 日期时间（内部自动完成 BCD -> 十进制）
  * @param  year/month/day/hour/min/sec  输出参数，均为十进制
  * @retval 0 = 成功；-1 = I2C 读失败
  * @note   一次连续读出 0x00~0x06 共 7 字节，避免"跨秒进位"造成的
  *         撕裂（例如分已经进位、秒还没读到导致的 59:00 -> 00:59 跳变）。
  *         小时强制按 24 小时制解析（屏蔽模式位与 AM/PM 位）。
  */
int8_t DS3231_GetDateTime(uint8_t *year,  uint8_t *month, uint8_t *day,
                          uint8_t *hour,  uint8_t *min,   uint8_t *sec);

/**
  * @brief  读 DS3231 内部温度传感器（温补晶振用）
  * @retval 温度值（°C，0.25 °C 分辨率）；I2C 失败返回 -99.0f
  * @note   寄存器格式：0x11 为有符号整数部分，0x12 的 bit7:6 为小数部分。
  */
float DS3231_ReadTemp(void);

/**
  * @brief  I2C2 总线恢复（SDA 被从机卡低时释放总线）
  * @retval 0 = 已执行；-1 = 未执行
  * @note   当上一个字节传输被复位/掉电打断时，DS3231 可能仍拉住 SDA，
  *         此后主机的 START 永远发不出去（总线永远 BUSY）。
  *         本函数：关闭 I2C2 外设 -> PA8/PA9 切为 GPIO 开漏 ->
  *         打 9 个 SCL 脉冲让从机释放 SDA -> 产生一个 STOP 条件 ->
  *         恢复 I2C2 复用功能并重新初始化外设。
  *         **仅在初始化探测失败时调用**，正常运行时不会执行。
  */
int8_t DS3231_BusRecover(void);

#ifdef __cplusplus
}
#endif

#endif /* __DS3231_H */
