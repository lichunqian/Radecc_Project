/**
  ******************************************************************************
  * @file    bsp_sd_spi.h
  * @brief   TF/SD 卡 SPI 模式驱动（板级支持包）
  *
  * 硬件连接（见原理图 "TF卡电路"，网络标号写作 SPI1_*，实际引脚为 PB13~PB15）：
  *   PB12 -> TF_CS    （片选，推挽输出，空闲高电平）
  *   PB13 -> SPI1_CLK  = SPI2_SCK（AF5）
  *   PB14 -> SPI1_MISO = SPI2_MISO（AF5）
  *   PB15 -> SPI1_MOSI = SPI2_MOSI（AF5）
  *
  * ⚠️ 注意：网络标号虽叫 SPI1，但 PB13/14/15 的复用功能只有 SPI2（AF5）。
  *    SPI1 的 SCK/MISO/MOSI 在 PA5/PA6/PA7，与原理图不符 —— 代码必须用 SPI2。
  *
  * 时钟：SPI2 挂 APB1 = 120 MHz
  *   初始化阶段：预分频 256 -> 468.75 kHz（SD 卡要求 100~400 kHz）
  *   初始化完成后：预分频 8 -> 15 MHz（常规读写）
  ******************************************************************************
  */
#ifndef BSP_SD_SPI_H
#define BSP_SD_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 卡类型标志（diskio 可用 SD_GetCardType 区分寻址方式，仅诊断用） */
#define CT_NONE     0x00U
#define CT_SD1      0x02U   /* SD v1 (SDSC) */
#define CT_SD2      0x04U   /* SD v2 (SDSC) */
#define CT_SDHC     0x08U   /* SDHC/SDXC（块寻址） */
#define CT_BLOCK    0x10U   /* 强制块寻址 */

uint8_t SD_Init(void);                                  /* 0=成功，其它=失败 */
uint8_t SD_IsReady(void);                               /* 1=卡已初始化可用  */
uint8_t SD_GetCardType(void);                           /* 返回 CT_* 组合    */
uint8_t SD_ReadSectors(uint8_t *buf, uint32_t sector, uint32_t count);
uint8_t SD_WriteSectors(const uint8_t *buf, uint32_t sector, uint32_t count);
void    SD_DeInit(void);                                /* 卡拔出后复位状态  */

#ifdef __cplusplus
}
#endif

#endif /* BSP_SD_SPI_H */
