/**
  ******************************************************************************
  * @file    bsp_sd_spi.c
  * @brief   TF/SD 卡 SPI 模式驱动（SPI2，PB12~PB15）
  *
  * 实现要点：
  *  - 上电进入 SPI 模式：≥74 个时钟 + CMD0（CRC 固定 0x95）
  *  - SD v2 卡先 CMD8 探测，再用 ACMD41(HCS=1) 等待就绪，最后 CMD58 读 OCR
  *    判断 SDHC（块寻址）或 SDSC（字节寻址，CMD16 设 512B 块）
  *  - 初始化阶段 468.75 kHz，成功后切换 15 MHz
  *  - 所有命令帧前后必须多读 0xFF（SPI 模式时序要求）
  ******************************************************************************
  */
#include "bsp_sd_spi.h"
#include <string.h>

/* ------------------------------ 硬件定义 ------------------------------ */
#define SD_CS_PORT        GPIOB
#define SD_CS_PIN         GPIO_PIN_12

#define SD_SPI            hspi2
#define SD_TIMEOUT_MS     500U        /* 单字节 SPI 传输超时 */
#define SD_CMD_TIMEOUT    512U        /* 命令响应等待字节数上限 */
#define SD_BUSY_TIMEOUT   500U        /* 写忙等待（ms） */

SPI_HandleTypeDef hspi2;

/* ------------------------------ 内部状态 ------------------------------ */
static uint8_t sd_type = CT_NONE;   /* 卡类型 */
static uint8_t sd_ready = 0U;       /* 初始化成功标志 */

/* --------------------------- 底层收发原语 ---------------------------- */

static void SD_CS_Low(void)  { HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_RESET); }
static void SD_CS_High(void) { HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET);  }

/** 交换一个字节 */
static uint8_t SD_Xfer(uint8_t out)
{
  uint8_t in = 0xFFU;
  (void)HAL_SPI_TransmitReceive(&SD_SPI, &out, &in, 1U, SD_TIMEOUT_MS);
  return in;
}

/** 释放总线：拉高 CS 后再补 1 个字节，让 MISO 回高 */
static void SD_Release(void)
{
  SD_CS_High();
  (void)SD_Xfer(0xFFU);
}

/** 等待卡内部写忙完成：DO 变高 */
static uint8_t SD_WaitReady(void)
{
  uint32_t t0 = HAL_GetTick();
  do {
    if (SD_Xfer(0xFFU) == 0xFFU)
      return 1U;
  } while ((HAL_GetTick() - t0) < SD_BUSY_TIMEOUT);
  return 0U;
}

/**
  * @brief  发送一条命令并等待 R1 响应
  * @retval R1 字节（0x00 = 成功；0xFF = 超时无响应）
  */
static uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg)
{
  uint8_t crc;
  uint8_t r1;
  uint32_t n;

  if (!SD_WaitReady())
    return 0xFFU;

  /* CMD0 需要正确 CRC 0x95；CMD8 需要正确 CRC 0x87；其余在 SPI 模式下 CRC 关闭，填 0x01 */
  crc = (cmd == 0U) ? 0x95U : ((cmd == 8U) ? 0x87U : 0x01U);

  SD_CS_Low();
  (void)SD_Xfer(0x40U | cmd);
  (void)SD_Xfer((uint8_t)(arg >> 24));
  (void)SD_Xfer((uint8_t)(arg >> 16));
  (void)SD_Xfer((uint8_t)(arg >> 8));
  (void)SD_Xfer((uint8_t)arg);
  (void)SD_Xfer(crc);

  if (cmd == 12U)                       /* CMD12：先丢弃一个填充字节 */
    (void)SD_Xfer(0xFFU);

  for (n = 0U; n < SD_CMD_TIMEOUT; n++) /* R1 最高位为 0 即有效 */
  {
    r1 = SD_Xfer(0xFFU);
    if ((r1 & 0x80U) == 0U)
      return r1;
  }
  return 0xFFU;
}

/* ------------------------------ GPIO/SPI ------------------------------ */

/**
  * @brief  配置 SPI2 与 CS 引脚（自包含，不依赖 CubeMX 生成代码）
  * @param  slow : 1 = 初始化低速（468.75 kHz），0 = 常规高速（15 MHz）
  */
static void SD_SPI_PortInit(uint8_t slow)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_SPI2_CLK_ENABLE();

  /* PB12 = TF_CS，推挽输出，空闲高 */
  SD_CS_High();
  gpio.Pin = SD_CS_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SD_CS_PORT, &gpio);

  /* PB13/14/15 = SPI2 SCK/MISO/MOSI，复用推挽/上拉 */
  gpio.Pin = GPIO_PIN_13 | GPIO_PIN_15;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &gpio);

  gpio.Pin = GPIO_PIN_14;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &gpio);

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;          /* CPOL=0 */
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;              /* CPHA=0，SPI 模式 0 */
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = slow ? SPI_BAUDRATEPRESCALER_256   /* 468.75 kHz */
                                      : SPI_BAUDRATEPRESCALER_8;    /* 15 MHz */
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    /* 初始化失败：保持 sd_ready=0，由上层容错 */
  }
}

/** 初始化完成后提速 */
static void SD_SPI_FastSpeed(void)
{
  __HAL_SPI_DISABLE(&SD_SPI);
  MODIFY_REG(SD_SPI.Instance->CR1, SPI_CR1_BR, SPI_BAUDRATEPRESCALER_8);
  __HAL_SPI_ENABLE(&SD_SPI);
}

/* ------------------------------ 初始化 ------------------------------- */

uint8_t SD_Init(void)
{
  uint8_t r1;
  uint32_t t0;
  uint32_t i;
  uint8_t ocr[4];

  sd_ready = 0U;
  sd_type = CT_NONE;

  SD_SPI_PortInit(1U);          /* 低速模式搭建总线 */

  /* ≥74 个时钟周期，让卡完成上电内部初始化 */
  for (i = 0U; i < 10U; i++)
    (void)SD_Xfer(0xFFU);

  /* 进入 SPI 模式：CMD0，最多重试 255 次（插卡瞬间可能无响应） */
  for (i = 0U; i < 255U; i++)
  {
    r1 = SD_SendCmd(0U, 0U);
    if (r1 == 0x01U)
      break;
  }
  if (i >= 255U) { SD_Release(); return 1U; }   /* 无卡或卡故障 */

  /* 探测卡版本 */
  r1 = SD_SendCmd(8U, 0x1AAU);                  /* CMD8: 检查 v2.0 电压/校验 */
  if (r1 == 0x01U)
  {
    /* ---- SD v2 ---- */
    for (i = 0U; i < 4U; i++)
      ocr[i] = SD_Xfer(0xFFU);
    SD_Release();
    if (ocr[2] != 0x01U || ocr[3] != 0xAAU)     /* 电压窗口不匹配 */
      return 2U;

    /* ACMD41(HCS=1)：等待卡就绪，最多 1 s */
    t0 = HAL_GetTick();
    do {
      (void)SD_SendCmd(55U, 0U);                /* APP 前导 */
      r1 = SD_SendCmd(41U, 1UL << 30);
    } while (r1 != 0U && (HAL_GetTick() - t0) < 1000U);
    if (r1 != 0U)                               /* 卡初始化未完成 */
      return 3U;

    /* CMD58 读 OCR：CCS=1 -> SDHC/SDXC（块寻址） */
    if (SD_SendCmd(58U, 0U) != 0U) { SD_Release(); return 4U; }
    for (i = 0U; i < 4U; i++)
      ocr[i] = SD_Xfer(0xFFU);
    SD_Release();
    sd_type = (ocr[0] & 0x40U) ? (CT_SD2 | CT_SDHC | CT_BLOCK) : CT_SD2;
  }
  else
  {
    /* ---- SD v1 / MMC ---- */
    SD_Release();
    t0 = HAL_GetTick();
    do {
      (void)SD_SendCmd(55U, 0U);
      r1 = SD_SendCmd(41U, 0U);
      if (r1 == 0U) { sd_type = CT_SD1; break; }
      if (r1 == 0x05U)                          /* 不支持 ACMD41 -> MMC */
      {
        sd_type = 0U;
        r1 = SD_SendCmd(1U, 0U);                /* MMC 用 CMD1 */
        if (r1 == 0U) break;
        return 5U;
      }
    } while ((HAL_GetTick() - t0) < 1000U);
    if (r1 != 0U)
      return 6U;

    /* SDSC：固定 512B 块（字节寻址） */
    if (SD_SendCmd(16U, 512U) != 0U) { SD_Release(); return 7U; }
    SD_Release();
  }

  SD_SPI_FastSpeed();           /* 切换 15 MHz */

  sd_ready = 1U;
  return 0U;
}

void SD_DeInit(void)
{
  sd_ready = 0U;
  sd_type = CT_NONE;
}

uint8_t SD_IsReady(void)    { return sd_ready; }
uint8_t SD_GetCardType(void){ return sd_type;  }

/* ------------------------------ 读写块 -------------------------------- */

/**
  * @brief  等待数据块起始令牌 0xFE
  */
static uint8_t SD_WaitToken(uint8_t token)
{
  uint32_t t0 = HAL_GetTick();
  do {
    uint8_t b = SD_Xfer(0xFFU);
    if (b == token)
      return 1U;
    if (b != 0xFFU)             /* 收到错误令牌 */
      return 0U;
  } while ((HAL_GetTick() - t0) < SD_BUSY_TIMEOUT);
  return 0U;
}

/**
  * @brief  计算扇区地址（SDHC 块寻址 / SDSC 字节寻址）
  */
static uint32_t SD_SectorAddr(uint32_t sector)
{
  return (sd_type & CT_SDHC) ? sector : (sector * 512U);
}

uint8_t SD_ReadSectors(uint8_t *buf, uint32_t sector, uint32_t count)
{
  uint8_t ok = 1U;

  if (!sd_ready)
    return 0U;

  while (count--)
  {
    if (SD_SendCmd(17U, SD_SectorAddr(sector)) != 0U)   /* CMD17 单块读 */
    { ok = 0U; break; }

    if (!SD_WaitToken(0xFEU))                            /* 等起始令牌 */
    { SD_Release(); ok = 0U; break; }

    for (uint32_t i = 0U; i < 512U; i++)
      buf[i] = SD_Xfer(0xFFU);
    (void)SD_Xfer(0xFFU);                               /* CRC 高 */
    (void)SD_Xfer(0xFFU);                               /* CRC 低 */

    SD_Release();
    sector++;
    buf += 512U;
  }
  return ok;
}

uint8_t SD_WriteSectors(const uint8_t *buf, uint32_t sector, uint32_t count)
{
  uint8_t ok = 1U;

  if (!sd_ready)
    return 0U;

  while (count--)
  {
    if (!SD_WaitReady() || SD_SendCmd(24U, SD_SectorAddr(sector)) != 0U)
    { SD_Release(); ok = 0U; break; }

    (void)SD_Xfer(0xFEU);                               /* 写起始令牌 */
    for (uint32_t i = 0U; i < 512U; i++)
      (void)SD_Xfer(buf[i]);
    (void)SD_Xfer(0xFFU);                               /* CRC 占位 */
    (void)SD_Xfer(0xFFU);

    uint8_t resp = SD_Xfer(0xFFU);                      /* 数据响应令牌 */
    if ((resp & 0x1FU) != 0x05U)
    { SD_Release(); ok = 0U; break; }

    /* 等待卡内部编程完成 */
    if (!SD_WaitReady())
    { SD_Release(); ok = 0U; break; }

    SD_Release();
    sector++;
    buf += 512U;
  }
  return ok;
}
