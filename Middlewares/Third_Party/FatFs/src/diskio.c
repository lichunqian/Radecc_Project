/**
  ******************************************************************************
  * @file    diskio.c
  * @brief   FatFs 磁盘接口层 —— 对接 bsp_sd_spi.c 的 SPI 模式 SD 卡驱动
  *
  * 驱动器编号：pdrv 0 -> "0:" -> TF 卡（工程仅此一个卷）
  * 时间戳：get_fattime() 从 rtc_config 的 DS3231 RAM 时间镜像取值
  *         （只读内存不走 I2C，主循环上下文调用安全）
  ******************************************************************************
  */
#include "ff.h"         /* FatFs 顶层头 */
#include "diskio.h"     /* 磁盘接口声明 */
#include "bsp_sd_spi.h"
#include "rtc_config.h" /* DS3231 时间镜像（RTC_Now） */

#define DEV_SD  0U      /* 物理驱动器号：0 = SD 卡 */

/* --------------------------------------------------------------------- */

/**
  * @brief  获取磁盘状态
  * @return 0 = 就绪；STA_NOINIT = 尚未初始化
  */
DSTATUS disk_status(BYTE pdrv)
{
  if (pdrv != DEV_SD)
    return STA_NOINIT;

  return SD_IsReady() ? (DSTATUS)0 : STA_NOINIT;
}

/**
  * @brief  初始化磁盘
  */
DSTATUS disk_initialize(BYTE pdrv)
{
  if (pdrv != DEV_SD)
    return STA_NOINIT;

  return (SD_Init() == 0U) ? (DSTATUS)0 : STA_NOINIT;
}

/**
  * @brief  读扇区
  */
DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
  if (pdrv != DEV_SD || count == 0U)
    return RES_PARERR;

  if (!SD_IsReady())
    return RES_NOTRDY;

  return (SD_ReadSectors((uint8_t *)buff, (uint32_t)sector, (uint32_t)count) == 1U)
         ? RES_OK : RES_ERROR;
}

#if _FS_READONLY == 0
/**
  * @brief  写扇区
  */
DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
  if (pdrv != DEV_SD || count == 0U)
    return RES_PARERR;

  if (!SD_IsReady())
    return RES_NOTRDY;

  return (SD_WriteSectors((const uint8_t *)buff, (uint32_t)sector, (uint32_t)count) == 1U)
         ? RES_OK : RES_ERROR;
}
#endif

/**
  * @brief  其它磁盘控制功能
  * @note   本工程未启用 MKFS/TRIM/FASTSEEK，仅需 CTRL_SYNC。
  *         写路径每次都等待卡内部编程完成，故 CTRL_SYNC 直接返回成功。
  */
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
  (void)buff;

  if (pdrv != DEV_SD)
    return RES_PARERR;

  switch (cmd)
  {
    case CTRL_SYNC:
      return SD_IsReady() ? RES_OK : RES_NOTRDY;

    default:
      return RES_PARERR;
  }
}

/**
  * @brief  提供文件/目录的创建与修改时间戳
  * @return FAT 32 位时间戳（日期高 16 位 | 时间低 16 位）
  * @note   数据来自 DS3231 的 RAM 时间镜像（RTC_Now，只读内存，中断安全）
  */
DWORD get_fattime(void)
{
  RTC_DateTime_t dt;
  RTC_Now(&dt);

  uint32_t year = (uint32_t)dt.year + 2000U;    /* dt.year 为 0~99 */
  if (year < 1980U)                             /* 时间无效时的兜底值 */
    year = 2026U;

  uint32_t fat_date = (((year - 1980U) & 0x7FU) << 9)
                    | (((uint32_t)dt.month & 0x0FU) << 5)
                    | ((uint32_t)dt.day & 0x1FU);
  uint32_t fat_time = (((uint32_t)dt.hour & 0x1FU) << 11)
                    | (((uint32_t)dt.min  & 0x3FU) << 5)
                    | (((uint32_t)dt.sec  / 2U) & 0x1FU);

  return (fat_date << 16) | fat_time;
}
