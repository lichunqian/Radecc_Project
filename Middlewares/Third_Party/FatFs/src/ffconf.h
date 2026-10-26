/*---------------------------------------------------------------------------/
/  FatFs - Configuration file  (R0.12c, 适配本工程)
/  基于 STM32Cube_FW_G4_V1.4.0 的 ffconf_template.h 裁剪：
/  - 单卷、512B 扇区、SD 卡 SPI 模式
/  - 启用长文件名（静态缓冲）：日志名 LOG_YYYYMMDD.TXT 超出 8.3 短名限制
/  - 打开字符串函数（f_puts / f_printf，便于日志写入）
/  - 时间戳由 get_fattime() 提供DS3231 真实时间，见 diskio.c）
/---------------------------------------------------------------------------*/

#define _FFCONF 68300	/* Revision ID */

/*---------------------------------------------------------------------------/
/ Function Configurations
/---------------------------------------------------------------------------*/

#define _FS_READONLY	0
/* 0: 允许写（本工程需要往卡里追加日志） */

#define _USE_STRFUNC	1
/* 1: 启用 f_puts / f_printf 等字符串函数 */

#define _USE_FIND		0

#define _USE_MKFS		0
/* 不需要格式化功能（卡在 PC 上预先格式化为 FAT32） */

#define _USE_FASTSEEK	0

#define _USE_EXPAND		0

#define _USE_CHMOD		0

#define _USE_LABEL		0

#define _USE_FORWARD	0

/*---------------------------------------------------------------------------/
/ Locale and Namespace Configurations
/---------------------------------------------------------------------------*/

#define _CODE_PAGE	437
/* 437 = U.S.；日志内容为 ASCII，无需中文码页 */

#define	_USE_LFN	1
/* 1: 启用长文件名（静态缓冲，无需 malloc）。
      ⚠️ 日志文件名 LOG_20261020.TXT 的主名有 12 个字符，超出 8.3 短名限制，
      LFN 关闭时 f_open 会返回 FR_INVALID_NAME，写卡永远失败。
      _MAX_LFN = 32 已覆盖 16 字符的 LOG_YYYYMMDD.TXT；
      静态缓冲占用 (32+1)*2 = 66 字节 RAM，不占栈。 */

#define	_MAX_LFN	32
/* LFN 工作缓冲最大字符数（12~255），须 >= 本工程最长文件名 16 字符 */

#define	_LFN_UNICODE	0

#define _STRF_ENCODE	3

#define _FS_RPATH		0

#define _VOLUMES		1
/* 1 个逻辑驱动器："0:" = SD 卡 */

#define _MIN_SS			512
#define _MAX_SS			512
/* 扇区大小固定 512 B（SD 卡标准值，bsp_sd_spi 读写均按 512B）。
   这两个宏为必填项，缺失时 ff.c 报 "Wrong sector size configuration" */

#define _STR_VOLUME_ID	0

#define _FS_NOFSINFO	0

/*---------------------------------------------------------------------------/
/ System Configurations
/---------------------------------------------------------------------------*/

#define _FS_TINY		0

#define _FS_EXFAT		0

#define _FS_NORTC		0
/* 0: 启用真实时间戳 —— get_fattime() 在 diskio.c 中从 DS3231 RAM 时间镜像取值 */

#define _NORTC_MON		1
#define _NORTC_MDAY		1
#define _NORTC_YEAR		2026

#define _FS_LOCK		0

#define _FS_REENTRANT	0
/* 0: 非重入 —— 所有 FATFS 调用均发生在主循环上下文，无需互斥 */
