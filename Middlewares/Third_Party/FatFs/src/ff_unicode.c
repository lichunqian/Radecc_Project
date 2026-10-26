/**
  ******************************************************************************
  * @file    ff_unicode.c
  * @brief   FatFs R0.12c LFN 所需的 OEM/Unicode 转换函数
  *
  * 背景：ffconf.h 启用长文件名（_USE_LFN 1）后，ff.c 会在"文件名 ASCII/OEM 字符
  *       <-> FAT 目录项 UTF-16 字符"之间调用 ff_convert()，在"长文件名大小写
  *       不敏感比较"时调用 ff_wtoupper()。ChaN 原版这两个函数在 option/unicode.c
  *       （面向 932/936/949/950 双字节码页），本工程裁剪时未携带，链接会报：
  *         L6218E: Undefined symbol ff_convert / ff_wtoupper
  *
  * 实现约定：本工程 _CODE_PAGE 437，文件名与日志内容均为纯 ASCII
  *   （LOG_YYYYMMDD.TXT、时间戳、数值行），因此：
  *   - 0x00~0x7F：恒等映射（ASCII 在 Unicode 中码点相同）；
  *   - >= 0x80  ：映射失败返回 0（FatFs 视为非法字符，不会写入错误字符）；
  *   - ff_wtoupper：仅做 ASCII 小写转大写（0x61~0x7A -> 0x41~0x5A），
  *     其余原样返回，足够支撑 LFN 文件名比较。
  ******************************************************************************
  */
#include "ff.h"

/* OEM(437/ASCII) <-> Unicode 双向转换（dir: 0=OEM->Unicode, 非0=Unicode->OEM） */
WCHAR ff_convert(WCHAR chr, UINT dir)
{
  (void)dir;                    /* ASCII 区间双向映射相同，方向不影响结果 */

  if (chr < 0x80U)
    return chr;                 /* ASCII：OEM 码点与 Unicode 码点一致 */
  return 0U;                    /* 非 ASCII：本工程不使用，返回 0 = 转换失败 */
}

/* Unicode 转大写（用于 LFN 大小写不敏感比较） */
WCHAR ff_wtoupper(WCHAR chr)
{
  if (chr >= (WCHAR)'a' && chr <= (WCHAR)'z')
    return (WCHAR)(chr - 0x20U);   /* ASCII 小写 -> 大写 */
  return chr;
}
