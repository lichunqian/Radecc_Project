/**
  ******************************************************************************
  * @file    ds3231.c
  * @brief   DS3231 高精度实时时钟底层驱动实现
  *
  * 通信接口 : I2C2（PA8 = SDA, PA9 = SCL），标准模式 100 kHz
  * 从机地址 : 0x68（7 位），HAL 接口需要传入左移一位的 0xD0
  * 数据格式 : 时间/日期寄存器全部为 BCD；温度寄存器为有符号二进制
  *
  * 层次关系 :
  *     main.c  ──►  rtc_config.c (RTC_*)  ──►  ds3231.c (DS3231_*)  ──►  HAL_I2C
  *     应用层只认 RTC_* 接口，不直接接触本文件。
  *
  * @warning 本文件所有 API 均为阻塞式 I2C 传输，**禁止在中断上下文调用**。
  *          中断侧读时间请使用 rtc_config.h 的 RTC_NowMs() / RTC_Now()。
  ******************************************************************************
  */

#include "ds3231.h"
#include "main.h"          /* extern I2C_HandleTypeDef hi2c2 */

/* ============================ 内部常量与工具 ============================ */

#define DS3231_TIMEOUT_MS    100U      /* I2C 阻塞传输超时                     */
#define DS3231_PROBE_TRIALS  3U        /* 地址握手重试次数                     */
#define DS3231_RECOVER_CLK   9U        /* 总线恢复时发送的 SCL 脉冲数（≥9）    */

/* ---------------------------------------------------------------------------
 * 微秒级忙等待：用于 I2C 总线恢复时的软件时序。
 * SystemCoreClock = 120 MHz；一次循环约 4 个时钟周期（递减 + 比较分支 + NOP）。
 * ------------------------------------------------------------------------- */
static void i2c_delay_us(uint32_t us)
{
    if (us == 0U)
        return;
    uint32_t n = us * ((SystemCoreClock / 1000000U) / 4U);
    while (n--)
    {
        __NOP();
    }
}

/* ============================== 底层 I2C 收发 ============================== */

static int8_t ds3231_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return (HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)DS3231_I2C_WRITE,
                                    buf, 2, DS3231_TIMEOUT_MS) == HAL_OK) ? 0 : -1;
}

static int8_t ds3231_read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return (HAL_I2C_Mem_Read(&hi2c2, (uint16_t)DS3231_I2C_WRITE, reg,
                             I2C_MEMADD_SIZE_8BIT, buf, len,
                             DS3231_TIMEOUT_MS) == HAL_OK) ? 0 : -1;
}

static int8_t ds3231_write_regs(uint8_t reg, const uint8_t *buf, uint16_t len)
{
    return (HAL_I2C_Mem_Write(&hi2c2, (uint16_t)DS3231_I2C_WRITE, reg,
                              I2C_MEMADD_SIZE_8BIT, (uint8_t *)buf, len,
                              DS3231_TIMEOUT_MS) == HAL_OK) ? 0 : -1;
}

/* ========================== BCD <-> 十进制 转换 ========================== */

static uint8_t bin2bcd(uint8_t val)
{
    return (uint8_t)(((val / 10U) << 4) | (val % 10U));
}

static uint8_t bcd2bin(uint8_t val)
{
    return (uint8_t)(((val >> 4) * 10U) + (val & 0x0FU));
}

/* ---------------------------------------------------------------------------
 * Zeller（蔡勒）公式：由年月日推算星期
 *   入参 m 需为 1~12；当 m < 3 时按前一年的 13、14 月处理。
 *   返回 w: 0 = 周一, 1 = 周二, ... , 6 = 周日
 * ------------------------------------------------------------------------- */
static uint8_t calc_weekday(uint16_t y, uint8_t m, uint8_t d)
{
    int32_t yy = (int32_t)y;
    int32_t mm = (int32_t)m;
    int32_t dd = (int32_t)d;

    if (mm < 3)
    {
        mm += 12;
        yy--;
    }
    int32_t w = (dd + 2 * mm + 3 * (mm + 1) / 5 + yy + yy / 4 - yy / 100 + yy / 400) % 7;

    /* DS3231 的 DAY 寄存器习惯：1 = 周日, 2 = 周一, ... , 7 = 周六 */
    return (uint8_t)(((uint32_t)w + 1U) % 7U + 1U);
}

/* ================================ 公开 API ================================ */

int8_t DS3231_Probe(void)
{
    return (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)DS3231_I2C_WRITE,
                                  DS3231_PROBE_TRIALS,
                                  DS3231_TIMEOUT_MS) == HAL_OK) ? 0 : -1;
}

uint8_t DS3231_IsPresent(void)
{
    return (DS3231_Probe() == 0) ? 1U : 0U;
}

int8_t DS3231_ReadStatus(uint8_t *status)
{
    return ds3231_read_regs(DS3231_REG_STATUS, status, 1);
}

int8_t DS3231_ClearOSF(void)
{
    uint8_t status;
    if (ds3231_read_regs(DS3231_REG_STATUS, &status, 1) != 0)
    {
        return -1;
    }
    status &= (uint8_t)~DS3231_STATUS_OSF;      /* OSF 写 0 清除                */
    /* 同时关闭 32 kHz 输出（默认即为 0，此处显式清零以省电） */
    status &= (uint8_t)~DS3231_STATUS_EN32KHZ;
    return ds3231_write_reg(DS3231_REG_STATUS, status);
}

int8_t DS3231_EnableBatteryOscillator(void)
{
    uint8_t ctrl;
    if (ds3231_read_regs(DS3231_REG_CONTROL, &ctrl, 1) != 0)
    {
        return -1;
    }
    /* EOSC = 1 会让 DS3231 在**电池供电期间主动停振**，这是"掉电丢时间"最常见的
     * 软件根因，必须清零。BBSQW = 1 会在电池供电时输出方波，增加电池消耗，一并清零。 */
    ctrl &= (uint8_t)~(DS3231_CTRL_EOSC | DS3231_CTRL_BBSQW);
    ctrl |= DS3231_CTRL_INTCN;                  /* 保留中断/方波引脚为中断模式   */
    return ds3231_write_reg(DS3231_REG_CONTROL, ctrl);
}

int8_t DS3231_SetDateTime(uint8_t year, uint8_t month, uint8_t day,
                          uint8_t hour, uint8_t min, uint8_t sec)
{
    uint8_t regs[7];

    regs[0] = bin2bcd(sec);
    regs[1] = bin2bcd(min);
    regs[2] = bin2bcd(hour) & 0x3FU;                        /* bit6 = 0 -> 24 小时制 */
    regs[3] = calc_weekday((uint16_t)(2000U + year), month, day);
    regs[4] = bin2bcd(day);
    regs[5] = bin2bcd(month) & 0x1FU;                       /* bit7 = 世纪位，20xx -> 0 */
    regs[6] = bin2bcd(year);

    /* 一次连续写 0x00~0x06，保证秒/分/时/日/月/年在同一时刻生效 */
    if (ds3231_write_regs(DS3231_REG_SECONDS, regs, 7) != 0)
    {
        return -1;
    }

    /* 时间已写入且可信 -> 清除 OSF */
    (void)DS3231_ClearOSF();
    return 0;
}

int8_t DS3231_GetDateTime(uint8_t *year,  uint8_t *month, uint8_t *day,
                          uint8_t *hour,  uint8_t *min,   uint8_t *sec)
{
    uint8_t regs[7];

    /* 一次连续读 0x00~0x06，避免跨秒进位导致的数值撕裂 */
    if (ds3231_read_regs(DS3231_REG_SECONDS, regs, 7) != 0)
    {
        return -1;
    }

    if (sec   != NULL) { *sec   = bcd2bin(regs[0] & 0x7FU); }
    if (min   != NULL) { *min   = bcd2bin(regs[1] & 0x7FU); }
    if (hour  != NULL) { *hour  = bcd2bin(regs[2] & 0x3FU); }  /* 屏蔽 12/24 与 AM/PM 位 */
    if (day   != NULL) { *day   = bcd2bin(regs[4] & 0x3FU); }
    if (month != NULL) { *month = bcd2bin(regs[5] & 0x1FU); }  /* 屏蔽世纪位           */
    if (year  != NULL) { *year  = bcd2bin(regs[6]); }

    return 0;
}

float DS3231_ReadTemp(void)
{
    uint8_t buf[2];

    if (ds3231_read_regs(DS3231_REG_TEMP_MSB, buf, 2) != 0)
    {
        return -99.0f;
    }
    /* 0x11 = 有符号整数部分；0x12 的 bit7:6 = 小数部分（0.25 °C / LSB） */
    int16_t raw = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
    return (float)(raw >> 6) * 0.25f;
}

int8_t DS3231_BusRecover(void)
{
    GPIO_InitTypeDef gpio;
    uint8_t i;

    /* 1) 关闭 I2C2 外设（内部调用 HAL_I2C_MspDeInit -> 复位 PA8/PA9） */
    if (HAL_I2C_DeInit(&hi2c2) != HAL_OK)
    {
        /* 即便失败也继续尝试 GPIO 方式恢复 */
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* 2) PA8/PA9 切为 GPIO 开漏输出，先全部释放为高 */
    gpio.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Mode      = GPIO_MODE_OUTPUT_OD;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = 0U;
    HAL_GPIO_Init(GPIOA, &gpio);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);    /* SCL = 1 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);    /* SDA = 1 */
    i2c_delay_us(5U);

    /* 3) 打 SCL 脉冲：从机（DS3231）每收到一个完整时钟就移出一位数据，
     *    最多 9 个脉冲即可让它把当前字节发完并释放 SDA。 */
    for (i = 0U; i < DS3231_RECOVER_CLK; i++)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
        i2c_delay_us(5U);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
        i2c_delay_us(5U);

        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET)
        {
            break;                                          /* SDA 已释放         */
        }
    }

    /* 4) 产生 STOP 条件：SCL 保持高电平期间，SDA 由低跳变到高 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);   /* SDA = 0 */
    i2c_delay_us(5U);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);     /* SCL = 1 */
    i2c_delay_us(5U);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);     /* SDA 0->1 = STOP */
    i2c_delay_us(5U);

    /* 5) 复位 GPIO 后重新初始化 I2C2（HAL_I2C_Init 内部会调用
     *    HAL_I2C_MspInit 把 PA8/PA9 配回 AF4 开漏复用） */
    (void)HAL_I2C_DeInit(&hi2c2);
    if (HAL_I2C_Init(&hi2c2) != HAL_OK)
    {
        return -1;
    }
    return 0;
}
