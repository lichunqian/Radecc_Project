/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
 * @brief          : 主程序 —— PB0 下降沿外部中断 -> ADC1(PA1) DMA 连续采样10次
 *                    -> 去极值平均判定 -> 有效时 32 位定时器计时 ->
 *                    每次有效识别通过 USART1(DMA) 输出四项数据：
 *                    ① 运行时间 ② 当前时间(外部 DS3231 真实日历) ③ 相邻脉冲间隔(us) ④ 累计脉冲总数
 *                    PA2/PA3(USART2) 每秒输出调试信息（含真实时间/VREF/计数），并可收校时命令
 *
 * @note   时间基准：外部 DS3231（I2C2 = PA8/PA9，自带温补晶振 + 纽扣电池），
 *         **掉电后时间不丢失**。STM32 内部 RTC 已弃用（本板 VBAT 直接接 3.3V，
 *         无备份电池，断电即丢）。
 *         中断侧通过 rtc_config 维护的 RAM 时间镜像读时间，绝不在中断里跑 I2C。
  ******************************************************************************
  * @note   目标芯片：STM32G431CBT6
  *         关键参数：
  *           - 系统时钟 SYSCLK = 120 MHz（由 HSE 经 PLL 产生，见 SystemClock_Config）
  *           - ADC1 时钟 = SYSCLK / 4 = 30 MHz，采样时间 47.5 周期
  *             => 单次转换 = (12.5 + 47.5) 周期 = 60 周期 = 30MHz/60 = 500 KHz（精确）
  *           - TIM2（32 位定时器）计数时钟 = 120 MHz / 120 = 1 MHz，即 1 us 节拍（微秒级精度）
  *             32 位自由计数，单段最大可计 4.29e9 us ≈ 71.58 分钟；
  *             启用更新(溢出)中断，由 timer_overflow_us（64 位）软件累加，覆盖 1 ms ~ 24 h 量程
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rtc_config.h"   /* 系统时间服务（外部 DS3231，掉电不丢失） */
#include "sd_log.h"       /* UART2 输出同步落盘：SD 卡 FATFS 文本日志 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_SAMPLES           10U     /* 每次中断启动后连续转换次数            */
#define ADC_DROP              2U      /* 去掉 1 个最大值 + 1 个最小值         */
#define VREF_MV               3300UL  /* 参考电压 3.3V，单位 mV               */
#define ADC_FULL_SCALE        4095UL  /* 12 位 ADC 满量程码值                 */
#define VALID_THRESHOLD_MV    200UL   /* 判定“中断有效”的电压阈值（>500mV）  */

/* PA6 参考电压采集（动态阈值基准 VREF）相关 */
#define VREF_SAMPLE_PERIOD_MS 1000UL  /* PA6 采集周期：1 秒                        */
#define VREF_FILTER_K         8U      /* 一阶滤波系数：新样本权重 1/K，历史 (K-1)/K */

/* [DBG] 汇总行（经 SDLOG_UART2 写 USART2 + SD 卡）的输出周期。
   行内全部是那一刻的累计值快照，不做任何聚合：
   要拿计数率就用相邻两行的 Total/Rn219/Rn220 自己相减（开机首行提供 0 基准）。
   10 s/条：每天 8640 条 x 约 100 字节 ≈ 0.86 MB，SD 卡与 115200 串口带宽均绰绰有余
   （单条约 100 字节 @115200 ≈ 9 ms 发送时间，远小于 10 s 周期）。
   注意：VREF 采样与 RTC 同步仍保持 1 秒周期，不受本周期影响。 */
#define DBG_LOG_PERIOD_MS     10000UL
#define ADC2_PA6_CHANNEL      ADC_CHANNEL_3  /* PA6 = ADC12_IN3（ADC2 通道 3） */

/* 串口 DMA 发送 FIFO：避免两次有效中断间隔过短导致发送缓冲被覆盖 */
#define UART_TX_FIFO_SLOTS    8U
#define UART_TX_MSG_LEN       96U     /* 四项数据帧最长约 76 字节，留足余量 */

/* ---- 核素延迟符合窗（RaDeCC 法）：按“与上一有效脉冲的间隔”把第二个脉冲归类 ----
 * 219Rn -> 215Po（t1/2 = 1.78 ms）：间隔 10 us ~ 5.6 ms（约 3tau）判为 219Rn
 * 220Rn -> 216Po（t1/2 = 145 ms）：间隔 10 ms ~ 600 ms（约 4tau）判为 220Rn
 * 5.6 ms ~ 10 ms 为两窗之间的死区，不归入任何一种，避免互相污染。
 * 注意：受 ADC 采样窗口（约 21 us）限制，219 窗的实际有效下限约 21 us，
 *       小于该值的间隔会在 EXTI 回调里被计入 D=，进不到本判据。 */
#define RN219_MIN_US          10U        /* 219Rn 窗下限：10 us */
#define RN219_MAX_US          5600U      /* 219Rn 窗上限：5.6 ms */
#define RN220_MIN_US          10000U     /* 220Rn 窗下限：10 ms */
#define RN220_MAX_US          600000U    /* 220Rn 窗上限：600 ms */

/* 脉冲记录环形队列：ADC 中断里只做入队（纯内存拷贝，几十个时钟周期），
   64 位除法格式化与 USART1 发送全部移到主循环，避免把 ~10us 的组帧时间
   压在 ADC 中断的关键路径上（原 UART_SendRecord 直接在 ISR 内调用）。 */
#define PULSE_Q_DEPTH         64U     /* 必须是 2 的幂，用掩码取模 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* 本任务用到的外设句柄（全局，供 stm32g4xx_it.c 的 ISR 引用） */
ADC_HandleTypeDef   hadc1;
UART_HandleTypeDef  huart1;
UART_HandleTypeDef  huart2;             /* PA2/PA3：每秒调试信息输出（轮询发送，不占 DMA） */
I2C_HandleTypeDef   hi2c2;              /* PA8/PA9：I2C2，外接 DS3231 实时时钟（掉电不丢失） */
TIM_HandleTypeDef   htim2;
TIM_HandleTypeDef   htim6;              /* 1ms 单触发，用于 PB5 脉冲下降沿 */
TIM_HandleTypeDef   htim3;             /* 专用“运行时间”自由计数定时器（10kHz，常开，永不被复位） */
ADC_HandleTypeDef   hadc2;              /* PA6 参考电压采集（ADC2，1Hz 单次轮询） */
DMA_HandleTypeDef   hdma_adc1;          /* DMA1_Channel1 -> ADC1 */
DMA_HandleTypeDef   hdma_usart1_tx;     /* DMA1_Channel2 -> USART1_TX */

/* USER CODE BEGIN PV */
/* ---- 应用层状态与缓冲 ---- */
volatile uint8_t  adc_busy = 0;        /* 1 = 正在执行一次 ADC+DMA 采样序列      */
volatile uint8_t  timer_running = 0;   /* 1 = 定时器已启动，正在测量两次有效中断间隔 */
volatile uint8_t  uart_tx_busy = 0;    /* 1 = USART1 TX DMA 正在发送             */

uint16_t adc_buf[ADC_SAMPLES];         /* ADC DMA 接收缓冲（10 个 12 位采样值） */
uint64_t last_interval_us = 0;         /* 最近一次记录到的间隔（us）            */
volatile uint64_t timer_overflow_us = 0; /* TIM2 1us 计数溢出累加：每 ~71.58 min +1，覆盖 24h */
volatile uint32_t pulse_total = 0;     /* 累计脉冲总数（每次有效识别 +1） */
volatile uint32_t edge_drop_total = 0; /* PB0 边沿因上一轮 ADC 采样未完成而被忽略的次数
                                          （偶发近距离脉冲 <25us 会走到这里）；随记录帧 D= 上报 */
volatile uint32_t rn219_total = 0;     /* 219Rn 计数：与上一脉冲间隔落在 10us~5.6ms 的脉冲数 */
volatile uint32_t rn220_total = 0;     /* 220Rn 计数：与上一脉冲间隔落在 10ms~600ms 的脉冲数 */

/* ---- 脉冲记录环形队列（单生产者 = ADC-DMA 中断，单消费者 = 主循环）----
 * 目的：ISR 只保存"间隔 + 计数"两个数值，不碰字符串格式化与串口发送。
 * 队列满时只丢记录并计数，不影响脉冲计数 pulse_total 的准确性。 */
typedef struct
{
  uint64_t interval_us;   /* 与上一次有效脉冲的间隔（us）；首脉冲记为 0 */
  uint32_t count;         /* 该次判定完成后的累计脉冲数 */
} pulse_rec_t;

static pulse_rec_t       pulse_q[PULSE_Q_DEPTH];
static volatile uint16_t pulse_q_head = 0U;    /* 仅在 ADC 中断里推进 */
static volatile uint16_t pulse_q_tail = 0U;    /* 仅在主循环里推进 */
volatile uint32_t        pulse_q_drop = 0U;    /* 队列满导致记录被丢弃的次数 */
volatile uint64_t uptime_ms_total = 0;  /* （保留）原 TIM3 溢出累加；运行时间现改由 RTC 计算，此变量不再用于 U= */
/* 注：原 boot_sec_total（上电时刻 RTC 线性秒基准）已移入 rtc_config.c 内部
   （s_boot_ms），对外通过 RTC_GetUptimeMs() 读取，此处不再保留。 */

/* 串口发送环形 FIFO（每槽固定长度字符串，以 '\0' 结尾） */
static uint8_t  uart_tx_fifo[UART_TX_FIFO_SLOTS][UART_TX_MSG_LEN];
static volatile uint8_t uart_tx_head = 0;  /* 生产者指针 */
static volatile uint8_t uart_tx_tail = 0;  /* 消费者指针 */

/* PA6 采集得到的动态阈值基准（单位 mV），经一阶滤波稳定 */
uint32_t VREF = 0U;               /* PA6 电压（mV）；用作判定阈值基准 */
uint8_t  vref_ready = 0U;         /* 0 = 尚未完成首次采集，判定时回退到固定阈值 */
uint32_t last_vref_tick = 0U;    /* 上次采集 PA6 的 SysTick 时间戳（ms） */
uint32_t last_dbg_log_tick = 0U; /* 上次输出 [DBG] 汇总行的 SysTick 时间戳（ms），周期 DBG_LOG_PERIOD_MS */

/* USART2 接收行缓冲（用于通过串口命令校正 RTC 时间） */
static uint8_t  uart2_rx_byte = 0;            /* HAL_UART_Receive_IT 单字节接收缓冲 */
static char     uart2_rx_line[40];            /* 命令行缓冲（最长约 "set YYYY-MM-DD HH:MM:SS"） */
static uint8_t  uart2_rx_idx = 0;             /* 行缓冲写入下标 */
static volatile uint8_t uart2_cmd_ready = 0;  /* 1 = 收到完整一行，待主循环解析 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM3_Init(void);          /* 运行时间自由计数器（10kHz，常开） */
static void MX_ADC2_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);          /* I2C2: PA8/PA9，外部 DS3231 时钟总线 */
/* USER CODE BEGIN PFP */
static void UART_TrySendNext(void);
static void UART_SendRecord(uint64_t interval_us, uint32_t pulse_count);
static void PulseQ_Push(uint64_t interval_us, uint32_t pulse_count);
static int  fmt_clock_hms(uint32_t total_sec, uint8_t *buf);
static int  fmt_uptime(uint32_t total_sec, uint8_t *buf);
static uint64_t get_uptime_ms(void);    /* 读时间镜像 - 上电基准，得到运行时间(ms)，中断安全 */
static int  fmt_rtc(uint8_t *buf);
static int  u64_to_dec(uint64_t v, uint8_t *buf);
static uint32_t ADC2_ReadPA6(void);
static void DEBUG_Print(void);
static void UART2_CmdProcess(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  尝试从发送 FIFO 取出一条消息启动 USART1 TX DMA
  * @note   仅在 uart_tx_busy==0 且 FIFO 非空时启动；由 TxCplt 回调回收
  */
static void UART_TrySendNext(void)
{
  if (uart_tx_busy)
    return;
  if (uart_tx_head == uart_tx_tail)   /* FIFO 空 */
    return;

  uint8_t *msg = uart_tx_fifo[uart_tx_tail];
  uint16_t len = 0U;
  while (len < (UART_TX_MSG_LEN - 1U) && msg[len] != '\0')
    len++;

  uart_tx_busy = 1;
  if (HAL_UART_Transmit_DMA(&huart1, msg, len) == HAL_OK)
  {
    /* 注意：发送槽在 DMA 完成前不能被回收，tail 的推进放在
       HAL_UART_TxCpltCallback 中，避免下一帧覆盖正在发送的数据 */
  }
  else
  {
    uart_tx_busy = 0;   /* 启动失败，下一帧或下次调用重试 */
  }
}

/**
  * @brief  将 uint64_t 转为十进制 ASCII（不含结束符），返回长度（不含 '\0'）
  * @note   ARMCC microlib 的 snprintf 对 %llu 支持不完善，故手写实现。
  *         buf 至少需 21 字节（2^64 十进制最多 20 位）。
  */
static int u64_to_dec(uint64_t v, uint8_t *buf)
{
  uint8_t tmp[21];
  int i = 0;
  if (v == 0U)
  {
    tmp[i++] = '0';
  }
  else
  {
    while (v > 0U && i < 21)
    {
      tmp[i++] = (uint8_t)('0' + (int)(v % 10U));
      v /= 10U;
    }
  }
  int k = 0;
  while (k < i)
  {
    buf[k] = tmp[i - 1 - k];
    k++;
  }
  return k;
}

/**
  * @brief  将秒数格式化为 "HH:MM:SS"（每域 2 位），返回写入长度 8（不含 '\0'）
  * @note   用于"当前时间"与"运行时间"中的时分秒部分。
  */
static int fmt_clock_hms(uint32_t total_sec, uint8_t *buf)
{
  uint32_t h = total_sec / 3600U;
  uint32_t m = (total_sec % 3600U) / 60U;
  uint32_t s = total_sec % 60U;
  buf[0] = (uint8_t)('0' + (h / 10U)); buf[1] = (uint8_t)('0' + (h % 10U));
  buf[2] = ':';
  buf[3] = (uint8_t)('0' + (m / 10U)); buf[4] = (uint8_t)('0' + (m % 10U));
  buf[5] = ':';
  buf[6] = (uint8_t)('0' + (s / 10U)); buf[7] = (uint8_t)('0' + (s % 10U));
  return 8;
}

/**
  * @brief  将"自启动以来的运行时间"格式化为 "Dd HH:MM:SS"，返回写入长度
  */
static int fmt_uptime(uint32_t total_sec, uint8_t *buf)
{
  uint32_t d = total_sec / 86400U;
  int n = u64_to_dec(d, buf);            /* 天数（1 位以上） */
  buf[n++] = 'd';
  n += fmt_clock_hms(total_sec % 86400U, buf + n);
  return n;
}

/**
  * @brief  读取“自启动以来的运行时间”（ms）
  * @note   数据来自 rtc_config 维护的**时间镜像**（RTC_GetUptimeMs）：
  *         - 镜像由主循环每 1 s 通过 I2C 从 DS3231 刷新一次；
  *         - 本函数只读 RAM + SysTick 外推，**不触碰 I2C**，因此可安全地在
  *           中断上下文（HAL_ADC_ConvCpltCallback -> UART_SendRecord）中调用，
  *           不会像"中断里跑阻塞 I2C"那样把脉冲处理时间拖长上千倍；
  *         - 分辨率提升到 ms 级（旧实现受限于 RTC 秒寄存器，只能到 1 s）。
  *         运行时间零点 = 上电时 RTC_CaptureBoot() 抓取的基准。
  */
static uint64_t get_uptime_ms(void)
{
  return RTC_GetUptimeMs();
}

/**
  * @brief  组装并发送一条包含四项数据的记录
  * @param  interval_us : 相邻两次脉冲间隔（us）；首次脉冲记为 0
  * @param  pulse_count : 累计脉冲总数
 * @note   帧格式（例）：U=0d00:00:05,T=2026-10-14 22:07:53,I=123456us,N=1,D=0,Q=0\r\n
 *         ① U= 运行时间（自启动，Dd HH:MM:SS）
 *         ② T= 当前时间（RTC 真实日历，YYYY-MM-DD HH:MM:SS）
 *         ③ I= 相邻脉冲间隔(us)  ④ N= 累计脉冲总数
 *         ⑤ D= 因上一轮采样未完成被忽略的边沿累计数（近距离脉冲丢边沿）
 *         ⑥ Q= 记录队列满被丢弃的条数（脉冲率超过串口带宽）
 * @note   本函数在主循环上下文执行（由出队逻辑调用），不在中断里运行。
 */
static void UART_SendRecord(uint64_t interval_us, uint32_t pulse_count)
{
  uint8_t slot = uart_tx_head;
  uint8_t next = (uint8_t)((slot + 1U) % UART_TX_FIFO_SLOTS);
  if (next == uart_tx_tail)           /* FIFO 满，丢弃 */
    return;

  uint8_t *msg = uart_tx_fifo[slot];
  int n = 0;
  /* ① 运行时间(ms)：U = 当前 DS3231 线性时间 - 上电基准（rtc_config 内的 s_boot_ms），
     在每帧实时计算，与 ② T= 同源于 DS3231，保证同步增长（不再依赖易失缓存）。
     读的是 RAM 时间镜像，中断安全；分辨率由原来的 1 s 提升到 1 ms。 */
  uint64_t uptime_ms = get_uptime_ms();
  uint32_t up_sec = (uint32_t)(uptime_ms / 1000U);

  /* ① 设备自启动以来的运行时间 */
  msg[n++] = 'U'; msg[n++] = '=';
  n += fmt_uptime(up_sec, msg + n);
  msg[n++] = ',';
  /* ② 当前时间（RTC 真实日历） */
  msg[n++] = 'T'; msg[n++] = '=';
  n += fmt_rtc(msg + n);
  msg[n++] = ',';
  /* ③ 相邻两次脉冲之间的时间间隔（us） */
  msg[n++] = 'I'; msg[n++] = '=';
  n += u64_to_dec(interval_us, msg + n);
  msg[n++] = 'u'; msg[n++] = 's';
  msg[n++] = ',';
  /* ④ 累计脉冲总数 */
  msg[n++] = 'N'; msg[n++] = '=';
  n += u64_to_dec((uint64_t)pulse_count, msg + n);
  /* ⑤ 因上一轮采样未完成而被忽略的边沿累计数（诊断项：正常应为 0 或很小） */
  msg[n++] = ',';
  msg[n++] = 'D'; msg[n++] = '=';
  n += u64_to_dec((uint64_t)edge_drop_total, msg + n);
  /* ⑥ 记录队列满被丢弃的条数（诊断项：持续增长说明脉冲率超过串口带宽） */
  msg[n++] = ',';
  msg[n++] = 'Q'; msg[n++] = '=';
  n += u64_to_dec((uint64_t)pulse_q_drop, msg + n);

  msg[n++] = '\r'; msg[n++] = '\n'; msg[n] = '\0';

  uart_tx_head = next;
  UART_TrySendNext();
}

/**
  * @brief  脉冲记录入队（只在 ADC-DMA 完成回调里调用）
  * @param  interval_us : 与上一次有效脉冲的间隔（us），首脉冲传 0
  * @param  pulse_count : 当前累计脉冲数
  * @note   纯内存写入，无字符串格式化、无外设操作，耗时几十个时钟周期。
  *         队列满时丢弃本条记录并累加 pulse_q_drop——记录可丢，但
  *         pulse_total 计数不受影响，D=/Q= 字段保证事后可核对。
  */
static void PulseQ_Push(uint64_t interval_us, uint32_t pulse_count)
{
  uint16_t next = (uint16_t)((pulse_q_head + 1U) & (PULSE_Q_DEPTH - 1U));

  if (next == pulse_q_tail)          /* 队列满：主循环跟不上（串口带宽所限） */
  {
    pulse_q_drop++;
    return;
  }

  pulse_q[pulse_q_head].interval_us = interval_us;
  pulse_q[pulse_q_head].count       = pulse_count;
  pulse_q_head = next;               /* 最后再更新 head，保证消费者看不到半成品 */
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU 配置：HAL 初始化（含 SysTick） */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* 配置系统时钟（SYSCLK = 120 MHz） */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* 初始化所有用到的外设。
     注意：MX_DMA_Init / MX_USART1_UART_Init / MX_ADC1_Init 的调用顺序
     必须保证 DMA 句柄先于对应外设被链接（__HAL_LINKDMA）。 */

  MX_GPIO_Init();      /* PB0 下降沿 EXTI + NVIC；PA1 由 MSP 配置为模拟 */
  MX_DMA_Init();       /* 初始化并链接 DMA1_Channel1(ADC) / Channel2(UART TX) */
  MX_TIM2_Init();      /* 32 位定时器，1 us 节拍（溢出中断扩展至 24h） */
  MX_TIM3_Init();      /* 自由计数器（10kHz，常开）——U= 已改由 DS3231 计算，此定时器仅备用/扩展 */
  MX_TIM6_Init();      /* 1 ms 单触发脉冲发生器（PB5） */
  MX_ADC2_Init();      /* PA6 / CH3，每秒单次轮询采样，作为动态阈值基准 VREF */
  MX_ADC1_Init();      /* PA1 / CH2，连续转换 + DMA，采样率 500 KHz */
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();   /* PA2/PA3：调试串口（每秒输出一次 + 接收校时命令） */
  MX_I2C2_Init();          /* PA8/PA9：I2C2 总线，必须先于 RTC_Config_Init 初始化 */
  RTC_Config_Init();       /* 系统时基：外部 DS3231（自带电池，掉电不丢失）；
                              内部已自动抓取开机基准，无需再调用 capture_boot_time */

  /* SD 卡文件系统初始化：挂载卷 + 打开/创建当日日志文件（追加模式）。
     必须在 RTC_Config_Init 之后（文件名要取 DS3231 日期）；
     必须在下方第一条 UART2 输出之前，让开机横幅同样落盘。
     卡未插时返回 0，不影响运行，sd_log 内部每 5s 静默重试。 */
  SDLOG_Init();

  /* USER CODE BEGIN DBG_BOOT */
  /* 上电自检横幅：复位后约 100ms 内即通过 USART2(PA2/PA3) 输出一行。
     若 PC 端串口工具能看到此行，说明 USART2 硬件/时钟/波特率均正常；
     若看不到，则问题在系统时钟(HSE)、PA2/PA3 接线 或 PC 端端口/波特率设置。
     命令（USART2 输入一行）：
        set YYYY-MM-DD HH:MM:SS   —— 校时（例：set 2026-10-19 14:30:00）
        time                      —— 查询时基来源、DS3231 状态与温度 */
  {
    static const char boot_msg[] =
      "USART2 debug ready (115200,8N1)\r\n"
      "[RTC] src=DS3231: 'set YYYY-MM-DD HH:MM:SS' to calibrate, 'time' to query\r\n";
    SDLOG_UART2((uint8_t *)boot_msg,
                (uint16_t)(sizeof(boot_msg) - 1U));

    /* 时钟自检结果：DS3231 是否在线、是否曾用编译时间重置（OSF）、当前时间。
       这是判断"掉电后时间是否真的不丢失"最直接的现场依据。 */
    {
      char rtc_msg[128];
      RTC_DateTime_t dt;
      RTC_Now(&dt);
      int rn = snprintf(rtc_msg, sizeof(rtc_msg),
                        "[RTC] src=%s present=%u reset=%u syncfail=%lu now=20%02u-%02u-%02u %02u:%02u:%02u\r\n",
                        RTC_GetSourceName(),
                        (unsigned int)RTC_IsPresent(),
                        (unsigned int)RTC_WasTimeReset(),
                        (unsigned long)RTC_GetSyncFailCount(),
                     (unsigned int)dt.year, (unsigned int)dt.month, (unsigned int)dt.day,
                     (unsigned int)dt.hour, (unsigned int)dt.min,   (unsigned int)dt.sec);
      if (rn > 0)
      {
        /* snprintf 返回"本应写入"的长度，超长时必须钳位，否则会越界发送 */
        if (rn > (int)sizeof(rtc_msg) - 1) { rn = (int)sizeof(rtc_msg) - 1; }
        SDLOG_UART2((uint8_t *)rtc_msg, (uint16_t)rn);
      }
      if (RTC_IsPresent() == 0U)
      {
        static const char warn[] =
          "[RTC] WARN: DS3231 not found! Check PA8/PA9, I2C pull-ups, DS3231 power/VBAT.\r\n";
        SDLOG_UART2((uint8_t *)warn,
                    (uint16_t)(sizeof(warn) - 1U));
      }
      else if (RTC_WasTimeReset() != 0U)
      {
        static const char warn[] =
          "[RTC] WARN: oscillator had stopped (OSF=1). Check backup battery; "
          "time reset to build time.\r\n";
        SDLOG_UART2((uint8_t *)warn,
                    (uint16_t)(sizeof(warn) - 1U));
      }
    }

    /* 开机首条 [DBG] 快照：上电后立即落一行，作为后续所有差分的 0 基准。
       行内 Total/Rn219/Rn220 全为 0、Seq=1，看到它即说明 USART2 + SD 卡链路已通；
       之后每满 DBG_LOG_PERIOD_MS(10 s) 再落一行，相邻两行相减即得该时段计数率。
       先读一次 PA6 只为让首行 VREF 显示真实电压（仅显示用途）：
       不置 vref_ready，阈值判定仍等 1s 周期块建立基准后才切换到动态阈值。 */
    if (vref_ready == 0U)
    {
      VREF = (uint32_t)((uint32_t)ADC2_ReadPA6() * VREF_MV / ADC_FULL_SCALE);
    }
    last_dbg_log_tick = HAL_GetTick();   /* 以开机时刻作为 10 s 周期的起点 */
    DEBUG_Print();
  }
  /* 启动 USART2 单字节接收中断，用于接收校时命令 */
  if (HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END DBG_BOOT */

  /* USER CODE BEGIN 2 */
  /* ADC 电压调节器已在 MX_ADC1_Init()->HAL_ADC_Init() 内部使能并等待稳压完成，
     此处直接执行校准以提升精度（G4 系列建议上电后校准一次）。 */
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
    Error_Handler();
  /* USER CODE END 2 */

  /* 主循环：业务逻辑均由中断/回调驱动；PC13 改为“脉冲识别时翻转”，此处空转休眠 */
  while (1)
  {
    /* USER CODE BEGIN WHILE */
    /* 串口帧只在“脉冲识别”时产生：中断回调只入队（PulseQ_Push），
       主循环在上方出队并组帧发送，避免在 ADC 中断里做 64 位除法格式化。
       无脉冲时队列为空，不会产生重复帧。此处仅处理 USART2 校时命令。 */

    /* 脉冲记录出队：由主循环组帧并启动 USART1 TX DMA。
       一次只发一条，靠内部 8 槽 FIFO 与队列共同吸收突发；SysTick 每 1ms
       唤醒一次本循环，队列深度 64 足以跨过 RTC_Sync / SD 写卡的主循环停顿。 */
    if (!uart_tx_busy && (pulse_q_tail != pulse_q_head))
    {
      uint64_t iv  = pulse_q[pulse_q_tail].interval_us;
      uint32_t cnt = pulse_q[pulse_q_tail].count;
      pulse_q_tail = (uint16_t)((pulse_q_tail + 1U) & (PULSE_Q_DEPTH - 1U));
      UART_SendRecord(iv, cnt);
    }

    /* 处理 USART2 收到的校时命令（若有完整一行） */
    if (uart2_cmd_ready)
    {
      uart2_cmd_ready = 0;
      UART2_CmdProcess();
    }

    /* 每秒采集一次 PA6 电压 -> 动态阈值基准 VREF（mV），做一阶滤波以抑制抖动 */
    if ((HAL_GetTick() - last_vref_tick) >= VREF_SAMPLE_PERIOD_MS)
    {
      last_vref_tick = HAL_GetTick();

      /* 每秒用 DS3231 刷新一次 RAM 时间镜像（阻塞式 I2C，只能放在主循环）。
         放在 1s 周期块的开头，这样本周期后续输出的时间戳都是最新值。
         ⚠️ 绝不能放到中断里：一次 I2C 读约 0.5~1 ms，会毁掉微秒级计时精度。 */
      RTC_Sync();

      uint32_t raw = ADC2_ReadPA6();                       /* 12 位原始码值 */
      uint32_t mv  = raw * VREF_MV / ADC_FULL_SCALE;       /* 换算为 mV，单位与阈值一致 */

      if (!vref_ready)
      {
        VREF = mv;            /* 首次采集：直接赋值作为初值 */
        vref_ready = 1U;
      }
      else
      {
        /* 一阶指数滑动平均：VREF = VREF*(K-1)/K + mv/K，K = VREF_FILTER_K */
        VREF = (VREF * (VREF_FILTER_K - 1U) + mv) / VREF_FILTER_K;
      }
    }

    /* [DBG] 汇总行：每 10 秒 1 次（周期 DBG_LOG_PERIOD_MS）。
       经 SDLOG_UART2 同时送 USART2 与 SD 卡；行内是累计值快照，不聚合。
       开机那一条在 DBG_BOOT 里已输出并把 last_dbg_log_tick 置为开机时刻，
       所以这里是开机后第 1、2、3... 个 10 秒的整点，不是上电后随机相位。
       VREF 采样与 RTC 同步仍在上面的 1 秒块里，不随本周期放慢。 */
    if ((HAL_GetTick() - last_dbg_log_tick) >= DBG_LOG_PERIOD_MS)
    {
      last_dbg_log_tick = HAL_GetTick();
      DEBUG_Print();                 /* 轮询发送，主循环上下文，无阻塞风险 */
    }

    __WFI();                                   /* 进入休眠，等待中断唤醒（SysTick 1ms 唤醒后再次检查 1s 周期） */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  * @note  取 HSE（本板假设 8 MHz）作为 PLL 输入：
  *        VCO = HSE / PLLM(2) * PLLN(60) = 8/2*60 = 240 MHz
  *        PLLP / PLLQ / PLLR 均 DIV2 => 各路 120 MHz
  *        SYSCLK = 120 MHz，AHB/APB1/APB2 均不分频 => 120 MHz
  *        ADC1 时钟源 = SYSCLK，预分频 /4 => 30 MHz
  *        TIM2 时钟 = APB1(120 MHz)，因 APB1 预分频=1，定时器时钟=120 MHz
  *   若你的板载 HSE 不是 8 MHz，请相应调整 PLLN，或改为经 HSI 配置。
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** 配置主稳压器输出电压（Boost 模式支持到 170 MHz，120 MHz 余量充足） */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** 初始化 RCC 振荡器（PLL 由 HSE 提供） */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** 初始化 CPU、AHB、APB 总线时钟 */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @retval None
  * @note   PB0 -> EXTI0 下降沿触发输入（外部中断源）
  *         PA1 -> ADC1_IN2 模拟输入（在 HAL_ADC_MspInit 中配置）
  *         USART1 TX/RX 在 HAL_UART_MspInit 中配置（PB6/PB7）
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO 端口时钟使能 */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin : PB0（外部中断输入，下降沿） */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;   /* 下降沿触发 EXTI0 */
  GPIO_InitStruct.Pull = GPIO_PULLUP;            /* 外部电路已处理上下拉可改为 PULLUP */
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC13（脉冲识别指示灯，每次有效识别时翻转） */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;       /* 推挽输出 */
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;      /* PC13 驱动能力有限，用低速即可 */
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5（脉冲识别时在有效中断上输出 1ms 脉冲） */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;       /* 推挽输出 */
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* EXTI0 中断优先级与使能（PB0 映射到 EXTI 线 0） */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  /* USER CODE END MX_GPIO_Init_2 */
}

/**
  * @brief DMA Initialization Function
  * @retval None
  * @note   DMA1_Channel1 -> ADC1（外设到内存，半字，普通模式）
  *         DMA1_Channel2 -> USART1_TX（内存到外设，字节，普通模式）
  *         STM32G4 通过 DMAMUX 将 DMA 通道映射到外设请求，二者无硬冲突。
  */
static void MX_DMA_Init(void)
{
  /* USER CODE BEGIN MX_DMA_Init_1 */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
  /* USER CODE END MX_DMA_Init_1 */

  /* ---- ADC1 RX DMA ---- */
  hdma_adc1.Instance = DMA1_Channel1;
  hdma_adc1.Init.Request = DMA_REQUEST_ADC1;            /* DMAMUX: ADC1 */
  hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc1.Init.Mode = DMA_NORMAL;                     /* 非循环：传完 10 个即停止 */
  hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
  if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

  /* ---- USART1 TX DMA ---- */
  hdma_usart1_tx.Instance = DMA1_Channel2;
  hdma_usart1_tx.Init.Request = DMA_REQUEST_USART1_TX;  /* DMAMUX: USART1_TX */
  hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_usart1_tx.Init.Mode = DMA_NORMAL;
  hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_LINKDMA(&huart1, hdmatx, hdma_usart1_tx);

  /* USER CODE BEGIN MX_DMA_Init_2 */
  /* DMA 通道中断优先级与使能（用于传输完成回调） */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);   /* ADC DMA 最高 */
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 3, 0);   /* UART TX DMA */
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* USER CODE END MX_DMA_Init_2 */
}

/**
  * @brief TIM2 Initialization Function（32 位计时器）
  * @retval None
  * @note   计数时钟 = 120 MHz / 120 = 1 MHz => 1 us/计数（微秒级精度）
  *         32 位自由运行，ARR = 0xFFFFFFFF，单段最大 ≈ 71.58 分钟；
  *         启用更新(溢出)中断，由 timer_overflow_us 64 位累加，覆盖 1 ms ~ 24 h。
  *         注：本设计采用“常开自由计数 + 在有效中断处清零”，
  *         也可改为仅在首次有效中断时 HAL_TIM_Base_Start。
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM2_Init 0 */
  __HAL_RCC_TIM2_CLK_ENABLE();   /* 确保 TIM2 时钟使能 */
  /* USER CODE END TIM2_Init 0 */

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 120U - 1U;             /* 16 位 PSC：120 MHz / 120 = 1 MHz（1 us/计数） */
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFFU;              /* 32 位满量程，自由运行 */
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 1 */
  /* 预启动定时器（自由运行 + 溢出中断），首次有效中断时清零 CNT 即等效“启动计时” */
  HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);   /* 低于 ADC-DMA(1)/TIM6(4)/USART1(4)，不抢占采样处理 */
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END TIM2_Init 1 */
}

/**
  * @brief TIM6 Initialization Function（1ms 单触发脉冲发生器）
  * @retval None
  * @note   用于“脉冲识别”时在 PB5 输出精确 1ms 脉冲：
  *         TIM6 时钟 = APB1(120 MHz)，PSC = 12000-1 => 10 kHz（0.1 ms/计数），
  *         ARR = 10-1 => 10 计数 = 1 ms。有效中断里置 PB5 高并 HAL_TIM_Base_Start_IT，
  *         1ms 后 TIM6 更新中断把 PB5 拉低并停止定时器。
  *         注意：不能在 ADC-DMA 中断回调里用 HAL_Delay 延时（SysTick 优先级更低会死锁），
  *         故必须用硬件定时器产生脉冲。
  */
static void MX_TIM6_Init(void)
{
  /* USER CODE BEGIN TIM6_Init 0 */
  __HAL_RCC_TIM6_CLK_ENABLE();   /* 确保 TIM6 时钟使能 */
  /* USER CODE END TIM6_Init 0 */

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 12000U - 1U;          /* 16 位 PSC：120 MHz / 12000 = 10 kHz（0.1 ms/计数） */
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10U - 1U;                /* 10 计数 @ 0.1 ms = 1 ms（单触发） */
  htim6.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 1 */
  /* 使能 TIM6 更新中断（优先级低于 ADC-DMA(1)，避免抢占采样完成处理） */
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  /* PB5 初值低（无脉冲） */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
  /* USER CODE END TIM6_Init 1 */
}

/**
  * @brief TIM3 Initialization Function（保留的通用定时器，现不再用于运行时间）
  * @retval None
  * @note   运行时间 U= 已改为由外部 DS3231 计算（见 get_uptime_ms / rtc_config.c 的 s_boot_ms），
  *         因为 G431 的 TIM3 为 16 位定时器（ARR 被钳到 0xFFFF，每 ~6.55s 回绕），
  *         且其溢出中断未能可靠累加 64 位基准。此处保留 TIM3 为常开自由计数，仅供调试/扩展。
  *         时钟 = APB1(120 MHz)，PSC = 12000-1 => 10 kHz（0.1 ms/计数），32 位 ARR（硬件按 16 位有效）。
  */
static void MX_TIM3_Init(void)
{
  /* USER CODE BEGIN TIM3_Init 0 */
  __HAL_RCC_TIM3_CLK_ENABLE();   /* 确保 TIM3 时钟使能（基础定时器，无 GPIO，无需 MSP） */
  /* USER CODE END TIM3_Init 0 */

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 12000U - 1U;                 /* 16 位 PSC：120 MHz / 12000 = 10 kHz */
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFFFFFFU;                    /* 32 位满量程，自由运行 */
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 1 */
  /* 启动自由运行并允许溢出中断（用于 64 位运行时间累加） */
  uptime_ms_total = 0U;
  HAL_NVIC_SetPriority(TIM3_IRQn, 6, 0);   /* 最低优先级，不抢占采样/串口处理 */
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END TIM3_Init 1 */
}

/**
  * @brief ADC2 Initialization Function（PA6 参考电压采集）
  * @retval None
  * @note   通道：PA6 -> ADC2_IN3（ADC12_IN3，通道 3）
  *         单次转换模式 + 软件触发，由主循环每秒轮询一次（不使用 DMA/中断）。
  *         采样时钟 = SYSCLK/4 = 30 MHz（与 ADC1 一致）；采用较长采样时间提升精度。
  *         注意：本工程 MSP 的 ADC2 分支默认将 PA1 配为模拟，此处按需求改为 PA6；
  *               PA6 的模拟配置在下面内联完成（保持改动集中在 main.c）。
  */
static void MX_ADC2_Init(void)
{
  /* USER CODE BEGIN ADC2_Init 0 */
  /* PA6 模拟输入配置（MSP ADC2 分支已配 PA6；此处再确保一次，
     以防 CubeMX 用旧 .ioc 重新生成代码时丢失该引脚配置） */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef gpio_init = {0};
  gpio_init.Pin = GPIO_PIN_6;
  gpio_init.Mode = GPIO_MODE_ANALOG;
  gpio_init.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &gpio_init);
  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */
  /* USER CODE END ADC2_Init 1 */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;  /* ADC_CLK = 30 MHz，与 ADC1 一致 */
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;            /* 单通道 */
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;               /* 单次转换，主循环轮询驱动 */
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;      /* 软件触发 */
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;            /* 不使用 DMA */
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** 配置规则通道：PA6 -> ADC2_IN3（ADC12_IN3，通道 3） */
  sConfig.Channel = ADC2_PA6_CHANNEL;                     /* ADC_CHANNEL_3 */
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;      /* 较长采样时间，提升参考电压精度 */
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN ADC2_Init 2 */
  /* G4 系列建议上电后校准一次；HAL_ADC_Init 内部已使能稳压器并等待其稳定 */
  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END ADC2_Init 2 */
}

/**
  * @brief ADC1 Initialization Function
  * @retval None
  * @note   通道：ADC1_IN2 映射到 PA1
  *         连续转换模式 + DMA 连续请求：启动一次即连续采样，
  *         DMA 传送满 10 个后触发传输完成回调，在回调中 HAL_ADC_Stop_DMA 自动停止。
  *         采样率 = ADC_CLK / (12.5 + SamplingTime)
  *                = 30 MHz / (12.5 + 47.5) = 30 MHz / 60 = 500 KHz（精确）
  */
static void MX_ADC1_Init(void)
{
  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;  /* ADC_CLK = SYSCLK/4 = 30 MHz */
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;            /* 单通道 */
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;               /* 连续转换，直到 DMA 停止 */
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;      /* 由 HAL_ADC_Start_DMA 软件触发 */
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;            /* DMA 请求连续产生 */
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** 配置规则通道：PA1 -> ADC1_IN2 */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;     /* 47.5 周期采样 -> (12.5+47.5)=60 周期 = 30MHz/60 = 500 KHz（精确达标） */
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
  * @brief USART1 Initialization Function
  * @retval None
  * @note   波特率 115200，8N1；TX 使用 DMA（句柄在 MX_DMA_Init 中链接）
  *         引脚：PB6(TX)/PB7(RX)，AF7（见 HAL_UART_MspInit）
  */
static void MX_USART1_UART_Init(void)
{
  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
  * @brief USART2 Initialization Function（PA2/PA3 调试串口）
  * @retval None
  * @note   波特率 115200，8N1；TX=PA2 / RX=PA3，AF7。
  *         引脚与时钟由 HAL_UART_MspInit 自动配置（见 stm32g4xx_hal_msp.c 的
  *         USART2 分支：PA2/PA3 -> GPIO_AF7_USART2，时钟源 PCLK1）。
  *         本串口仅用于每秒打印调试信息，采用轮询发送（HAL_UART_Transmit），
  *         不占用 DMA 通道与中断资源，简单可靠。
  */
static void MX_USART2_UART_Init(void)
{
  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
  * @brief I2C2 Initialization Function（外部 DS3231 实时时钟总线）
  * @retval None
  *
  * 硬件：PA8 = I2C2_SDA，PA9 = I2C2_SCL（复用功能 AF4，开漏 + 内部上拉）
  *      引脚复用与时钟使能由 HAL_I2C_MspInit 完成（见 stm32g4xx_hal_msp.c）。
  *
  * 速率：标准模式 100 kHz（DS3231 最高支持 400 kHz；本板走线较长且有跳线，
  *      100 kHz 抗干扰更好，且每秒只同步一次时间，速率不敏感）。
  *
  * Timing 参数推算（I2C2 内核时钟 = PCLK1 = 120 MHz，tI2CCLK = 8.333 ns）：
  *      tPRESC  = (PRESC  + 1) * tI2CCLK = (7 + 1) * 8.333  = 66.67 ns
  *      tSCLH   = (SCLH   + 1) * tPRESC  = (74 + 1) * 66.67 = 5000   ns  (>= 4000 ns ✓)
  *      tSCLL   = (SCLL   + 1) * tPRESC  = (74 + 1) * 66.67 = 5000   ns  (>= 4700 ns ✓)
  *      tSCLDEL = (SCLDEL + 1) * tPRESC  = (4  + 1) * 66.67 = 333    ns  (>=  250 ns ✓)
  *      tSDADEL =   SDADEL     * tPRESC  =   2      * 66.67 = 133    ns  (>=    0 ns ✓)
  *      => tSCL = tSCLH + tSCLL = 10000 ns，即恰好 100 kHz
  *      Timing = (PRESC<<28)|(SCLDEL<<20)|(SDADEL<<16)|(SCLH<<8)|SCLL = 0x70424A4A
  *
  * @note 若把 PCLK1 改成其它频率，上面的 Timing 必须重算，否则 I2C 速率会偏离。
  */
static void MX_I2C2_Init(void)
{
  hi2c2.Instance             = I2C2;
  hi2c2.Init.Timing          = 0x70424A4AU;      /* 100 kHz @ I2CCLK = 120 MHz，见上方推算 */
  hi2c2.Init.OwnAddress1     = 0U;               /* MCU 作为主机，无需自身地址 */
  hi2c2.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2     = 0U;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN I2C2_Init 2 */
  /* 使能模拟噪声滤波器（默认已开）；DS3231 在 100 kHz 下无需配置数字滤波器 */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END I2C2_Init 2 */
}

/* USER CODE BEGIN 4 */
/* ====================== 中断回调与处理逻辑 ====================== */

/**
  * @brief  在 PB5 上触发一个 1ms 高电平脉冲
  * @note   由 ADC-DMA 完成回调（脉冲识别有效时）调用，运行于中断上下文。
  *         置 PB5 高后启动 TIM6（1ms 单触发），到时由
  *         HAL_TIM_PeriodElapsedCallback 拉低，避免在 ISR 内用 HAL_Delay 死锁。
  */
static void Pulse_TriggerPB5(void)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);   /* 脉冲起始：PB5 高 */
  __HAL_TIM_SET_COUNTER(&htim6, 0U);
  HAL_TIM_Base_Start_IT(&htim6);                        /* 1ms 后触发更新中断拉低 */
}

/**
  * @brief  对 PA6（ADC2_IN3）做一次单次轮询采样，返回 12 位原始码值
  * @note   由主循环每秒调用一次。阻塞时间极短（约 (12.5+247.5)/30MHz ≈ 8.7 us），
  *         不会明显影响其它中断响应。采样后即停止 ADC2，避免长期占用。
  * @retval 12 位 ADC 原始码值（0 ~ 4095）；若转换启动/等待失败返回 0。
  */
static uint32_t ADC2_ReadPA6(void)
{
  uint32_t val = 0U;
  if (HAL_ADC_Start(&hadc2) == HAL_OK)
  {
    if (HAL_ADC_PollForConversion(&hadc2, 2U) == HAL_OK)  /* 2ms 超时，余量充足 */
    {
      val = HAL_ADC_GetValue(&hadc2);
    }
    HAL_ADC_Stop(&hadc2);                               /* 单次转换后停止 */
  }
  return val;
}

/**
  * @brief  通过 USART2(PA2/PA3) 输出一行汇总信息（每 10 秒一次）
  * @note   调用点有两处，均运行于线程（主循环）上下文：
  *           1) 开机 —— DBG_BOOT 区块末尾，输出 Seq=1 的 0 基准行；
  *           2) 周期 —— 主循环按 DBG_LOG_PERIOD_MS(10 s)，输出 Seq=2,3,4...
  *         使用轮询发送 HAL_UART_Transmit（超时 10ms），不占用 DMA/中断资源。
  *         每条就是"当时的快照"：VREF(mV)、VREF 就绪标志、计时器标志、
  *         Total/Rn219/Rn220 累计值，外加 Seq= 第几条。
  *         不聚合——计数率由上位机用相邻两条的 Total/Rn219/Rn220 相减得到；
  *         总事件率 N_T = (Total 增量) + (本条 D= 的增量)。
  */
static void DEBUG_Print(void)
{
  static char dbg[192];               /* 静态：本函数只在主循环调用、无重入，避免大缓冲压栈（栈仅 1KB）；
                                         各计数取 10 位十进制时单条约 120 字节，192 余量充足 */
  uint8_t now_buf[20];
  int nl = fmt_rtc(now_buf);
  now_buf[nl] = '\0';                 /* 供 %s 使用 */
  static uint32_t seq_idx = 0U;    /* 本行是开机后第几条记录，用于上位机核对有无漏行 */
  seq_idx++;

  int n = snprintf(dbg, sizeof(dbg),
                   "%s VREF=%lu mV rdy=%u timer=%u Total=%lu Rn219=%lu Rn220=%lu"
                   " D=%lu Q=%lu Seq=%lu\r\n",
                   (char *)now_buf,
                   //RTC_GetSourceName(),
                   (unsigned long)VREF,
                   (unsigned int)vref_ready,
                   (unsigned int)timer_running,
                   (unsigned long)pulse_total,
                   (unsigned long)rn219_total,
                   (unsigned long)rn220_total,
                   (unsigned long)edge_drop_total,
                   (unsigned long)pulse_q_drop,
                   (unsigned long)seq_idx);
  if (n < 0) n = 0;
  if ((size_t)n > sizeof(dbg) - 1U)
    n = (int)(sizeof(dbg) - 1U);

  SDLOG_UART2((uint8_t *)dbg, (uint16_t)n);
}

/**
  * @brief  TIM 周期溢出（更新）回调
  * @note   TIM2：1 us/计数，32 位每 ~71.58 min 溢出一次，累加 timer_overflow_us 扩展量程；
  *         TIM6：1 ms 单触发到点，拉低 PB5 并停止定时器；
  *         TIM3：16 位保留计数器（每 ~6.55 s 溢出），仅累加 uptime_ms_total 供调试，不参与 U=。
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    /* 1 us/计数下 32 位每 ~71.58 分钟溢出一次，软件累加以扩展量程至 24 h */
    timer_overflow_us += 0x100000000ULL;
  }
  else if (htim->Instance == TIM6)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);  /* 1ms 到点，PB5 拉低 */
    HAL_TIM_Base_Stop_IT(&htim6);
  }
  else if (htim->Instance == TIM3)
  {
    /* （保留）TIM3 为 16 位定时器，每 ~6.55s 溢出一次；运行时间现由 RTC 计算，
       此分支累加的 uptime_ms_total 已不用于 U=，仅作调试/扩展保留。 */
    uptime_ms_total += (uint64_t)((0xFFFFU + 1U) / 10U);
  }
}

/**
  * @brief  EXTI 外部中断回调（PB0 下降沿）
  * @param  GPIO_Pin : 触发引脚
  * @note   每次有效下降沿：若当前无采样序列进行，则启动一次
  *         “ADC 连续转换 + DMA 接收 10 个采样值”，转换完成后由
  *         HAL_ADC_ConvCpltCallback 自动处理。
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_0)        /* PB0 -> EXTI0 */
  {
    if (adc_busy)
    {
      /* 上一次 10 次采样（约 20us）尚未完成，本次边沿无法启动采样。
         不能静默丢弃：计数后由记录帧的 D= 字段上报，事后可核对丢了多少、
         以及是否存在系统性近距离脉冲（<25us 间隔必丢第二个）。 */
      edge_drop_total++;
      return;
    }

    adc_busy = 1;
    /* 清除上一次序列可能遗留的 EOC/OVR 标志，确保连续模式能干净重启
       （连续转换下，DMA 传完 10 个后 ADC 可能多转 1~2 次并置 OVR） */
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC | ADC_FLAG_OVR);
    /* 启动连续 ADC + DMA；DMA 传送满 10 个半字后触发 ConvCplt 回调 */
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, ADC_SAMPLES) != HAL_OK)
    {
      adc_busy = 0;                   /* 启动失败则释放标志，允许下次重试 */
    }
  }
}

/**
  * @brief  ADC DMA 传输完成回调
  * @param  hadc : ADC 句柄
  * @note   在此完成：停止 ADC+DMA -> 去最大值/最小值 -> 剩余 8 次平均 ->
  *         换算电压 -> 与 500mV 比较判定“中断有效”。
  *         有效且为首次：清零并（已）启动定时器；
  *         有效且非首次：读取定时器计数值（间隔 ms），清零重启，并通过 USART1 DMA 输出。
  *         此函数运行于 DMA1_Channel1 中断上下文。
  */
uint32_t voltage_mv=0;
uint32_t valid_thr_mv =0;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance != ADC1)
    return;

  /* 1) 停止 ADC 与 DMA（连续模式下自动停止，实现“10 次后自动停止”） */
  HAL_ADC_Stop_DMA(&hadc1);
  adc_busy = 0;

  /* 2) 去最大值、最小值，求剩余 8 次平均 */
  uint16_t maxv = 0U, minv = 0xFFFFU;
  uint32_t sum = 0U;
  for (uint32_t i = 0U; i < ADC_SAMPLES; i++)
  {
    uint16_t v = adc_buf[i];
    if (v > maxv) maxv = v;
    if (v < minv) minv = v;
    sum += v;
  }
  sum -= (uint32_t)(maxv + minv);                       /* 去掉最大、最小 */
  uint16_t avg = (uint16_t)(sum / (ADC_SAMPLES - ADC_DROP)); /* 剩余 8 次平均 */

  /* 3) 换算为电压（mV）：代码值 -> 电压 */
  voltage_mv = (uint32_t)avg * VREF_MV / ADC_FULL_SCALE;

  /* 4) 判定“中断有效”：动态阈值 (VREF + 50) mV；VREF 尚未就绪时回退原固定阈值 */
//  valid_thr_mv = vref_ready ? (VREF + 50U) : VALID_THRESHOLD_MV;  /* 动态阈值 = VREF + 50 mV（需求） */
   valid_thr_mv = vref_ready ? (VREF ) : VALID_THRESHOLD_MV;  /* 动态阈值 = VREF（需求） */
  /* 动态阈值按需求用“>=（VREF+50）”；未就绪回退时沿用原“>固定阈值”语义 */
  if (vref_ready ? (voltage_mv >= valid_thr_mv) : (voltage_mv > valid_thr_mv))
  {
    /* 脉冲识别：PB5 输出 1ms 脉冲（硬件 TIM6 单触发，ISR 内不阻塞） */
    Pulse_TriggerPB5();
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);    /* PC13 每次识别到脉冲翻转一次 */

    pulse_total++;                              /* ④ 累计脉冲总数 +1（每次有效识别） */

    if (!timer_running)
    {
      /* 首次有效中断：以“清零计数器”作为计时起点 */
      __HAL_TIM_SET_COUNTER(&htim2, 0U);
      HAL_TIM_Base_Start(&htim2);     /* 已在 MX_TIM2_Init 里 Start_IT 预启动，此处只保证计数器在跑 */
      /* 关键：清掉“上电 → 首脉冲”期间累积的溢出量。
         TIM2 在 MX_TIM2_Init 就已 Start_IT 自由运行，若开机到首脉冲超过 71.58 min，
         timer_overflow_us 已累加 ≥1，不清零会让第一个间隔多出 71.58 min 的整数倍。 */
      timer_overflow_us = 0U;
      timer_running = 1;
      /* 首次脉冲尚无间隔，记为 0；仍入队，由主循环补上时间戳后发出 */
      PulseQ_Push(0ULL, pulse_total);
    }
    else
    {
      /* 后续有效中断：读取两次有效中断之间的间隔（us），清零重启，发送 */
      /* 节拍为 1 MHz（1 us/计数）；叠加溢出累加值以覆盖 > 71.58 分钟的长间隔 */
      uint64_t elapsed_us = timer_overflow_us + (uint64_t)__HAL_TIM_GET_COUNTER(&htim2);
      timer_overflow_us = 0U;                              /* 重置溢出累加 */
      __HAL_TIM_SET_COUNTER(&htim2, 0U);                   /* 重启计时，准备下一个间隔 */
      last_interval_us = elapsed_us;

      /* ---- 按间隔归类核素（延迟符合法）----
       * 只有“第 2 个及以后”的脉冲才有间隔可言；首脉冲在上面的 if 分支里，不参与归类。
       * elapsed_us 已经是本次与上一次有效脉冲的微秒间隔，直接落窗判断即可。 */
      if ((elapsed_us >= (uint64_t)RN219_MIN_US) && (elapsed_us <= (uint64_t)RN219_MAX_US))
      {
        rn219_total++;                       /* 10us ~ 5.6ms：判为 219Rn（215Po 符合） */
      }
      else if ((elapsed_us >= (uint64_t)RN220_MIN_US) && (elapsed_us <= (uint64_t)RN220_MAX_US))
      {
        rn220_total++;                       /* 10ms ~ 600ms：判为 220Rn（216Po 符合） */
      }
      else
      {
        /* 落在 5.6ms~10ms 死区、或 >600ms（长时间无脉冲后重新开始）、或 <10us：
           只计入 pulse_total，不归入任何一个核素，避免污染两类计数。 */
      }

      /* 记录入队；组帧与 USART1 发送由主循环完成，中断内不再做字符串格式化 */
      PulseQ_Push(elapsed_us, pulse_total);
    }
  }
}

/**
  * @brief  UART TX DMA 传输完成回调
  * @param  huart : UART 句柄
  * @note   发送完成后释放忙标志，并尝试发送 FIFO 中下一条消息。
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uart_tx_tail = (uint8_t)((uart_tx_tail + 1U) % UART_TX_FIFO_SLOTS); /* 回收已发送槽 */
    uart_tx_busy = 0;
    UART_TrySendNext();   /* 尝试发送 FIFO 中的下一条 */
  }
}

/**
  * @brief  ADC 错误回调（如 DMA 传输错误）
  * @note   防止 adc_busy 被永久置位导致后续中断无法再触发采样。
  */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    HAL_ADC_Stop_DMA(&hadc1);
    adc_busy = 0;
  }
}

/**
  * @brief  UART 错误回调
  * @note   防止 uart_tx_busy 被永久置位导致 FIFO 卡死。
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uart_tx_busy = 0;
    UART_TrySendNext();
  }
}

/**
  * @brief  将当前日历时间格式化为 "YYYY-MM-DD HH:MM:SS"（19 字节，不含 '\0'）
  * @note   时间取自 rtc_config 的 RAM 镜像（RTC_Now），源头是外部 DS3231。
  *         只读内存、不走 I2C，因此可在中断上下文中安全调用。
  */
static int fmt_rtc(uint8_t *buf)
{
  RTC_DateTime_t dt;
  RTC_Now(&dt);

  uint16_t y = (uint16_t)dt.year + 2000U;
  buf[0]  = (uint8_t)('0' + (y / 1000U) % 10U);
  buf[1]  = (uint8_t)('0' + (y / 100U) % 10U);
  buf[2]  = (uint8_t)('0' + (y / 10U)  % 10U);
  buf[3]  = (uint8_t)('0' + (y % 10U));
  buf[4]  = '-';
  buf[5]  = (uint8_t)('0' + (dt.month / 10U)); buf[6]  = (uint8_t)('0' + (dt.month % 10U));
  buf[7]  = '-';
  buf[8]  = (uint8_t)('0' + (dt.day   / 10U)); buf[9]  = (uint8_t)('0' + (dt.day   % 10U));
  buf[10] = ' ';
  buf[11] = (uint8_t)('0' + (dt.hour  / 10U)); buf[12] = (uint8_t)('0' + (dt.hour  % 10U));
  buf[13] = ':';
  buf[14] = (uint8_t)('0' + (dt.min   / 10U)); buf[15] = (uint8_t)('0' + (dt.min   % 10U));
  buf[16] = ':';
  buf[17] = (uint8_t)('0' + (dt.sec   / 10U)); buf[18] = (uint8_t)('0' + (dt.sec   % 10U));
  return 19;
}

/**
  * @brief  处理 USART2 收到的命令
  *           校时：set YYYY-MM-DD HH:MM:SS      （例：set 2026-10-19 14:30:00）
  *           查询：time                          （输出时基来源 / 状态 / 温度）
  * @note   由主循环在 uart2_cmd_ready 置位时调用，运行在线程上下文，
  *         可以安全执行阻塞式 I2C（RTC_SetDateTime 内部会写 DS3231）。
  */
static void UART2_CmdProcess(void)
{
  /* --- 查询命令：time --- */
  if ((uart2_rx_line[0] == 't' || uart2_rx_line[0] == 'T') &&
      (uart2_rx_line[1] == 'i' || uart2_rx_line[1] == 'I') &&
      (uart2_rx_line[2] == 'm' || uart2_rx_line[2] == 'M') &&
      (uart2_rx_line[3] == 'e' || uart2_rx_line[3] == 'E'))
  {
    char out[128];
    RTC_DateTime_t dt;
    RTC_Now(&dt);
    /* 温度拆成整数 + 一位小数后打印：避免 %f（ARMCC 下需显式开启浮点 printf 支持） */
    float tf = RTC_GetTemp();
    int   ti = (tf >= 0.0f) ? (int)(tf * 10.0f + 0.5f) : (int)(tf * 10.0f - 0.5f);
    int   tip = ti / 10;
    int   tfp = ti - tip * 10;
    if (tfp < 0) { tfp = -tfp; }
    int n = snprintf(out, sizeof(out),
                     "[RTC] src=%s present=%u reset=%u syncfail=%lu temp=%d.%dC now=20%02u-%02u-%02u %02u:%02u:%02u\r\n",
                     RTC_GetSourceName(),
                     (unsigned int)RTC_IsPresent(),
                     (unsigned int)RTC_WasTimeReset(),
                     (unsigned long)RTC_GetSyncFailCount(),
                     tip, tfp,
                     (unsigned int)dt.year,  (unsigned int)dt.month, (unsigned int)dt.day,
                     (unsigned int)dt.hour,  (unsigned int)dt.min,   (unsigned int)dt.sec);
    if (n > 0)
    {
      /* snprintf 返回"本应写入"的长度，超长时必须钳位，否则会越界发送 */
      if (n > (int)sizeof(out) - 1) { n = (int)sizeof(out) - 1; }
      SDLOG_UART2((uint8_t *)out, (uint16_t)n);
    }
    return;
  }

  /* --- 校时命令：set YYYY-MM-DD HH:MM:SS --- */
  {
    RTC_DateTime_t dt;
    if (RTC_ParseString(uart2_rx_line, &dt) != 0)
    {
      const char h[] = "[RTC] usage: set YYYY-MM-DD HH:MM:SS  |  time\r\n";
      SDLOG_UART2((uint8_t *)h, (uint16_t)(sizeof(h) - 1U));
      return;
    }

    if (RTC_SetDateTime(dt.year, dt.month, dt.day, dt.hour, dt.min, dt.sec) == 0)
    {
      /* 写入 DS3231 成功 -> 掉电后由纽扣电池保持 */
      const char ok[] = "[RTC] time set OK (kept by DS3231 backup battery)\r\n";
      SDLOG_UART2((uint8_t *)ok, (uint16_t)(sizeof(ok) - 1U));
    }
    else if (RTC_IsPresent())
    {
      /* 芯片在线但写失败 -> I2C 通信异常 */
      const char er[] = "[RTC] set failed (I2C error)\r\n";
      SDLOG_UART2((uint8_t *)er, (uint16_t)(sizeof(er) - 1U));
    }
    else
    {
      /* DS3231 不存在 -> 仅更新软件时钟，掉电即丢 */
      const char er[] = "[RTC] DS3231 missing: software clock only, LOST on power-down\r\n";
      SDLOG_UART2((uint8_t *)er, (uint16_t)(sizeof(er) - 1U));
    }
  }
}

/**
  * @brief  UART 接收完成回调
  * @note   USART1：DMA TX 完成由 HAL_UART_TxCpltCallback 处理（见上）。
  *         USART2：单字节中断接收，拼成一行后由主循环解析（校时命令）。
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    if (uart2_rx_byte == '\r' || uart2_rx_byte == '\n')
    {
      if (uart2_rx_idx > 0)
      {
        uart2_rx_line[uart2_rx_idx] = '\0';
        uart2_rx_idx = 0;
        uart2_cmd_ready = 1;        /* 通知主循环解析 */
      }
    }
    else if (uart2_rx_idx < (sizeof(uart2_rx_line) - 1U))
    {
      uart2_rx_line[uart2_rx_idx++] = (char)uart2_rx_byte;
    }
    HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);  /* 继续接收下一字节 */
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
