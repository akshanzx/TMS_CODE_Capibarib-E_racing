/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (TMS mux + ADC robusto)
  ******************************************************************************
*/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TMS_NUM_SENSORS   80
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifndef MIN
#define MIN(a,b) ((a)<(b)?(a):(b))
#endif
#ifndef MAX
#define MAX(a,b) ((a)>(b)?(a):(b))
#endif
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
COM_InitTypeDef BspCOMInit;
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
ADC_HandleTypeDef hadc4;
ADC_HandleTypeDef hadc5;
FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN PV */
uint16_t mux_raw[TMS_NUM_SENSORS];
float    mux_voltage[TMS_NUM_SENSORS];
float    mux_temp[TMS_NUM_SENSORS];

uint32_t last_print_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_ADC4_Init(void);
static void MX_ADC5_Init(void);

/* USER CODE BEGIN PFP */
static inline void wait_us_approx(uint32_t us);
static uint16_t adc_read_once(ADC_HandleTypeDef *hadc);
static uint16_t median5_u16(uint16_t a[5]);
static uint16_t read_adc_by_index(uint16_t index);
static inline float clampf(float x, float lo, float hi);
static int8_t temp_to_i8(float t);
static float voltage_to_temperature(float vout);
void send_address_claim(void);
void send_thermistor_summary(int8_t minT, int8_t maxT, int8_t avgT, uint8_t count, uint8_t id_max, uint8_t id_min);
void send_all_temps_to_esp(int8_t temps[80]);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Seleção do canal do mux ---------------------------------------------------*/
void select_mux_channel(uint8_t channel)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, (GPIO_PinState)(channel & 0x01));        // S0
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, (GPIO_PinState)((channel >> 1) & 0x01)); // S1
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, (GPIO_PinState)((channel >> 2) & 0x01)); // S2
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, (GPIO_PinState)((channel >> 3) & 0x01)); // S3
}

/* --------- Utilitários de leitura robusta do ADC --------- */

static inline void wait_us_approx(uint32_t us)
{
  /* ~50 NOP/µs @ ~170 MHz (ajuste fino se necessário) */
  for (volatile uint32_t i = 0; i < (us * 50U); i++) __NOP();
}

static uint16_t adc_read_once(ADC_HandleTypeDef *hadc)
{
  HAL_ADC_Start(hadc);
  HAL_ADC_PollForConversion(hadc, 10);
  return (uint16_t)HAL_ADC_GetValue(hadc);
}

static uint16_t median5_u16(uint16_t a[5])
{
  for (int i = 1; i < 5; i++) {
    uint16_t x = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > x) { a[j + 1] = a[j]; j--; }
    a[j + 1] = x;
  }
  return a[2];
}

static uint16_t read_adc_by_index(uint16_t index)
{
  ADC_HandleTypeDef *hadc = NULL;
  switch (index) {
    case 1: hadc = &hadc1; break;
    case 2: hadc = &hadc2; break;
    case 3: hadc = &hadc3; break;
    case 4: hadc = &hadc4; break;
    case 5: hadc = &hadc5; break;
    default: return 0;
  }

  /* Dummy para carregar Csample com o novo canal */
  (void)adc_read_once(hadc);

  /* 5 leituras -> mediana */
  uint16_t v[5];
  for (int i = 0; i < 5; i++) v[i] = adc_read_once(hadc);
  return median5_u16(v);
}

/* --------- Conversões auxiliares --------- */

static inline float clampf(float x, float lo, float hi)
{
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

static int8_t temp_to_i8(float t)
{
  if (t < -40.0f) t = -40.0f;
  if (t > 120.0f) t = 120.0f;
  return (int8_t)lrintf(t);
}

/* Tensão -> temperatura (sensor por diodo/Zener)
   Substitua A/B por coeficientes calibrados (2 pontos) */
/* Tensão -> temperatura (Enepaq sensor tipo shunt/Zener)
   Fonte: tabela Vout x Temp do datasheet Enepaq (interpolação linear). */
static float voltage_to_temperature(float vout)
{
  /* Tabela oficial (Vout diminui quando a temperatura sobe) */
  static const float V[] = {
    2.44,2.42,2.40,2.38,2.35,2.32,2.27,2.23,2.17,2.11,2.05,1.99,1.92,1.86,1.80,1.74,1.68,
    1.63,1.59,1.55,1.51,1.48,1.45,1.43,1.40,1.38,1.37,1.35,1.34,1.33,1.32,1.31,1.30
  };
  static const float T[] = {
    -40,-35,-30,-25,-20,-15,-10, -5,  0,  5, 10, 15, 20, 25, 30, 35, 40,
     45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95,100,105,110,115,120
  };
  const int N = sizeof(V)/sizeof(V[0]);

  /* Clamp à faixa da tabela */
  if (vout >= V[0])   return T[0];
  if (vout <= V[N-1]) return T[N-1];

  /* Acha o intervalo V[i] >= vout >= V[i+1] e interpola */
  for (int i = 0; i < N-1; ++i) {
    if (vout <= V[i] && vout >= V[i+1]) {
      float v1 = V[i],   v2 = V[i+1];
      float t1 = T[i],   t2 = T[i+1];
      float f  = (vout - v1) / (v2 - v1);   /* f in [0..1] */
      return t1 + f * (t2 - t1);
    }
  }
  /* fallback (não deve chegar aqui) */
  return 25.0f;
}


/* ------------------- CAN helpers ------------------- */

void send_address_claim(void)
{
  FDCAN_TxHeaderTypeDef txHeader;
  uint8_t data[8] = {0xF3, 0x00, 0x80, 0x00, 0x40, 0x1E, 0x90, 0x00};

  txHeader.Identifier         = 0x18EEFF80;
  txHeader.IdType             = FDCAN_EXTENDED_ID;
  txHeader.TxFrameType        = FDCAN_DATA_FRAME;
  txHeader.DataLength         = FDCAN_DLC_BYTES_8;
  txHeader.ErrorStateIndicator= FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch      = FDCAN_BRS_OFF;
  txHeader.FDFormat           = FDCAN_CLASSIC_CAN;
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker      = 0;

  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, data);
}

void send_thermistor_summary(int8_t minT, int8_t maxT, int8_t avgT,
                             uint8_t count, uint8_t id_max, uint8_t id_min)
{
  FDCAN_TxHeaderTypeDef txHeader;
  uint8_t data[8];

  data[0] = 0x00;
  data[1] = (uint8_t)minT;
  data[2] = (uint8_t)maxT;
  data[3] = (uint8_t)avgT;
  data[4] = count;
  data[5] = id_max;
  data[6] = id_min;

  uint16_t checksum = 0x39 + 8;
  for (int i = 0; i < 7; i++) checksum += data[i];
  data[7] = (uint8_t)(checksum & 0xFF);

  txHeader.Identifier         = 0x1839F380;
  txHeader.IdType             = FDCAN_EXTENDED_ID;
  txHeader.TxFrameType        = FDCAN_DATA_FRAME;
  txHeader.DataLength         = FDCAN_DLC_BYTES_8;
  txHeader.ErrorStateIndicator= FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch      = FDCAN_BRS_OFF;
  txHeader.FDFormat           = FDCAN_CLASSIC_CAN;
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker      = 0;

  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, data);
}

void send_all_temps_to_esp(int8_t temps[80])
{
  FDCAN_TxHeaderTypeDef txHeader;
  uint8_t data[8];

  txHeader.IdType             = FDCAN_STANDARD_ID;
  txHeader.TxFrameType        = FDCAN_DATA_FRAME;
  txHeader.DataLength         = FDCAN_DLC_BYTES_8;
  txHeader.ErrorStateIndicator= FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch      = FDCAN_BRS_OFF;
  txHeader.FDFormat           = FDCAN_CLASSIC_CAN;
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

  for (uint8_t i = 0; i < 10; i++) {
    for (uint8_t j = 0; j < 8; j++) data[j] = (uint8_t)temps[i * 8 + j];
    txHeader.Identifier    = 0x321 + i;   // 0x321..0x330
    txHeader.MessageMarker = i;
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, data);
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_ADC4_Init();
  MX_ADC5_Init();

  for (int i = 0; i < TMS_NUM_SENSORS; i++) {
    mux_raw[i]     = 0;
    mux_voltage[i] = 0.0f;
    mux_temp[i]    = 25.0f;
  }

  last_print_time = HAL_GetTick();
  HAL_FDCAN_Start(&hfdcan1);

  BSP_LED_Init(LED_GREEN);
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  BspCOMInit.BaudRate  = 115200;
  BspCOMInit.WordLength= COM_WORDLENGTH_8B;
  BspCOMInit.StopBits  = COM_STOPBITS_1;
  BspCOMInit.Parity    = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE) {
    Error_Handler();
  }

  /* Loop principal: exemplo varrendo 16 canais do ADC1 (ajuste p/ 80 no seu caso) */
  while (1)
  {
    float soma = 0.0f;
    int8_t minT = 127, maxT = -128;
    uint8_t id_min = 0, id_max = 0;

    for (uint8_t adc = 1; adc <= 2; adc++)
    {
    printf("\n========== ADC %d ==========\n", adc);
    printf("| Canal |   RAW |  Vout(V) |  Temp(°C) |\n");
    printf("|-------|-------|----------|-----------|\n");

    for (uint8_t ch = 0; ch < 16; ch++) {
      select_mux_channel(ch);
      wait_us_approx(20);                 /* tempo de assentamento pós-MUX */

      uint16_t raw = read_adc_by_index(adc);
      float volt  = (raw / 4095.0f) * 3.3f;
      float temp  = voltage_to_temperature(volt);

      uint8_t index = (adc - 1) * 16 + ch;
      mux_raw[index]     = raw;
      mux_voltage[index] = volt;
      mux_temp[index]    = temp;

      printf("|  %2d  | %5u |   %6.3f |   %7.2f |\n",
             ch, (unsigned)mux_raw[index], mux_voltage[index], mux_temp[index]);

      if (temp > maxT && temp < 240) {
    	  maxT = temp;
    	  id_max = index;
      	  }
      if (temp < minT) {
    	  minT = temp;
    	  id_min = index;
      	  }
      if(temp < 240 && temp < 0){

    	  soma += temp;
      	  }
    }
}
    /* Média dos 16 lidos neste exemplo (mude para 56/80 ao varrer todos) */
    int8_t avgT = (int8_t)lrintf(soma / 56.0f);

    /* CAN */
    send_address_claim();
    HAL_Delay(15);
    send_thermistor_summary(minT, maxT, avgT, 56, id_max, id_min);

    int8_t temps_i8[80];
    for (int i = 0; i < 80; i++) temps_i8[i] = temp_to_i8(mux_temp[i]);
    send_all_temps_to_esp(temps_i8);

    last_print_time = HAL_GetTick();
    HAL_Delay(50);
  }
}

/* -------------------- System / Peripherals init -------------------- */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM            = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN            = 85;
  RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

/* === ADC1 === */
static void MX_ADC1_Init(void)
{
  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance                      = ADC1;
  hadc1.Init.ClockPrescaler           = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution               = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign                = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation         = 0;
  hadc1.Init.ScanConvMode             = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait         = DISABLE;
  hadc1.Init.ContinuousConvMode       = DISABLE;
  hadc1.Init.NbrOfConversion          = 1;
  hadc1.Init.DiscontinuousConvMode    = DISABLE;
  hadc1.Init.ExternalTrigConv         = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests    = DISABLE;
  hadc1.Init.Overrun                  = ADC_OVR_DATA_PRESERVED;

  /* Oversampling (8x, >>3) */
  hadc1.Init.OversamplingMode                         = ENABLE;
  hadc1.Init.Oversampling.Ratio                       = ADC_OVERSAMPLING_RATIO_8;
  hadc1.Init.Oversampling.RightBitShift               = ADC_RIGHTBITSHIFT_3;
  hadc1.Init.Oversampling.TriggeredMode               = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset       = ADC_REGOVERSAMPLING_CONTINUED_MODE;

  if (HAL_ADC_Init(&hadc1) != HAL_OK) { Error_Handler(); }

  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel      = ADC_CHANNEL_1;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;  /* *** */
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset       = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
}

/* === ADC2 === */
static void MX_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc2.Instance                      = ADC2;
  hadc2.Init.ClockPrescaler           = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution               = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign                = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation         = 0;
  hadc2.Init.ScanConvMode             = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait         = DISABLE;
  hadc2.Init.ContinuousConvMode       = DISABLE;
  hadc2.Init.NbrOfConversion          = 1;
  hadc2.Init.DiscontinuousConvMode    = DISABLE;
  hadc2.Init.ExternalTrigConv         = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests    = DISABLE;
  hadc2.Init.Overrun                  = ADC_OVR_DATA_PRESERVED;

  hadc2.Init.OversamplingMode                         = ENABLE;
  hadc2.Init.Oversampling.Ratio                       = ADC_OVERSAMPLING_RATIO_8;
  hadc2.Init.Oversampling.RightBitShift               = ADC_RIGHTBITSHIFT_3;
  hadc2.Init.Oversampling.TriggeredMode               = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc2.Init.Oversampling.OversamplingStopReset       = ADC_REGOVERSAMPLING_CONTINUED_MODE;

  if (HAL_ADC_Init(&hadc2) != HAL_OK) { Error_Handler(); }

  sConfig.Channel      = ADC_CHANNEL_2;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;  /* *** */
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset       = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) { Error_Handler(); }
}

/* === ADC3 === */
static void MX_ADC3_Init(void)
{
  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc3.Instance                      = ADC3;
  hadc3.Init.ClockPrescaler           = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution               = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign                = ADC_DATAALIGN_RIGHT;
  hadc3.Init.GainCompensation         = 0;
  hadc3.Init.ScanConvMode             = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait         = DISABLE;
  hadc3.Init.ContinuousConvMode       = DISABLE;
  hadc3.Init.NbrOfConversion          = 1;
  hadc3.Init.DiscontinuousConvMode    = DISABLE;
  hadc3.Init.ExternalTrigConv         = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DMAContinuousRequests    = DISABLE;
  hadc3.Init.Overrun                  = ADC_OVR_DATA_PRESERVED;

  hadc3.Init.OversamplingMode                         = ENABLE;
  hadc3.Init.Oversampling.Ratio                       = ADC_OVERSAMPLING_RATIO_8;
  hadc3.Init.Oversampling.RightBitShift               = ADC_RIGHTBITSHIFT_3;
  hadc3.Init.Oversampling.TriggeredMode               = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc3.Init.Oversampling.OversamplingStopReset       = ADC_REGOVERSAMPLING_CONTINUED_MODE;

  if (HAL_ADC_Init(&hadc3) != HAL_OK) { Error_Handler(); }

  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel      = ADC_CHANNEL_1;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;  /* *** */
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset       = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) { Error_Handler(); }
}

/* === ADC4 === */
static void MX_ADC4_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc4.Instance                      = ADC4;
  hadc4.Init.ClockPrescaler           = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc4.Init.Resolution               = ADC_RESOLUTION_12B;
  hadc4.Init.DataAlign                = ADC_DATAALIGN_RIGHT;
  hadc4.Init.GainCompensation         = 0;
  hadc4.Init.ScanConvMode             = ADC_SCAN_DISABLE;
  hadc4.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;
  hadc4.Init.LowPowerAutoWait         = DISABLE;
  hadc4.Init.ContinuousConvMode       = DISABLE;
  hadc4.Init.NbrOfConversion          = 1;
  hadc4.Init.DiscontinuousConvMode    = DISABLE;
  hadc4.Init.ExternalTrigConv         = ADC_SOFTWARE_START;
  hadc4.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc4.Init.DMAContinuousRequests    = DISABLE;
  hadc4.Init.Overrun                  = ADC_OVR_DATA_PRESERVED;

  hadc4.Init.OversamplingMode                         = ENABLE;
  hadc4.Init.Oversampling.Ratio                       = ADC_OVERSAMPLING_RATIO_8;
  hadc4.Init.Oversampling.RightBitShift               = ADC_RIGHTBITSHIFT_3;
  hadc4.Init.Oversampling.TriggeredMode               = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc4.Init.Oversampling.OversamplingStopReset       = ADC_REGOVERSAMPLING_CONTINUED_MODE;

  if (HAL_ADC_Init(&hadc4) != HAL_OK) { Error_Handler(); }

  sConfig.Channel      = ADC_CHANNEL_3;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;  /* *** */
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset       = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK) { Error_Handler(); }
}

/* === ADC5 === */
static void MX_ADC5_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc5.Instance                      = ADC5;
  hadc5.Init.ClockPrescaler           = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc5.Init.Resolution               = ADC_RESOLUTION_12B;
  hadc5.Init.DataAlign                = ADC_DATAALIGN_RIGHT;
  hadc5.Init.GainCompensation         = 0;
  hadc5.Init.ScanConvMode             = ADC_SCAN_DISABLE;
  hadc5.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;
  hadc5.Init.LowPowerAutoWait         = DISABLE;
  hadc5.Init.ContinuousConvMode       = DISABLE;
  hadc5.Init.NbrOfConversion          = 1;
  hadc5.Init.DiscontinuousConvMode    = DISABLE;
  hadc5.Init.ExternalTrigConv         = ADC_SOFTWARE_START;
  hadc5.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc5.Init.DMAContinuousRequests    = DISABLE;
  hadc5.Init.Overrun                  = ADC_OVR_DATA_PRESERVED;

  hadc5.Init.OversamplingMode                         = ENABLE;
  hadc5.Init.Oversampling.Ratio                       = ADC_OVERSAMPLING_RATIO_8;
  hadc5.Init.Oversampling.RightBitShift               = ADC_RIGHTBITSHIFT_3;
  hadc5.Init.Oversampling.TriggeredMode               = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc5.Init.Oversampling.OversamplingStopReset       = ADC_REGOVERSAMPLING_CONTINUED_MODE;

  if (HAL_ADC_Init(&hadc5) != HAL_OK) { Error_Handler(); }

  sConfig.Channel      = ADC_CHANNEL_1;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;  /* *** */
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset       = 0;
  if (HAL_ADC_ConfigChannel(&hadc5, &sConfig) != HAL_OK) { Error_Handler(); }
}

/* === FDCAN1 === */
static void MX_FDCAN1_Init(void)
{
  hfdcan1.Instance                 = FDCAN1;
  hfdcan1.Init.ClockDivider        = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat         = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode                = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission  = DISABLE;
  hfdcan1.Init.TransmitPause       = DISABLE;
  hfdcan1.Init.ProtocolException   = DISABLE;
  hfdcan1.Init.NominalPrescaler    = 34;
  hfdcan1.Init.NominalSyncJumpWidth= 1;
  hfdcan1.Init.NominalTimeSeg1     = 4;
  hfdcan1.Init.NominalTimeSeg2     = 5;
  hfdcan1.Init.DataPrescaler       = 1;
  hfdcan1.Init.DataSyncJumpWidth   = 1;
  hfdcan1.Init.DataTimeSeg1        = 1;
  hfdcan1.Init.DataTimeSeg2        = 1;
  hfdcan1.Init.StdFiltersNbr       = 0;
  hfdcan1.Init.ExtFiltersNbr       = 0;
  hfdcan1.Init.TxFifoQueueMode     = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) { Error_Handler(); }
}

/* === GPIO === */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin   = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* Error Handler -------------------------------------------------------------*/
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif
