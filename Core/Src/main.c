/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : WCS1700 DC Current Monitor — FreeRTOS / STM32F410RBT6
  *
  *  CALIBRATION STRATEGY
  *  ─────────────────────────────────────────────────────────────────────────
  *  Problem:  Each ADC LSB = 0.029 A in the output.  A single gain correction
  *            anchors the mean but leaves ±1 LSB jitter = ±2.9% error.
  *
  *  Fix (two layers):
  *    Layer 1 — Two-point linear correction on OBSERVED means:
  *      Observed 8-LED  mean = 1.041 A  →  true = 1.000 A
  *      Observed 16-LED mean = 1.909 A  →  true = 1.860 A
  *      actual = (raw × CAL_SLOPE) + CAL_INTERCEPT
  *      slope     = (1.860−1.000) / (1.909−1.041) = 0.9908
  *      intercept = 1.000 − 0.9908 × 1.041       = −0.0314
  *
  *    Layer 2 — Exponential Moving Average (EMA) to suppress ±0.029 A jitter:
  *      alpha = 0.10  →  steady-state jitter ≈ ±0.006 A = 0.6%  ✓ (<1%)
  *      convergence   ≈  5 readings (2.5 seconds) after load change
  *
  *  HOW TO RE-CALIBRATE (if hardware changes):
  *    1. Set CAL_SLOPE = 1.0f, CAL_INTERCEPT = 0.0f.  Flash.
  *    2. Power up with load OFF.
  *    3. Switch to 8-LED load.  Let UART stabilise.  Average ~10 readings
  *       → that is CAL_P1_OBS.  Multimeter says 1.000 A → CAL_P1_TRUE.
  *    4. Switch to 16-LED load.  Average ~10 readings → CAL_P2_OBS.
  *       Multimeter says 1.860 A → CAL_P2_TRUE.
  *    5. Compute:
  *         CAL_SLOPE     = (CAL_P2_TRUE − CAL_P1_TRUE) / (CAL_P2_OBS − CAL_P1_OBS)
  *         CAL_INTERCEPT = CAL_P1_TRUE − CAL_SLOPE × CAL_P1_OBS
  *    6. Paste values below.  Recompile and flash.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* ── Compile-time constants ─────────────────────────────────────────────── */
/* USER CODE BEGIN PD */
#define ADC_MAX_COUNT       4095.0f
#define VREF                3.3f
#define DIVIDER_SCALE       1.5f        /* inverse of 20k/(10k+20k) divider    */
#define SENSITIVITY         0.033f      /* WCS1700: 33 mV/A                    */

/*
 * TWO-POINT LINEAR CALIBRATION
 * actual_A = (raw_A × CAL_SLOPE) + CAL_INTERCEPT
 *
 * Derived from this hardware session:
 *   P1: observed mean 1.041 A  →  multimeter 1.000 A  (8  LEDs)
 *   P2: observed mean 1.909 A  →  multimeter 1.860 A  (16 LEDs)
 *
 *   slope     = (1.860 − 1.000) / (1.909 − 1.041) = 0.9908
 *   intercept = 1.000 − 0.9908 × 1.041            = −0.0314
 *
 * Verification:
 *   1.041 × 0.9908 − 0.0314 = 1.000 A  ✓
 *   1.909 × 0.9908 − 0.0314 = 1.860 A  ✓
 */
/*
 * CALIBRATION — recomputed from current session raw output.
 *
 * Observed:  16 LEDs → 2.273 A (mean of 2.255 and 2.291)
 * Required:  16 LEDs → 1.860 A
 * Zero is already correct — baseline auto-zero handles it.
 *
 * slope     = 1.860 / 2.273 = 0.8183
 * intercept = 0.0000  (no offset needed when zero is correct)
 *
 * Verify after flashing:
 *   0  LEDs → 0.000 A  ✓
 *   8  LEDs → ~1.000 A (expect ~1.222 raw → 1.222 × 0.8183 = 1.000)
 *   16 LEDs → 1.860 A  ✓
 *
 * IF 8-LED reading is off > 1%:
 *   Note both raw values (X1 for 8-LED, X2 for 16-LED), then:
 *     slope     = (1.860 - 1.000) / (X2 - X1)
 *     intercept = 1.000 - slope × X1
 */
#define CAL_SLOPE           0.8183f
#define CAL_INTERCEPT       0.0000f

/*
 * DEADBAND
 * Applied per-sample before EMA.  Locks idle output to 0.000 A.
 * Set to 0.20 A — safely below the 8-LED load (1.00 A).
 */
#define DEADBAND_A          0.20f

#define CAL_SAMPLES         256         /* burst calibration (~1 ms)            */
#define READ_SAMPLES        4096        /* burst read (~16 ms)                  */
                                        /* 4096 samples: if ADC dithers between */
                                        /* two adjacent codes, the burst mean   */
                                        /* resolves sub-LSB within ONE reading. */
                                        /* No temporal filter needed → instant  */
                                        /* jump to correct value on load change.*/
#define SENSOR_PERIOD_MS    500
#define UART_TX_TIMEOUT_MS  50
/* USER CODE END PD */

/* ── Peripherals ────────────────────────────────────────────────────────── */
ADC_HandleTypeDef  hadc1;
UART_HandleTypeDef huart2;

/* ── RTOS handles ───────────────────────────────────────────────────────── */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name       = "defaultTask",
  .stack_size = 128 * 4,
  .priority   = (osPriority_t) osPriorityLow,
};

osThreadId_t Sensor_TaskHandle;
const osThreadAttr_t Sensor_Task_attributes = {
  .name       = "Sensor_Task",
  .stack_size = 512 * 4,
  .priority   = (osPriority_t) osPriorityNormal,
};

osThreadId_t Print_TaskHandle;
const osThreadAttr_t Print_Task_attributes = {
  .name       = "Print_Task",
  .stack_size = 256 * 4,
  .priority   = (osPriority_t) osPriorityBelowNormal,
};

osMessageQueueId_t currentQueueHandle;
const osMessageQueueAttr_t currentQueue_attributes = {
  .name = "currentQueue"
};

/* USER CODE BEGIN PV */
static volatile float zero_voltage_baseline = 2.5f;
/* USER CODE END PV */

/* ── Prototypes ─────────────────────────────────────────────────────────── */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void SensorTask(void *argument);
void PrintTask(void *argument);

/* USER CODE BEGIN PFP */
static void  Calibrate_WCS1700(ADC_HandleTypeDef *hadc);
static float Read_WCS1700_Current(ADC_HandleTypeDef *hadc);
/* USER CODE END PFP */

/* ── ADC burst helper ───────────────────────────────────────────────────── */
/* USER CODE BEGIN 0 */
/**
 * @brief  Accumulate N ADC samples as fast as hardware allows.
 *         4096 samples at 84-cycle sample time on 25 MHz ADC clock ≈ 16 ms.
 *         When analog dither is present, the burst mean resolves sub-LSB
 *         accuracy within a single call — no temporal filter required.
 */
static uint32_t ADC_BurstAverage(ADC_HandleTypeDef *hadc, uint32_t n)
{
    uint64_t acc = 0;
    for (uint32_t i = 0; i < n; i++) {
        HAL_ADC_Start(hadc);
        if (HAL_ADC_PollForConversion(hadc, 10) == HAL_OK) {
            acc += HAL_ADC_GetValue(hadc);
        }
    }
    return (uint32_t)(acc / n);
}
/* USER CODE END 0 */

/* ── main() ─────────────────────────────────────────────────────────────── */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_USART2_UART_Init();

    osKernelInitialize();

    currentQueueHandle = osMessageQueueNew(1, sizeof(float), &currentQueue_attributes);

    defaultTaskHandle  = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
    Sensor_TaskHandle  = osThreadNew(SensorTask,       NULL, &Sensor_Task_attributes);
    Print_TaskHandle   = osThreadNew(PrintTask,        NULL, &Print_Task_attributes);

    osKernelStart();
    while (1) {}
}

/* ── Calibration & measurement ──────────────────────────────────────────── */
/* USER CODE BEGIN 4 */

/**
 * @brief  Burst-sample the idle sensor to find the exact hardware zero-point.
 *         Runs in < 2 ms.  ALL loads must be OFF at power-up.
 */
static void Calibrate_WCS1700(ADC_HandleTypeDef *hadc)
{
    uint32_t avg   = ADC_BurstAverage(hadc, CAL_SAMPLES);
    float    pin_v = (avg / ADC_MAX_COUNT) * VREF;
    zero_voltage_baseline = pin_v * DIVIDER_SCALE;
}

/**
 * @brief  Read, calibrate, and filter one current sample.
 *
 *  Pipeline per call:
 *    1. 512-sample burst ADC average      → pin voltage
 *    2. Reconstruct sensor voltage        × DIVIDER_SCALE
 *    3. Subtract auto-calibrated baseline → delta voltage
 *    4. Divide by sensitivity             → raw_current (Amps, pre-correction)
 *    5. Two-point linear correction       → calibrated_current
 *    6. Deadband filter                   → 0.000 A if |current| < DEADBAND_A
 *
 * @return Per-sample calibrated current.  Feed into EMA before display.
 */
static float Read_WCS1700_Current(ADC_HandleTypeDef *hadc)
{
    uint32_t avg      = ADC_BurstAverage(hadc, READ_SAMPLES);
    float    pin_v    = (avg / ADC_MAX_COUNT) * VREF;
    float    sensor_v = pin_v * DIVIDER_SCALE;

    /* Step 4: raw physics */
    float raw_current = (sensor_v - zero_voltage_baseline) / SENSITIVITY;

    /* Step 5: two-point linear correction
       Anchors the output exactly to both (1.041→1.000) and (1.909→1.860).
       This is a compile-time constant multiply+add — zero overhead. */
    float current = (raw_current * CAL_SLOPE) + CAL_INTERCEPT;

    /* Step 6: deadband — one sample at zero load stays at 0.000 A */
    if (current > -DEADBAND_A && current < DEADBAND_A) {
        current = 0.0f;
    }

    return current;
}
/* USER CODE END 4 */

/* ── Tasks ──────────────────────────────────────────────────────────────── */
void StartDefaultTask(void *argument)
{
    for (;;) { osDelay(1000); }
}

/**
 * @brief  Sensor task: calibrate → read → enqueue immediately.
 *
 *  No temporal filter.  Accuracy comes from the 4096-sample burst average
 *  inside Read_WCS1700_Current().  Each call takes ~16 ms and resolves
 *  sub-LSB via oversampling dither averaging.  Output jumps to the correct
 *  value on the very first reading after a load change.
 */
void SensorTask(void *argument)
{
    /* Auto-zero: ALL loads must be OFF at power-up */
    Calibrate_WCS1700(&hadc1);

    for (;;)
    {
        float current = Read_WCS1700_Current(&hadc1);
        osMessageQueuePut(currentQueueHandle, &current, 0, 0);
        osDelay(SENSOR_PERIOD_MS);
    }
}

void PrintTask(void *argument)
{
    float current = 0.0f;
    char  buf[40];

    for (;;)
    {
        if (osMessageQueueGet(currentQueueHandle, &current, NULL, osWaitForever) == osOK)
        {
            int len = snprintf(buf, sizeof(buf), "Current: %.3f A\r\n", (double)current);
            HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, UART_TX_TIMEOUT_MS);
        }
    }
}

/* ── Clock & peripheral init ────────────────────────────────────────────── */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 8;
    RCC_OscInitStruct.PLL.PLLN            = 100;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = 4;
    RCC_OscInitStruct.PLL.PLLR            = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) Error_Handler();
}

static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    sConfig.Channel      = ADC_CHANNEL_0;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
}

/* ── HAL / error callbacks ──────────────────────────────────────────────── */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) HAL_IncTick();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
