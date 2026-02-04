/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Minimal offline calibration: MCP4728 4ch sync update (no CAN)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "i2c.h"
#include <stdint.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ======== Set your DAC output voltages here (Volts) ========
#define VOUT_CH_A   (3.300f)   // CH-A 输出电压
#define VOUT_CH_B   (1.650f)   // CH-B 输出电压
#define VOUT_CH_C   (0.800f)   // CH-C 输出电压
#define VOUT_CH_D   (2.500f)   // CH-D 输出电压

// DAC reference voltage (usually VDD for MCP4728)
#define DAC_VREF    (3.300f)   // 如果你的 MCP4728 供电不是 3.3V，请改这里

// ======== MCP4728 I2C 7-bit address ========
// 如果你扫描/确认的地址不是 0x60，把这里改成实际 7-bit 地址，8-bit 地址除以2即可为7-bit 地址
#define MCP4728_ADDR_7BIT   (0x60)

// ======== LDAC pin (sync update) ========
// 你前面用 PA8 做 GPIO 输出非常合适：LDAC 接 PA8
#define LDAC_GPIO_Port      GPIOA
#define LDAC_Pin            GPIO_PIN_8

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static inline void LDAC_High(void) { HAL_GPIO_WritePin(LDAC_GPIO_Port, LDAC_Pin, GPIO_PIN_SET); }
static inline void LDAC_Low(void)  { HAL_GPIO_WritePin(LDAC_GPIO_Port, LDAC_Pin, GPIO_PIN_RESET); }

static inline uint16_t clamp_u16(int32_t x, uint16_t lo, uint16_t hi)
{
  if (x < (int32_t)lo) return lo;
  if (x > (int32_t)hi) return hi;
  return (uint16_t)x;
}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

static uint16_t volt_to_code(float v, float vref);
static HAL_StatusTypeDef MCP4728_FastWrite4ch(uint16_t codeA, uint16_t codeB, uint16_t codeC, uint16_t codeD);
static HAL_StatusTypeDef MCP4728_SyncUpdate4ch(uint16_t codeA, uint16_t codeB, uint16_t codeC, uint16_t codeD);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 将目标电压转换为 12-bit DAC code (0..4095)
static uint16_t volt_to_code(float v, float vref)
{
  if (vref <= 0.1f) vref = 3.3f;
  if (v < 0.0f) v = 0.0f;
  if (v > vref) v = vref;

  float code_f = (v / vref) * 4095.0f;
  int32_t code_i = (int32_t)(code_f + 0.5f);
  return clamp_u16(code_i, 0, 4095);
}

// MCP4728 Fast Write: 4 channels, each 2 bytes
// Byte0: [C1 C0 PD1 PD0 D11 D10 D9 D8]
// Byte1: [D7..D0]
// C1C0: 00=A, 01=B, 10=C, 11=D
// PD=00: normal mode
static HAL_StatusTypeDef MCP4728_FastWrite4ch(uint16_t codeA, uint16_t codeB, uint16_t codeC, uint16_t codeD)
{
  uint8_t buf[8];
  uint16_t code[4] = {codeA, codeB, codeC, codeD};

  for (int ch = 0; ch < 4; ch++) {
    uint16_t v = code[ch] & 0x0FFF;                 // 12-bit
    uint8_t high = (uint8_t)((v >> 8) & 0x0F);      // D11..D8
    uint8_t low  = (uint8_t)(v & 0xFF);             // D7..D0

    buf[ch*2 + 0] = (uint8_t)(((ch & 0x03) << 6) | (0x0 << 4) | high);
    buf[ch*2 + 1] = low;
  }

  return HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(MCP4728_ADDR_7BIT << 1), buf, sizeof(buf), 50);
}

// 同步更新：LDAC 高锁住输出 → 写4通道 → LDAC 低脉冲 → 4通道同时更新
static HAL_StatusTypeDef MCP4728_SyncUpdate4ch(uint16_t codeA, uint16_t codeB, uint16_t codeC, uint16_t codeD)
{
  LDAC_High();  // lock outputs

  HAL_StatusTypeDef st = MCP4728_FastWrite4ch(codeA, codeB, codeC, codeD);
  if (st != HAL_OK) return st;

  LDAC_Low();
  for (volatile int i = 0; i < 300; i++) { __NOP(); }  // short pulse
  LDAC_High();

  return HAL_OK;
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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  
  // 上电默认把 LDAC 拉高（防止写入过程中输出乱跳）
  LDAC_High();

  // 计算你设定的 4 路目标电压对应的 DAC code
  uint16_t codeA = volt_to_code(VOUT_CH_A, DAC_VREF);
  uint16_t codeB = volt_to_code(VOUT_CH_B, DAC_VREF);
  uint16_t codeC = volt_to_code(VOUT_CH_C, DAC_VREF);
  uint16_t codeD = volt_to_code(VOUT_CH_D, DAC_VREF);

  // 4 通道同步输出一次
  (void)MCP4728_SyncUpdate4ch(codeA, codeB, codeC, codeD);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
    HAL_Delay(1000);
    /* USER CODE END WHILE */


    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
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
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
