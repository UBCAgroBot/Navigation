/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---- TMC5160 Register Addresses ---- */
#define TMC5160_GCONF       0x00
#define TMC5160_GSTAT       0x01
#define TMC5160_IHOLD_IRUN  0x10
#define TMC5160_CHOPCONF    0x6C
#define TMC5160_VACTUAL     0x22
#define TMC5160_RAMPMODE    0x20
#define TMC5160_XACTUAL     0x21
#define TMC5160_XTARGET     0x2D

/* Velocity mode registers */
#define TMC5160_VSTART      0x23
#define TMC5160_A1          0x24
#define TMC5160_V1          0x25
#define TMC5160_AMAX        0x28
#define TMC5160_VMAX        0x27
#define TMC5160_DMAX        0x29
#define TMC5160_D1          0x2A
#define TMC5160_VSTOP       0x2B

/* SPI write flag */
#define TMC5160_WRITE_BIT   0x80

/* ---- Pin Assignments ---- */

/* PC10 = DRV_ENN (driver enable, active LOW) */
#define DRV_EN_PIN    GPIO_PIN_10
#define DRV_EN_PORT   GPIOC

/* PC11 = CSN (SPI chip select for stepper 1) */
#define CS1_PIN       GPIO_PIN_11
#define CS1_PORT      GPIOC

/* PB6 = CSN for stepper 2 (unused for now) */
#define CS2_PIN       GPIO_PIN_6
#define CS2_PORT      GPIOB

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

void     TMC5160_CS_Low(GPIO_TypeDef *port, uint16_t pin);
void     TMC5160_CS_High(GPIO_TypeDef *port, uint16_t pin);
uint32_t TMC5160_ReadReg(GPIO_TypeDef *cs_port, uint16_t cs_pin, uint8_t reg);
void     TMC5160_WriteReg(GPIO_TypeDef *cs_port, uint16_t cs_pin, uint8_t reg, uint32_t value);
void     TMC5160_Init(GPIO_TypeDef *cs_port, uint16_t cs_pin);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void TMC5160_CS_Low(GPIO_TypeDef *port, uint16_t pin)
{
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

void TMC5160_CS_High(GPIO_TypeDef *port, uint16_t pin)
{
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

uint32_t TMC5160_ReadReg(GPIO_TypeDef *cs_port, uint16_t cs_pin, uint8_t reg)
{
  uint8_t tx[5] = {0};
  uint8_t rx[5] = {0};

  tx[0] = reg & 0x7F;

  TMC5160_CS_Low(cs_port, cs_pin);
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 5, HAL_MAX_DELAY);
  TMC5160_CS_High(cs_port, cs_pin);

  HAL_Delay(1);

  memset(tx, 0, sizeof(tx));
  memset(rx, 0, sizeof(rx));

  TMC5160_CS_Low(cs_port, cs_pin);
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 5, HAL_MAX_DELAY);
  TMC5160_CS_High(cs_port, cs_pin);

  uint32_t value = ((uint32_t)rx[1] << 24) |
                   ((uint32_t)rx[2] << 16) |
                   ((uint32_t)rx[3] <<  8) |
                   ((uint32_t)rx[4]);
  return value;
}

void TMC5160_WriteReg(GPIO_TypeDef *cs_port, uint16_t cs_pin, uint8_t reg, uint32_t value)
{
  uint8_t tx[5];
  uint8_t rx[5];

  tx[0] = reg | TMC5160_WRITE_BIT;
  tx[1] = (value >> 24) & 0xFF;
  tx[2] = (value >> 16) & 0xFF;
  tx[3] = (value >>  8) & 0xFF;
  tx[4] = (value >>  0) & 0xFF;

  TMC5160_CS_Low(cs_port, cs_pin);
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 5, HAL_MAX_DELAY);
  TMC5160_CS_High(cs_port, cs_pin);
}

void TMC5160_Init(GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
  /* 1. Clear error flags */
  TMC5160_WriteReg(cs_port, cs_pin, TMC5160_GSTAT, 0x07);

  /* 2. GCONF: enable stealthChop */
  TMC5160_WriteReg(cs_port, cs_pin, TMC5160_GCONF, 0x00000004);

  /* 3. CHOPCONF: TOFF=4, HSTRT=4, HEND=1, TBL=2, MRES=4 (16 microsteps) */
  TMC5160_WriteReg(cs_port, cs_pin, TMC5160_CHOPCONF, 0x04010094);

  /* 4. IHOLD_IRUN: IHOLDDELAY=6, IRUN=10, IHOLD=5 */
  TMC5160_WriteReg(cs_port, cs_pin, TMC5160_IHOLD_IRUN, 0x00060A05);

  /* 5. RAMPMODE = 0 → positioning mode */
  TMC5160_WriteReg(cs_port, cs_pin, TMC5160_RAMPMODE, 0x00000000);

  /* 6. Gentle ramp profile */
  TMC5160_WriteReg(cs_port, cs_pin, TMC5160_VSTART, 1);
  TMC5160_WriteReg(cs_port, cs_pin, TMC5160_A1,     500);
  TMC5160_WriteReg(cs_port, cs_pin, TMC5160_V1,     5000);
  TMC5160_WriteReg(cs_port, cs_pin, TMC5160_AMAX,   500);
  TMC5160_WriteReg(cs_port, cs_pin, TMC5160_VMAX,   50000);
  TMC5160_WriteReg(cs_port, cs_pin, TMC5160_DMAX,   500);
  TMC5160_WriteReg(cs_port, cs_pin, TMC5160_D1,     500);
  TMC5160_WriteReg(cs_port, cs_pin, TMC5160_VSTOP,  10);

  /* 7. Zero the position */
  TMC5160_WriteReg(cs_port, cs_pin, TMC5160_XACTUAL, 0);
  TMC5160_WriteReg(cs_port, cs_pin, TMC5160_XTARGET, 0);
}

/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */

  /* Start PWM for DC motor */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  /* Enable the TMC5160 driver (DRV_ENN LOW) */
  HAL_GPIO_WritePin(DRV_EN_PORT, DRV_EN_PIN, GPIO_PIN_RESET);

  /* Deselect CSN lines (HIGH) */
  TMC5160_CS_High(CS1_PORT, CS1_PIN);
  TMC5160_CS_High(CS2_PORT, CS2_PIN);

  /* Let TMC5160 boot */
  HAL_Delay(100);

  /* Initialize stepper 1 */
  TMC5160_Init(CS1_PORT, CS1_PIN);

  /* Sanity check: should read 0x00000004 */
  uint32_t gconf = TMC5160_ReadReg(CS1_PORT, CS1_PIN, TMC5160_GCONF);
  (void)gconf;  /* breakpoint here */

  /* ---- TEST: one full revolution forward ---- */
  TMC5160_WriteReg(CS1_PORT, CS1_PIN, TMC5160_XTARGET, 64000);

  uint32_t pos = 0;
  do {
    HAL_Delay(50);
    pos = TMC5160_ReadReg(CS1_PORT, CS1_PIN, TMC5160_XACTUAL);
  } while (pos != 64000);

  /* Pause then return to zero */
  HAL_Delay(500);
  TMC5160_WriteReg(CS1_PORT, CS1_PIN, TMC5160_XTARGET, 0);

  do {
    HAL_Delay(50);
    pos = TMC5160_ReadReg(CS1_PORT, CS1_PIN, TMC5160_XACTUAL);
  } while (pos != 0);

  /* USER CODE END 2 */

  while (1)
  {
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;          /* CPOL=1 for SPI Mode 3 */
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;               /* CPHA=1 for SPI Mode 3 */
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;  /* ~2 MHz */
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM1_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 79;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim1);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* PC10 (DRV_ENN) LOW = enabled, PC11 (CSN) HIGH = deselected */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
