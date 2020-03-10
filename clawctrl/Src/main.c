/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "claw_hal.h"
#include "digiled.h"
#include "lighteffect.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DO_REFERENCE_MOVE 1

#define STATE_INITIALIZING 0
#define STATE_TRACKING 1
#define STATE_CLAW_DOWN 2
#define STATE_CLAW_UP 3

#define REF_MOVE_TIMEOUT_MS 5000

#define RANDOM_LED_UPDATE_INTERVAL_MS 200

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

static uint8_t state = STATE_INITIALIZING;
static uint32_t tick_at_claw_down = 0;
static uint32_t tick_at_last_random_led_switch = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

static void claw_main(void);
static void claw_init(void);
static void claw_reference_move(void);

static void led_init();
static uint32_t led_rand_red_green_or_blue();
static void led_random();
static void led_danger();

static void log_string(const char* str);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  led_init();
  claw_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

      claw_main();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable 
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_SPI1
                              |RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable USB Voltage detector 
  */
  HAL_PWREx_EnableUSBVoltageDetector();
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, BUZZER_Pin|MTR_UP_Pin|CLAW_ENABLE_Pin|MTR_DOWN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, MTR_LEFT_Pin|MTR_RIGHT_Pin|MTR_BACKWARD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, MTR_FORWARD_Pin|LED_1_Pin|LED_2_Pin|USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BUZZER_Pin MTR_UP_Pin CLAW_ENABLE_Pin MTR_DOWN_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin|MTR_UP_Pin|CLAW_ENABLE_Pin|MTR_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_B1_Pin */
  GPIO_InitStruct.Pin = USER_B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(USER_B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MTR_LEFT_Pin MTR_RIGHT_Pin MTR_BACKWARD_Pin */
  GPIO_InitStruct.Pin = MTR_LEFT_Pin|MTR_RIGHT_Pin|MTR_BACKWARD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MTR_FORWARD_Pin LED_1_Pin LED_2_Pin USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = MTR_FORWARD_Pin|LED_1_Pin|LED_2_Pin|USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : JOY_DOWN_Pin JOY_UP_Pin JOY_RIGHT_Pin JOY_LEFT_Pin 
                           BTN_1_Pin */
  GPIO_InitStruct.Pin = JOY_DOWN_Pin|JOY_UP_Pin|JOY_RIGHT_Pin|JOY_LEFT_Pin 
                          |BTN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_2_Pin LS5_Pin LS4_Pin LS3_Pin 
                           LS2_Pin LS1_Pin */
  GPIO_InitStruct.Pin = BTN_2_Pin|LS5_Pin|LS4_Pin|LS3_Pin 
                          |LS2_Pin|LS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

static void claw_init() {

    beep(1, 250);

    HAL_GPIO_WritePin(MTR_DOWN_GPIO_Port, MTR_DOWN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MTR_UP_GPIO_Port, MTR_UP_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MTR_FORWARD_GPIO_Port, MTR_FORWARD_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MTR_BACKWARD_GPIO_Port, MTR_BACKWARD_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MTR_LEFT_GPIO_Port, MTR_LEFT_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MTR_RIGHT_GPIO_Port, MTR_RIGHT_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CLAW_ENABLE_GPIO_Port, CLAW_ENABLE_Pin, GPIO_PIN_SET);

    // 1 second blink button and onboard LEDs
    for (int i = 0; i < 8; i++) {
        uint8_t even = (i % 2 == 0) ? 0 : 1;
        set_user_leds(even, even, even);
        set_btn1_led(even);
        set_btn2_led(even);
        HAL_Delay(1000 / 8);
    }

    // do the reference move
#if DO_REFERENCE_MOVE
    claw_reference_move();
#endif

    // double beep after reference move
    beep(2, 250);

    // start in initial state
    state = STATE_INITIALIZING;
    log_string("BOOT COMPLETE -> INITIALIZING");
}

static void claw_main() {
    uint8_t btn_red_pressed, btn_blue_pressed;
    uint8_t user_b1_pressed;
    uint8_t joy_up_pressed, joy_down_pressed, joy_left_pressed, joy_right_pressed;
    uint8_t ls_1, ls_2, ls_3, ls_4, ls_5;
    uint32_t tick_now = HAL_GetTick();

    switch (state) {
        case STATE_INITIALIZING:
            // set everything to disabled
            set_user_leds(0, 0, 0);
            HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);

            HAL_GPIO_WritePin(MTR_DOWN_GPIO_Port, MTR_DOWN_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MTR_UP_GPIO_Port, MTR_UP_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MTR_FORWARD_GPIO_Port, MTR_FORWARD_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MTR_BACKWARD_GPIO_Port, MTR_BACKWARD_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MTR_LEFT_GPIO_Port, MTR_LEFT_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MTR_RIGHT_GPIO_Port, MTR_RIGHT_Pin, GPIO_PIN_SET);

            HAL_GPIO_WritePin(CLAW_ENABLE_GPIO_Port, CLAW_ENABLE_Pin, GPIO_PIN_SET);

            log_string("INITIALIZING -> TRACKING");
            state = STATE_TRACKING;
            set_user_leds(1, 0, 0);
            led_random();
            break;
        case STATE_TRACKING:
            ls_1 = HAL_GPIO_ReadPin(LS1_GPIO_Port, LS1_Pin) == GPIO_PIN_RESET;
            ls_2 = HAL_GPIO_ReadPin(LS2_GPIO_Port, LS2_Pin) == GPIO_PIN_RESET;
            ls_4 = HAL_GPIO_ReadPin(LS4_GPIO_Port, LS4_Pin) == GPIO_PIN_RESET;

            if (tick_at_last_random_led_switch < tick_now - RANDOM_LED_UPDATE_INTERVAL_MS) {
                tick_at_last_random_led_switch = tick_now;
                led_random();
            }

            // buzz on any limit
            if (ls_1 || ls_2 || ls_4) {
                set_user_leds(1, 1, 0);
            } else {
                set_user_leds(1, 0, 0);
            }

            btn_red_pressed = read_btn1();
            btn_blue_pressed = read_btn2();
            joy_up_pressed = HAL_GPIO_ReadPin(JOY_UP_GPIO_Port, JOY_UP_Pin) == GPIO_PIN_RESET;
            joy_down_pressed = HAL_GPIO_ReadPin(JOY_DOWN_GPIO_Port, JOY_DOWN_Pin) == GPIO_PIN_RESET;
            joy_left_pressed = HAL_GPIO_ReadPin(JOY_LEFT_GPIO_Port, JOY_LEFT_Pin) == GPIO_PIN_RESET;
            joy_right_pressed = HAL_GPIO_ReadPin(JOY_RIGHT_GPIO_Port, JOY_RIGHT_Pin) == GPIO_PIN_RESET;

            //led_update_tracking(joy_up_pressed, joy_down_pressed, joy_left_pressed, joy_right_pressed);

            // active button LED is on
            set_btn1_led(1);
            set_btn2_led(0);

            if (btn_red_pressed) {
                log_string("TRACKING -> CLAW_DOWN");
                tick_at_claw_down = HAL_GetTick();
                state = STATE_CLAW_DOWN;
                led_danger();
            } else if (btn_blue_pressed) {
                // TODO: some effect
            } else {
                // left-right axis
                if (joy_left_pressed && !ls_4) {
                    HAL_GPIO_WritePin(MTR_RIGHT_GPIO_Port, MTR_RIGHT_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(MTR_LEFT_GPIO_Port, MTR_LEFT_Pin, GPIO_PIN_RESET);
                } else if (joy_right_pressed) { // TODO: no limit switch here
                    HAL_GPIO_WritePin(MTR_LEFT_GPIO_Port, MTR_LEFT_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(MTR_RIGHT_GPIO_Port, MTR_RIGHT_Pin, GPIO_PIN_RESET);
                } else {
                    HAL_GPIO_WritePin(MTR_LEFT_GPIO_Port, MTR_LEFT_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(MTR_RIGHT_GPIO_Port, MTR_RIGHT_Pin, GPIO_PIN_SET);
                }

                // forward-backward axis
                if (joy_up_pressed && !ls_1) {
                    HAL_GPIO_WritePin(MTR_FORWARD_GPIO_Port, MTR_FORWARD_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(MTR_BACKWARD_GPIO_Port, MTR_BACKWARD_Pin, GPIO_PIN_RESET);
                } else if (joy_down_pressed && !ls_2) {
                    HAL_GPIO_WritePin(MTR_BACKWARD_GPIO_Port, MTR_BACKWARD_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(MTR_FORWARD_GPIO_Port, MTR_FORWARD_Pin, GPIO_PIN_RESET);
                } else {
                    HAL_GPIO_WritePin(MTR_BACKWARD_GPIO_Port, MTR_BACKWARD_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(MTR_FORWARD_GPIO_Port, MTR_FORWARD_Pin, GPIO_PIN_SET);
                }
            }
            break;
        case STATE_CLAW_DOWN:
            // active button LED is on
            set_btn1_led(0);
            set_btn2_led(1);

            btn_blue_pressed = read_btn2();
            ls_3 = read_down_ls();
            uint32_t now_ticks = HAL_GetTick();
            if (ls_3) {
                log_string("CLAW_DOWN -> CLAW_UP [bottom reached]");
                state = STATE_CLAW_UP;
                set_user_leds(1, 1, 1);
                HAL_GPIO_WritePin(MTR_DOWN_GPIO_Port, MTR_DOWN_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(MTR_UP_GPIO_Port, MTR_UP_Pin, GPIO_PIN_SET);
            } else if (btn_blue_pressed) {
                log_string("CLAW_DOWN -> CLAW_UP [button pressed]");
                state = STATE_CLAW_UP;
                set_user_leds(1, 1, 1);
                HAL_GPIO_WritePin(MTR_DOWN_GPIO_Port, MTR_DOWN_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(MTR_UP_GPIO_Port, MTR_UP_Pin, GPIO_PIN_SET);
            } else if (now_ticks - tick_at_claw_down > 5000) {
                log_string("CLAW_DOWN -> CLAW_UP [max sink time reached]");
                state = STATE_CLAW_UP;
                set_user_leds(1, 1, 1);
                HAL_GPIO_WritePin(MTR_DOWN_GPIO_Port, MTR_DOWN_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(MTR_UP_GPIO_Port, MTR_UP_Pin, GPIO_PIN_SET);
            } else { // lower claw
                HAL_GPIO_WritePin(MTR_UP_GPIO_Port, MTR_UP_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(MTR_DOWN_GPIO_Port, MTR_DOWN_Pin, GPIO_PIN_RESET);
            }
            break;
        case STATE_CLAW_UP:
            // no buttons are active
            set_btn1_led(0);
            set_btn2_led(0);

            // grab
            HAL_GPIO_WritePin(CLAW_ENABLE_GPIO_Port, CLAW_ENABLE_Pin, GPIO_PIN_RESET);

            HAL_Delay(2000);

            // raise claw
            HAL_GPIO_WritePin(MTR_DOWN_GPIO_Port, MTR_DOWN_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MTR_UP_GPIO_Port, MTR_UP_Pin, GPIO_PIN_RESET);

            // move to start position
            claw_reference_move();

            // dispense goods
            HAL_GPIO_WritePin(CLAW_ENABLE_GPIO_Port, CLAW_ENABLE_Pin, GPIO_PIN_SET);
            HAL_Delay(1000);

            // audible feedback
            beep(1, 250);

            log_string("CLAW_UP -> TRACKING");
            state = STATE_TRACKING;
            set_user_leds(0, 0, 0);
            led_random();
            break;
        default:
            HAL_NVIC_SystemReset();
            break;
    }
    HAL_Delay(1);
}

static void claw_reference_move() {
    uint8_t ls_2, ls_4, ls_5;
    uint32_t ticks_at_start;
    log_string("REFERENCE MOVE");

    ticks_at_start = HAL_GetTick();

    // no buttons are active
    set_btn1_led(0);
    set_btn2_led(0);

    // move claw at upper position
    ls_5 = read_up_ls();
    DigiLed_setAllColor(255, 0, 0);
    DigiLed_update(1);
    while (!ls_5 && (HAL_GetTick() - ticks_at_start < REF_MOVE_TIMEOUT_MS)) {
        HAL_GPIO_WritePin(MTR_UP_GPIO_Port, MTR_UP_Pin, GPIO_PIN_RESET);
        ls_5 = read_up_ls();
        HAL_Delay(1);
    }
    HAL_GPIO_WritePin(MTR_UP_GPIO_Port, MTR_UP_Pin, GPIO_PIN_SET);
    log_string("UP LIMIT OR MAX MOVE TIME");

    // move claw to left position
    DigiLed_setAllColor(0, 255, 0);
    DigiLed_update(1);

    ticks_at_start = HAL_GetTick();
    ls_4 = HAL_GPIO_ReadPin(LS4_GPIO_Port, LS4_Pin) == GPIO_PIN_RESET;
    while (!ls_4 && (HAL_GetTick() - ticks_at_start < REF_MOVE_TIMEOUT_MS)) {
        HAL_GPIO_WritePin(MTR_LEFT_GPIO_Port, MTR_LEFT_Pin, GPIO_PIN_RESET);
        ls_4 = HAL_GPIO_ReadPin(LS4_GPIO_Port, LS4_Pin) == GPIO_PIN_RESET;
        HAL_Delay(1);
    }
    HAL_GPIO_WritePin(MTR_LEFT_GPIO_Port, MTR_LEFT_Pin, GPIO_PIN_SET);
    log_string("LEFT LIMIT OR MAX MOVE TIME");

    // move claw to front position
    DigiLed_setAllColor(0, 0, 255);
    DigiLed_update(1);

    ticks_at_start = HAL_GetTick();
    ls_2 = HAL_GPIO_ReadPin(LS2_GPIO_Port, LS2_Pin) == GPIO_PIN_RESET;
    while (!ls_2 && (HAL_GetTick() - ticks_at_start < REF_MOVE_TIMEOUT_MS)) {
        HAL_GPIO_WritePin(MTR_FORWARD_GPIO_Port, MTR_FORWARD_Pin, GPIO_PIN_RESET);
        ls_2 = HAL_GPIO_ReadPin(LS2_GPIO_Port, LS2_Pin) == GPIO_PIN_RESET;
        HAL_Delay(1);
    }
    log_string("FORWARD LIMIT OR MAX MOVE TIME");
    HAL_GPIO_WritePin(MTR_FORWARD_GPIO_Port, MTR_FORWARD_Pin, GPIO_PIN_SET);

    log_string("REFERENCE MOVE DONE");
}

static void led_init() {
    DigiLed_init(&hspi1);
    DigiLed_setAllIllumination(31);
    int num_leds = DigiLed_getFrameSize();
    for (int i = 0; i < num_leds; i++) {
        DigiLed_setAllColor(0, 0, 0);
        DigiLed_setRGB(i, led_rand_red_green_or_blue());
        DigiLed_update(1);
        HAL_Delay(15);
    }
    DigiLed_setAllColor(0, 0, 0);
    DigiLed_update(1);
}

static uint32_t led_rand_red_green_or_blue() {
    uint32_t r = rand();
    if (r & 0x1) {
        return 0x00ff0000;
    } else if (r & 0x2) {
        return 0x0000ff00;
    } else {
        return 0x000000ff;
    }
}

static void led_random() {
    int num_leds = DigiLed_getFrameSize();
    for (int i = 0; i < num_leds; i++) {
        DigiLed_setRGB(i, led_rand_red_green_or_blue());
    }
    DigiLed_update(1);
}

static void led_danger() {
    DigiLed_setAllColor(255, 0, 0);
    DigiLed_update(1);
}

static void log_string(const char* str) {
    HAL_UART_Transmit(&huart3, (uint8_t *)str, (uint16_t) strlen(str), HAL_MAX_DELAY);

    const char* newline = "\n";
    HAL_UART_Transmit(&huart3, (uint8_t *)newline, 1, HAL_MAX_DELAY);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
    while(1)
    {
    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
