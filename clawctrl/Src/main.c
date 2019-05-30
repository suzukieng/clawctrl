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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdint.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DO_REFERENCE_MOVE 1

#define STATE_INITIALIZING 0
#define STATE_TRACKING 1
#define STATE_CLAW 2
#define STATE_DISPENSING 3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */

#pragma location=0x30040000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30040060
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
#pragma location=0x30040200
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffers */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30040000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30040060))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
__attribute__((at(0x30040200))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffer */

#elif defined ( __GNUC__ ) /* GNU Compiler */ 

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE] __attribute__((section(".RxArraySection"))); /* Ethernet Receive Buffers */

#endif

ETH_TxPacketConfig TxConfig; 

ETH_HandleTypeDef heth;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

static int state = STATE_INITIALIZING;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */

static void claw_main(void);
static void claw_init(void);
static void claw_reference_move(void);
static void log_string(const char* str);

static void set_buzzer(uint8_t on);
static void set_user_led1(uint8_t on);
static void set_user_led2(uint8_t on);
static void set_user_led3(uint8_t on);
static void set_user_leds(uint8_t on1, uint8_t on2, uint8_t on3);


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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */

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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_USB;
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
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   uint8_t MACAddr[6] ;

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */
    
  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : JOY_LEFT_Pin JOY_RIGHT_Pin JOY_UP_Pin JOY_DOWN_Pin 
                           BTN_1_Pin */
  GPIO_InitStruct.Pin = JOY_LEFT_Pin|JOY_RIGHT_Pin|JOY_UP_Pin|JOY_DOWN_Pin 
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

}

/* USER CODE BEGIN 4 */

static void claw_init() {

    HAL_GPIO_WritePin(MTR_DOWN_GPIO_Port, MTR_DOWN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MTR_UP_GPIO_Port, MTR_UP_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MTR_FORWARD_GPIO_Port, MTR_FORWARD_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MTR_BACKWARD_GPIO_Port, MTR_BACKWARD_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MTR_LEFT_GPIO_Port, MTR_LEFT_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MTR_RIGHT_GPIO_Port, MTR_RIGHT_Pin, GPIO_PIN_SET);

    // 1 second beep
    set_buzzer(1);
    HAL_Delay(1000);
    set_buzzer(0);

    // 1 second blink LEDS
    for (int i = 0; i < 8; i++) {
        uint8_t even = (i % 2 == 0) ? 0 : 1;
        set_user_leds(even, even, even);
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, even ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, even ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_Delay(1000 / 8);
    }

    // do the reference move
#if DO_REFERENCE_MOVE
    claw_reference_move();
#endif

    // double beep after reference move
    set_buzzer(1);
    HAL_Delay(250);
    set_buzzer(0);
    HAL_Delay(250);
    set_buzzer(1);
    HAL_Delay(250);
    set_buzzer(0);

    // start in initial state
    state = STATE_INITIALIZING;
}

static void claw_main() {
    uint8_t btn_red_pressed, btn_blue_pressed;
    uint8_t user_b1_pressed;
    uint8_t joy_up_pressed, joy_down_pressed, joy_left_pressed, joy_right_pressed;
    uint8_t ls_1, ls_2, ls_3, ls_4, ls_5;

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

            HAL_GPIO_WritePin(CLAW_ENABLE_GPIO_Port, CLAW_ENABLE_Pin, GPIO_PIN_RESET);

            log_string("INITIALIZING -> TRACKING");
            state = STATE_TRACKING;
            set_user_leds(1, 0, 0);
            break;
        case STATE_TRACKING:
            ls_1 = HAL_GPIO_ReadPin(LS1_GPIO_Port, LS1_Pin) == GPIO_PIN_RESET;
            ls_2 = HAL_GPIO_ReadPin(LS2_GPIO_Port, LS2_Pin) == GPIO_PIN_RESET;
            ls_3 = HAL_GPIO_ReadPin(LS3_GPIO_Port, LS3_Pin) == GPIO_PIN_RESET;
            ls_4 = HAL_GPIO_ReadPin(LS4_GPIO_Port, LS4_Pin) == GPIO_PIN_RESET;
            ls_5 = HAL_GPIO_ReadPin(LS5_GPIO_Port, LS5_Pin) == GPIO_PIN_RESET;

            // buzz on any limit
            if (ls_1 || ls_2 || ls_3 || ls_4 || ls_5) {
                set_user_leds(1, 1, 0);
            } else {
                set_user_leds(1, 0, 0);
            }

            user_b1_pressed = HAL_GPIO_ReadPin(USER_B1_GPIO_Port, USER_B1_Pin) == GPIO_PIN_SET;
            btn_red_pressed = HAL_GPIO_ReadPin(BTN_1_GPIO_Port, BTN_1_Pin) == GPIO_PIN_RESET;
            btn_blue_pressed = HAL_GPIO_ReadPin(BTN_2_GPIO_Port, BTN_2_Pin) == GPIO_PIN_RESET;
            joy_up_pressed = HAL_GPIO_ReadPin(JOY_UP_GPIO_Port, JOY_UP_Pin) == GPIO_PIN_RESET;
            joy_down_pressed = HAL_GPIO_ReadPin(JOY_DOWN_GPIO_Port, JOY_DOWN_Pin) == GPIO_PIN_RESET;
            joy_left_pressed = HAL_GPIO_ReadPin(JOY_LEFT_GPIO_Port, JOY_LEFT_Pin) == GPIO_PIN_RESET;
            joy_right_pressed = HAL_GPIO_ReadPin(JOY_RIGHT_GPIO_Port, JOY_RIGHT_Pin) == GPIO_PIN_RESET;

            // button LEDs turn on/off when their buttons are pressed
            HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, btn_red_pressed ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, btn_blue_pressed ? GPIO_PIN_SET : GPIO_PIN_RESET);

            if (user_b1_pressed) {
                HAL_GPIO_WritePin(CLAW_ENABLE_GPIO_Port, CLAW_ENABLE_Pin, GPIO_PIN_RESET);
            } else {
                HAL_GPIO_WritePin(CLAW_ENABLE_GPIO_Port, CLAW_ENABLE_Pin, GPIO_PIN_SET);
            }

            if (btn_red_pressed && !ls_5) {
                HAL_GPIO_WritePin(MTR_DOWN_GPIO_Port, MTR_DOWN_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(MTR_UP_GPIO_Port, MTR_UP_Pin, GPIO_PIN_RESET);
            } else if (btn_blue_pressed && !ls_3) {
                HAL_GPIO_WritePin(MTR_DOWN_GPIO_Port, MTR_DOWN_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(MTR_UP_GPIO_Port, MTR_UP_Pin, GPIO_PIN_SET);
            } else {
                HAL_GPIO_WritePin(MTR_UP_GPIO_Port, MTR_UP_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(MTR_DOWN_GPIO_Port, MTR_DOWN_Pin, GPIO_PIN_SET);

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
        case STATE_CLAW:
            HAL_GPIO_WritePin(CLAW_ENABLE_GPIO_Port, CLAW_ENABLE_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MTR_DOWN_GPIO_Port, MTR_DOWN_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MTR_UP_GPIO_Port, MTR_UP_Pin, GPIO_PIN_SET);

            HAL_Delay(2500);

            HAL_GPIO_WritePin(MTR_DOWN_GPIO_Port, MTR_DOWN_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(CLAW_ENABLE_GPIO_Port, CLAW_ENABLE_Pin, GPIO_PIN_RESET);

            HAL_Delay(1500);

            HAL_GPIO_WritePin(MTR_UP_GPIO_Port, MTR_UP_Pin, GPIO_PIN_RESET);

            HAL_Delay(2500);

            HAL_GPIO_WritePin(MTR_UP_GPIO_Port, MTR_UP_Pin, GPIO_PIN_SET);

            log_string("CLAW -> DISPENSING");
            state = STATE_DISPENSING;
            set_user_leds(1, 1, 1);

            break;
        case STATE_DISPENSING:
            // TODO: move to hole

            // dispense goods
            HAL_GPIO_WritePin(CLAW_ENABLE_GPIO_Port, CLAW_ENABLE_Pin, GPIO_PIN_SET);

            HAL_Delay(1000);
            log_string("DISPENSING -> INITIALIZING");
            state = STATE_INITIALIZING;
            set_user_leds(0, 0, 0);
            break;
        default:
            HAL_NVIC_SystemReset();
            break;
    }
    HAL_Delay(1);
    //dump_state();
}

static void claw_reference_move() {
    uint8_t ls_2, ls_4, ls_5;

    log_string("REFERENCE MOVE");

    // move claw at upper position
    ls_5 = HAL_GPIO_ReadPin(LS5_GPIO_Port, LS5_Pin) == GPIO_PIN_RESET;
    while (!ls_5) {
        HAL_GPIO_WritePin(MTR_UP_GPIO_Port, MTR_UP_Pin, GPIO_PIN_RESET);
        ls_5 = HAL_GPIO_ReadPin(LS5_GPIO_Port, LS5_Pin) == GPIO_PIN_RESET;
        HAL_Delay(1);
    }
    HAL_GPIO_WritePin(MTR_UP_GPIO_Port, MTR_UP_Pin, GPIO_PIN_SET);
    log_string("UP LIMIT");

    // move claw to left position

    ls_4 = HAL_GPIO_ReadPin(LS4_GPIO_Port, LS4_Pin) == GPIO_PIN_RESET;
    while (!ls_4) {
        HAL_GPIO_WritePin(MTR_LEFT_GPIO_Port, MTR_LEFT_Pin, GPIO_PIN_RESET);
        ls_4 = HAL_GPIO_ReadPin(LS4_GPIO_Port, LS4_Pin) == GPIO_PIN_RESET;
        HAL_Delay(1);
    }
    HAL_GPIO_WritePin(MTR_LEFT_GPIO_Port, MTR_LEFT_Pin, GPIO_PIN_SET);
    log_string("LEFT LIMIT");

    // move claw to front position
    ls_2 = HAL_GPIO_ReadPin(LS2_GPIO_Port, LS2_Pin) == GPIO_PIN_RESET;
    while (!ls_2) {
        HAL_GPIO_WritePin(MTR_FORWARD_GPIO_Port, MTR_FORWARD_Pin, GPIO_PIN_RESET);
        ls_2 = HAL_GPIO_ReadPin(LS2_GPIO_Port, LS2_Pin) == GPIO_PIN_RESET;
        HAL_Delay(1);
    }
    log_string("FORWARD LIMIT");
    HAL_GPIO_WritePin(MTR_FORWARD_GPIO_Port, MTR_FORWARD_Pin, GPIO_PIN_SET);

    log_string("REFERENCE MOVE DONE");
}

static void log_string(const char* str) {
    HAL_UART_Transmit(&huart3, (uint8_t *)str, (uint16_t) strlen(str), HAL_MAX_DELAY);

    const char* newline = "\n";
    HAL_UART_Transmit(&huart3, (uint8_t *)newline, 1, HAL_MAX_DELAY);
}

static void set_user_led1(uint8_t on) {
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void set_user_led2(uint8_t on) {
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void set_user_led3(uint8_t on) {
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void set_user_leds(uint8_t on1, uint8_t on2, uint8_t on3) {
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, on1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, on2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, on3 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void set_buzzer(uint8_t on) {
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
