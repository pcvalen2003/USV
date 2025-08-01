/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "NRF24.h"
#include "NRF24_conf.h"
#include "NRF24_reg_addresses.h"
#include "gps.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define _BV(x) (1 << (x))
#define GPS_RX_BUFFER_SIZE 128
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* Definitions for ACQ_GPS */
osThreadId_t ACQ_GPSHandle;
const osThreadAttr_t ACQ_GPS_attributes = {
  .name = "ACQ_GPS",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ACQ_MPU */
osThreadId_t ACQ_MPUHandle;
const osThreadAttr_t ACQ_MPU_attributes = {
  .name = "ACQ_MPU",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for WatchDog */
osThreadId_t WatchDogHandle;
const osThreadAttr_t WatchDog_attributes = {
  .name = "WatchDog",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Proc_datos */
osThreadId_t Proc_datosHandle;
const osThreadAttr_t Proc_datos_attributes = {
  .name = "Proc_datos",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
/*****	ERROR Managment	****/
typedef enum {
	NRF24_NOT_INIT,
	NRF24_TX_LOST,
	NRF24_EXTRA_NOT_CPY
}error_t;

error_t error;
/***** Variables NRF24	*****/
uint8_t NRF_addr[5] = {0xE1, 0xF0, 0xF0, 0xE8, 0xE8};	//Adress del módulo NRF24

typedef struct{		// Estructura del mensaje a recibir
	uint8_t MODO;
	uint8_t POTENCIA;
	uint8_t DIRECCION;
	char EXTRA[29];
}msg_t;

msg_t mensaje = {
		.MODO = 0,
		.POTENCIA = 0,
		.DIRECCION = 0,
		.EXTRA = {0}
};

uint16_t PLD_SIZE =  sizeof(msg_t);

/***** Variables FSM onBoard	******/
typedef enum{
	MANUAL = 0b00000001,
	WAYPOINT = 0b00000011
}modo_t;

/***** Buffer de recepción NMEA *****/
uint8_t GPSrx_buffer1[128];
uint8_t GPSrx_buffer2[128];
uint8_t GPSrx_buffer3[128];
typedef enum {
	MSG_ON_BUF1,
	MSG_ON_BUF2,
	MSG_ON_BUF3
}notifyGPS_t;

uint8_t len;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
void StartACQGPS(void *argument);
void StartACQMPU(void *argument);
void StartWatchDog(void *argument);
void StartProcDatos(void *argument);

/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (GPIO_Pin == GPIO_PIN_15)
    {
    	if (nrf24_data_available()) {
    		uint8_t rx_buffer[sizeof(msg_t)];

    		nrf24_receive(rx_buffer, sizeof(msg_t));

    		mensaje.MODO = rx_buffer[0];
    		mensaje.POTENCIA = rx_buffer[1];
    		mensaje.DIRECCION = rx_buffer[2];

    		char *ret = memcpy(mensaje.EXTRA, &rx_buffer[3], sizeof(mensaje.EXTRA));
    		if(ret != mensaje.EXTRA){
    			error = NRF24_EXTRA_NOT_CPY;
    		}
    		nrf24_flush_rx();
    	}

    	//Activada la IRQ de msj recibido, notifico a la task proc. datos
    	vTaskNotifyGiveFromISR(Proc_datosHandle, &xHigherPriorityTaskWoken); //Notificar a procesar datos que hay uno nuevo
    	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }

}
void USART1_IRQHandler(void) {
	static uint8_t buffer_idx;
	const uint8_t *buffers[] = { GPSrx_buffer1, GPSrx_buffer2, GPSrx_buffer3};
	notifyGPS_t NotifyGPS;

    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);

        // Desactivo DMA momentáneamente
        HAL_UART_DMAStop(&huart1);

        switch (buffer_idx) {
			case 0:
				NotifyGPS = MSG_ON_BUF1;
				break;
			case 1:
				NotifyGPS = MSG_ON_BUF2;
				break;
			case 2:
				NotifyGPS = MSG_ON_BUF3;
				break;
		}

        // Calculás cuántos bytes llegaron
        len = GPS_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
        buffer_idx = (buffer_idx + 1) % 3;

        ulTaskNotifyValueClear(ACQ_GPSHandle, 0xFFFFFFFF);   // limpio notificaciones previas
        xTaskNotify(ACQ_GPSHandle, NotifyGPS, eSetBits);  // nueva notificación

        // Reinicio DMA desde el buffer correspondiente
        HAL_UART_Receive_DMA(&huart1, buffers[buffer_idx], GPS_RX_BUFFER_SIZE);
    }
}
bool NRF24_Init(void){
	ce_low();
	csn_high();

	HAL_Delay(5);

	// Velocidad inicial confiable (1 Mbps)
	nrf24_data_rate(_1mbps);

    // Habilitar ACK payload y payload dinámico
	nrf24_en_ack_pld(enable);
	nrf24_en_dyn_ack(enable);
	nrf24_dpl(enable);

    // Habilitar auto-ack en todos los pipes (puede ajustarse)
	nrf24_auto_ack_all(enable);

    // Habilitar solo pipes RX 0 y 1 por defecto
	nrf24_open_rx_pipe(0, addr);

    // Tamaño fijo de payload (por defecto 32 bytes)
	nrf24_pipe_pld_size(0, PLD_SIZE);
	nrf24_pipe_pld_size(1, PLD_SIZE);
	nrf24_pipe_pld_size(2, PLD_SIZE);
	nrf24_pipe_pld_size(3, PLD_SIZE);
	nrf24_pipe_pld_size(4, PLD_SIZE);
	nrf24_pipe_pld_size(5, PLD_SIZE);

    // Ancho de dirección: 5 bytes (máximo permitido)
	nrf24_set_addr_width(5);

	// Canal por defecto (76 → dentro de ISM band y sin interferencia típica)
	nrf24_set_channel(76);

    // Limpiar flags de status
	nrf24_clear_max_rt();
	nrf24_clear_rx_dr();
	nrf24_clear_tx_ds();

    // Flush buffers
	nrf24_flush_rx();
	nrf24_flush_tx();

	//Retransmission setup
	//0110 (ARD) = 1.5ms, 0101 (ARC) = 5 retries.
	uint8_t setup = 0b01100101;
	uint8_t* p = &setup;
	nrf24_w_reg(SETUP_RETR, p, sizeof(setup));

	uint8_t SETUP_reg = nrf24_r_reg(SETUP_RETR, 1);

	//Para debug
	uint8_t CONFIG_reg = nrf24_r_reg(CONFIG, 1);

    // Configurar registro CONFIG:
    // - CRC de 2 bytes (CRCO)
    // - CRC habilitado (EN_CRC)
    // - Power Up (PWR_UP)
    // - Modo PTX (PRIM_RX = 0)
	nrf24_set_crc(en_crc, _2byte);
	nrf24_pwr_up();
	// Pongo en Modo PTX (PRIM_RX = 0)
	nrf24_set_bit(CONFIG, 0, 0);

    // Delay de power-up a standby (~1.5ms recomendado)
    HAL_Delay(2);

    nrf24_open_rx_pipe(0, addr);
    nrf24_listen();

    // Verificación de escritura de CONFIG
    return (nrf24_r_reg(CONFIG, 1) == (_BV(EN_CRC) | _BV(CRCO) | _BV(PWR_UP))) ? true : false;
}
void Motor_Init(void){
	//Inicialización MOTORES
	  HAL_TIM_Base_Start(&htim1);
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

	  htim1.Instance->CCR1 = 256;
	  htim1.Instance->CCR2 = 256;

	  HAL_Delay(2000);

	  htim1.Instance->CCR1 = 128;
	  htim1.Instance->CCR2 = 128;

	  HAL_Delay(2000);

	  // Innicialización TIMóN
	  htim1.Instance->CCR3 = 128; //0 grados

	  HAL_Delay(500);

	  htim1.Instance->CCR3 = 256;	//180 grados (max)

	  HAL_Delay(500);

	  htim1.Instance->CCR3 = 192;	//90 grados (centrado)
}
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
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  if (NRF24_Init() == 0) {
	  error = NRF24_NOT_INIT;
	  return error;
  }
  Motor_Init();
  HAL_UART_Receive_DMA(&huart1, GPSrx_buffer1, GPS_RX_BUFFER_SIZE); //Inicia en el Buffer 1
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ACQ_GPS */
  ACQ_GPSHandle = osThreadNew(StartACQGPS, NULL, &ACQ_GPS_attributes);

  /* creation of ACQ_MPU */
  ACQ_MPUHandle = osThreadNew(StartACQMPU, NULL, &ACQ_MPU_attributes);

  /* creation of WatchDog */
  WatchDogHandle = osThreadNew(StartWatchDog, NULL, &WatchDog_attributes);

  /* creation of Proc_datos */
  Proc_datosHandle = osThreadNew(StartProcDatos, NULL, &Proc_datos_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 125 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2560 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : NRF_IRQ_Pin */
  GPIO_InitStruct.Pin = NRF_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NRF_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : VBAT_IN_Pin IBAT_IN_Pin */
  GPIO_InitStruct.Pin = VBAT_IN_Pin|IBAT_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF_CE_Pin */
  GPIO_InitStruct.Pin = NRF_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NRF_CE_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartACQGPS */
/**
  * @brief  Function implementing the ACQ_GPS thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartACQGPS */
void StartACQGPS(void *argument)
{
  /* USER CODE BEGIN 5 */
	notifyGPS_t NotifiedValue;
  /* Infinite loop */
  for(;;)
  {
		xTaskNotifyWait(0x0, 0xFFFFFFFF, &NotifiedValue, portMAX_DELAY);

		switch (NotifiedValue) {	//La notif. determina el buffer a leer.
			case MSG_ON_BUF1:
				for (int i = 0; i < len; i++) { // len siempre reserva el valor de cantidad de bytes escritos en el buffer
					rx_data = GPSrx_buffer1[i];
					GPS_UART_CallBack();
				}
				break;
			case MSG_ON_BUF2:
				for (int i = 0; i < len; i++) {
					rx_data = GPSrx_buffer2[i];
					GPS_UART_CallBack();
				}
				break;
			case MSG_ON_BUF3:
				for (int i = 0; i < len; i++) {
					rx_data = GPSrx_buffer3[i];
					GPS_UART_CallBack();
				}
				break;
		}
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartACQMPU */
/**
* @brief Function implementing the ACQ_MPU thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartACQMPU */
void StartACQMPU(void *argument)
{
  /* USER CODE BEGIN StartACQMPU */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartACQMPU */
}

/* USER CODE BEGIN Header_StartWatchDog */
/**
* @brief Function implementing the WatchDog thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartWatchDog */
void StartWatchDog(void *argument)
{
  /* USER CODE BEGIN StartWatchDog */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartWatchDog */
}

/* USER CODE BEGIN Header_StartProcDatos */
/**
* @brief Function implementing the Proc_datos thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartProcDatos */
void StartProcDatos(void *argument)
{
  /* USER CODE BEGIN StartProcDatos */
	modo_t modo;
	uint8_t direccion;
	uint8_t potencia;

	typedef enum {
		IZQUIERDA = 0,
		DERECHA,
		CENTRO
	}dir_t;
	dir_t i_or_d;

	uint8_t k_giro;
	uint8_t k_timon;

	typedef struct {
		uint8_t m_der;
		uint8_t m_izq;
	}vel_t;
	vel_t velocidad;

  /* Infinite loop */
  for(;;)
  {
	  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	  // Copia de los datos del mensaje recibido por RF
	  modo = (modo_t)mensaje.MODO;
	  direccion = mensaje.DIRECCION;
	  potencia = mensaje.POTENCIA;

	  // Lógica de asignación de dirección
	  if (direccion < 128) i_or_d = IZQUIERDA;
	  else if (direccion > 128) i_or_d = DERECHA;
	  else i_or_d = CENTRO;

	  //Ganancia de giro
	  k_giro = lut_giro[direccion];	//Asigna valor según función abs, parabola, etc.
	  //Ganancia de giro del Timon
	  k_timon = lut_timon[direccion];	//Asigna valor clampeado entre 45 y 135 grados.

	  // Asignación de potencia de los motores
	  velocidad.m_der = i_or_d == DERECHA? (potencia * k_giro)/256 : potencia;
	  velocidad. m_izq = i_or_d == IZQUIERDA? (potencia * k_giro)/256 : potencia;

	  switch (modo) {
		case MANUAL:

			htim1.Instance->CCR1 = mensaje.POTENCIA*kdir;

			break;
		case WAYPOINT:

			break;
	}
  }
  /* USER CODE END StartProcDatos */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
