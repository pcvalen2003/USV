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
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

HAL_StatusTypeDef I2C_ReadRegister(uint8_t devAddr, uint8_t regAddr, uint8_t *pBuffer, uint16_t bufferSize)
{
	return HAL_I2C_Mem_Read(&hi2c2, devAddr << 1, regAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, bufferSize, HAL_MAX_DELAY);
}

HAL_StatusTypeDef I2C_WriteRegister(uint8_t devAddr, uint8_t regAddr, uint8_t *pData, uint16_t dataSize)
{
    return HAL_I2C_Mem_Write(&hi2c2, devAddr << 1, regAddr,
                            I2C_MEMADD_SIZE_8BIT, pData, dataSize, HAL_MAX_DELAY);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Definiciones para el MPU9250
#define MPU9250_ADDRESS 	0x68
#define ACCEL_CONFIG 		0x1C
#define ACCEL_XOUT_H    	0x3B
#define GYRO_XOUT_H     	0x43
#define EXT_SENS_DATA_00 	0x49  // Primer registro del magnetómetro

// Variables para los datos de los sensores
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;
int16_t magX, magY, magZ;
uint8_t sensorData[14]; // Buffer para datos (6 acc + 6 gyro + 8 mag)
float heading;

// Configuración inicial de los sensores
void MPU9250_Init(void)
{
    uint8_t config;

    // Despertar el dispositivo
    config = 0x00;
    I2C_WriteRegister(MPU9250_ADDRESS, 0x6B, &config, 1);

    // Configurar acelerómetro ±2g
    config = 0x00;
    I2C_WriteRegister(MPU9250_ADDRESS, 0x1C, &config, 1);

    // Configurar giroscopio ±250°/s
    config = 0x00;
    I2C_WriteRegister(MPU9250_ADDRESS, 0x1B, &config, 1);

//    // Configurar el magnetómetro (modo continuo medición 1)
//    I2C_WriteRegister(MPU9250_ADDRESS, 0x37, 0x02, 1); // Bypass I2C
//    I2C_WriteRegister(0x0C, 0x0A, 0x16, 1); // AK8963 (magnetómetro)

    HAL_Delay(100);
}


// Dirección del magnetómetro AK8963 (dentro del MPU9250)
#define AK8963_ADDRESS 0x0C
#define AK8963_WHO_AM_I 0x00
#define AK8963_CNTL1 0x0A
#define AK8963_ST1 0x02
#define AK8963_HXL 0x03

// Función de inicialización mejorada del magnetómetro
void Magnetometer_Init(void)
{
    uint8_t data[3];

    // 1. Habilitar bypass I2C para acceder directamente al AK8963
    uint8_t bypass_enable = 0x02;
    I2C_WriteRegister(MPU9250_ADDRESS, 0x37, &bypass_enable, 1);
    HAL_Delay(10);

    // 2. Verificar conexión con el magnetómetro
    I2C_ReadRegister(AK8963_ADDRESS, AK8963_WHO_AM_I, data, 1);
    if(data[0] != 0x48) {
        // Error: El magnetómetro no responde correctamente
        while(1); // Bloquear o manejar el error
    }

    // 3. Configurar el magnetómetro en modo Fuse ROM
    data[0] = 0x0F; // Modo Fuse ROM access
    I2C_WriteRegister(AK8963_ADDRESS, AK8963_CNTL1, data, 1);
    HAL_Delay(10);

    // 4. Leer los valores de ajuste de sensibilidad (ASA)
    I2C_ReadRegister(AK8963_ADDRESS, 0x10, data, 3);
    float mag_sens_adj[3];
    mag_sens_adj[0] = (data[0] - 128) / 256.0f + 1.0f;
    mag_sens_adj[1] = (data[1] - 128) / 256.0f + 1.0f;
    mag_sens_adj[2] = (data[2] - 128) / 256.0f + 1.0f;

    // 5. Configurar modo de medición continua (16-bit, 100Hz)
    data[0] = 0x16; // Modo continuo 2 (100Hz), 16 bits
    I2C_WriteRegister(AK8963_ADDRESS, AK8963_CNTL1, data, 1);
    HAL_Delay(10);
}

// Función para leer el magnetómetro
uint8_t Read_Magnetometer(int16_t *magX, int16_t *magY, int16_t *magZ)
{
    uint8_t status;
    uint8_t magData[7];

    // 1. Leer registro de estado
    I2C_ReadRegister(AK8963_ADDRESS, AK8963_ST1, &status, 1);

    if(status & 0x01) { // Si hay datos nuevos
        // 2. Leer los 7 registros (6 datos + ST2)
        I2C_ReadRegister(AK8963_ADDRESS, AK8963_HXL, magData, 7);

        // 3. Verificar overflow (bit HOFL en ST2)
        if(!(magData[6] & 0x08)) {
            // 4. Convertir datos (el orden es little-endian: LSB primero)
            *magX = (int16_t)(magData[1] << 8 | magData[0]);
            *magY = (int16_t)(magData[3] << 8 | magData[2]);
            *magZ = (int16_t)(magData[5] << 8 | magData[4]);
            return 1; // Lectura exitosa
        }
    }
    return 0; // No hay datos nuevos o hubo overflow
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
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  // Configuración inicial del acelerómetro
  MPU9250_Init();
  Magnetometer_Init();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Leer acelerómetro (6 bytes)
	      if (I2C_ReadRegister(MPU9250_ADDRESS, ACCEL_XOUT_H, sensorData, 6) == HAL_OK)
	      {
	          accX = (int16_t)((sensorData[0] << 8) | sensorData[1]);
	          accY = (int16_t)((sensorData[2] << 8) | sensorData[3]);
	          accZ = (int16_t)((sensorData[4] << 8) | sensorData[5]);
	      }

	      // Leer giroscopio (6 bytes)
	      if (I2C_ReadRegister(MPU9250_ADDRESS, GYRO_XOUT_H, sensorData, 6) == HAL_OK)
	      {
	          gyroX = (int16_t)((sensorData[0] << 8) | sensorData[1]);
	          gyroY = (int16_t)((sensorData[2] << 8) | sensorData[3]);
	          gyroZ = (int16_t)((sensorData[4] << 8) | sensorData[5]);
	      }

	      // Leer magnetómetro (7 bytes - el último es registro de estado)
//	      if (I2C_ReadRegister(MPU9250_ADDRESS, EXT_SENS_DATA_00, sensorData, 7) == HAL_OK)
//	      {
//	          // El magnetómetro tiene los bytes invertidos (primero LOW luego HIGH)
//	          magX = (int16_t)((sensorData[1] << 8) | sensorData[0]);
//	          magY = (int16_t)((sensorData[3] << 8) | sensorData[2]);
//	          magZ = (int16_t)((sensorData[5] << 8) | sensorData[4]);
//
//	          // El byte 6 contiene el ST2 register (status)
//	          uint8_t magStatus = sensorData[6];
//	      }

	      Read_Magnetometer(&magX, &magY, &magZ);
	      heading = (3 - atan2f(-(magY - 128), magX - 128)) * 60;

	      // Aquí puedes procesar los datos:
	      // - accX, accY, accZ (aceleración en counts)
	      // - gyroX, gyroY, gyroZ (velocidad angular en counts)
	      // - magX, magY, magZ (campo magnético en counts)

	      // Conversión opcional a unidades físicas:
	      // float accX_g = accX / 16384.0f;  // Para ±2g
	      // float gyroX_dps = gyroX / 131.0f; // Para ±250°/s
	      // float magX_uT = magX * 0.15f;    // Conversión a microteslas

	      HAL_Delay(20); // Frecuencia de muestreo ~50Hz
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
