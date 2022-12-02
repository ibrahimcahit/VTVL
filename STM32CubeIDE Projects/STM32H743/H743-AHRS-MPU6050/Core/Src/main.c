/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "stdio.h"
#include "math.h"
#include "mpu6050.h"
#include "micros.h"
#include "quaternion.h"
#include "FusionAHRS.h"
#include "LowPassFilter.h"
#include "NotchFilter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
MPU6050_t imu_t;

Quaternion_t quaternion_t;
FusionBias fusionBiasIMU1;
FusionAhrs fusionAhrsIMU1;
FusionAHRS_t AHRS_IMU1;

NotchFilter_t NF_gyro_x, NF_gyro_y, NF_gyro_z;
LPFTwoPole_t LPF_accel_x, LPF_accel_y, LPF_accel_z, LPF_gyro_x, LPF_gyro_y, LPF_gyro_z;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define 	SAMPLE_FREQ_HZ 		1000.0f	// Data sample frequency in Hz
#define 	LPF_ACCEL_CTOFF_HZ 	260.0f	// LPF Cut-Off or accelerometer
#define 	LPF_GYRO_CTOFF_HZ 	256.0f	// LPF Cut-Off or gyro
#define 	NF_GYRO_CFREQ_HZ	74.0f	// NF center frequency for Gyro
#define 	NF_GYRO_NWDTH_HZ	5.0f	// NF notch-width frequency for Gyro
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
	return len;
}

float sample_time_sec_f32 = 1.0f / SAMPLE_FREQ_HZ;
float sample_time_us_f32 = (1.0f / SAMPLE_FREQ_HZ) * 1000000.0f;

float accelLowPassFiltered_f32[3], gyroLowPassFiltered_f32[3], gyroNotchFiltered_f32[3];

float eulerAngles_f32[3];

uint64_t timer_u64 = 0;
uint64_t lastTime_u64 = 0;
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
  MX_LPUART1_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  //Init DWT Clock for proper us time tick
  DWT_Init();

  //Init filter with predefined settings
  LPFTwoPole_Init(&LPF_accel_x, LPF_TYPE_BESSEL, LPF_ACCEL_CTOFF_HZ, sample_time_sec_f32);
  LPFTwoPole_Init(&LPF_accel_y, LPF_TYPE_BESSEL, LPF_ACCEL_CTOFF_HZ, sample_time_sec_f32);
  LPFTwoPole_Init(&LPF_accel_z, LPF_TYPE_BESSEL, LPF_ACCEL_CTOFF_HZ, sample_time_sec_f32);

  LPFTwoPole_Init(&LPF_gyro_x, LPF_TYPE_BESSEL, LPF_GYRO_CTOFF_HZ, sample_time_sec_f32);
  LPFTwoPole_Init(&LPF_gyro_y, LPF_TYPE_BESSEL, LPF_GYRO_CTOFF_HZ, sample_time_sec_f32);
  LPFTwoPole_Init(&LPF_gyro_z, LPF_TYPE_BESSEL, LPF_GYRO_CTOFF_HZ, sample_time_sec_f32);

  NotchFilterInit(&NF_gyro_x, NF_GYRO_CFREQ_HZ, NF_GYRO_NWDTH_HZ, sample_time_sec_f32);
  NotchFilterInit(&NF_gyro_y, NF_GYRO_CFREQ_HZ, NF_GYRO_NWDTH_HZ, sample_time_sec_f32);
  NotchFilterInit(&NF_gyro_z, NF_GYRO_CFREQ_HZ, NF_GYRO_NWDTH_HZ, sample_time_sec_f32);

  //Init state estimators
  quaternionInit(&quaternion_t, sample_time_us_f32);
  initFusionAHRS(&fusionBiasIMU1, &fusionAhrsIMU1, &AHRS_IMU1, sample_time_sec_f32);

  //Init sensors
	while (MPU6050_Init(&hi2c1, &imu_t));

	if (imu_t.CALIBRATIN_OK_u8 == TRUE)
	{
	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
	}

	uint8_t newData_u8;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	//Get system time in us
	timer_u64 = micros();

	if ( ((timer_u64 - lastTime_u64) >= sample_time_us_f32) && (imu_t.CALIBRATIN_OK_u8 == TRUE) )
	{
		lastTime_u64 = micros();

		//Read MPU6050 sensor data
		readMPU6050(&hi2c1, &imu_t);

		//Get accelerometer data in "g" and run LPF
		accelLowPassFiltered_f32[0] = (LPFTwoPole_Update(&LPF_accel_x, imu_t.MPU6050_Accel_f32[0]));
		accelLowPassFiltered_f32[1] = (LPFTwoPole_Update(&LPF_accel_y, imu_t.MPU6050_Accel_f32[1]));
		accelLowPassFiltered_f32[2] = (LPFTwoPole_Update(&LPF_accel_z, imu_t.MPU6050_Accel_f32[2]));

		//Get gyro data in "deg/s" and run LPF
		gyroLowPassFiltered_f32[0] = NotchFilter_Update(&NF_gyro_x, imu_t.MPU6050_Gyro_f32[0]);
		gyroLowPassFiltered_f32[1] = NotchFilter_Update(&NF_gyro_y, imu_t.MPU6050_Gyro_f32[1]);
		gyroLowPassFiltered_f32[2] = NotchFilter_Update(&NF_gyro_z, imu_t.MPU6050_Gyro_f32[2]);

		//Put gyro data into Notch Filter to flat-out any data in specific frequency band
		gyroNotchFiltered_f32[0] = (LPFTwoPole_Update(&LPF_gyro_x, gyroLowPassFiltered_f32[0]));
		gyroNotchFiltered_f32[1] = (LPFTwoPole_Update(&LPF_gyro_y, gyroLowPassFiltered_f32[1]));
		gyroNotchFiltered_f32[2] = (LPFTwoPole_Update(&LPF_gyro_z, gyroLowPassFiltered_f32[2]));

		//Get state estimations, using quaternion and fusion-quaternion based estimators
		quaternionUpdate(&quaternion_t, accelLowPassFiltered_f32[0], accelLowPassFiltered_f32[1], accelLowPassFiltered_f32[2],
				gyroNotchFiltered_f32[0]*(M_PI/180.0f), gyroNotchFiltered_f32[1]*(M_PI/180.0f),
					gyroNotchFiltered_f32[2]*(M_PI/180.0f));

		getFusionAHRS_6DoF(&fusionBiasIMU1, &fusionAhrsIMU1, &AHRS_IMU1, accelLowPassFiltered_f32[0], accelLowPassFiltered_f32[1],
				accelLowPassFiltered_f32[2], gyroNotchFiltered_f32[0]*(M_PI/180.0f), gyroNotchFiltered_f32[1]*(M_PI/180.0f),
					gyroNotchFiltered_f32[2]*(M_PI/180.0f));

		newData_u8 = TRUE; //Set newData to high for activate UART printer

	}//end of timer if

	if(newData_u8)
	{
		printf("%f, %f, %f\r\n",
				quaternion_t.yaw, quaternion_t.pitch, quaternion_t.roll);
//				AHRS_IMU1.YAW, AHRS_IMU1.PITCH, AHRS_IMU1.ROLL);
		newData_u8 = FALSE;
	}

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 1;
  RCC_OscInitStruct.PLL.PLLR = 8;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

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

