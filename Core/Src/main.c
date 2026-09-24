/* USER CODE BEGIN Header */
/**
 * By:  Armando S. Sanca
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define _USE_MATH_DEFINES
#include "lcd.h"
#include <stdio.h>
#include "bno055_stm32.h"
#include "usbd_cdc_if.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct ControleMotor {
	float Kp, Kd, Ki;
	float setpoint;
	float erro_anterior, integral, derivada;
	float rpm_atual;
} controleMotor_t;

typedef struct RoboDiferencial {
	controleMotor_t motor_esquerdo, motor_direito;
	float L, raio_roda;
	float rpm_max, rpm_min;
	float v_lin, v_ang;
	float pos_x, pos_y;
	float theta;
} roboDiferencial_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PPR 15336
#define PWM_MAX 10000
#define TS_S 0.01
#define TS_MS 10
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a):(b))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

////////////////////////////////////////////////////////////////////////////////////////////////////////



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
__IO uint16_t uhADCxConvertedValue[4];  // Interno do código de demostração
__IO uint32_t uhADCxInputVoltage[4];    // para acionar os ADCs
uint32_t adc3_inp0,vbat,tempsensor,vrefint;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /* Configure the MPU attributes for the QSPI 256MB without instruction access */
  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress      = QSPI_BASE;
  MPU_InitStruct.Size             = MPU_REGION_SIZE_256MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL1;
  MPU_InitStruct.SubRegionDisable = 0x00;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Configure the MPU attributes for the QSPI 8MB (QSPI Flash Size) to Cacheable WT */
  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress      = QSPI_BASE;
  MPU_InitStruct.Size             = MPU_REGION_SIZE_8MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_PRIV_RO;
  MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;
  MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL1;
  MPU_InitStruct.SubRegionDisable = 0x00;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

void LED_Pisca(uint32_t delay)
{
	HAL_GPIO_WritePin(PE3_GPIO_Port,PE3_Pin,GPIO_PIN_SET);
	HAL_Delay(delay);
	HAL_GPIO_WritePin(PE3_GPIO_Port,PE3_Pin,GPIO_PIN_RESET);
	HAL_Delay(delay);
}


float LerEncoder(int dif_count, float Ts) {
	float RPM;
	RPM = (dif_count*60)/(Ts*(PPR/1000.0));
	return RPM;
}

float CalcularPI(controleMotor_t* ctrl, float dt) {
	float erro;
	float P, I;
	float saida;

	erro = ctrl->setpoint - ctrl->rpm_atual;
	P = ctrl->Kp * erro;
	ctrl->integral = ctrl->integral + ctrl->Ki*erro*dt;
	I = ctrl->integral;
	saida = P+I;

	if (saida > 1) {
		ctrl->integral = ctrl->integral - ctrl->Ki*erro*dt;
		saida = 1;
	} else if (saida < -1) {
		ctrl->integral = ctrl->integral - ctrl->Ki*erro*dt;
		saida = -1;
	}

	ctrl->erro_anterior = erro;
	return saida;
}


void CalcVelAng(roboDiferencial_t*robo) {
	float omegaE, omegaD;

	omegaE = (1/robo->raio_roda)*robo->v_lin - (robo->L/(robo->raio_roda*2))*robo->v_ang;

	omegaD = (1/robo->raio_roda)*robo->v_lin + (robo->L/(robo->raio_roda*2))*robo->v_ang;

	robo->motor_direito.setpoint = (omegaD*(60/(2*M_PI)))/PWM_MAX;
	robo->motor_esquerdo.setpoint = (omegaE*(60/(2*M_PI)))/PWM_MAX;
}

void ControleReferenciaVariavelPosicao(roboDiferencial_t* robo, float x_d, float y_d, float K_l, float K_theta, float v_max, float omega_max, float tol) {
	float delta_x, delta_y, delta_l, delta_theta;
	float theta_d;
	float delta_l_projetado;

	delta_x = x_d - robo->pos_x;
	delta_y = y_d - robo->pos_y;
	delta_l = sqrt(pow(delta_x, 2) + pow(delta_y, 2));

	if (delta_l < tol) {
		robo->v_ang = 0.0;
		robo->v_lin = 0.0;
		robo->motor_direito.integral = 0.0;
		robo->motor_esquerdo.integral = 0.0;
		robo->motor_direito.setpoint = 0.0;
		robo->motor_esquerdo.setpoint = 0.0;
		robo->motor_direito.rpm_atual = 0.0;
		robo->motor_esquerdo.rpm_atual = 0.0;
		return;
	}
	theta_d = atan2(delta_y, delta_x);
	delta_theta = theta_d - robo->theta;
	delta_theta = atan2(sin(delta_theta), cos(delta_theta));
	delta_l_projetado = delta_l*cos(delta_theta);
	robo->v_ang = K_theta*delta_theta;
	robo->v_lin = K_l*delta_l_projetado;

	robo->v_ang = MAX(-omega_max, MIN(omega_max, robo->v_ang));
	robo->v_lin = MAX(-v_max, MIN(v_max, robo->v_lin));
	CalcVelAng(robo);
	robo->pos_x = robo->pos_x + (robo->v_lin*cos(robo->theta)*TS_S);
	robo->pos_y = robo->pos_y + (robo->v_lin*sin(robo->theta)*TS_S);
	robo->theta = robo->theta + robo->v_ang*TS_S;
	robo->theta = atan2(sin(robo->theta), cos(robo->theta));
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  #ifdef W25Qxx
    SCB->VTOR = QSPI_BASE;
  #endif
  MPU_Config();
  CPU_CACHE_Enable();

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
  MX_SPI4_Init();
  MX_TIM1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC3_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  bno055_assignI2C(&hi2c1);
  bno055_setup();
  bno055_setOperationModeNDOF();

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // Encoder02  start Right Motor
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // Encoder01  start  Left Motor

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // PWM01 start  Left Motor
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); // PWM02 start Right Motor

  HAL_TIM_Base_Start(&htim8);

  LCD_Test();

	/* Run the ADC calibration in single-ended mode */
  if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler();
  }

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_SET);


  int difCountLeft=0, difCountRight=0;
  float RPMLeft=0.0, RPMRight=0.0;
  int oldCountLeft=0, oldCountRight=0;
  int newCountLeft=0, newCountRight=0;
  float time = 0;
  char dados[200];
  float pidL=0, pidR=0;

  controleMotor_t motor_esquerdo = {
	  .Kd = 0.0,
	  .Kp = 0.238988603135938,
	  .Ki = 47.7977206271875,
	  .derivada = 0.0,
	  .erro_anterior = 0.0,
	  .rpm_atual = 0.0,
	  .integral = 0.0,
	  .setpoint = 0.0
  };

  controleMotor_t motor_direito = {
  	  .Kd = 0.0,
  	  .Kp = 0.248745422786589,
  	  .Ki = 49.7490845573178,
  	  .derivada = 0.0,
  	  .erro_anterior = 0.0,
  	  .rpm_atual = 0.0,
  	  .integral = 0.0,
  	  .setpoint = 0.0
    };

  roboDiferencial_t robo ={
		  .L = 0.22,
		  .motor_direito = motor_direito,
		  .motor_esquerdo = motor_esquerdo,
		  .pos_x = 0.0,
		  .pos_y = 0.0,
		  .raio_roda = 0.033,
		  .rpm_max = 201,
		  .rpm_min = 201,
		  .theta = 0.0,
		  .v_ang = 0.0,
		  .v_lin = 0.0
  };
  TIM2->CNT = 0;
  TIM3->CNT = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  for(uint32_t i = 0;i<(sizeof(uhADCxConvertedValue)/sizeof(uint16_t));i++)
		{
			/*##-1- Start the conversion process #######################################*/
			if (HAL_ADC_Start(&hadc3) != HAL_OK)
			{
				/* Start Conversation Error */
				Error_Handler();
			}
			/*##-2- Wait for the end of conversion #####################################*/
			/*  For simplicity reasons, this example is just waiting till the end of the
					conversion, but application may perform other tasks while conversion
					operation is ongoing. */
			if (HAL_ADC_PollForConversion(&hadc3,HAL_MAX_DELAY) != HAL_OK)
			{
				/* End Of Conversion flag not set on time */
				Error_Handler();
			}
			else
			{
				/* ADC conversion completed */
				/*##-3- Get the converted value of regular channel  ########################*/
				uhADCxConvertedValue[i] = HAL_ADC_GetValue(&hadc3);

				/* Convert the result from 16 bit value to the voltage dimension (mV unit) */
				/* Vref = 3.3 V */
				uhADCxInputVoltage[i] = ((uhADCxConvertedValue[i] * 3300) / 0xFFFF);
			}
		}
		HAL_ADC_Stop(&hadc3);

		#define V30  (620)  // mV, V30: 0.62V,datasheet P278
		#define Avg_Slope (2) // mV/  C

		adc3_inp0  = uhADCxInputVoltage[0]; // mv
		vrefint    = uhADCxInputVoltage[1]; // type. 1200mV
		tempsensor = ((int32_t)uhADCxInputVoltage[2] - V30)/Avg_Slope + 30; //   C
		vbat       = uhADCxInputVoltage[3] * 4;

		//zerar os contadores

		//calcular o deslocamento
		ControleReferenciaVariavelPosicao(&robo, 0.3, 0.0, 1.5, 1.5, 0.2, 0.2, 0.015);

		newCountLeft = __HAL_TIM_GET_COUNTER(&htim2);
		newCountRight = __HAL_TIM_GET_COUNTER(&htim3);

		oldCountLeft = newCountLeft;
		oldCountRight = newCountRight;

		HAL_Delay(TS_MS);

		newCountLeft = __HAL_TIM_GET_COUNTER(&htim2);
		newCountRight = __HAL_TIM_GET_COUNTER(&htim3);

		time = time+TS_MS;
		difCountLeft = (int)(newCountLeft - oldCountLeft);
		difCountRight = (int)(newCountRight - oldCountRight);

		RPMLeft = LerEncoder(difCountLeft, TS_MS);
		RPMRight = LerEncoder(difCountRight, TS_MS);

		motor_direito.rpm_atual = RPMRight/PWM_MAX;
		motor_esquerdo.rpm_atual = RPMLeft/PWM_MAX;

		pidR = CalcularPI(&robo.motor_direito, TS_S);
		pidL = CalcularPI(&robo.motor_esquerdo, TS_S);

		if (pidL > 0) {
			TIM2->CNT = 0;
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8,GPIO_PIN_RESET);  // LEFT MOTOR
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9,GPIO_PIN_SET);
		} else {
			TIM2->CNT = 0-1;
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8,GPIO_PIN_SET);  // LEFT MOTOR
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9,GPIO_PIN_RESET);
		}
		pidL = abs(pidL);


		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,  pidL*PWM_MAX);
		if (pidR > 0) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10,GPIO_PIN_RESET); // RIGHT MOTOR
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11,GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10,GPIO_PIN_SET); // RIGHT MOTOR
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11,GPIO_PIN_RESET);
		}
		pidR = abs(pidR);

		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,  pidR*PWM_MAX);

		sprintf(dados, "RPM L: %.2f RPM R: %.2f pid L %.5f, pid R: %.5f\r\n", RPMLeft, RPMRight, pidL, pidR);
		CDC_Transmit_FS((uint8_t *)dados, strlen(dados));
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
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
  while(1)
		LED_Pisca(500);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
