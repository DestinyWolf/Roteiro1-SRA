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
	float Kp, Ki, Kd;
	float erro_anterior, integral, derivada;
	float setpoint, medida;
	float saida;
} controleMotor_t;

typedef struct RoboDiferencial {
	controleMotor_t motor_esq, motor_dir;
	float L, raio_roda;
	float rpm_max, rpm_min;
	float velocidade_linear, velocidade_angular;
} roboDiferencial_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//valor da velocidade linear
float vel_linear = 0.1;

// Sinal de controle, ciclo de duração PWM
int CH1_PWM_duty = 0; // Inicializa ciclo de duração PWM
int CH2_PWM_duty = 0; // Inicializa ciclo de duração PWM
int CH1_PWM = GPIO_PIN_12;
int CH2_PWM = GPIO_PIN_13;
// Encoder references
uint32_t oldPosLeft  = 0;
uint32_t oldPosRight  = 0;
uint32_t newPosLeft, newPosRight;
uint32_t newPos, oldPos, wheelSpeed;
int32_t dif_countLeft = 0;
int32_t dif_countRight = 0;


float Ts = 1.0;  // Ts = 1ms
float Time = 0; // increment in ms

float wheelMEASLeft, wheelMEASRight;



roboDiferencial_t robo = {
		.L = 0.22,
		.raio_roda = 0.033,
		.rpm_max = 209,
		.rpm_min = -209,
		.velocidade_angular = 0.0,
		.velocidade_linear = 0.0
};
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

float calcular_PID(controleMotor_t *ctrl, float dt) {

	uint8_t text[200];
	/*ctrl é o motor a ser controlado, dt é o tempo de amostragem*/
	float erro, P, I, D;

	float saida_bruta;
	float derivada;

	erro = ctrl->setpoint - ctrl->medida;
	P = ctrl->Kp * erro;
	ctrl->integral = ctrl->integral + ctrl->Ki*(erro*dt);
	I =  ctrl->integral;

	derivada =  (erro - ctrl->erro_anterior)/dt;
	D = ctrl->Kd * derivada;

	saida_bruta = P + I + D;

	//sprintf((char *)&text, "pid: %.5f erro: %.5f\r\n", saida_bruta, erro);
    //CDC_Transmit_FS((uint8_t *)text, strlen(text));


	/*saturação*/
	if (saida_bruta > 1) {
		ctrl->integral = ctrl->integral - erro*dt;
		saida_bruta = 1;
	} else if (saida_bruta < -1) {
		ctrl->integral = ctrl->integral - erro*dt;
		saida_bruta = -1;
	}

	ctrl->erro_anterior = erro;

	return saida_bruta; // saida referente a uma porcentagem da ativação do pwm, varia ve -1 a 1

}


// Calcula a velocidade angular
float OmegaSpeedLeft(float dif_countLeft, float Ts){
   float wheelSpeedLeft = (float)((dif_countLeft/Ts)*(60.0/15.336));  // Wheel Side in RPM ****** where 15336 PPR / 1000ms
   return wheelSpeedLeft;
}


float OmegaSpeedRight(float dif_countRight, float Ts){
   float wheelSpeedRight = (float)((dif_countRight/Ts)*(60.0/15.336));  // Wheel Side in RPM ****** where 15336 PPR / 1000ms
   return wheelSpeedRight;
}

// Calcula a velocidade angular
float Linear2RPM(float V_lin, roboDiferencial_t robo){
   return V_lin * 60 / (robo.raio_roda * 2 * M_PI);
}

// Delay us
//void delay_us(uint16_t us)
//{
//  __HAL_TIM_SET_COUNTER(&htim8,0); // set the counter value 0
//  while (__HAL_TIM_GET_COUNTER(&htim8) < us);  // wait for the counter to reach the us input in the parameter
//}
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

  // ============================================
  // Definição dos parametros do pid
  // ============================================
  //esse aqui esta pronto
  robo.motor_dir.Kd = 0.0;
  robo.motor_dir.Kp = 43.3526254527816;
  robo.motor_dir.Ki = 1817.9632631973;

  robo.motor_esq.Kd = 0.0;
  robo.motor_esq.Kp = 43.3526254527816;
  robo.motor_esq.Ki = 1817.9632631973;

  // =================================
  // Definição dos setpoints iniciais
  // =================================

  robo.motor_dir.setpoint = 0.0;
  robo.motor_esq.setpoint = 0.0;

  // ===================================
  // 	Definição do estado inicial
  // ===================================

  robo.motor_esq.erro_anterior = 0.0;
  robo.motor_esq.integral = 0.0;
  robo.motor_esq.medida = 0.0;

  robo.motor_dir.erro_anterior = 0.0;
  robo.motor_dir.integral = 0.0;
  robo.motor_dir.medida = 0.0;


	/* Run the ADC calibration in single-ended mode */
  if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler();
  }

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_SET);

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
		robo.motor_dir.setpoint = ((float)Linear2RPM(vel_linear,robo))/10000.0;
		robo.motor_esq.setpoint = ((float)Linear2RPM(vel_linear,robo))/10000.0;

		if (Time < 2000)
			vel_linear = vel_linear;
		else
			vel_linear = 0;
		uint8_t text[200];
		// 1. Leitura dos Encoders
		        newPosLeft =  __HAL_TIM_GET_COUNTER(&htim3);
		        newPosRight = __HAL_TIM_GET_COUNTER(&htim2);

		        // 2. Calcula a diferença dos contadores e resolve o overflow CORRETAMENTE
		        // TIM3 (Left) é 16-bits, então obrigatório o cast para int16_t
		        int16_t deltaLeft = (int16_t)(newPosLeft - oldPosLeft);
		        // TIM2 (Right) é 32-bits, então obrigatório o cast para int32_t
		        int32_t deltaRight = (int32_t)(newPosRight - oldPosRight);

		        oldPosLeft = newPosLeft;
		        oldPosRight = newPosRight;

		        // 3. Tempo de amostragem
		        HAL_Delay(Ts);
		        Time = Time + Ts;

		        // 4. Calcula Velocidades Reais (Pode desfazer a inversão, agora vai bater certo!)
		        wheelMEASLeft = (float)OmegaSpeedLeft((float)deltaLeft, Ts);
		        wheelMEASRight = (float)OmegaSpeedRight((float)deltaRight, Ts);

		        // 5. Alimenta a estrutura do robô
		        robo.motor_esq.medida =  wheelMEASRight / 10000.0;
		        robo.motor_dir.medida =  wheelMEASLeft / 10000.0;

		        // 6. Calcula o PID (Passando Ts em SEGUNDOS)
		        float pid1 = calcular_PID(&robo.motor_esq, Ts / 1000.0);
		        float pid2 = calcular_PID(&robo.motor_dir, Ts / 1000.0);

		        // 7. Calcula Duty Cycle Linearizado (Como no Simulink)
		        CH1_PWM_duty = (int)(abs(pid1) * 10000.0);
		        CH2_PWM_duty = (int)(abs(pid2) * 10000.0);

		        // 8. Controle de Direção - MOTOR ESQUERDO
		        if (pid1 < 0) {
		            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);  // FRENTE
		            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);
		        } else {
		            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET); // RÉ (CORRIGIDO)
		            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
		        }
		        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, CH1_PWM_duty);

		        // 9. Controle de Direção - MOTOR DIREITO
		        if (pid2 < 0) {
		            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);  // FRENTE
		            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
		        } else {
		            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET); // RÉ (CORRIGIDO)
		            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
		        }
		        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, CH2_PWM_duty);


		        // Print para debug
		        sprintf((char *)&text, "WrL: %5.2f WrR: %5.2f | med D: %f med E: %f pid1 %.5f pid2 %.5f\r\n",
		        		wheelMEASLeft, wheelMEASRight, robo.motor_dir.medida, robo.motor_esq.medida, pid1, pid2);
		        CDC_Transmit_FS((uint8_t *)text, strlen(text));
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
