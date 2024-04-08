/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "com.h"
#include "step.h"
#include "math.h"
#include "pid.h"

extern int position_start;   //pid 스타트

///////////////////////////////ADC

int k;
float battery_V;
float battery_V_avg;
float battery_V_sum;

extern uint16_t ADC3_value[3];
uint16_t ADC3_IN12;
uint16_t ADC3_IN13;
uint16_t ADC3_IN14;
///////////////////////////////////타이머 플래그
int tim7_flag = 0;
int tim6_flag = 0;
int tim14_flag = 0;
int tim13_flag = 0;
int tim12_flag = 0;
int tim12_test = 0;
int tim11_flag = 0;

//////////////////////////////////  dc모터 엔코더 , 속도측정

uint32_t ENCODER_NEW_X, ENCODER_OLD_X;
float RPM_X;
float RPS_X;
float Omega_X;

//////////////////////전류센서

float sensitivity = 0.255;

int rawVoltage_count = 0;
float rawVoltage_avg = 0;
float rawVoltage_sum = 0;

uint16_t readValue;

float rawVoltage;
float current;
float current_mA;
float current_A;
float current_A_floor;

////////////////////////////////    스텝모터
uint32_t step_pulse_count_tim13 = 0;
uint16_t step_pulse_count_tim11 = 0;

extern uint16_t shaft_step;
extern uint16_t shaft_pulse_cycle;

extern uint16_t ball_screw_step;
extern uint16_t ball_screw_pulse_cycle;

//////////////////////////////////

//extern double p_encoder;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc3;
extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {

	if (hadc->Instance == hadc3.Instance)

	{

		//GPIOB->ODR ^= 1 << 0;  //ok

		ADC3_IN14 = ADC3_value[2];

		battery_V = 16.059 * ADC3_IN14 / 67.8 * 27.667 + 2.2

		;

		battery_V_sum = battery_V + battery_V_sum;

		k++;
		if (k == 2000) {
			battery_V_avg = battery_V_sum / 2001;

			k = 0;
			battery_V_sum = 0;
		}

		ADC3_IN12 = ADC3_value[0];

		/*
		 readValue = ADC3_IN13;
		 rawVoltage = (float) readValue * 3.3 * 2 / 4095;
		 // If rawVoltage is not 2.5Volt, multiply by a factor.In my case it is 1.035
		 // This is due to tolerance in voltage divider resister & ADC accuracy
		 current =(rawVoltage - 2.5)/sensitivity;
		 */

		readValue = ADC3_value[1];
		//  readValue=readValue*0.6870; //
		rawVoltage_sum = rawVoltage_sum + readValue;
		rawVoltage_count++;
		if (rawVoltage_count == 200) {
			rawVoltage_avg = rawVoltage_sum / (200);
			rawVoltage_sum = 0;
			rawVoltage_count = 0;
		}

		//rawVoltage_avg=rawVoltage_avg*0.6887;
		current_A = (rawVoltage_avg - (3256)) * 5 / 4095 / 0.253;

		current_A_floor = floor(current_A * 100) / 100; // 소수점 둘재짜리까지 표시
		current_mA = current_A * 1000;

	}

}

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
	while (1) {
	}
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles ADC1, ADC2 and ADC3 global interrupts.
  */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc3);
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
  * @brief This function handles TIM1 trigger and commutation interrupts and TIM11 global interrupt.
  */
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 0 */

  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim11);
  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 1 */
	step_pulse_count_tim11++;

	GPIOB->ODR ^= 1 << 0;

//	tim11_flag = 1;
	z_axis_step_motor(ball_screw_step, ball_screw_pulse_cycle); //(스텝,주기) 펄스:200당 한바퀴

  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles TIM8 break interrupt and TIM12 global interrupt.
  */
void TIM8_BRK_TIM12_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 0 */

  /* USER CODE END TIM8_BRK_TIM12_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  HAL_TIM_IRQHandler(&htim12);
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 1 */

//	tim12_flag = 1;
	//tim12_test++;
  /* USER CODE END TIM8_BRK_TIM12_IRQn 1 */
}

/**
  * @brief This function handles TIM8 update interrupt and TIM13 global interrupt.
  */
void TIM8_UP_TIM13_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 0 */

  /* USER CODE END TIM8_UP_TIM13_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  HAL_TIM_IRQHandler(&htim13);
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 1 */

	if ((TIM13->SR & 0x01) != RESET)	// CC1 interrupt flag
			{
		TIM13->SR &= ~0x01;	// CC1 Interrupt Claer

		GPIOB->ODR ^= 1 << 14;

		step_pulse_count_tim13++;

		shaft_step_motor(shaft_step, shaft_pulse_cycle); //(스텝,주기) 펄스:200당 한바퀴

	}

  /* USER CODE END TIM8_UP_TIM13_IRQn 1 */
}

/**
  * @brief This function handles TIM8 trigger and commutation interrupts and TIM14 global interrupt.
  */
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 0 */

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 1 */
	// GPIOB->ODR ^= 1 << 7;
	tim14_flag++;

//motor1
	ENCODER_OLD_X = ENCODER_NEW_X;
	ENCODER_NEW_X = TIM2->CNT;

	RPM_X = (abs(ENCODER_NEW_X - ENCODER_OLD_X) * 60) / 0.01 / 34 / 512; // ?��코더 ?��?��: 1 turn?�� 3 pulse 출력
	RPS_X = RPM_X / 60;
	Omega_X = RPS_X * 2 * M_PI;

	//	printf("RPS = %d \r\n",RPS);
	// 	 printf("RPM = %d \r\n",RPM);

	if (( GPIOE->ODR & 1 << 0) == 0)	// if  back?
			{
		RPM_X = RPM_X;
		RPS_X = RPS_X;
		Omega_X = Omega_X;

		current_A = current_A;
	} else {
		RPM_X = -RPM_X;
		RPS_X = -RPS_X;
		Omega_X = -Omega_X;

		current_A = -current_A;
	}

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

	tim6_flag = 1;

	if (position_start == 1) {
		position_pid_x();

		position_pid_y();
	}

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc3);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
