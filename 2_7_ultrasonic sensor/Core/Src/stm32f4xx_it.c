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
extern uint32_t ADC1_value[2];
extern uint32_t ADC1_0;
extern uint32_t ADC1_1;

int tim6_flag=0;
extern uint8_t key_value_flag;
extern uint32_t encoder_count;

float V;
float V1_mV;
float V1_V;
float amp_A;
float amp_mA;
double sum=0;
float i_avg_mA=0;
float i_avg_A=0;
uint8_t i=0;




 uint32_t ENCODER_NEW_tim2, ENCODER_OLD_tim2;
 float RPM_tim2, RPS_tim2, Omega_tim2;

 uint32_t ENCODER_NEW, ENCODER_OLD;
extern float RPM;
extern float RPS;
//extern uint16_t ADC1_value[1];
extern uint16_t current;

extern float Omega;
#define pi 3.141592


extern float i_input;
extern float i_sensor;
extern float i_error;
extern float i_error_sum;
extern float i_kp;
extern float i_ki;
extern float i_ka;
extern float i_vs;
extern float i_v_ref;

extern float I_KP;
extern float I_KI;



extern float speed_input;
extern float speed_sensor;
extern float speed_error;
extern float speed_error_sum;
extern float speed_kp ;  //a*Js*Wcw/Kt
extern float speed_ki ;   //Js*Wcw*Wcw/(5*Kt)
extern float speed_ka ;  //1/(Js*Wcw/Kt)
extern float speed_is;
extern float speed_i_ref;
extern float speed_kps ;  //(1-a)*Js*Wcw/Kt

extern float S_KP;
extern float S_KI;

extern float x;
extern float duty;


extern uint8_t key_flag;
////////////////////////////////////////////////
/*       position test    */
float error;
float error1=0;
float set_position;
float position_sensor;
float i_err;
float d_err;
float kp = 2;
float ki = 0.01;
float kd = 0.1;
float kp_term;
float ki_term;
float kd_term;
float PID;
float b_err;
uint8_t time_1s=0;

uint8_t stop_flag=0;


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
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_tim4_up;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart3;


/* USER CODE BEGIN EV */

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
   while (1)
  {
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
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim4_up);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
  * @brief This function handles ADC1, ADC2 and ADC3 global interrupts.
  */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC_IRQn 1 */


  ADC1_0= ADC1_value[0];

  V=ADC1_0 * 0.6547877;  //3227 > 2113(ref)= 2.58V
  V1_mV = V *5000 / 4095.0; //[mV]
  V1_V = V *5 / 4095.0; //[V]


  if(V1_V - 2.58>=0)
  {
	  amp_A = (V1_V -2.58) /285*1000;
  }
  else if(V1_V - 2.58 < 0)
  {
	  amp_A = -(V1_V - 2.58) /285*1000;
  }


  amp_mA = amp_A * 1000;

  sum=amp_mA + sum;
 	  i++;
 	 	 if(i==100000)
 	 	  {
 	 		i_avg_mA=sum/100001;
 	 		i_avg_A = i_avg_mA / 1000;
 	 			  i=0;
 	 			  sum=0;
 	 	  }


  /* USER CODE END ADC_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */


  /* USER CODE END TIM3_IRQn 1 */
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
  * @brief This function handles TIM8 update interrupt and TIM13 global interrupt.
  */
void TIM8_UP_TIM13_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 0 */

  /* USER CODE END TIM8_UP_TIM13_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  HAL_TIM_IRQHandler(&htim13);
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 1 */


  //GPIOB->ODR ^=1<<0;  //ok


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

  time_1s++;

//motor1
  ENCODER_OLD= ENCODER_NEW;
  ENCODER_NEW = TIM4->CNT;


   	RPM =  ((abs(ENCODER_NEW-ENCODER_OLD)*60))/512.0;	// ?��코더 ?��?��: 1 turn?�� 3 pulse 출력
    	RPS=RPM/60;
    	Omega= RPS*2*pi;
    //	printf("RPS = %d \r\n",RPS);
   // 	 printf("RPM = %d \r\n",RPM);

/*
    	if(( GPIOE->ODR & 1<<0) == 0)	// if  back?
    	{
    		RPM = -RPM;
			RPS = -RPS;
			Omega = -Omega;
    	}
    	else
    	{
    		RPM = RPM;
         	RPS = RPS;
    		Omega = Omega;
    	}
*/








    	GPIOB->ODR ^= 1<<7;
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

  tim6_flag=1;










  encoder_count=TIM4->CNT;



	if(key_flag==1)
	  {
		  GPIOE->ODR |= 1<<0;  // forward

	      GPIOG->ODR &= ~1<<0;  //start

		  TIM3->CCR1 = 250;

		    if(TIM4->CNT>=10220)
			{
		    	TIM3->CCR1 = 100;

		 		if(TIM4->CNT>=10512)
		  		{
			         GPIOG->ODR |= 1<<0;  //stop
			        //  key_flag=0;
			          stop_flag=1;
			          key_flag=0;
			          HAL_Delay(2000);
		  		}

		  	}


	  }


	  if(stop_flag==1)
	      	    {
	      	          GPIOE->ODR &= ~1<<0;  //back
	      	          GPIOG->ODR &= ~1<<0;  //start
	      	          TIM3->CCR1 = 250;

	      	            if(encoder_count<=10280)
	      	            {
	      	            	  TIM3->CCR1 = 87;
	      	             	  if(encoder_count<=10000)
	      	    	      	    {
	      	    	  		 		  GPIOG->ODR |= 1<<0;  //stop
	      	    	          		  GPIOE->ODR |= 1<<0;  //go
	      	    	  		         stop_flag=0;

	      	    	  		    }
	      	           }

	      	    }





////////////////////////////////////////////////////////////////////////


/*
  position_sensor=RPS*9.5*pi*time_1s;

  error=set_position-position_sensor;

 //   i_err = i_err + error;
  //     d_err=(error-error1);  //d ?��차값



       kp_term=kp*error;          //kp_term=kp*err;

  //     ki_term=ki*i_err;         //i_err+=err*dt;        //ki_term=ki*i_err;

    //   kd_term=kd*d_err;            //d_err=(err-duty)/dt;     //kd_term=kd*d_err;

       PID=kp_term ;
    		   //+ ki_term + kd_term;       //조작?��=Kp×?���????+Ki×?��차의 ?��?���????+Kd×?��?�� ?��차�??�� �????

      error1=error;

      if(PID>=15)
      {
    	  PID=15;
      }
      else if(PID<=-15)
      {
          	  PID=-15;
      }

      if(PID<=0)
	{
    	  GPIOE->ODR &= ~1<<0; // back
    	  TIM3->CCR1 = 1000*PID/100;
	}
      else
      {
    	  GPIOE->ODR |= 1<<0;   // forward
    	   TIM3->CCR1 = 1000*PID/100;
      }

*/









  //position= encoder_count*0.0001/512;
  //  RPS*9.5*pi;//1초에 ?��?��거리


 // tim6_flag=1;
/*
 	   speed_sensor=Omega;
       i_sensor=i_avg_A;


	 speed_error = (speed_input - speed_sensor);

	 S_KP = speed_error*speed_kp;

	  speed_error_sum = speed_error_sum + speed_error*0.0001;
	//    S_KI = (speed_error_sum - (speed_ka*(speed_is-speed_i_ref))) * speed_ki;

	  if(duty==0) speed_error_sum=0;
	  S_KI = speed_error_sum  * speed_ki;

	  speed_is = S_KP + S_KI -speed_sensor*speed_kps;


	  if(speed_is>=0.4)
	  {
	  	speed_i_ref=0.4;
	  }
	  else if(speed_is<=-0.4)
	  {
	  	speed_i_ref=-0.4;
	  }


	  i_input= speed_i_ref;

	  i_error =( i_input - i_sensor);

	  I_KP = i_error*i_kp;

	  i_error_sum = i_error_sum + i_error*0.0001;
	//   I_KI = (i_error_sum - (i_ka*(i_vs-i_v_ref))) * i_ki;

	  if(duty==0) i_error_sum=0;
	  I_KI = i_error_sum  * i_ki;


	  i_vs = I_KP + I_KI;

	  if(i_vs>=24)
	  {
	  	i_v_ref=24;
	  }
	  else if(i_vs<=-24)
	  {
	  	i_v_ref=-24;
	  }


	//  x=i_v_ref+ 4.534*Omega;  //[V]

	  x=i_v_ref;

//	  y = 2E-05x6 - 0.0008x5 + 0.0075x4 + 0.105x3 - 2.3402x2 + 13.285x - 12.008

	  if(x>=0)
	  {
		  x=x;
	  }
	  else
	  {
		  x=-x;
	  }



	  if(x<=20)
	  {
		  duty = 0.0018*pow(x,3) - 0.0191*pow(x,2) + 0.4468*x + 4.2488;

	  }
	  else if(x>20)
	  {
        duty = -3.6513*pow(x,4) + 327.08*pow(x,3) - 10973*pow(x,2) + 163412*x - 911605;

	   }
	//  duty= 2/100000*pow(x,6) - 0.0008*pow(x,5) + 0.0075*pow(x,4)+ 0.105*pow(x,3) - 2.3402*pow(x,2) + 13.285*pow(x,1) - 12.008;
	  	  //[%]

	 if(i_v_ref>=0)
	 {
		 duty=duty;
		 GPIOG->ODR &= ~1<<1;
	 }
	 else if(i_v_ref<0)
	 {
		 duty=duty;

		 GPIOG->ODR |= 1<<1;
	 }

*/





  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /*if(stop_flag==1)
  {      //  HAL_Delay(500);
  	  		          GPIOE->ODR &= ~1<<0;  //back
  	  		          GPIOG->ODR &= ~1<<0;  //start
  	  		          TIM3->CCR1 = 250;

  	  		          if(TIM4->CNT<=250)
  	  		          {
  	  		        	  TIM3->CCR1 = 90;
  	  		        	  if(TIM4->CNT<=2)
  	  		        	    {
  	  		        		  GPIOG->ODR |= 1<<0;  //stop
  	  		        		   key_flag=0;
  	  		        		 stop_flag=0;
  	  		        	    }
  	  		          }

  }*/

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
