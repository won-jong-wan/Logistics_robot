/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "string.h"
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

 // USER CODE BEGIN 4 ,USER CODE BEGIN PFP  +while?�� printf 추�? ?��
void SerialSendChar_PC(uint8_t c1);
void SerialSendChar_WIFI(uint8_t c2);





uint8_t buffer[256];


uint8_t key_value;
UART_HandleTypeDef huart3;
uint8_t data;
UART_HandleTypeDef huart4;

uint32_t encoder_count=0;
char uart_buf[30];

int8_t RX_flag=0;

uint32_t ENCODER_NEW, ENCODER_OLD;
float RPM;
float RPS;
uint16_t RPM_uint16;

uint32_t ADC1_value[2];
uint32_t ADC1_0;
uint32_t ADC1_1;

uint8_t ENC[4];




#define pi 3.141592




//       @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@  PID


float Ra =12.1;
float Kt = 4.534;
float Ke = 4.534;
float B = 0.09;
float Js = 1.84;
float La = 0.0003342;
//float Wci = 804.2;
float Wci = 3142;
//float Wcw = 80.42;
float Wcw = 314;
float a = 0.9;

//I control
float i_input;
float i_sensor;
float i_error;
float i_error_sum=0;
float i_kp = 0.0003342 * 3142 ;
float i_ki = 12.1 * 3142;
float i_ka = 1/(0.0003342 * 3142);
float i_vs=0;
float i_v_ref=0;

float I_KP;
float I_KI;

//Speed control
float speed_input;
float speed_sensor;
float speed_error;
float speed_error_sum = 0.0;
//float speed_kp = 0.9* 1.84*314/4.534;  //a*Js*Wcw/Kt =114.6
//float speed_ki =1.84*314*314/(5*4.534);   //Js*Wcw*Wcw/(5*Kt) = 8002
float speed_kp=114.6;
float speed_ki=5;

float speed_ka = 1/(1.84*314/4.534);  //1/(Js*Wcw/Kt) =
float speed_is=0;
float speed_i_ref=0;
float speed_kps = (1-0.9)*1.84*314/4.534 ;  //(1-a)*Js*Wcw/Kt =12.7

float S_KP;
float S_KI;
float Omega;

float x=0;
double duty=0;


uint8_t key_flag;


extern uint8_t time_1s;
extern float set_position;

extern uint8_t stop_flag;
extern int tim14_flag;
extern int tim6_flag;
extern int tim13_flag;
  int count=0;
float f = 1.234;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void SerialSendChar_PC(uint8_t Ch1) // 1문자 보내�?????????????????????????????? ?��?��
{
	// USART_SR_TXE(1<<7)=0?, TX Buffer NOT Empty?
	// TX buffer Empty?���?????????????????????????????? ?��?���?????????????????????????????? 계속 ??�??????????????????????????????(?��?�� �???????????????????????????????��?�� ?��?��까�? ??�??????????????????????????????)
        while((USART3->SR & 1<<7) == RESET);
	USART3->DR = (Ch1 & 0x01FF);	// ?��?�� (최�? 9bit ?���??????????????????????????????�?????????????????????????????? 0x01FF�?????????????????????????????? masking)
}

void SerialSendChar_WIFI(uint8_t Ch2) // 1문자 보내�?????????????????????????????? ?��?��
{
	// USART_SR_TXE(1<<7)=0?, TX Buffer NOT Empty?
	// TX buffer Empty?���?????????????????????????????? ?��?���?????????????????????????????? 계속 ??�??????????????????????????????(?��?�� �???????????????????????????????��?�� ?��?��까�? ??�??????????????????????????????)
        while((UART4->SR & 1<<7) == RESET);
	UART4->DR = (Ch2 & 0x01FF);	// ?��?�� (최�? 9bit ?���??????????????????????????????�?????????????????????????????? 0x01FF�?????????????????????????????? masking)
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	encoder_count=TIM4->CNT;

  if(huart->Instance == USART3)
  {

	  HAL_UART_Receive_IT(&huart3, &key_value, 1);
	  printf("%d,^^ %c \r\n", key_value,key_value);

	  if(key_value=='q')  // forward
	  	 	  {
	  	 		  GPIOE->ODR |= 1<<0;  // forward
	  	 		  GPIOB->ODR |= 1<<0;  //LD1

	  	 	  }
	  else if(key_value=='w')  // back
		  	 	  {
		  	 		  GPIOE->ODR &= ~1<<0;  // back
		  	 		GPIOB->ODR &= ~1<<0;  //LD1
		  	 	  }

	  else if(key_value=='1')
	 	  {
	 		  GPIOG->ODR |= 1<<0;  //stop

	 	  }

	  else  if(key_value=='2')
	  {
		  GPIOG->ODR &= ~1<<0;  //start

	  }
	  else if(key_value=='3')
	  {

			TIM3->CCR1 = 50;
			TIM3->CCR2 = 50;
	  }
	  else if(key_value=='4')
	  	  {

	  			TIM3->CCR1 = 100;
	  			TIM3->CCR2 = 100;
	  	  }
	  else if(key_value=='5')
	  	  	  {

	  	  			TIM3->CCR1 = 500;
	  	  		TIM3->CCR2 = 500;
	  	  	  }
	  else if(key_value=='6')
	  	  	  {

	  	  			TIM3->CCR1 = 800;
	  	  		TIM3->CCR2 = 800;
	  	  	  }
	  else if(key_value=='7')
	  	  	  {

	  	  			TIM3->CCR1 = 1000;
	  	  		TIM3->CCR2 = 1000;
	  	  	  }
	  else if(key_value=='a')
	  {
		  TIM4->CNT=10000;

		  key_flag=1;
	  }
	  else if(key_value=='b')
	 {

		 speed_input=(2*pi/60)*20;
     }
	  else if(key_value=='z')
	  	 	  {



	  	  	  	 speed_input=10*2*pi/60;

	//  	  	  	x=speed_input;
	//  	  	    duty = 2.0881*pow(x,6) - 36.163*pow(x,5) + 248.4*pow(x,4) - 851.05*pow(x,3) + 1486.5*pow(x,2) - 1164.9*x + 249.29;
	  	  //[%]

	  	//  	TIM3->CCR1 = 1000*duty/100;
	  	//  		TIM3->CCR2 = 1000*duty/100;
	  	  	  	  }
	  else if(key_value=='x')
	  	  	  	  {
	  	  	  	GPIOB->ODR &= ~1<<0;


	  	  	   speed_input=45*2*pi/60;

  	// 	   x=speed_input;
	 //		  duty = 2.0881*pow(x,6) - 36.163*pow(x,5) + 248.4*pow(x,4) - 851.05*pow(x,3) + 1486.5*pow(x,2) - 1164.9*x + 249.29;
	  	  		 	  //[%]
	  	//	  TIM3->CCR1 = 1000*duty/100;
	  	//		  	TIM3->CCR2 = 1000*duty/100;

	  	  	  	  }
	  else if(key_value=='c')
 	  	  	  {
	  	  	  	GPIOB->ODR &= ~1<<0;

	//  	  	HAL_TIM_Base_Stop_IT(&htim6);
	 	  	  	  	  }





  }
  if(huart->Instance == UART4)
    {
	  HAL_UART_Receive_IT(&huart4, &data, 1);


		switch(data)
		{
		case '0':
			 GPIOB->ODR |= 1<<0;
			GPIOG->ODR &= ~1<<0;// start

			break;

		case '1':  //stop
			  GPIOB->ODR &= ~1<<0;
	          GPIOG->ODR |= 1<<0;
			break;

		case '2':  //dir
			  GPIOE->ODR |= 1<<0;  // forward
			break;

		case '3'://dir
			 GPIOE->ODR &= ~1<<0;  //back
			break;

		case '4':

		    TIM3->CCR1 = 0;
			TIM3->CCR2 = 0;

			break;

		case '5':
			TIM3->CCR1 = 50;
			TIM3->CCR2 = 50;
			break;

		case '6':
			TIM3->CCR1 = 200;
			TIM3->CCR2 = 200;
			break;

		case '7':
			TIM3->CCR1 = 1000;
			TIM3->CCR2 = 1000;
			break;

		case '!':
			RX_flag=1;
			 TIM2->CNT=0;
			 TIM4->CNT=0;
			break;
		case 'A':  //Tx

			RX_flag=2;
				break;
		}



    }

}


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
int _write(int file, char* p, int len){
	for(int i=0; i<len; i++){
		ITM_SendChar((*p++));
	}
	return len;
}


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ETH_TxPacketConfig TxConfig;
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;
DMA_HandleTypeDef hdma_tim4_up;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
#define TRIG_PIN GPIO_PIN_7
#define TRIG_PORT GPIOD
#define ECHO_PIN GPIO_PIN_6
#define ECHO_PORT GPIOD
uint32_t pMillis;
uint32_t Value1 = 0;
uint32_t Value2 = 0;
float Distance  = 0;  // cm
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__           //추�?****************************
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE              //추�?****************************
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART3 and Loop until the end of transmission */
  if(ch=='\n')
	  HAL_UART_Transmit(&huart3, (uint8_t *)"\r", 1, 0xFFFF);
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//HAL_UART_Receive_IT(&huart3, &key_value, 1);

void delay_us(uint16_t time) {
	__HAL_TIM_SET_COUNTER(&htim7, 0);              // ???��머�?? 0?���???? 초기?��
	while((__HAL_TIM_GET_COUNTER(&htim7))<time);   // ?��?��?�� ?��간까�???? ??�????
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
  MX_DMA_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM13_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */


  HAL_UART_Receive_IT(&huart3, &key_value, 1);     //추�?****************************
  HAL_UART_Receive_IT(&huart4, &data, 1);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1_value, 2);




   sprintf((char *)buffer, "Hello, World!\n");
   HAL_UART_Transmit(&huart4, buffer, strlen((char *)buffer), 100);


   HAL_TIM_Base_Start_IT(&htim6);
   HAL_TIM_Base_Start_IT(&htim7);

   HAL_TIM_Base_Start(&htim1);
     HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low


//  HAL_ADC_Start_DMA(&hadc1,&ADC1_value[1], 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(tim6_flag==1)
	  {

	  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	    	      __HAL_TIM_SET_COUNTER(&htim1, 0);
	    	      while (__HAL_TIM_GET_COUNTER (&htim1) < 10);  // wait for 10 us
	    	      HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	    	      pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
	    	      // wait for the echo pin to go high
	    	      while (!(HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 10 >  HAL_GetTick());
	    	      Value1 = __HAL_TIM_GET_COUNTER (&htim1);

	    	      pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
	    	      // wait for the echo pin to go low
	    	      while ((HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 50 > HAL_GetTick());
	    	      Value2 = __HAL_TIM_GET_COUNTER (&htim1);

	    	      Distance = (Value2-Value1)* 0.017/2;
	    	 //     printf("Distance = %f \r\n",Distance);
	    	//      sprintf((char *)buffer, "%d \r\n",Distance);
	    	 //     HAL_UART_Transmit(&huart3, buffer, strlen((char *)buffer), 100);
	    	      tim6_flag=0;
	  }
	  if(tim14_flag==200) //2s
	  {
		    RPM_uint16=RPM*100;
		    ENC[3]=RPM_uint16 / 1000;		// 10000 ?���??????????????????????????????
    	    ENC[2]=RPM_uint16 % 1000/100;	// 1000 ?���??????????????????????????????
			ENC[1]= RPM_uint16 % 100/10;	// 100 ?���??????????????????????????????
		 	ENC[0]=RPM_uint16 % 10/1;		// 10 ?���??????????????????????????????

		   	SerialSendChar_WIFI('?');

			SerialSendChar_WIFI('0');
			SerialSendChar_WIFI('0');
			SerialSendChar_WIFI('0');
	     	SerialSendChar_WIFI('0');

		    SerialSendChar_WIFI(ENC[3]+0x30);
		    SerialSendChar_WIFI(ENC[2]+0x30);
		 	SerialSendChar_WIFI(ENC[1]+0x30);
	        SerialSendChar_WIFI(ENC[0]+0x30);

	  		  sprintf((char *)buffer, "%d \r\n",RPM_uint16);
  		  		 	  		  	  HAL_UART_Transmit(&huart3, buffer, strlen((char *)buffer), 100);



		  	SerialSendChar_PC('\n');
		 	SerialSendChar_PC('\r');

		 //	  sprintf((char *)buffer, "%d \r\n",ADC1_1);
		 	//  	  HAL_UART_Transmit(&huart4, buffer, strlen((char *)buffer), 100);


		 	//  	  HAL_Delay(2000);

		  tim14_flag=0;
	  }

	  if(tim13_flag==1)
	  {
		  ENCODER_OLD= ENCODER_NEW;
		   ENCODER_NEW = TIM4->CNT;


		    	RPM =  ((abs(ENCODER_NEW-ENCODER_OLD)*60))/512.0/0.01;	// ?��코더 ?��?��: 1 turn?�� 3 pulse 출력
		     	RPS=RPM/60;
		     	Omega= RPS*2*pi;


		  speed_sensor=Omega;

		 	 speed_error = (speed_input - speed_sensor);

		 	 S_KP = speed_error*speed_kp;

		// 	  speed_error_sum = speed_error_sum + speed_error*0.0001;
		 	//    S_KI = (speed_error_sum - (speed_ka*(speed_is-speed_i_ref))) * speed_ki;

		 	 // if(duty==0) speed_error_sum=0;
	//	 	  S_KI = speed_error_sum  * speed_ki;

		 	//  speed_is = S_KP + S_KI -speed_sensor*speed_kps;
		 	 speed_is = S_KP -speed_sensor*speed_kps;

		 	  if(speed_is>=0.5)
		 	  {
		 	  	speed_i_ref=0.5;
		 	  }
		 	  else if(speed_is<=0.1)
		 	  {
		 	  	speed_i_ref=0.1;
		 	  }

		 /*	 x=speed_i_ref*48;

		 	  if(x<=20)
		 		  {
		 			  duty = 0.0018*pow(x,3) - 0.0191*pow(x,2) + 0.4468*x + 4.2488;

		 		  }
		 		  else if(x>20)
		 		  {
		 	        duty = -1*(-3.6513*pow(x,4) + 327.08*pow(x,3) - 10973*pow(x,2) + 163412*x - 911605);


		 		   }

		 	  TIM3->CCR1 = 50+1000*duty/100;
*/

		 	 TIM3->CCR1= 1000*speed_i_ref;
		  tim13_flag=0;
	  }

//	  TIM3->CCR1 = 1000*duty/100;
 // 	TIM3->CCR2 = 1000*duty/100;










/*
if(encoder_count>=512)
{
	TIM3->CCR1 = 0;  //TIM3->CCR1
}
*/
	  if(RX_flag==1)
	  {
		 // RX_flag=0;
		//  TIM4->CNT=0;
	  	 		TIM3->CCR1 = 1000;
	  	 		TIM3->CCR2 = 1000;
	  	 			if(encoder_count>=1200)
	  		 		{
	  	 				TIM3->CCR1 = 50;

	  	 				GPIOG->ODR |= 1<<0;
	  	 				HAL_Delay(100);
	  	 				GPIOG->ODR &= ~1<<0;
	  	 				TIM3->CCR1 = 50;
	  	 				if(encoder_count>=512*3)
	  	 				{
	  	 					TIM3->CCR1 = 0;
	  	 					RX_flag=0;
	  	 				}

	  	 	     	}

	  }






//	  HAL_Delay(1);


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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  ADC1->CR1 |=  (1<<5);		// EOCIE=1: Interrupt enable for EOC
  NVIC->ISER[0] |= (1<<18);	// Enable ADC global Interrupt             ?��링파?��?��?�� 추�?




  //HAL_ADC_Start_DMA(&hadc1, &ADC1_value,2);
  // HAL_ADC_Start_DMA(&hadc1, &ADC1_value[1], 1);
  /* USER CODE END ADC1_Init 2 */

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

   static uint8_t MACAddr[6];

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 340;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 840-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim3, TIM_CHANNEL_1);
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim3, TIM_CHANNEL_2);
  /* USER CODE BEGIN TIM3_Init 2 */
  TIM3->DIER |= (1<<1);   // CC1IE: Enable the Tim3 CC1 interrupt
   NVIC->ISER[0] |= (1<<29); // TIM3_CC
   TIM3->CCER	|= (1<<0);	// CC1E=1: OC1(TIM5_CH1) Active(Capture/Compare 1 output enable)
    					// ?��?��??(40�?????????????????????????????????)?�� ?��?�� ?��?��출력
   TIM3->CR1	|= (1<<0);	// CEN: Counter TIM3 enable

  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 340;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 84-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 100-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 84-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1000-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */
  HAL_TIM_Base_Start(&htim7);
  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 840-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 50-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 2000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */
  HAL_TIM_Base_Start_IT(&htim8);
  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 21-1;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 100-1;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */
  HAL_TIM_MspPostInit(&htim13);
  /* USER CODE END TIM13_Init 2 */
  HAL_TIM_MspPostInit(&htim13);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 8400-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 100-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */
  TIM14->DIER |= (1<<0);   // CC1IE: Enable the Tim14 UG interrupt
   NVIC->ISER[1] |= (1<<(45-32)); // TIM14_CC

   TIM14->CR1	|= (1<<0);	// CEN: Counter TIM14 enable

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_EVEN;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  if (HAL_UART_Init(&huart3) != HAL_OK)
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
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|LD3_Pin|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 LD3_Pin PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|LD3_Pin|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 PG1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)  //1ms마다 ???���????????????????????????????????????????????????????????????? ?��??
{
	encoder_count = __HAL_TIM_GET_COUNTER(&htim4);
//	 encoder_count=TIM4->CNT;

		  printf("encoder_count = %d \r\n",encoder_count);

}*/

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
