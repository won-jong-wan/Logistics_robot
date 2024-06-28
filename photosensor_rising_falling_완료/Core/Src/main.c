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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "sensor.h"
#include "step.h"
#include "linear.h"
#include "pp.h"
#include "mpu6050.h"

uint8_t key_value;

uint8_t data;

uint32_t encoder_count_x = 1000000;
uint16_t encoder_count_y = 30000;
extern uint32_t tim4_encoder_overflow;

uint16_t ADC3_value[3];

extern uint8_t tim6_flag;
extern uint8_t tim14_flag;
extern uint8_t tim5_flag;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern uint16_t shaft_pulse_cycle;
extern uint16_t ball_screw_pulse_cycle;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM13_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM5_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

struct {
	bool target_2_1;
	bool target_3_1;
	bool target_1_2;
	bool target_2_2;
	bool target_3_2;
} Manual_mode = { FALSE, FALSE, FALSE };

struct flag Finish_flag = { FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, 0,
		0, FALSE, FALSE };

float targetS_X_GO = 52;
float targetS_X_BACK = 52;

float targetS_Y_GO = 52;
float targetS_Y_BACK = 52;
float go_time = 1.0;
float back_time = 1.3;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern int pathLength;

cell path[CELL_DOMAIN_MAX];

extern uint32_t save_X_IN;

extern float maxV;
extern float fallingN;
extern float fallingN_;
extern float raisingN;
extern float staticN;
extern float orderN;
extern uint16_t i_v_t_count;

uint8_t target_1_2_flag=0;
uint8_t target_2_1_flag=0;

void FALSE_Init() {
	Finish_flag.x_go = FALSE;
	Finish_flag.x_back = FALSE;
	Finish_flag.y_go = FALSE;
	Finish_flag.y_back = FALSE;
	Finish_flag.ball_high = FALSE;
	Finish_flag.ball_mid = FALSE;
	Finish_flag.ball_low = FALSE;
	Finish_flag.downpart_high = 0;
	Finish_flag.downpart_low = 0;
	Finish_flag.linear_go = FALSE;
	Finish_flag.linear_back = FALSE;

	target_1_2_flag=0;
	target_2_1_flag=0;
}


//수동 모드 경우의 수는 3가지: x만 이동, y만 이동, xy이동
//void Manual_Mode_Y(float row, float col) {
//
//	if (Manual_mode.target_1_2 == TRUE) { //y축 1칸 왕복,  볼스크류 왕복  다운파트 생략 ,
//
//		if(	target_1_2_flag == 0)
//		{
//			v_t_graph_DIR_GO_Y(targetS_Y_GO * col);  //앞으로
//			target_1_2_flag=1;
//		}
//
//		if (Finish_flag.y_go == TRUE) {
//			target_1_2_flag=2;
//			HAL_Delay(700 - 1);
//			z_axis_UP(400 * 6 * 2, ball_screw_pulse_cycle);
//
//			Finish_flag.y_go = FALSE;
//		}
//		if ((Finish_flag.ball_high == TRUE) &&  (target_1_2_flag ==2) ) {
//			target_1_2_flag=3;
//			HAL_Delay(700 - 1);
//			z_axis_DOWN(400 * 6 * 2, ball_screw_pulse_cycle);
//
//			Finish_flag.ball_high = FALSE;
//		}
//		if ((Finish_flag.ball_low == TRUE) && (target_1_2_flag ==3) ) {
//
//			HAL_Delay(700 - 1);
//			v_t_graph_DIR_BACK_Y(targetS_Y_BACK * col);
//
//			FALSE_Init();
//			Manual_mode.target_1_2 = FALSE;
//
//
//		}
//	}
//}

//y축 1칸 왕복,  볼스크류 왕복  다운파트 생략 ,
void Manual_Mode_Y(float row, float col) {  //볼스크류 먼저 >> y 이동

	if (Manual_mode.target_1_2 == TRUE) {


		if(	target_1_2_flag == 0)
		{
			//v_t_graph_DIR_GO_Y(targetS_Y_GO * col);  //앞으로
			z_axis_UP(400 * 6 * 2, ball_screw_pulse_cycle);
			target_1_2_flag=1;
		}

		if ((Finish_flag.ball_high == TRUE) &&  (target_1_2_flag ==1) ) {
			target_1_2_flag=2;
			HAL_Delay(700 - 1);
			v_t_graph_DIR_GO_Y(targetS_Y_GO * (col-1) );  //앞으로

			Finish_flag.ball_high = FALSE;
		}
		if ((Finish_flag.y_go == TRUE) &&  (target_1_2_flag ==2) ) {
			target_1_2_flag=3;
			HAL_Delay(700 - 1);

			v_t_graph_DIR_BACK_Y(targetS_Y_BACK * (col-1) );
			Finish_flag.y_go = FALSE;
		}
		if ((Finish_flag.y_back == TRUE) && (target_1_2_flag ==3) ) {

			HAL_Delay(700 - 1);
			z_axis_DOWN(400 * 6 * 2, ball_screw_pulse_cycle);

			FALSE_Init();
			Manual_mode.target_1_2 = FALSE;


		}
	}
}
//(2,1)  (3,1)    여기는 볼스크류 필요없음  , 다운파트 나중
void Manual_Mode_X(float row, float col) {   // x전진 , x후진 끝

	if ((Manual_mode.target_2_1 == TRUE) || (Manual_mode.target_3_1 == TRUE)) {

		if(	target_2_1_flag == 0)
		{
			v_t_graph_DIR_GO_X(targetS_X_GO * (row-1) );  //앞으로

			target_2_1_flag=1;
		}

		if ((Finish_flag.x_go == TRUE) && (target_2_1_flag ==1) ){
			target_2_1_flag=2;
			HAL_Delay(700 - 1);
			v_t_graph_DIR_BACK_X(targetS_X_BACK  * (row-1) );  //앞으로

			Finish_flag.x_go = FALSE;
			FALSE_Init();
			Manual_mode.target_2_1 = FALSE;
			Manual_mode.target_3_1 = FALSE;
		}
	}
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
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_TIM6_Init();
  MX_TIM13_Init();
  MX_ADC3_Init();
  MX_TIM12_Init();
  MX_TIM11_Init();
  MX_TIM4_Init();
  MX_TIM7_Init();
  MX_TIM5_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

	MPU6050_Init();

	HAL_UART_Receive_IT(&huart3, &key_value, 1);
	HAL_UART_Receive_IT(&huart4, &data, 1);

	HAL_ADC_Start_DMA(&hadc3, (uint32_t*) ADC3_value, 3);

	HAL_TIM_Base_Start_IT(&htim6);  //pid
	HAL_TIM_Base_Start_IT(&htim7);  //센서
	HAL_TIM_Base_Start_IT(&htim14);  //RPM
	HAL_TIM_Base_Start_IT(&htim5);  //vt

	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(&htim4);

	HAL_TIM_Base_Start(&htim12);  //초음파 센서
	HAL_TIM_Base_Start_IT(&htim8);  //ADC 타이머

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	setvbuf(stdin, NULL, _IONBF, 0);

	gridInit(IsEmpty);
	setDpp(0, 0, 5, 5); //0,0 에서 1,2로
	readPath(lpp(), path);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//		encoder_count_x = TIM2->CNT;  //32bit timer
//		encoder_count_y = TIM4->CNT;
		//+ tim4_encoder_overflow; //16bit timer 에서 32비트로

//		printf("%d %d\r\n",save_X_IN,encoder_count_x);

		if (tim6_flag == 1)  //1ms
		{
			distance_sensor();  //거리 센서
	//		MPU6050_Read_Accel();  //MPU떄문에 거리센서 평균이 계산이 느려짐
	//		MPU6050_Read_Gyro();

			tim6_flag = 0;
		}
#if 0
		//수동 조작
		if (Manual_mode.target_2_2 == TRUE) { //ㄱ자 이동

			v_t_graph_DIR_GO_X(targetS_X_GO);  //앞으로  // 출발 신호 밑으로 내릴 것

			if (Finish_flag.x_go == TRUE) {
				HAL_Delay(500 - 1);
				z_axis_DOWN(400 * 6 * 2, ball_screw_pulse_cycle);

				if (Finish_flag.ball_low == TRUE) {
					HAL_Delay(500 - 1);
					v_t_graph_DIR_GO_Y(targetS_Y_GO);  //앞으로

					if (Finish_flag.y_go == TRUE) {
						HAL_Delay(500 - 1);
						//다운 파트 임의 설정
						down_part_DOWN(4000, shaft_pulse_cycle);

						if (Finish_flag.downpart_low >= 1) {
							HAL_Delay(500 - 1);
							linear_motor_GO(go_time);

							if (Finish_flag.linear_go == TRUE) {
								HAL_Delay(500 - 1);
								down_part_UP(4000, shaft_pulse_cycle);

								if (Finish_flag.downpart_high >= 1) {
									HAL_Delay(500 - 1);
									v_t_graph_DIR_BACK_Y(targetS_Y_BACK); //y 후진

									if (Finish_flag.y_back == TRUE) {
										HAL_Delay(500 - 1);
										z_axis_UP(400 * 6 * 2,
												ball_screw_pulse_cycle);

										if (Finish_flag.ball_high == TRUE) {
											HAL_Delay(500 - 1);
											v_t_graph_DIR_BACK_X(
													targetS_X_BACK); //앞으로      //초기 위치로

											if (Finish_flag.x_back == TRUE) {
												HAL_Delay(500 - 1);
												down_part_DOWN(4000,
														shaft_pulse_cycle);

												if (Finish_flag.downpart_low
														== 2) {
													HAL_Delay(500 - 1);
													linear_motor_BACK(
															back_time);

													if (Finish_flag.linear_back
															== TRUE) {
														HAL_Delay(500 - 1);
														down_part_UP(4000,
																shaft_pulse_cycle);

														FALSE_Init();
														Manual_mode.target_2_2 =
																FALSE;
													}
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}
#endif
		Manual_Mode_Y(1,2);
		if(key_value == '!') Manual_Mode_X(2,1);

		if(key_value == '@') Manual_Mode_X(3,1);




		/*		if(Manual_mode.target_1_2 == TRUE){ //y축 1칸 왕복,  볼스크류 왕복  다운파트 생략 ,

		 if(Finish_flag.y_go ==TRUE)
		 {
		 HAL_Delay(700-1);
		 z_axis_UP(450 * 6*2, ball_screw_pulse_cycle);


		 if(Finish_flag.ball_high == TRUE)
		 {
		 HAL_TIM_OC_Stop_IT(&htim11,TIM_CHANNEL_1);  // 타이머 정지
		 HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, 1);  //정지

		 //	HAL_Delay(700-1);
		 z_axis_DOWN(450 * 6*2, ball_screw_pulse_cycle);




		 if(Finish_flag.ball_low == TRUE)
		 {

		 HAL_Delay(700-1);
		 v_t_graph_DIR_BACK_Y(targetS_Y_BACK);

		 FALSE_Init();
		 Manual_mode.target_1_2 = FALSE;
		 }
		 }

		 //FALSE_Init();
		 //Manual_mode.target_2_1 = FALSE;
		 }
		 //			if(Finish_flag.y_go ==TRUE)
		 //			{
		 //				HAL_Delay(700-1);
		 //				v_t_graph_DIR_BACK_Y(targetS_Y_BACK);  //앞으로
		 //
		 //				FALSE_Init();
		 //				Manual_mode.target_1_2 = FALSE;
		 //			}
		 }
		 */






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
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ENABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = ENABLE;
  hadc3.Init.NbrOfDiscConversion = 3;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 3;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  htim2.Init.Prescaler = 5-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffffffff;
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
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000-1;
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
  sConfigOC.Pulse = 600;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

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
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xFFFF-1;
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

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 84-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 10000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */
	//0.001s = 1ms
	//PID 샘플링 주기
  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 84-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000-1;
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
  htim7.Init.Period = 100-1;
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
	//ADC 측정 타이머 , 나중엔 필요 x
  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 84-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 300-1;
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

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 168-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 800-1;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */
	// 초음파 센서
  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 42-1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 65535;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */
// 테스트 용
  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 84-1;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 800-1;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 130;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim13, TIM_CHANNEL_1);
  /* USER CODE BEGIN TIM13_Init 2 */
	//HAL_TIM_MspPostInit(&htim13);
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
//RPM 측정
  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 84-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 10000-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */
//	TIM14->DIER |= (1 << 0);   // CC1IE: Enable the Tim14 UG interrupt
//	NVIC->ISER[1] |= (1 << (45 - 32)); // TIM14_CC
//	TIM14->CR1 |= (1 << 0);	// CEN: Counter TIM14 enable
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
  huart4.Init.BaudRate = 230400;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  HAL_GPIO_WritePin(GPIOF, linear_DIR_Pin|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(linear_goback_GPIO_Port, linear_goback_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|DC_M_GO_Y_Pin|DC_M_DIR_Y_Pin|LD3_Pin
                          |LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, DC_M_GO_X_Pin|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DC_M_DIR_X_GPIO_Port, DC_M_DIR_X_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BALL_MIN_Pin */
  GPIO_InitStruct.Pin = BALL_MIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BALL_MIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : linear_DIR_Pin PF13 PF14 PF15 */
  GPIO_InitStruct.Pin = linear_DIR_Pin|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : linear_goback_Pin */
  GPIO_InitStruct.Pin = linear_goback_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(linear_goback_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin DC_M_GO_Y_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|DC_M_GO_Y_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_M_GO_X_Pin PG2 PG3 */
  GPIO_InitStruct.Pin = DC_M_GO_X_Pin|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : DC_M_DIR_Y_Pin */
  GPIO_InitStruct.Pin = DC_M_DIR_Y_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(DC_M_DIR_Y_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BALL_MAX_Pin BALL_MID_Pin */
  GPIO_InitStruct.Pin = BALL_MAX_Pin|BALL_MID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD2 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : sensor_X_Pin sensor_Y_Pin */
  GPIO_InitStruct.Pin = sensor_X_Pin|sensor_Y_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DC_M_DIR_X_Pin */
  GPIO_InitStruct.Pin = DC_M_DIR_X_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(DC_M_DIR_X_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

float input_encodor_X_GO = 34816 * 2;
float input_encodor_X_BACK = -34816 * 2;

float input_encodor_Y_GO = 10880 * 2;
float input_encodor_Y_BACK = -10880 * 2;

PUTCHAR_PROTOTYPE  //테라텀
{
	HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}

void SerialSendChar_PC(uint8_t Ch1) // 1문자 보내기 함수
{
	// USART_SR_TXE(1<<7)=0?, TX Buffer NOT Empty?
	// TX buffer Empty되지 않으면 계속 대기(송신 가능한 상태까지 대기)
	while ((USART3->SR & 1 << 7) == RESET)
		;
	USART3->DR = (Ch1 & 0x01FF);	// 전송 (최대 9bit 이므로 0x01FF과 masking)
}

void SerialSendChar_ESP(uint8_t Ch2) // 1문자 보내기 함수
{
	while ((UART4->SR & 1 << 7) == RESET)
		;
	UART4->DR = (Ch2 & 0x01FF);
}

extern uint16_t ADC3_value[3];

extern float battery_V;
uint16_t RPM_uint16;
uint16_t battery_uint16;

extern float RPM_X;
uint8_t buffer[256];

void STM32_to_ESP(void)                    // ESP로 데이터 보냄
{

#define test 1
#if test ==1
//	GPIOB->ODR ^= 1 << 7;
	RPM_uint16 = RPM_X * 10;
	battery_uint16 = battery_V * 100;

	SerialSendChar_ESP('?');

	//테스트

	if (ADC3_value[0] >= 1000 && ADC3_value[0] <= 4095) {

		sprintf((char*) buffer, "%d\n\r", ADC3_value[0]);
		//		HAL_UART_Transmit(&huart3, buffer, strlen((char*) buffer), 100);
		sprintf((char*) buffer, "%d\n\r", ADC3_value[0]);
		//		HAL_UART_Transmit(&huart3, buffer, strlen((char*) buffer), 100);
		sprintf((char*) buffer, "%d\n\r", ADC3_value[0]);
		//	HAL_UART_Transmit(&huart3, buffer, strlen((char*) buffer), 100);

		//배터리, 속도 전송
		sprintf((char*) buffer, "%d", ADC3_value[0]);
		HAL_UART_Transmit(&huart4, buffer, strlen((char*) buffer), 100);
		sprintf((char*) buffer, "%d", ADC3_value[0]);
		HAL_UART_Transmit(&huart4, buffer, strlen((char*) buffer), 100);
		sprintf((char*) buffer, "%d", ADC3_value[0]);
		HAL_UART_Transmit(&huart4, buffer, strlen((char*) buffer), 100);
	}

#elif test == 2
		 GPIOB->ODR ^= 1 << 7;
		 RPM_uint16 = RPM_X * 10;
		 battery_uint16 = battery_V * 100;

		 SerialSendChar_ESP('?');
		 sprintf((char*) buffer, "%d", battery_uint16);
		 HAL_UART_Transmit(&huart4, buffer, strlen((char*) buffer), 100);
		 sprintf((char*) buffer, "%d", RPM_uint16);
		 HAL_UART_Transmit(&huart4, buffer, strlen((char*) buffer), 100);
		 sprintf((char*) buffer, "%d", battery_uint16);
		 HAL_UART_Transmit(&huart4, buffer, strlen((char*) buffer), 100);
		 #endif

}

extern uint16_t shaft_pulse_cycle;
extern uint16_t ball_screw_pulse_cycle;

uint16_t ball_step = 100;
uint32_t downpart_step = 25;
extern uint32_t step_pulse_count_tim13;
extern uint16_t step_pulse_count_tim11;

void UART3_RX_PC_to_STM32(void) // UART3: STM32 <-> PC
{

	HAL_UART_Receive_IT(&huart3, &key_value, 1);
	printf("%d,^^ %c \r\n", key_value, key_value);

	switch (key_value) {
	case 'q':
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, 0);  //방향 go

		GPIOB->ODR |= 1 << 0;  //LD1

		break;

	case 'w':

		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, 1);  //방향 back
		GPIOB->ODR &= ~1 << 0;  //LD1

		break;

	case '1':
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 1);  //stop

		break;

	case '2':
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 0);  //start

		break;

	case '3':
		TIM3->CCR1 = 400;

		break;

	case '4':
		TIM3->CCR1 = 1700;

		break;

	case '5':
		TIM3->CCR1 = 6000;

		break;

		/////////////////////////////////////////////////////////////////y축 이동

	case 'd':
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);  //y축 방향 go

		GPIOB->ODR |= 1 << 0;  //LD1

		break;

	case 'f':

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);  //y축 방향 back
		GPIOB->ODR &= ~1 << 0;  //LD1

		break;

	case 'a':
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 1);  //stop

		break;

	case 's':
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 0);  //start

		break;

	case 'e':
		TIM3->CCR2 = 400;

		break;

	case 'r':
		TIM3->CCR2 = 1700;

		break;
	case 't':
		TIM3->CCR2 = 6000;

		break;
		///////////////////////////////////////////////////////////////// x축 바퀴 PID

	case 'i':

		dc_motor_pid_X(34816);

		break;

	case 'o':
		dc_motor_pid_X(-34816);

		break;

	case 'p':
		dc_motor_pid_X(input_encodor_X_GO);

		break;

	case '[':
		dc_motor_pid_X(input_encodor_X_BACK);

		break;
		////////////////////////////////////////////////////////////////x축 vt
	case 'I':  //30cm

		v_t_graph_DIR_GO_X(30);  //앞으로 30cm

		break;
	case 'O':

		v_t_graph_DIR_BACK_X(30); //뒤로 30cm

		break;
	case 'P':

		v_t_graph_DIR_GO_X(targetS_X_GO);  //앞으로

		break;
	case '{':

		v_t_graph_DIR_BACK_X(targetS_X_BACK); //뒤로

		break;
		///////////////////////////////////////////////////////// y축  바퀴  PID

	case '8':

		dc_motor_pid_Y(10880);
		break;

	case '9':
		dc_motor_pid_Y(-10880);
		break;

	case '0':
		dc_motor_pid_Y(input_encodor_Y_GO);
		break;
	case '-':
		dc_motor_pid_Y(input_encodor_Y_BACK);
		break;
//////////////////////////////////////////////////////////      Y  VT
	case '*':

		v_t_graph_DIR_GO_Y(30);   //Y축 vt 앞
		break;

	case '(':
		v_t_graph_DIR_BACK_Y(30);   //뒤
		break;

	case ')':
		v_t_graph_DIR_GO_Y(targetS_Y_GO);
		break;

	case '_':
		v_t_graph_DIR_BACK_Y(targetS_Y_BACK);
		break;

		//////////////////////////////////////////////////////z축 볼스크류 스텝모터,  0625수정: 동작시에만 enable
	case 'j':
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, 1);  //정지
		step_pulse_count_tim11 = 0;
		break;

	case 'k':
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, 0);  //출발

		break;

	case 'l':   //cw 미소 이동             시계방향  위쪽
		z_axis_UP(ball_step, ball_screw_pulse_cycle);

		break;
	case ';':    //ccw 미소 이동           반시계방향  아래쪽
		z_axis_DOWN(ball_step, ball_screw_pulse_cycle);

		break;

	case 'L':   //cw              시계방향  위쪽
		z_axis_UP(430 * 6, ball_screw_pulse_cycle);

		break;
	case ':':    //ccw            반시계방향  아래쪽

		z_axis_DOWN(430 * 6, ball_screw_pulse_cycle);

		break;

		/////////////////////////////////////////////////////////////////////다운파트 축 스텝모터
	case 'm':
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 1);  //정지
		step_pulse_count_tim13 = 0;
		break;

	case ',':
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 0);  //출발

		break;

	case 'M':  //가속
		down_part_UP_accel(downpart_step);

		break;
	case '<': //가속
		down_part_DOWN_accel(downpart_step);

		break;
	case '.':

		down_part_UP(downpart_step, shaft_pulse_cycle);

		break;

	case '/':

		down_part_DOWN(downpart_step, shaft_pulse_cycle);

		break;
	case '>':

		down_part_UP(400, shaft_pulse_cycle);

		break;

	case '?':

		down_part_DOWN(400, shaft_pulse_cycle);

		break;
//////////////////////////////////////////////////////////////////////////////////리니어 모터
//	case 'z':
//
//		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, 1); //정지
//
//		break;
//
//	case 'x':
//
//		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, 0);  //출발
//
//		break;
//	case 'c':
//
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1);   //방향 후진
//
//		break;
//
//	case 'v':
//
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);  //방향  전진
//
//		break;
//
//	case 'b': //방향  전진
//
//		TIM3->CCR4 = 1500;
//
//		break;
//	case 'n': //방향  전진
//
//		TIM3->CCR4 = 4200;
//
//		break;
//	case 'h': //방향  전진
//
//		TIM3->CCR4 = 8000;
//
//		break;

	case 'z': //방향 초 전진

		linear_motor_GO(go_time);

		break;
	case 'x': //방향 초 전진

		linear_motor_BACK(back_time);

		break;

		/////////////////////////////////////////////////////////수동 조작
	case '!':  // (2,1)
		FALSE_Init();
		Manual_mode.target_2_1 = TRUE;

		break;
	case '@': //  (3,1)
		FALSE_Init();
		Manual_mode.target_3_1 = TRUE;

		break;

	case '#':  //(1,2)
		FALSE_Init();
		Manual_mode.target_1_2 = TRUE;

		break;

	}

}

void UART4_RX_ESP_to_STM32(void) // UART4: esp <-> stm32
{
	HAL_UART_Receive_IT(&huart4, &data, 1);

	printf("%d -- %c \r\n", data, data);
	switch (data) {
	case '0':
		v_t_graph_DIR_GO_X(targetS_X_GO);  //앞으로
		break;

	case '1':
		v_t_graph_DIR_BACK_X(targetS_X_BACK); //뒤로
		break;

	case '2':
		v_t_graph_DIR_GO_Y(targetS_Y_GO);

		break;

	case '3':

		v_t_graph_DIR_BACK_Y(targetS_Y_BACK);
		break;

	case '4':

		z_axis_UP(400 * 6, ball_screw_pulse_cycle);
		break;

	case '5':
		z_axis_DOWN(400 * 6, ball_screw_pulse_cycle);
		break;

	case '6':

		break;

	}

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
	__disable_irq();
	while (1) {
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
