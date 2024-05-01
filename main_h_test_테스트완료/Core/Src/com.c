#include "com.h"
#include "main.h"
#include "pid.h"





float input_encodor;
float targetS;

int position_start = 0;
int position_start_Y=0;

extern uint32_t tim4_encoder_overflow;
extern uint32_t encoder_count_y;
extern uint32_t encoder_count_x;
extern uint8_t key_value;

extern float input_position_X;
extern float input_position_Y;
float input_angle_X;

extern float speed_input_X;

extern float err_sum_s_X;
extern float err_sum_X;
extern float err_sum_s_Y;
extern float err_sum_Y;

extern uint8_t data;
int8_t RX_flag = 0;
extern int tim14_flag;

extern float input_speed;

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim11;

extern uint32_t step_pulse_count_tim13;
extern uint16_t step_pulse_count_tim11;

uint8_t rev10_flag;
uint8_t rev1_flag;

//extern int us;
extern uint16_t step_count;
int step_enable = 0;
uint16_t step_start = 0;







PUTCHAR_PROTOTYPE  //테라텀
{
	HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}

int _write(int file, char *p, int len) {        //테라텀
	HAL_UART_Transmit(&huart3, p, len, 10);
	return len;
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

uint16_t RPM_uint16;
uint16_t battery_uint16;
extern float RPM_X;
extern float RPS_X;
extern float Omega_X;
extern uint16_t ADC3_value[3];
uint8_t buffer[256];
extern float battery_V;

void STM32_to_ESP(void)                    // ESP로 데이터 보냄
{

#define test 1
#if test ==1
			GPIOB->ODR ^= 1 << 7;
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

void UART3_RX_PC_to_STM32(void) // UART3: STM32 <-> PC
{
	HAL_UART_Receive_IT(&huart3, &key_value, 1);
	printf("%d,^^ %c \r\n", key_value, key_value);

	switch (key_value) {
	case 'q':
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, 1);  //방향 back
		//	GPIOE->ODR |= 1 << 0;  // back
		GPIOB->ODR |= 1 << 0;  //LD1

		break;

	case 'w':
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, 0);  //방향 go
		//	GPIOE->ODR &= ~1 << 0;  // go
		GPIOB->ODR &= ~1 << 0;  //LD1

		break;

	case '1':
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 1);  //stop
		//GPIOG->ODR |= 1 << 0;  //stop

		break;

	case '2':
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 0);  //start
		//GPIOG->ODR &= ~1 << 0;  //start

		break;

	case '3':
		TIM3->CCR1 = 700;


		break;

	case '4':
		TIM3->CCR1 = 1500;


		break;

	case '5':
		TIM3->CCR1 = 5000;


		break;

		//////////////////////////y축 이동

	case 'd':
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);  //y축 방향 back

		GPIOB->ODR |= 1 << 0;  //LD1

		break;

	case 'f':
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);  //y축 방향 go

		GPIOB->ODR &= ~1 << 0;  //LD1

		break;

	case 'a':
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 1);  //stop

		break;

	case 's':
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 0);  //start

		break;

	case 'e':
		TIM3->CCR2 = 700;

		break;

	case 'r':
		TIM3->CCR2 = 1500;

		break;
	case 't':
		TIM3->CCR2 = 5000;

		break;
		///////////////////////////////////////////////////////////////// x축 바퀴 PID
	case 'u':
		if (position_start == 0) {
			position_start = 1;

		} else if (position_start == 1) {
			position_start = 0;

		}
		break;

	case 'i':  //i, o, p : PID 동작 버튼
		TIM2->CNT = 1000000;
		input_position_X = 1000000 + 34000;
		err_sum_X = 0;
		err_sum_s_X = 0;
		//			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 0);  //start

		break;

	case 'o':

		dc_motor_pid_X(34000);

		break;

	case 'p':
		dc_motor_pid_X(-34000);

		break;

	case '[':
		dc_motor_pid_X(input_encodor);  //디버그모드에서 직접 테스트 용도

		break;

	case ']':

		v_t_graph(targetS);


		break;
	case 'z':
		speed_input_X = 0;
		err_sum_X = 0;
		err_sum_s_X = 0;
		break;

	case 'x':
		speed_input_X = 7;
		err_sum_X = 0;
		err_sum_s_X = 0;
		break;
	case 'c':
		speed_input_X = 15;
		err_sum_X = 0;
		err_sum_s_X = 0;
		break;
	case 'v':
		speed_input_X = 23;
		err_sum_X = 0;
		err_sum_s_X = 0;
		break;

	///////////////////////////////////////////////////////// y축  바퀴  PID
	case '7':
			if (position_start_Y == 0) {
				position_start_Y = 1;

			} else if (position_start_Y == 1) {
				position_start_Y = 0;

			}
			break;

 	case '8':  //8, 9, 0 : PID y축 동작 버튼




		TIM4->CNT=30000;
 		tim4_encoder_overflow=1000000-TIM4->CNT;


		input_position_Y = 1000000 + 34000;
		err_sum_Y = 0;
		err_sum_s_Y = 0;
		break;

	case '9':
		TIM4->CNT=30000;
		 		tim4_encoder_overflow=1000000-TIM4->CNT;
		input_position_Y = 1000000 + 34000 * 2;
		err_sum_Y = 0;
		err_sum_s_Y = 0;
		break;

	case '0':
		TIM4->CNT=30000;
		 		tim4_encoder_overflow=1000000-TIM4->CNT;
		input_position_Y = 1000000 - 34000;
		err_sum_Y = 0;
		err_sum_s_Y = 0;
		break;

		///////////////////////////////////////z축 볼스크류 스텝모터
	case 'j':
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, 1);  //정지

		break;

	case 'k':
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, 0);  //출발

		break;
	case 'l':   //cw 1바퀴               시계방향  위쪽
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, 0);  //방향

		step_pulse_count_tim11 = 0;  //펄스 기준값 =0
		HAL_TIM_OC_Start_IT(&htim11, TIM_CHANNEL_1);

		break;
	case ';':    //ccw 1바퀴            반시계방향  아래쪽

		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, 1);  //방향

		step_pulse_count_tim11 = 0;
		HAL_TIM_OC_Start_IT(&htim11, TIM_CHANNEL_1);

		///////////////////////////////////////다운파트 샤프트 스텝모터
		break;

	case 'm':
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 1);  //정지

		break;

	case ',':
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 0);  //출발

		break;

	case '.': //cw 1바퀴

		step_pulse_count_tim13 = 0;  //펄스 기준값 =0
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, 0);  //방향

		HAL_TIM_OC_Start_IT(&htim13, TIM_CHANNEL_1);

		break;

	case '/': //ccw 1바퀴

		step_pulse_count_tim13 = 0;
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, 1);  //방향

		HAL_TIM_OC_Start_IT(&htim13, TIM_CHANNEL_1);

		break;

	}

}

void UART4_RX_ESP_to_STM32(void) // UART4: esp <-> stm32
{
	HAL_UART_Receive_IT(&huart4, &data, 1);

	printf("%d -- %c \r\n", data, data);
	switch (data) {
	case '0':
		GPIOB->ODR |= 1 << 0;
		GPIOG->ODR &= ~1 << 0;  // start

		break;

	case '1':  //stop
		GPIOB->ODR &= ~1 << 0;
		GPIOG->ODR |= 1 << 0;
		break;

	case '2':  //dir
		GPIOE->ODR |= 1 << 0;  // back
		break;

	case '3':  //dir
		GPIOE->ODR &= ~1 << 0; // go
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
		RX_flag = 1;
		TIM2->CNT = 0;
		TIM4->CNT = 0;
		break;
	case 'A':  //Tx

		RX_flag = 2;
		break;
	}

}

