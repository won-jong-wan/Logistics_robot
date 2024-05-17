#include "sensor.h"
#include "main.h"
#include "pid.h"
#define TRIG_PIN GPIO_PIN_7
#define TRIG_PORT GPIOD
#define ECHO_PIN GPIO_PIN_6
#define ECHO_PORT GPIOD

float Distance = 0;  // cm
float Distance_Sum = 0;
float Distance_Avg;
uint16_t j = 0;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim5;


void distance_sensor(void) {
	static uint32_t pMillis;
	static uint32_t Value1 = 0;
	static uint32_t Value2 = 0;

	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET); // pull the TRIG pin HIGH
	__HAL_TIM_SET_COUNTER(&htim12, 0);
	while (__HAL_TIM_GET_COUNTER (&htim12) < 10)
		;  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET); // pull the TRIG pin low

	pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
	// wait for the echo pin to go high
	while (!(HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN))
			&& pMillis + 10 > HAL_GetTick())
		;
	Value1 = __HAL_TIM_GET_COUNTER(&htim12);

	pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
	// wait for the echo pin to go low
	while ((HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN))
			&& pMillis + 50 > HAL_GetTick())
		;
	Value2 = __HAL_TIM_GET_COUNTER(&htim12);

	Distance = (Value2 - Value1) * 0.034 / 2 / 2;

	Distance_Sum = Distance + Distance_Sum;
	j++;
	if (j == 100) {
		Distance_Avg = Distance_Sum / (100 + 1);

		j = 0;
		Distance_Sum = 0;
	}

	//     printf("Distance = %f \r\n",Distance);
	//      sprintf((char *)buffer, "%d \r\n",Distance);
	//     HAL_UART_Transmit(&huart3, buffer, strlen((char *)buffer), 100);

}

uint32_t save_X_IN = 0;
uint32_t save_X_OUT = 0;
uint32_t save_Y_IN = 0;
uint32_t save_Y_OUT = 0;

uint32_t save_X_avg;

extern uint8_t vt_start;  //전
extern uint8_t vt_start_Y;  //전

extern uint8_t position_start;
extern uint8_t position_start_Y;
extern uint32_t encoder_count_x;

extern float speed_sensor_X;



int photo_sensor_flag_x = 0;
int photo_sensor_flag_y = 0;
int photo_sensor_flag_y__ = 0;

extern float speed_sensor_X_vt;
extern float speed_sensor_X_pid;
extern float speed_input_X_vt;

extern float maxV;

extern uint16_t i_v_t_count;

extern float maxV;
extern float fallingN;
extern float fallingN_;
extern float raisingN;
extern float staticN;
extern float orderN;




#define sensor_test 2  // 1: 센서 입력 시 멈춤 .
//2: 센서 입력 시  pid
void photo_sensor_1(void) {

	static int sensor_status = 0;
	static int edge_status = 0;

	sensor_status++;

	if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 0)) {
		edge_status = 0;
		if (sensor_status == 2) {

#if sensor_test == 1
				save_X_IN = encoder_count_x;
				GPIOB->ODR |= 1 << 7;
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, 1);  //stop
				vt_start=0;
				GPIOB->ODR &= ~1 << 14;  //LD3
#elif sensor_test == 2

			GPIOB->ODR &= ~1 << 14;

			if (i_v_t_count >= raisingN + staticN + fallingN) {
				photo_sensor_flag_x = 1;

				save_X_IN = encoder_count_x;
				GPIOB->ODR |= 1 << 0;
				dc_motor_pid_X_with_photosensor1(save_X_IN);
			}

#endif
		}

	} else {
		sensor_status = 0;
		if (sensor_status <= 1) {
			edge_status++;
		}
	}
	if (edge_status == 2) {
		save_X_OUT = encoder_count_x;
		GPIOB->ODR &= ~1 << 0;

	}
//	save_X_avg = (save_X_IN + save_X_OUT)/2;
	//save_X_avg = save_X_IN ;
	//printf("%d ,. %d  ,   %d\n\r",save_X_IN,save_X_OUT,sensor_status);

}
extern int v_t_dir_back_Y_flag;  //0: 전진 , 1: 후진


void photo_sensor_2(void) {

	static int sensor_status = 0;
	static int edge_status = 0;
	extern uint32_t encoder_count_y;

	sensor_status++;

	if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == 0))
	{
		edge_status = 0;
		if (sensor_status == 2)
		{
		//	photo_sensor_flag_y__=1;

			GPIOB->ODR &= ~1 << 14;

			if (i_v_t_count >= raisingN + staticN + fallingN)
						{

				v_t_dir_back_Y_flag = 0;  //0: 전진 , 1: 후진

							photo_sensor_flag_y = 1;

							save_Y_IN = encoder_count_y;
							GPIOB->ODR |= 1 << 0;
							dc_motor_pid_Y_with_photosensor2(save_Y_IN);
							photo_sensor_flag_y__=0;

						}

		}

	} else {
		sensor_status = 0;
		if (sensor_status <= 1) {
			edge_status++;
		}
	}
	if (edge_status == 2) {
		//save_Y_OUT = encoder_count_y;
		GPIOB->ODR &= ~1 << 7;
		//	save_Y_avg = (save_X_IN + save_X_OUT)/2;
	}
	//	printf("%d ,. %d  ,   %d\n\r",save_X,save_Y,sensor_status);
}

