#include "step.h"
#include "main.h"

extern uint32_t step_pulse_count_tim13;
extern uint16_t step_pulse_count_tim11;

extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim11;

uint16_t shaft_step = 400*1;//400스텝 : 1바퀴
uint16_t shaft_pulse_cycle = 3000; //속도

uint16_t ball_screw_step=400*1;
uint16_t ball_screw_pulse_cycle=300;

void shaft_step_motor(uint16_t step1,uint16_t cycle_time1)  // (스텝,주기)  // 다운 파트 스텝모터
{

	TIM13->ARR = cycle_time1;

	 if(step_pulse_count_tim13 >= step1*2)  //
	 {

	 HAL_TIM_OC_Stop_IT(&htim13,TIM_CHANNEL_1);  // 타이머 정지
	 }
}


void z_axis_step_motor(uint16_t step2,uint16_t cycle_time2)  // (스텝,주기)      // 볼스크류 스텝모터
{

	TIM11->ARR = cycle_time2;

	 if(step_pulse_count_tim11 >= step2*2)  //
	 {

	 HAL_TIM_OC_Stop_IT(&htim11,TIM_CHANNEL_1);  // 타이머 정지
	 }
}



void z_axis_UP(uint16_t step2,uint16_t cycle_time2)  // (스텝,주기)  //UP = 정방향
{
	ball_screw_step=step2;
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, 0);  //방향

			step_pulse_count_tim11 = 0;  //펄스 기준값 =0
			HAL_TIM_OC_Start_IT(&htim11, TIM_CHANNEL_1);

}
void z_axis_DOWN(uint16_t step2,uint16_t cycle_time2)  // (스텝,주기)  //DOWN = 역방향
{
	ball_screw_step=step2;
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, 1);  //방향

			step_pulse_count_tim11 = 0;
			HAL_TIM_OC_Start_IT(&htim11, TIM_CHANNEL_1);


}



void down_part_UP(uint16_t step1,uint16_t cycle_time1)  // (스텝,주기)
{
	shaft_step = step1;

	step_pulse_count_tim13 = 0;  //펄스 기준값 =0
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, 1);  //방향  //일단 업이라 가정

		HAL_TIM_OC_Start_IT(&htim13, TIM_CHANNEL_1);
}

void down_part_DOWN(uint16_t step1,uint16_t cycle_time1)  // (스텝,주기)
{
	shaft_step = step1;

	step_pulse_count_tim13 = 0;
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, 0);  //방향

			HAL_TIM_OC_Start_IT(&htim13, TIM_CHANNEL_1);
}


