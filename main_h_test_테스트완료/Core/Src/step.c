#include "step.h"
#include "main.h"

extern uint32_t step_pulse_count_tim13;
extern uint16_t step_pulse_count_tim11;

extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim11;

uint16_t shaft_step = 400*1;//400스텝 : 1바퀴
uint16_t shaft_pulse_cycle = 800;

uint16_t ball_screw_step=400*1;
uint16_t ball_screw_pulse_cycle=500;

void shaft_step_motor(uint16_t step1,uint16_t cycle_time1)  // (스텝,주기)
{

	TIM13->ARR = cycle_time1;

	 if(step_pulse_count_tim13 >= step1*2)  //
	 {

	 HAL_TIM_OC_Stop_IT(&htim13,TIM_CHANNEL_1);  // 타이머 정지
	 }
}

void z_axis_step_motor(uint16_t step2,uint16_t cycle_time2)  // (스텝,주기)
{

	TIM11->ARR = cycle_time2;

	 if(step_pulse_count_tim11 >= step2*2)  //
	 {

	 HAL_TIM_OC_Stop_IT(&htim11,TIM_CHANNEL_1);  // 타이머 정지
	 }
}
