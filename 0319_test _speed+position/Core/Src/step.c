#include "step.h"
#include "main.h"

extern uint16_t step_pulse_count;

extern TIM_HandleTypeDef htim13;

void z_axis_step_motor(uint16_t step_pulse,uint16_t cycle_time)  // (펄스,주기)
{

	TIM13->ARR = cycle_time;

	 if(step_pulse_count >= step_pulse*2)  //
	 {

	 HAL_TIM_OC_Stop_IT(&htim13,TIM_CHANNEL_1);  // 타이머 정지
	 }
}
