#include "pid.h"
#include "main.h"



extern uint32_t encoder_count;
uint16_t input_position;

int32_t err;
float err_sum;
double p_PID;
float p_kp;
float p_ki;
float p_kd;

float P_KP;
float P_KI;


double P_KD;
double err_d;
float previous_err;




void position_pid(void)
{
	p_kp = 4;
	p_ki = 2;
	p_kd = 0.01;

	err = input_position - encoder_count;
	P_KP = err * p_kp;

	err_sum = err_sum + err * 0.0001;
	if (p_PID == 0)
	{
		err_sum = 0;
	}
	P_KI = err_sum * p_ki;

	err_d = (err - previous_err) / 0.0001;
	previous_err = err;
	P_KD = err_d * p_kd;

	p_PID = P_KP + P_KI + P_KD;

	if (p_PID >= 400)
	{
		p_PID = 400;
	}
	else if (p_PID <= -400)
	{
		p_PID = -400;
	}

	if (p_PID > 0)
	{
		GPIOE->ODR &= ~1 << 0; // go
		TIM3->CCR1 = p_PID;
	}
	else if (p_PID < 0)
	{
		GPIOE->ODR |= 1 << 0; // back
		p_PID = -p_PID;
		TIM3->CCR1 = p_PID;
	}
	else
	{
		p_PID = 0;
		TIM3->CCR1 = p_PID;
	}




}
