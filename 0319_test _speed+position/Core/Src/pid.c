#include "pid.h"
#include "main.h"
#include "com.h"

extern uint32_t encoder_count;
float input_speed;
float input_position;

float err;
double p_PID;

float p_kp = 20;
float p_ki;
float p_kd;
float s_kp = 2.5;
float s_ki = 0;
float s_kd;

float P_KP;
float P_KI;

double P_KD;
double err_d;
float previous_err;

float err_s;
float err_sum_s;
float err_sum;
double s_PID;

float S_KP;
float S_KI;

double S_KD;
double err_d_s;
float previous_err_s;

float input_duty;

extern float RPM;
extern float RPM_Avg;

void position_pid(void) {

//		p_kp = 10;
	//	p_ki = 1;
	//	p_kd = 0.02;

	err = input_position - TIM4->CNT;
	P_KP = err * p_kp;

	err_sum = err_sum + err * 0.001;
	if (p_PID == 0) {
		err_sum = 0;
	}
	P_KI = err_sum * p_ki;

	err_d = (err - previous_err) / 0.001;   //1ms
	previous_err = err;
	P_KD = err_d * p_kd;

	p_PID = P_KP + P_KI + P_KD;

	if (p_PID >= 800) {
		p_PID = 800;
	} else if (p_PID <= -800) {
		p_PID = -800;
	}

//////////////////////////////////////속도제어기
//	s_kp = 50;
//	s_ki = 5;
//  s_kd = 0;

	input_speed = p_PID;
	err_s = input_speed - RPM / 60 * 512;
	S_KP = err_s * s_kp;

	//y = 0.05e0.158x
	input_duty = 0.05 * pow(2.71828, 0.158 * input_speed);

	err_sum_s = err_sum_s + err_s * 0.001;
	if (s_PID == 0) {
		err_sum_s = 0;
	}
	S_KI = err_sum_s * s_ki;

	err_d_s = (err_s - previous_err_s) / 0.001;
	previous_err_s = err_s;
	S_KD = err_d_s * s_kd;

	s_PID = S_KP + S_KI + S_KD;

	/*	if (s_PID >= 400)
	 {
	 s_PID = 400;
	 }
	 else if (s_PID <= -400)
	 {
	 s_PID = -400;
	 }

	 */
	/*
	 if (s_PID > 0)
	 {
	 GPIOE->ODR &= ~1 << 0; // go
	 TIM3->CCR1 = s_PID + input_duty;
	 }
	 else if (s_PID < 0)
	 {
	 //	GPIOE->ODR |= 1 << 0; // back
	 //	p_PID = -p_PID;
	 TIM3->CCR1 = s_PID + input_duty;
	 }
	 else
	 {
	 s_PID = 0;
	 TIM3->CCR1 = s_PID + input_duty;
	 }

	 */

	if (s_PID > 0) {
		GPIOE->ODR &= ~1 << 0; // go
		TIM3->CCR1 = s_PID;
	} else if (s_PID < 0) {
		GPIOE->ODR |= 1 << 0; // back
		s_PID = -s_PID;
		TIM3->CCR1 = s_PID;
	} else {
		s_PID = 0;
		TIM3->CCR1 = s_PID;
	}

}
