#include "pid.h"
#include "main.h"
#include "com.h"

//extern uint32_t encoder_count;


float err_sum_s_Y;
float err_sum_Y;
float input_position_Y;







///////////
float input_speed_X;
float input_position_X;

float input_current;

float err_X;


float p_kp_X = 1000;
float p_ki_X;
float p_kd_X = 10;

float s_kp_X = 1.2;
float s_ki_X = 0;
float s_kd_X;

float c_kp = 1;
float c_ki = 0;
float c_kd;

float P_KP_X;
float P_KI_X;
double P_KD_X;
double p_PID_X;
double err_d_X;
float previous_err_X;


double err_d_s_X;
float previous_err_s_X;

float err_s_X;
float err_sum_s_X;
float err_sum_X;
float S_KP_X;
float S_KI_X;
double S_KD_X;
double s_PID_X;


float err_c;
float err_sum_c;

float C_KP;
float C_KI;
double C_KD;
double c_PID;

double err_d_c;
float previous_err_c;

float input_duty;

extern float RPM_X;
extern float RPM_Avg;
extern float current_A;

float speed_sensor_X;
float speed_input_X;

float current_sensor;
float current_input;

extern float current_A_floor;

float errorgap_p_X;
float errorgap_s_X;
float errorgap_c;



#define SOURCE 5  //1 위치 속도  ,  2 위치속도전류,   3 위치, 4위치 다른방식(가능)
//5: 위치(4)+속도
//6: 위치4+속도5+전류
void position_pid_x(void) {
#if SOURCE == 1


	//////////////////////////////////위치

//		p_kp = 10;
	//	p_ki = 1;
	//	p_kd = 0.02;

	err = input_position_X - TIM2->CNT;
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

	if (p_PID >= 500) {
		p_PID = 500;
	} else if (p_PID <= -500) {
		p_PID = -500;
	}

//////////////////////////////////////속도제어기
//	s_kp = 50;
//	s_ki = 5;
//  s_kd = 0;

	input_speed = p_PID;
	err_s = input_speed - RPM_X / 60 * 512;
	S_KP = err_s * s_kp;

	//y = 0.05e0.158x
//	input_duty = 0.05 * pow(2.71828, 0.158 * input_speed);

	err_sum_s = err_sum_s + err_s * 0.001;
	if (s_PID == 0) {
		err_sum_s = 0;
	}
	S_KI = err_sum_s * s_ki;

	err_d_s = (err_s - previous_err_s) / 0.001;
	previous_err_s = err_s;
	S_KD = err_d_s * s_kd;

	s_PID = S_KP + S_KI + S_KD;

		if (s_PID >= 600)
	 {
	 s_PID = 600;
	 }
	 else if (s_PID <= -600)
	 {
	 s_PID = -600;
	 }


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


#elif SOURCE == 2

	//////////////////////////////////위치
	//		p_kp = 10;
	//	p_ki = 1;
	//	p_kd = 0.02;

	err = input_position_X - TIM2->CNT;
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

	if (p_PID >= 700) {
		p_PID = 700;
	} else if (p_PID <= -700) {
		p_PID = -700;
	}

	//////////////////////////////////////속도제어기
	//	s_kp = 50;
	//	s_ki = 5;
	//  s_kd = 0;

	input_speed = p_PID;
	err_s = input_speed - RPM_X / 60 * 512;
	S_KP = err_s * s_kp;

	err_sum_s = err_sum_s + err_s * 0.001;
	if (s_PID == 0) {
		err_sum_s = 0;
	}
	S_KI = err_sum_s * s_ki;

	err_d_s = (err_s - previous_err_s) / 0.001;
	previous_err_s = err_s;
	S_KD = err_d_s * s_kd;

	s_PID = S_KP + S_KI + S_KD;

	/*		if (s_PID >= 400)
	 {
	 s_PID = 400;
	 }
	 else if (s_PID <= -400)
	 {
	 s_PID = -400;
	 }
	 */
/*
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
*/
	///////////////////////////////////////전류

	//	c_kp = ;
	//	c_ki = ;
	//  c_kd = ;

	input_current = s_PID;
	err_c = input_current - current_A * 512 ;
	C_KP = err_c * c_kp;

	err_sum_c = err_sum_c + err_c * 0.001;
	if (c_PID == 0) {
		err_sum_c = 0;
	}
	C_KI = err_sum_c * c_ki;

	err_d_c = (err_c - previous_err_c) / 0.001;
	previous_err_s = err_c;
	C_KD = err_d_c * c_kd;

	c_PID = C_KP + C_KI + C_KD;

		if (c_PID >= 400)
	 {
			c_PID = 400;
	 }
	 else if (c_PID <= -400)
	 {
		 c_PID = -400;
	 }


	if (c_PID > 0) {
		GPIOE->ODR &= ~1 << 0; // go
		TIM3->CCR1 = c_PID;
	} else if (c_PID < 0) {
		GPIOE->ODR |= 1 << 0; // back
		c_PID = -c_PID;
		TIM3->CCR1 = c_PID;
	} else {
		c_PID = 0;
		TIM3->CCR1 = c_PID;
	}



#elif SOURCE == 3
	//////////////////////////////////위치

	//		p_kp = 10;
		//	p_ki = 1;
		//	p_kd = 0.02;

		err = input_position_X - TIM2->CNT;
		P_KP = err * p_kp;

		err_sum = err_sum + err * 0.0001;
//		if (p_PID == 0) {
//			err_sum = 0;
//		}
		P_KI = err_sum * p_ki;

		err_d = (err - previous_err) / 0.0001;   //1ms
		previous_err = err;
		P_KD = err_d * p_kd;

		p_PID = P_KP + P_KI + P_KD;

		if (p_PID >= 500) {
			p_PID = 500;
		} else if (p_PID <= -500) {
			p_PID = -500;
		}


		if (p_PID > 0) {
			GPIOE->ODR &= ~1 << 0; // go
			TIM3->CCR1 = p_PID;
		} else if (p_PID < 0) {
			GPIOE->ODR |= 1 << 0; // back
			p_PID = -p_PID;
			TIM3->CCR1 = p_PID;
		} else {
			p_PID = 0;
			TIM3->CCR1 = p_PID;
		}


#elif SOURCE == 4
	//////////////////////////////////위치

	//		p_kp = 10;
		//	p_ki = 1;
		//	p_kd = 0.02;


//참고	  errorGap = target - current - realError;
//		realError = target - current;	// 실시간 에러는 단순히 목표값 - 현재값을 의미합니다.
//	    accError += realError;

	    errorgap_p= input_position_X - TIM2->CNT- err;
     	err = input_position_X - TIM2->CNT;
		err_sum += err * 0.001;


		P_KP = err * p_kp;
		P_KI = err_sum * p_ki;

		if (p_PID == 0) {
			err_sum = 0;
		}



	//	err_d = (err - previous_err) / 0.0001;   //1ms
	//	previous_err = err;

		P_KD =  p_kd * errorgap_p/0.001;

		p_PID = P_KP + P_KI + P_KD;

		if (p_PID >= 4000) {
			p_PID = 4000;
		} else if (p_PID <= -4000) {
			p_PID = -4000;
		}


		if (p_PID > 0) {
			GPIOE->ODR &= ~1 << 0; // go
			TIM3->CCR1 = p_PID;
		} else if (p_PID < 0) {
			GPIOE->ODR |= 1 << 0; // back
			p_PID = -p_PID;
			TIM3->CCR1 = p_PID;
		} else {
			p_PID = 0;
			TIM3->CCR1 = p_PID;
		}




#elif SOURCE == 5
		//////////////////////////////////위치

			//		p_kp = 250;
			//	p_ki = 0;
			//	p_kd = 15;
			//		s_kp = 2;
			//	s_ki = 0;
			//	s_kd = 0;

		//참고	  errorGap = target - current - realError;
		//		realError = target - current;	// 실시간 에러는 단순히 목표값 - 현재값을 의미합니다.
		//	    accError += realError;

			errorgap_p_X = input_position_X - TIM2->CNT - err_X;
			err_X = input_position_X - TIM2->CNT;
			err_sum_X += err_X * 0.001;

			if (p_PID_X == 0) {
				err_sum_X = 0;
			}

			P_KP_X = err_X * p_kp_X;
			P_KI_X = err_sum_X * p_ki_X;

			//	err_d_X = (err_X - previous_err_X) / 0.0001;   //1ms
			//	previous_err_X = err_X;

			P_KD_X = p_kd_X * errorgap_p_X / 0.001;

			p_PID_X = P_KP_X + P_KI_X + P_KD_X;

			if (p_PID_X >= 12000) {
				p_PID_X = 12000;
			} else if (p_PID_X <= -12000) {
				p_PID_X = -12000;
			}
		/////////////////////////////////////속도

			//참고	  errorGap = target - current - realError;
			//		realError = target - current;	// 실시간 에러는 단순히 목표값 - 현재값을 의미합니다.
			//	    accError += realError;

			speed_sensor_X = (RPM_X / 60 * 34 * 512);
			speed_input_X = p_PID_X;
			errorgap_s_X = speed_input_X - speed_sensor_X - err_s_X;
			err_s_X = speed_input_X - speed_sensor_X;
			err_sum_s_X += err_s_X * 0.001;

			if (s_PID_X == 0) {
				err_sum_s_X = 0;
			}
			S_KP_X = err_s_X * s_kp_X;
			S_KI_X = err_sum_s_X * s_ki_X;

			//	err_d = (err - previous_err) / 0.0001;   //1ms
			//	previous_err = err;

			S_KD_X = s_kd_X * errorgap_s_X / 0.001;

			s_PID_X = S_KP_X + S_KI_X + S_KD_X;

			if (s_PID_X >= 6000) {
				s_PID_X = 6000;
			} else if (s_PID_X <= -6000) {
				s_PID_X = -6000;
			}

			if (s_PID_X > 0) {
				GPIOE->ODR &= ~1 << 0; // go
				TIM3->CCR1 = s_PID_X;
			} else if (s_PID_X < 0) {
				GPIOE->ODR |= 1 << 0; // back
				s_PID_X = -s_PID_X;
				TIM3->CCR1 = s_PID_X;
			} else {
				s_PID_X = 0;
				TIM3->CCR1 = s_PID_X;
			}




#elif SOURCE == 6
	//////////////////////////////////위치

	//		p_kp = 10;
	//	p_ki = 1;
	//	p_kd = 0.02;

//참고	  errorGap = target - current - realError;
//		realError = target - current;	// 실시간 에러는 단순히 목표값 - 현재값을 의미합니다.
//	    accError += realError;

	errorgap_p = input_position_X - TIM2->CNT - err;
	err = input_position_X - TIM2->CNT;
	err_sum += err * 0.001;

	if (p_PID == 0) {
		err_sum = 0;
	}

	P_KP = err * p_kp;
	P_KI = err_sum * p_ki;

	//	err_d = (err - previous_err) / 0.0001;   //1ms
	//	previous_err = err;

	P_KD = p_kd * errorgap_p / 0.001;

	p_PID = P_KP + P_KI + P_KD;

	if (p_PID >= 12000) {
		p_PID = 12000;
	} else if (p_PID <= -12000) {
		p_PID = -12000;
	}
/////////////////////////////////////속도

	//		s_kp = 10;
	//	s_ki = 1;
	//	s_kd = 0.02;

	//참고	  errorGap = target - current - realError;
	//		realError = target - current;	// 실시간 에러는 단순히 목표값 - 현재값을 의미합니다.
	//	    accError += realError;

	speed_sensor = (RPM_X / 60 * 34 * 512);
	speed_input = p_PID;
	errorgap_s = speed_input - speed_sensor - err_s;
	err_s = speed_input - speed_sensor;
	err_sum_s += err_s * 0.001;

	if (s_PID == 0) {
		err_sum_s = 0;
	}
	S_KP = err_s * s_kp;
	S_KI = err_sum_s * s_ki;

	//	err_d = (err - previous_err) / 0.0001;   //1ms
	//	previous_err = err;

	S_KD = s_kd * errorgap_s / 0.001;

	s_PID = S_KP + S_KI + S_KD;

	 if (s_PID >= 3000) {
	 s_PID = 3000;
	 } else if (s_PID <= -3000) {
	 s_PID = -3000;
	 }



////////////////////////////    전류

	//		c_kp = ;
	//	c_ki = ;
	//	c_kd = ;

	//참고	  errorGap = target - current - realError;
	//		realError = target - current;	// 실시간 에러는 단순히 목표값 - 현재값을 의미합니다.
	//	    accError += realError;
	current_sensor = (current_A_floor/(3.14) * 34 * 512);
	current_input = s_PID;
	errorgap_c = current_input - current_sensor - err_c;
	err_c = current_input - current_sensor;
	err_sum_c += err_c * 0.001;

	if (c_PID == 0) {
		err_sum_c = 0;
	}
	C_KP = err_c * c_kp;
	C_KI = err_sum_c * c_ki;

	//	err_d = (err - previous_err) / 0.0001;   //1ms
	//	previous_err = err;

	C_KD = c_kd * errorgap_c / 0.001;

	c_PID = C_KP + C_KI + C_KD;



	 if (c_PID >= 8000) {
	 c_PID = 8000;
	 } else if (c_PID <= -8000) {
	 c_PID = -8000;
	 }



	if (c_PID > 40) {
		GPIOE->ODR &= ~1 << 0; // go
		TIM3->CCR1 = c_PID;
	} else if (c_PID < 40) {
		GPIOE->ODR |= 1 << 0; // back
		c_PID = -c_PID;
		TIM3->CCR1 = c_PID;
	} else {
		c_PID = 0;
		TIM3->CCR1 = c_PID;
	}

#endif
}



#define SOURCE2 11
void position_pid_y(void) {   //일단은 rpm은 tim5로 해야되지만 기존의 tim2로 사용함 ,,


#if SOURCE2 == 10

	//변수만 바꿨을 떄 되는지

	//////////////////////////////////위치

		//		p_kp = 250;
		//	p_ki = 0;
		//	p_kd = 15;
		//		s_kp = 2;
		//	s_ki = 0;
		//	s_kd = 0;

	//참고	  errorGap = target - current - realError;
	//		realError = target - current;	// 실시간 에러는 단순히 목표값 - 현재값을 의미합니다.
	//	    accError += realError;

		errorgap_p_X = input_position_X - TIM5->CNT - err_X;
		err_X = input_position_X - TIM5->CNT;
		err_sum_X += err_X * 0.001;

		if (p_PID_X == 0) {
			err_sum_X = 0;
		}

		P_KP_X = err_X * p_kp_X;
		P_KI_X = err_sum_X * p_ki_X;

		//	err_d_X = (err_X - previous_err_X) / 0.0001;   //1ms
		//	previous_err_X = err_X;

		P_KD_X = p_kd_X * errorgap_p_X / 0.001;

		p_PID_X = P_KP_X + P_KI_X + P_KD_X;

		if (p_PID_X >= 12000) {
			p_PID_X = 12000;
		} else if (p_PID_X <= -12000) {
			p_PID_X = -12000;
		}
	/////////////////////////////////////속도

		//참고	  errorGap = target - current - realError;
		//		realError = target - current;	// 실시간 에러는 단순히 목표값 - 현재값을 의미합니다.
		//	    accError += realError;

		speed_sensor_X = (RPM_X / 60 * 34 * 512);
		speed_input_X = p_PID_X;
		errorgap_s_X = speed_input_X - speed_sensor_X - err_s_X;
		err_s_X = speed_input_X - speed_sensor_X;
		err_sum_s_X += err_s_X * 0.001;

		if (s_PID_X == 0) {
			err_sum_s_X = 0;
		}
		S_KP_X = err_s_X * s_kp_X;
		S_KI_X = err_sum_s_X * s_ki_X;

		//	err_d = (err - previous_err) / 0.0001;   //1ms
		//	previous_err = err;

		S_KD_X = s_kd_X * errorgap_s_X / 0.001;

		s_PID_X = S_KP_X + S_KI_X + S_KD_X;

		if (s_PID_X >= 6000) {
			s_PID_X = 6000;
		} else if (s_PID_X <= -6000) {
			s_PID_X = -6000;
		}

		if (s_PID_X > 0) {
			GPIOE->ODR &= ~1 << 0; // go
			TIM3->CCR2 = s_PID_X;
		} else if (s_PID_X < 0) {
			GPIOE->ODR |= 1 << 0; // back
			s_PID_X = -s_PID_X;
			TIM3->CCR2 = s_PID_X;
		} else {
			s_PID_X = 0;
			TIM3->CCR2 = s_PID_X;
		}

#endif
}

