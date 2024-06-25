#include "step.h"
#include "main.h"

extern struct flag Finish_flag ;




extern uint32_t step_pulse_count_tim13;
extern uint16_t step_pulse_count_tim11;

extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim11;

uint16_t shaft_step = 400*1;//400스텝 : 1바퀴   200*24
uint16_t shaft_pulse_cycle = 420; //속도   420

uint16_t ball_screw_step=400*1;
uint16_t ball_screw_pulse_cycle=250;


int step_count_for_flag=0;

uint8_t shaft_accel_flag=0; //0:등속 ,1 가속

uint8_t ball_dir_flag=0; // 0:위, 1: 아래
uint8_t downpart_dir_flag=0; // 0:위, 1: 아래


void shaft_step_motor(uint16_t step1,uint16_t cycle_time1)  // (스텝,주기)  // 다운 파트 스텝모터
{
	if(shaft_accel_flag==0)  //0: 등속
	{
		TIM13->ARR = cycle_time1;
	}

	if(step_pulse_count_tim13 >= step1*2)  //
	{
		step_count_for_flag=0;
		HAL_TIM_OC_Stop_IT(&htim13,TIM_CHANNEL_1);  // 타이머 정지

		if(downpart_dir_flag==0)	 Finish_flag.downpart_high++;     //올라가면
	    else if(downpart_dir_flag==1)     Finish_flag.downpart_low++;   //내려가면
	}
}


void z_axis_step_motor(uint16_t step2,uint16_t cycle_time2)  // (스텝,주기)      // 볼스크류 스텝모터
{

	TIM11->ARR = cycle_time2;

	if(step_pulse_count_tim11 >= step2*2)  //
	{

		HAL_TIM_OC_Stop_IT(&htim11,TIM_CHANNEL_1);  // 타이머 정지

		if(ball_dir_flag==0)	  Finish_flag.ball_high = TRUE;     //올라가면
		else if(ball_dir_flag==1)     Finish_flag.ball_low = TRUE;   //내려가면

	}
}



void z_axis_UP(uint16_t step2,uint16_t cycle_time2)  // (스텝,주기)  //UP = 정방향
{
	ball_dir_flag=0;

	ball_screw_step=step2;
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, 0);  //방향

	step_pulse_count_tim11 = 0;  //펄스 기준값 =0
	HAL_TIM_OC_Start_IT(&htim11, TIM_CHANNEL_1);

}

void z_axis_DOWN(uint16_t step2,uint16_t cycle_time2)  // (스텝,주기)  //DOWN = 역방향
{
	ball_dir_flag=1;

	ball_screw_step=step2;
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, 1);  //방향

	step_pulse_count_tim11 = 0;
	HAL_TIM_OC_Start_IT(&htim11, TIM_CHANNEL_1);


}



void down_part_UP(uint16_t step1,uint16_t cycle_time1)  // (스텝,주기)
{
	shaft_accel_flag=0; //0:등속 ,1 가속
	downpart_dir_flag=0; // 0:위, 1: 아래
	shaft_step = step1;

	step_pulse_count_tim13 = 0;  //펄스 기준값 =0
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, 1);  //방향  //일단 업이라 가정

	HAL_TIM_OC_Start_IT(&htim13, TIM_CHANNEL_1);
}

void down_part_DOWN(uint16_t step1,uint16_t cycle_time1)  // (스텝,주기)
{
	shaft_accel_flag=0; //0:등속 ,1 가속
	downpart_dir_flag=1; // 0:위, 1: 아래
	shaft_step = step1;

	step_pulse_count_tim13 = 0;
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, 0);  //방향

	HAL_TIM_OC_Start_IT(&htim13, TIM_CHANNEL_1);
}

int step_count__flag=0;



int step_count__;       //미스미 스텝: max=400~500,  min=261;
int step_count__max=500;
int step_count__min=261;      //전류 줄이면 190까지 가능 : ofo   , 정격은 240~250  ,260
int count_arr[1000];
//다운스텝은 6000쯤


void down_part_UP_accel(uint16_t step1)  // (스텝,주기)
{
	shaft_accel_flag=1; //0:등속 ,1 가속

	for(step_count__=step_count__min ; step_count__<step_count__max; step_count__++) //1초당 100카운트
	{
		count_arr[step_count__]=step_count__;
	}
	step_count_for_flag=1;

	shaft_step = step1;
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, 1);  //방향
	step_pulse_count_tim13 = 0;  //펄스 기준값 =0
	HAL_TIM_OC_Start_IT(&htim13, TIM_CHANNEL_1);
}

void down_part_DOWN_accel(uint16_t step1)  // (스텝,주기)
{
	shaft_accel_flag=1; //0:등속 ,1 가속

	for(step_count__=step_count__min ; step_count__<step_count__max; step_count__++) //1초당 100카운트
	{
		count_arr[step_count__]=step_count__;
	}
	step_count_for_flag=1;

	shaft_step = step1;
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, 0);  //방향
	step_pulse_count_tim13 = 0;  //펄스 기준값 =0
	HAL_TIM_OC_Start_IT(&htim13, TIM_CHANNEL_1);
}




