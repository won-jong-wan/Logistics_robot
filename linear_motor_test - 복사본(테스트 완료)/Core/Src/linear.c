#include "linear.h"
#include "main.h"

uint8_t linear_count = 0;
void linear_motor_GO(float time)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);  //방향  전진
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, 0);  //출발
	TIM3->CCR4=8200;

}

void linear_motor_BACK(float time)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1);   //방향 후진
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, 0);  //출발
	TIM3->CCR4=8200;

}

void linear_time_count_GO(float time)
{
	if((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == 0) && (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_4) == 0))
		{ //리니어      전진  && 출발

				linear_count++;
				if(linear_count==100*time)  //time=1 이면 1초
				{
					linear_count=0;
					HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, 1); //정지
					TIM3->CCR4=0;
				}
		}
}

void linear_time_count_BACK(float time)
{
	if((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == 1) && (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_4) == 0))
		{ //리니어      후진  && 출발

				linear_count++;

				if(linear_count== (uint8_t)100.0*time )  //time=1 이면 1초
				{
					linear_count=0;
					HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, 1); //정지
					TIM3->CCR4=0;
				}
		}
}
