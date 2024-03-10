#include "sensor.h"
#include "main.h"


#define TRIG_PIN GPIO_PIN_7
#define TRIG_PORT GPIOD
#define ECHO_PIN GPIO_PIN_6
#define ECHO_PORT GPIOD
uint32_t pMillis;
uint32_t Value1 = 0;
uint32_t Value2 = 0;
float Distance  = 0;  // cm
float Distance_Sum=0;
float Distance_Avg;
uint16_t j=0;

extern TIM_HandleTypeDef htim1;

void distance_sencor (void)
{
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	__HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER (&htim1) < 10);  // wait for 10 us
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

    pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
    // wait for the echo pin to go high
    while (!(HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 10 >  HAL_GetTick());
    Value1 = __HAL_TIM_GET_COUNTER (&htim1);

    pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
    // wait for the echo pin to go low
    while ((HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 50 > HAL_GetTick());
    Value2 = __HAL_TIM_GET_COUNTER (&htim1);

    Distance = (Value2-Value1)* 0.017/2;

    Distance_Sum = Distance + Distance_Sum;
    	  j++;
    	 	 if(j==200)
    	 	  {
    	 		Distance_Avg = Distance_Sum/201;

    	 			  j=0;
    	 			 Distance_Sum=0;
    	 	  }



    //     printf("Distance = %f \r\n",Distance);
	//      sprintf((char *)buffer, "%d \r\n",Distance);
		 //     HAL_UART_Transmit(&huart3, buffer, strlen((char *)buffer), 100);

}
