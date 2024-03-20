#include "com.h"
#include "main.h"


int position_start=0;
float speed_input;
uint8_t key_flag;
extern uint32_t encoder_count;
extern uint8_t key_value;
extern float input_position;
extern float err_sum_s;
extern float err_sum;
extern uint8_t data;
extern int8_t RX_flag;

extern float input_speed;

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart3;

uint8_t rev10_flag;
uint8_t rev1_flag;

//extern int us;
extern uint16_t step_count;
int step_enable=0;

PUTCHAR_PROTOTYPE  //테라텀
{
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

int _write(int file, char* p, int len){        //테라텀
	HAL_UART_Transmit(&huart3, p, len, 10);
	return len;
}









void SerialSendChar_PC(uint8_t Ch1) // 1문자 보내기 함수
{
	// USART_SR_TXE(1<<7)=0?, TX Buffer NOT Empty?
	// TX buffer Empty되지 않으면 계속 대기(송신 가능한 상태까지 대기)
	while((USART3->SR & 1<<7) == RESET);
	USART3->DR = (Ch1 & 0x01FF);	// 전송 (최대 9bit 이므로 0x01FF과 masking)
}

void SerialSendChar_WIFI(uint8_t Ch2) // 1문자 보내기 함수
{
	while((UART4->SR & 1<<7) == RESET);
	UART4->DR = (Ch2 & 0x01FF);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	encoder_count=TIM4->CNT;

  if(huart->Instance == USART3)
  {

	  HAL_UART_Receive_IT(&huart3, &key_value, 1);
	  printf("%d,^^ %c \r\n", key_value,key_value);

	  switch(key_value)
	  	{
	  		case 'q':  // forward
	  			GPIOE->ODR |= 1<<0;  // back
	  	  	 	GPIOB->ODR |= 1<<0;  //LD1

	  			break;

	  		case 'w':  // back
	  			GPIOE->ODR &= ~1<<0;  // go
	  		    GPIOB->ODR &= ~1<<0;  //LD1

	  			break;

	  		case '1':
	  			GPIOG->ODR |= 1<<0;  //stop

	  			break;

	  		case '2':
	  			  GPIOG->ODR &= ~1<<0;  //start

	  			break;

	  		case '3':
	  			TIM3->CCR1 = 50;
	  		//	TIM3->CCR2 = 50;

	  			break;

	  		case '4':
	  			TIM3->CCR1 = 100;
	  	  		//	TIM3->CCR2 = 100;

	  			break;

	  		case '5':
	  			TIM3->CCR1 = 500;
	  	  	  	//	TIM3->CCR2 = 500;

	  			break;

	  		case '6':
	  			TIM3->CCR1 = 800;
	  	  	  	//	TIM3->CCR2 = 800;
	  			break;

	  		case '7':
	  			TIM3->CCR1 = 1000;
	  	  	//  		TIM3->CCR2 = 1000;

	  			break;

	  		case 'a':
	  			  TIM4->CNT=10000;

	  		  key_flag=1;

	  			break;


	  		case 'u':
	  			if(position_start==0)
	  			{
	  				position_start=1;
	  			}
	  			else if(position_start==1)
	  		    {
	  				position_start=0;

	  			}
	  			  			 break;
	  			case 'i':
	  			      TIM4->CNT=10000;
	  			    input_position=10000+500*1;
	  			  	  err_sum=0;
	  			  	err_sum_s=0;
	  			 break;
	  			case 'o':
	  				  TIM4->CNT=10000;
	  				input_position=10000+512*2;
	  				  err_sum=0;
	  				err_sum_s=0;
	  			break;
	  			case 'p':
	  				   TIM4->CNT=10000;
	  				 input_position=10000-500*1;
	  			 	   err_sum=0;
	  			 	err_sum_s=0;
	  			 break;

	  			case 'b':

	  				if(step_enable==0)
	     			{
	  					step_enable=1;
	  			 	}
	  				else if(step_enable==1)
	  			   {
	  					step_enable=0;
                    }
	  			  break;
	  			case 'n':
	  			 HAL_GPIO_WritePin(GPIOF,  GPIO_PIN_13, 1);  //정지

	  				  	break;

	  			case 'm':
	  				 HAL_GPIO_WritePin(GPIOF,  GPIO_PIN_13, 0);  //출발

	  		  	break;
	  			case ',':
	  				 HAL_GPIO_WritePin(GPIOF,  GPIO_PIN_14, 0);  //방향

	  				  	break;
	  			case '.':
	  				 HAL_GPIO_WritePin(GPIOF,  GPIO_PIN_14, 1);  //방향

	  				   break;
	  			case '/': //
	  				rev10_flag=1;
	  				step_count=0;
	  				HAL_GPIO_WritePin(GPIOF,  GPIO_PIN_13, 0);  //출발

	  				   break;
	  			case '?': //
	  		     	rev1_flag=1;
	  				step_count=0;
	  				HAL_GPIO_WritePin(GPIOF,  GPIO_PIN_13, 0);  //출발
             		   break;


	  		case 'j':
	  			 TIM4->CNT=10000;
	  			input_speed=20;
	  		     err_sum_s=0;
	  			   break;
	  	 	case 'k': //
	  	 	     TIM4->CNT=10000;
	  	 	  input_speed=30;
	  	     	  err_sum_s=0;
	  	          break;
	  		case 'l': //
	  			 TIM4->CNT=10000;
	  			input_speed=40;
	  			 err_sum_s=0;
	  			   break;


	  }





  }
  if(huart->Instance == UART4)
  {
	  HAL_UART_Receive_IT(&huart4, &data, 1);

	  printf("%d -- %c \r\n", data,data);
		switch(data)
		{
		case '0':
			 GPIOB->ODR |= 1<<0;
			GPIOG->ODR &= ~1<<0;// start

			break;

		case '1':  //stop
			  GPIOB->ODR &= ~1<<0;
	          GPIOG->ODR |= 1<<0;
			break;

		case '2':  //dir
			  GPIOE->ODR |= 1<<0;  // back
			break;

		case '3'://dir
			 GPIOE->ODR &= ~1<<0; // go
			break;

		case '4':

		    TIM3->CCR1 = 0;
			TIM3->CCR2 = 0;

			break;

		case '5':
			TIM3->CCR1 = 50;
			TIM3->CCR2 = 50;
			break;

		case '6':
			TIM3->CCR1 = 200;
			TIM3->CCR2 = 200;
			break;

		case '7':
			TIM3->CCR1 = 1000;
			TIM3->CCR2 = 1000;
			break;

		case '!':
			RX_flag=1;
			 TIM2->CNT=0;
			 TIM4->CNT=0;
			break;
		case 'A':  //Tx

			RX_flag=2;
				break;
		}



    }

}
