#include <stdint.h>
//
//#include "sensor.h"
//#include "pid.h"
//#include "step.h"

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif


void SerialSendChar_PC(uint8_t Ch1);
void SerialSendChar_ESP(uint8_t Ch2);

void UART3_RX_PC_to_STM32(void);
void UART4_RX_ESP_to_STM32(void);
void STM32_to_ESP(void);
