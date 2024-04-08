#include <stdint.h>

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif


void SerialSendChar_PC(uint8_t Ch1);
void SerialSendChar_WIFI(uint8_t Ch2);
