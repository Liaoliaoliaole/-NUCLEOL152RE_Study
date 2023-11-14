/*
Create LED-blink (kit internal LED) for frequencies 1 Hz, 10 Hz, 500 Hz with delay_Ms() function
and search User LED pin from Arduino connector and measure period of the signal with oscilloscope.

Some lines has comments and those comments are from STM32L152_reference_manual.pdf
*/

/* Includes */
#include "stm32l1xx.h"
#define HSI_VALUE    ((uint32_t)16000000)
#include "nucleo152start.h"

/* Private typedef */
/* Private define  */
/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */

void delay_Ms(int delay);
/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void)
{

  /* Configure the system clock to 32 MHz and update SystemCoreClock */
  SetSysClock();
  SystemCoreClockUpdate();

  /* TODO - Add your application code here */

  RCC->AHBENR|=1; //GPIOA ABH bus clock ON. p154
  GPIOA->MODER|=0x400; //MODER5 01: General purpose output mode. p184

  /* Infinite loop */
  while (1)
  {
	  //1Hz
	  GPIOA->ODR|=0x20; //0010 0000 set bit 5,ODR5: Port output data. p186
	  delay_Ms(500);
	  GPIOA->ODR&=~0x20; //0000 0000 clear bit 5. p186
	  delay_Ms(500);

	  //10Hz
	  GPIOA->ODR|=0x20;
	  delay_Ms(50);
	  GPIOA->ODR&=~0x20;
	  delay_Ms(50);

	  //500Hz
	  GPIOA->ODR|=0x20;
	  delay_Ms(1);
	  GPIOA->ODR&=~0x20;
	  delay_Ms(1);
  }
  return 0;
}

void delay_Ms(int delay)
{
	int i=0;
	for(; delay>0;delay--)
		for(i=0;i<2460;i++); //measured with oscilloscope
}
