/*
Create your own 500 µs delay function.
Create pulse for Arduino connector D12 (not led pin) for frequency 1 kHz
and measure period of the signal with oscilloscope.

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

void delay_500us(int delay);
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
  GPIOA->MODER|=0x1000; //MODER5 01: General purpose output mode. p184

  /* Infinite loop */
  while (1)
  {
	  //500Hz
	  GPIOA->ODR|=1<<6;
	  delay_500us(1);
	  GPIOA->ODR&=~1<<6;
	  delay_500us(1);
  }
  return 0;
}

void delay_500us(int delay)
{
	int i=0;
	for(; delay>0;delay--)
		for(i=0;i<1230;i++); //measured with oscilloscope
}