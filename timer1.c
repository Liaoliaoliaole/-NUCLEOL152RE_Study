/*
 Create accurate 1-second pulse with 32-bit timer 2 (TIM5) to LD2 GPIO pin.
 Take oscilloscope screen to Spreadsheet and measure frequency.

 Create your own 10 µs delay function with 16-bit timer 6 (TIM6).
 Create 50 kHz signal by using your own delay function and take oscilloscope figure to spreadsheet.

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

void delay_ms(unsigned long delay);
void delay_s(unsigned long delay);
void delay_10us(unsigned long delay);
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

  RCC->AHBENR|=1; 		//GPIOA ABH bus clock ON. p154
  GPIOA->MODER|=0x400; 	//GPIOA pin 5 to output. p184

  /* Infinite loop */
  while (1)
  {
	  GPIOA->ODR^=0x20; //0010 0000 xor bit 5. p186
	  //delay_ms(1000);
	  //delay_s(3);
	  delay_10us(1);
  }
  return 0;
}

void delay_ms(unsigned long delay)
{
	unsigned long i=0;
	RCC->APB2ENR|=(1<<4); 	//TIM11EN: Timer 11 clock enable. p156
	TIM11->PSC=32-1; 		//32 000 000 MHz / 32 = 1 000 000 Hz. p435,The counter clock frequency CK_CNT is equal to fCK_PSC / (PSC[15:0] + 1).
	TIM11->ARR=1000-1; 		//TIM11 counter. 1 000 000 Hz / 1000 = 1000 Hz ~ 1ms. p435,Auto-reload value'	ARR is the value to be loaded in the actual auto-reload register.
	TIM11->CNT=0;			//counter start value = 0
	TIM11->CR1=1; 			//TIM11 Counter enabled. p421

	  while(i<delay)
	  {
		  while(!((TIM11->SR)&1)){} //Update interrupt flag. p427
		  i++;
		  TIM11->SR &= ~1; 	//flag cleared. p427
		  TIM11->CNT=0;	  	//counter start value = 0
	  }
	  TIM11->CR1=0; 		//TIM11 Counter disabled. p421
}

void delay_s(unsigned long delay)
{
	unsigned long i=0;
	RCC->APB1ENR|=(1<<3); 	//TIM5EN: Timer 5 clock enable.P160
	TIM5->PSC = 32000 - 1;  // 32 MHz / 32000 = 1000 Hz.
	TIM5->ARR = 1000 - 1;   // Auto-reload value: 1000 cycles = 1 second.
	TIM5->CNT = 0;	//counter start value = 0
	TIM5->CR1 = 1; 			//TIM5 Counter enabled. p421

	  while(i<delay)
	  {
		  while(!((TIM5->SR)&1)){} //Update interrupt flag. p427
		  i++;
		  TIM5->SR &= ~1; 	//flag cleared. p427
		  TIM5->CNT=0;	  	//counter start value = 0
	  }
	  TIM5->CR1=0; 		//TIM11 Counter disabled. p421
}

void delay_10us(unsigned long delay)
{
	unsigned long i=0;
	RCC->APB1ENR|=(1<<4); 	//TIM5EN: Timer 5 clock enable.P160
	TIM6->PSC = 32 - 1;
	TIM6->ARR = 10 - 1;
	TIM6->CNT = 0;
	TIM6->CR1 = 1;

	  while(i<delay)
	  {
		  while(!((TIM6->SR)&1)){} //Update interrupt flag. p427
		  i++;
		  TIM6->SR &= ~1; 	//flag cleared. p427
		  TIM6->CNT=0;	  	//counter start value = 0
	  }
	  TIM6->CR1=0; 		//TIM11 Counter disabled. p421
}
