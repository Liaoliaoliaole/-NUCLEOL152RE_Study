/*
Create HC-SR04 sensor reader with timer/interrupt.
Use ARM external interrupt EXTI6 (page 222, reference manual and function void EXTI9_5_IRQHandler(void))
Arduino connector D12 (HC-SR04: Echo Pulse Output).
External interrupt EXT6 will start and stop timer (TIM4).

Use Arduino connector D11 (HC-SR04: Trigger Input) as a physical pin for start pulse
and use ARM peripheral timer (TIM6) to create start pulse with aid of chapter 3.3 code.

Use timer (TIM3) to start distance measurement one second interval.
You need to send distance to terminal and your distance range is 10 cm to 100 cm

and when you use timer (TIM4) make sure that you calculate pre-scaler so that your timer 4 increments every 1 µs.
There must be 1 mm resolution in terminal.

*/

/* Includes */

#include "stm32l1xx.h"
#define HSI_VALUE    ((uint32_t)16000000)
#include "nucleo152start.h"
#include <stdio.h>
#include <stdlib.h>

/* Private typedef */
/* Private define  */
/* Private variables */
int pulse_start_time = 0;
int pulse_end_time = 0;

/* Private function prototypes */

void EXTI6_Configuration(void);
void USART2_Init(void);
void USART2_write(char data);
void delay_s(unsigned long delay);//TIM3
void delay_10us(unsigned long delay);//TIM6

/* Private functions */
/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void)
{
	__disable_irq();
	/* Configure the system clock to 32 MHz and update SystemCoreClock */
  SetSysClock();
  SystemCoreClockUpdate();
  USART2_Init();

  //Set output pin, pin7
  RCC->AHBENR |= 1;
  GPIOA->MODER |= 0X4000;
  GPIOA->MODER |= 0X400;

  EXTI6_Configuration();

  __enable_irq();



  //char debug = 'g';

  /* Infinite loop */
  while (1)
  {
	  /* Generate trigger pulse */
	GPIOA->ODR |= 0X80; // Set PA11 high (HC-SR04 trigger)
	GPIOA->ODR |= 0X20;
	delay_10us(1);
	GPIOA->ODR &= ~0X80; // Set PA11 low (HC-SR04 trigger)
	GPIOA->ODR &= ~0X20;
	//USART2_write(debug);

	/* Calculate and display the distance in millimeters */
	float distance_value=(pulse_end_time - pulse_start_time) *10 / 58;
	int temp_cm=(int)distance_value/10;
	int temp_mm=(int)distance_value%10;
	char distance_buf[100];
	int len = sprintf(distance_buf, "Distance: %d.%d cm \n\r", temp_cm, temp_mm);
	for (int i = 0; i < len; i++)
	{
		USART2_write(distance_buf[i]);
	}

	/* Delay before the next measurement (1 second) */
	delay_s(1);
  }
  return 0;
}


void EXTI6_Configuration(void)
{
	RCC->APB2ENR |= 1; // Enable SYSCFG clock
	GPIOA->MODER &= ~0X3000; //set input for PA6
	SYSCFG->EXTICR[1] &= ~0xf00; // Connect EXTI6 to PA6
	EXTI->IMR |= (1<<6); // Enable EXTI6 interrupt
	EXTI->RTSR |= (1<<6); // Trigger on rising edge
	EXTI->FTSR |= (1<<6); // Trigger on falling edge
	NVIC_EnableIRQ(EXTI9_5_IRQn); // Enable EXTI9_5_IRQn in NVIC
}

void EXTI9_5_IRQHandler(void)
{
	if ((GPIOA->IDR & (1<<6)) == (1<<6)) //rising edge: echo start time
	{
		RCC->APB1ENR |= (1<<2); // Enable TIM4 clock
		TIM4->PSC = (32000000 / 1000000) - 1; // 1 µs resolution,The ARR setting is used in timers when trigger an event based on a specific count, but in this case, you just want to read the current count in 1us interval.
		TIM4->CNT = 0;
		TIM4->CR1 |= 1; // Enable the timer
		pulse_start_time = TIM4->CNT;
	}
	else //Falling edge: echo end time
	{
		pulse_end_time = TIM4->CNT;
		TIM4->CR1 = 0;
	}
	EXTI->PR |= (1<<6); // Clear EXTI6 interrupt pending bit
}

void USART2_Init(void)
{
	RCC->APB1ENR|=0x00020000; 	//set bit 17 (USART2 EN)
	RCC->AHBENR|=0x00000001; 	//enable GPIOA port clock bit 0 (GPIOA EN)
	GPIOA->AFR[0]=0x00000700;	//GPIOx_AFRL p.188,AF7 p.177
	GPIOA->MODER|=0x00000020; 	//MODER2=PA2(TX) to mode 10=alternate function mode. p184

	USART2->BRR = 0x00000116;	//115200 BAUD and crystal 32MHz. p710, 116
	USART2->CR1 = 0x00000008;	//TE bit. p739-740. Enable transmit
	USART2->CR1 |= 0x00002000;	//UE bit. p739-740. Uart enable
}

void USART2_write(char data)
{
	//wait while TX buffer is empty
	while(!(USART2->SR&0x0080)){} 	//TXE: Transmit data register empty. p736-737
		USART2->DR=(data);			//p739
}

void delay_s(unsigned long delay)
{
	unsigned long i=0;
	RCC->APB1ENR|= 1<<1; 	//TIM5EN: Timer 5 clock enable.P160
	TIM3->PSC = 32000 - 1;  // 32 MHz / 32000 = 1000 Hz.
	TIM3->ARR = 1000 - 1;   // Auto-reload value: 1000 cycles = 1 second.
	TIM3->CNT = 0;	//counter start value = 0
	TIM3->CR1 = 1; 			//TIM5 Counter enabled. p421

	  while(i<delay)
	  {
		  while(!((TIM3->SR)&1)){} //Update interrupt flag. p427
		  i++;
		  TIM3->SR &= ~1; 	//flag cleared. p427
		  TIM3->CNT=0;	  	//counter start value = 0
	  }
	  TIM3->CR1=0; 		//TIM11 Counter disabled. p421
}

void delay_10us(unsigned long delay)
{
	unsigned long i=0;
	RCC->APB1ENR|=(1<<4); 	//TIM5EN: Timer 6 clock enable.P160
	TIM6->PSC = 32 - 1;
	TIM6->ARR = 8 - 1;
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
