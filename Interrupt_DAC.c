/*
Create project for NUCLEO-L152RE board where PA6 (D12, encoder A) is interrupt input
and PA7 (D11, encoder B) is GPIO input pin and PA4 (A2) is analog DAC1 output.
Use function void EXTI9_5_IRQHandler(void) for interrupts. Put LD2(D13-PA5) on when rising edge in PA6 detected.
*/

/* Includes */

#include "stm32l1xx.h"
#include "nucleo152start.h"
#include <stdio.h>
#include <string.h>

/* Private typedef */
/* Private define  */
/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */
void delay_Ms(int delay);
void USART_write(char data);
void USART2_Init(void);
/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void)
{
	__disable_irq();			//global disable IRQs, M3_Generic_User_Guide p135.
	USART2_Init();

  /* Configure the system clock to 32 MHz and update SystemCoreClock */
  SetSysClock();
  SystemCoreClockUpdate();

  /* TODO - Add your application code here */

  RCC->AHBENR|=1; 				//GPIOA ABH bus clock ON. p154
  RCC->APB2ENR |=1;				//Bit 0 SYSCFGEN: System configuration controller clock enable. p157
  RCC->APB1ENR |=1<<29; 			//enable DAC clock

  GPIOA->MODER &= ~0x00003000; 	// Configure PA6 (Encoder A) as an input
  GPIOA->MODER &= ~0x0000C000; 	// Configure PA7 (Encoder B) as an input
  GPIOA->MODER |= 0x00000400; 	// GPIOA LD2 (PA5) to output.
  GPIOA->MODER |= 0x00000300;		//PA4 analog mode: 11,1100000000

  DAC->CR|=1;						//enable DAC,P314
  DAC->CR|=2;						//disable Buffer

  SYSCFG->EXTICR[1] |= 0;		// Connect EXTI6 (PA6) to EXTI line,0x0000000,index 1:0-3;2:4-7;3:8-11.....s
  EXTI->IMR |= 1<<6;	// Unmask EXTI6 (PA6) interrupt, 1000000
  EXTI->RTSR |= 1<<6; 	// TR6, 1000000, Configure EXTI6 (PA6) to trigger on rising edge
  EXTI->FTSR |= 1<<6;

  NVIC_EnableIRQ(EXTI9_5_IRQn);	// Enable EXTI9_5_IRQn
  __enable_irq();			//global enable IRQs, M3_Generic_User_Guide p135

  int data=0;

  /* Infinite loop */
   while (1)
  {
	   DAC->DHR12R1=data++;
	   delay_Ms(10);

	   delay_Ms(10);

	   if(data>=4095)
	   {
	   		data=0;
	   }
  }
  return 0;
}

void delay_Ms(int delay)
{
	int i=0;
	for(; delay>0;delay--)
		for(i=0;i<2460;i++); 	//measured with oscilloscope
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

void USART_write(char data)
{
	//wait while TX buffer is empty
	while(!(USART2->SR&0x0080)){} 	//TXE: Transmit data register empty. p736-737
		USART2->DR=(data);			//p739
}

void EXTI9_5_IRQHandler(void)
{
	if (EXTI->PR & (1<<6)) // Check if PA6 (Encoder A) triggered the interrupt
	    {
	        if ((GPIOA->IDR & GPIO_IDR_IDR_6) != 0)
	        {
	        	char buf[]="risign edge interrupt";
	        	for(int i=0;i<strlen(buf);i++)
	        	{
	        		USART_write(buf[i]);
	        	}
	        		USART_write('\n');
	        		USART_write('\r');
	        		GPIOA->ODR |= 0x20; // Set LD2 (LED2) ON
	        }
	        else
	        {
	        	char buf[]="falling edge interrupt";
	        	for(int i=0;i<strlen(buf);i++)
	        	{
	        		USART_write(buf[i]);
	        	}
	        		USART_write('\n');
	        		USART_write('\r');
	        		GPIOA->ODR &= ~0x20;
	        }

	        EXTI->PR = EXTI_PR_PR6; // Clear the interrupt flag
	    }
}
