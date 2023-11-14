/*
Transmit array with 40 elements with some integer numbers.
Send whole array with 200 ms intervals and use LD2 to show data transfer.
Change baudrate to 38400 and check STM com port from laboratory PC (device manager).
Use Realterm to capture your data.
Create such software that by pressing USER button B1 you can send data to realterm.
Make system so that B1 button input works as normal input.
*/

/* Includes */
#include <stddef.h>
#include "stm32l1xx.h"
#include "nucleo152start.h"
#include <stdio.h>
#include <stdlib.h>

/* Private typedef */
/* Private define  */
/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */
void USART2_Init(void);
void USART2_write(int data);
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
  USART2_Init();
  /* TODO - Add your application code here */
  RCC->AHBENR|=0x1; //GPIOA enable. LD2 connected to PA5.
  RCC->AHBENR|=0x4; //GPIOC enable, IO port C clock enable,because B1 push button is connected to PC13.
  GPIOA->MODER|=0x400; //GPIOA pin 5 to output. p184
  GPIOC->MODER&=~0xC000000; //bit 27& 28 clear to 0, makes sure that pc13 is input.

int arr[40];
for(int i=0;i<40;i++)
	{
	arr[i]=i+1;
	}


  /* Infinite loop */
  while (1)
  {

	  if(GPIOC->IDR & (1<<13)) 	//GPIO port input data register (GPIOx_IDR),if PC13 is high state (button released)
	  {
	  	GPIOA->BSRR=0x200000;	//turn led off (PA5=low state),"reset" section (16-31 bit),1: Resets the corresponding ODRx bit
	  }
	  else //(button pressed)
	  {
	  	GPIOA->BSRR=0x20;		//turn led on (PA5=high state),"set" section (0-15 bits),1: Sets the corresponding ODRx bit
	  	for(int i=0;i<40;i++)
	  	{
	  		USART2_write(arr[i]);
	  	}
	  }

	  delay_Ms(200);

  }
  return 0;
}

void USART2_Init(void)
{
	RCC->APB1ENR|=0x00020000; 	//set bit 17 (USART2 EN)
	RCC->AHBENR|=0x00000001; 	//enable GPIOA port clock bit 0 (GPIOA EN)
	GPIOA->AFR[0]=0x00000700;	//GPIOx_AFRL p.188,AF7 p.177
	GPIOA->AFR[0]|=0x00007000;	//GPIOx_AFRL p.188,AF7 p.177
	GPIOA->MODER|=0x00000020; 	//MODER2=PA2(TX) to mode 10=alternate function mode. p184
	GPIOA->MODER|=0x00000080; 	//MODER2=PA3(RX) to mode 10=alternate function mode. p184

	USART2->BRR = 0x00000341;	//38400 BAUD and crystal 32MHz. p710, 833(341hex)=32M/38400
	USART2->CR1 = 0x00000008;	//TE bit. p739-740. Enable transmit
	USART2->CR1 |= 0x00000004;	//RE bit. p739-740. Enable receiver
	USART2->CR1 |= 0x00002000;	//UE bit. p739-740. Uart enable
}

void USART2_write(int data)
{
	//wait while TX buffer is empty
	while(!(USART2->SR&0x0080)){} 	//6. p736-737
		USART2->DR=(data);		//p739
}

void delay_Ms(int delay)
{
	int i=0;
	for(; delay>0;delay--)
		for(i=0;i<2460;i++); //measured with oscilloscope
}
