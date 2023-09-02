/*
Transmit array with 40 elements with some integer numbers.
Send whole array with 200 ms intervals and use LD2 to show data transfer.
Change baudrate to 38400 and check STM com port from laboratory PC (device manager).
Use Realterm to capture your data.
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

//float value=34.54;
//int temp_degree=0;
//float temp=0;
//int temp_decimals=0;
//char buf[100];
//
//	  temp=value*100; //remove decimals and 34.54 = 3454
//      temp_degree=(int)temp/100;
//      temp_decimals=abs((int)temp%100);
//	  sprintf(buf,"%d.%d Celcius",temp_degree,temp_decimals);
//
//int len=0;
//while(buf[len]!='\0')
//	len++;
int arr[40];
for(int i=0;i<40;i++)
	{
	arr[i]=i+1;
	}


  /* Infinite loop */
  while (1)
  {
		delay_Ms(1000);
		for(int i=0;i<40;i++)
		{
			USART2_write(arr[i]);
		}

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
