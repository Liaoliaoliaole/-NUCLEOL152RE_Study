/*
Build PT-100 electronics according to figure and create such software which can measure temperature -50°C to +50°C and show it in terminal window. 
PT100 sensor will change resistance 80,306 Ω - 119,397 Ω when temperature changes -50°C to +50°C. 
Calibrate system with external adjustable resistor and adjust Zero and Span. 
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
void USART2_write(char data);
char USART2_read(void);
void delay_Ms(int delay);
void read_adc_and_print_temperature(uint8_t channel);
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

  //set up pin PA5 for LED
  RCC->AHBENR |= 1;				//enable GPIOA clock
  GPIOA->MODER&=~0x00000C00;	//clear pin mode
  GPIOA->MODER|=0x00000400;		//set pin PA5 to output model

  //set up pin PA0 and PA1 for analog input
  RCC->AHBENR|=1;				//enable GPIOA clock
  GPIOA->MODER|=0x3;			//PA0 analog (A0)
  GPIOA->MODER|=0xC;			//PA1 analog (A1)

  //setup ADC1. p272
  RCC->APB2ENR|=0x00000200;		//enable ADC1 clock
  ADC1->CR2=0;					//bit 1=0: Single conversion mode
  ADC1->SMPR3=7;				//384 cycles sampling time for channel 0 (longest)
  ADC1->CR1&=~0x03000000;		//resolution 12-bit


  /* Infinite loop */
  while (1)
  {
	  read_adc_and_print_temperature(0);
	  delay_Ms(1000);
	  read_adc_and_print_temperature(0);
	  delay_Ms(1000);
  }
  return 0;
}


void read_adc_and_print_temperature(uint8_t channel)
{
  // Set the ADC channel
  ADC1->SQR5 = channel;      // Conversion sequence starts at selected channel
  ADC1->CR2 |= 1;            // Bit 0, ADC on/off (1=on, 0=off)
  ADC1->CR2 |= 0x40000000;   // Start conversion
  while (!(ADC1->SR & 2)){}  // Wait for conversion complete
  int result = ADC1->DR;     // Read conversion result
  //float voltage = (float)result * (3.3 / 4095.0); // Convert to voltage

  char buf[100];
  float adc_value = 0;
  int temp = 0;
  int temp_degree=0;
  int temp_decimals=0;

  adc_value=(100.0/4095.0)*((float)result-50.0);
  temp = adc_value*100;
  temp_degree=(int)temp/100;
  temp_decimals= abs((int)temp%100);
  sprintf(buf,"%d.%d Degree",temp_degree,temp_decimals);

  int len=0;
  while(buf[len]!='\0')
  len++;

  for(int i=0;i<len;i++)
  {
  	USART2_write(buf[i]);
  }

  USART2_write('\n');
  USART2_write('\r');

  ADC1->CR2 &= ~1;           // Bit 0, ADC on/off (1=on, 0=off)
}


void USART2_Init(void)
{

	RCC->APB1ENR|=0x00020000; 	//set bit 17 (USART2 EN)
	RCC->AHBENR|=0x00000001; 	//enable GPIOA port clock bit 0 (GPIOA EN)
	GPIOA->AFR[0]=0x00000700;	//GPIOx_AFRL p.188,AF7 p.177
	GPIOA->AFR[0]|=0x00007000;	//GPIOx_AFRL p.188,AF7 p.177
	GPIOA->MODER|=0x00000020; 	//MODER2=PA2(TX) to mode 10=alternate function mode. p184
	GPIOA->MODER|=0x00000080; 	//MODER2=PA3(RX) to mode 10=alternate function mode. p184

	USART2->BRR = 0x00000D05;	//9600BAUD and crystal 32MHz. p710, 116
	USART2->CR1 = 0x00000008;	//TE bit. p739-740. Enable transmit
	USART2->CR1 |= 0x00000004;	//RE bit. p739-740. Enable receiver
	USART2->CR1 |= 0x00002000;	//UE bit. p739-740. Uart enable
}


void USART2_write(char data)
{
	//wait while TX buffer is empty
	while(!(USART2->SR&0x0080)){} 	//TXE: Transmit data register empty. p736-737
		USART2->DR=(data);		//p739
}

char USART2_read()
{
	char data=0;
	//wait while RX buffer data is ready to be read
	while(!(USART2->SR&0x0020)){} //Bit 5 RXNE: Read data register not empty
		data=USART2->DR;			//p739
		return data;
}

void delay_Ms(int delay)
{
	int i=0;
	for(; delay>0;delay--)
		for(i=0;i<2460;i++); //measured with oscilloscope
}
