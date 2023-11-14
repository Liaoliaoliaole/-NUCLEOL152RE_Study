/*
DHT22 connected to PA6 (D12)
ESP USART1
*/

/* Includes */
#include <stddef.h>
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
void USART1_Init(void);
void USART2_Init(void);
void USART2_write(char data);
void delay_Ms(int delay);
void read_dht22_humidity_and_temperature(int *hum, int *temp);
void delay_Us(int delay);
char USART1_read();
char USART2_read();
void wifi_init(void);
void send_to_thigspeak_temperature(int temperature);
void send_to_thigspeak_humidity(int humidity);
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
	__disable_irq();			//global disable IRQs, M3_Generic_User_Guide p135.
  SetSysClock();
  SystemCoreClockUpdate();
  USART1_Init();
  USART2_Init();


	USART1->CR1 |= 0x0020;			//enable RX interrupt
	NVIC_EnableIRQ(USART1_IRQn); 	//enable interrupt in NVIC
	__enable_irq();					//global enable IRQs, M3_Generic_User_Guide p135

  /* TODO - Add your application code here */
  RCC->AHBENR|=1; //GPIOA ABH bus clock ON. p154
  GPIOA->MODER|=0x400; //GPIOA pin 5 to output. p184
  /* Infinite loop */
  int hum=0;
  int temp=0;
  
  wifi_init();

  while (1)
  {
		  //GPIOA->ODR|=0x20; //0010 0000 set bit 5. p186
	  	  GPIOA->ODR|=0x20; //0010 0000 set bit 5. p186
		  delay_Ms(1500);
		  GPIOA->ODR&=~0x20; //0000 0000 clear bit 5. p186
		  delay_Ms(1500);
		  read_dht22_humidity_and_temperature(&hum,&temp);
		  delay_Ms(1500);
		  send_to_thigspeak_temperature(temp);
		  delay_Ms(15000);
		  send_to_thigspeak_humidity(hum);
		  delay_Ms(15000);
  }
  return 0;
}

void USART1_Init(void)
{
	RCC->APB2ENR|=(1<<14);	 	//set bit 14 (USART1 EN) p.156
	RCC->AHBENR|=0x00000001; 	//enable GPIOA port clock bit 0 (GPIOA EN)
	GPIOA->AFR[1]=0x00000700;	//GPIOx_AFRL p.189,AF7 p.177
	GPIOA->AFR[1]|=0x00000070;	//GPIOx_AFRL p.189,AF7 p.177
	GPIOA->MODER|=0x00080000; 	//MODER2=PA9(TX) to mode 10=alternate function mode. p184
	GPIOA->MODER|=0x00200000; 	//MODER2=PA10(RX) to mode 10=alternate function mode. p184

	USART1->BRR = 0x00000116;	//9600 BAUD and crystal 32MHz. p710, D05
	USART1->CR1 = 0x00000008;	//TE bit. p739-740. Enable transmit
	USART1->CR1 |= 0x00000004;	//RE bit. p739-740. Enable receiver
	USART1->CR1 |= 0x00002000;	//UE bit. p739-740. Uart enable
}

void USART2_Init(void)
{
	RCC->APB1ENR|=0x00020000; 	//set bit 17 (USART2 EN)
	RCC->AHBENR|=0x00000001; 	//enable GPIOA port clock bit 0 (GPIOA EN)
	GPIOA->AFR[0]=0x00000700;	//GPIOx_AFRL p.188,AF7 p.177
	GPIOA->AFR[0]|=0x00007000;	//GPIOx_AFRL p.188,AF7 p.177
	GPIOA->MODER|=0x00000020; 	//MODER2=PA2(TX) to mode 10=alternate function mode. p184
	GPIOA->MODER|=0x00000080; 	//MODER2=PA3(RX) to mode 10=alternate function mode. p184

	USART2->BRR = 0x00000116;	//9600 BAUD and crystal 32MHz. p710, D05
	USART2->CR1 = 0x00000008;	//TE bit. p739-740. Enable transmit
	USART2->CR1 |= 0x00000004;	//RE bit. p739-740. Enable receiver
	USART2->CR1 |= 0x00002000;	//UE bit. p739-740. Uart enable
}

void USART1_write(char data)
{
	//wait while TX buffer is empty
	while(!(USART1->SR&0x0080)){} 	//TXE: Transmit data register empty. p736-737
		USART1->DR=(data);		//p739
}

char USART1_read()
{
	char data=0;
	//wait while RX buffer is data is ready to be read
	while(!(USART1->SR&0x0020)){} 	//Bit 5 RXNE: Read data register not empty
		data=USART1->DR;			//p739
		return data;
}

char USART2_read()
{
	char data=0;
	//wait while RX buffer is data is ready to be read
	while(!(USART2->SR&0x0020)){} 	//Bit 5 RXNE: Read data register not empty
		data=USART2->DR;			//p739
		return data;
}

void USART2_write(char data)
{
	//wait while TX buffer is empty
	while(!(USART2->SR&0x0080)){} 	//TXE: Transmit data register empty. p736-737
		USART2->DR=(data);		//p739
}

void delay_Ms(int delay)
{
	int i=0;
	for(; delay>0;delay--)
		for(i=0;i<2460;i++); //measured with oscilloscope
}

void delay_Us(int delay)
{
	for(int i=0;i<(delay*2);i++) //accurate range 10us-100us
	{
		asm("mov r0,r0");
	}
}

void read_dht22_humidity_and_temperature(int *hum, int *temp)
{
	unsigned int humidity=0, i=0,temperature=0;
	unsigned int mask=0x80000000;
	RCC->AHBENR|=1; //GPIOA ABH bus clock ON. p154
	GPIOA->MODER|=0x1000; //GPIOA pin 6 to output. p184
	GPIOA->ODR|=0x40; //0100 0000 set bit 6. p186
	delay_Ms(10);
	GPIOA->ODR&=~0x40; //low-state at least 500 us
	delay_Ms(1);
	GPIOA->ODR|=0x40; //pin 6 high state and sensor gives this 20us-40us
	GPIOA->MODER&=~0x3000; //GPIOA pin 6 to input. p184

	//response from sensor
	while((GPIOA->IDR & 0x40)){}
	while(!(GPIOA->IDR & 0x40)){}
	while((GPIOA->IDR & 0x40)){}

	//read values from sensor
	while(i<32)
	{
		while(!(GPIOA->IDR & 0x40)){}

		delay_Us(35);

		if((GPIOA->IDR & 0x40)&&i<16)
		{
			humidity=humidity|(mask>>16);
		}
		if((GPIOA->IDR & 0x40)&&i>=16)
		{
			temperature=temperature|mask;
		}
		mask=(mask>>1);
		i++;

		while((GPIOA->IDR & 0x40)){}
	}
	*hum=(int)humidity;
	*temp=(int)temperature;
}

void USART1_IRQHandler(void)
{
	char c=0;

	//This bit is set by hardware when the content of the
	//RDR shift register has been transferred to the USART_DR register.
	if(USART1->SR & 0x0020) 		//if data available in DR register. p737
	{
			c = USART1->DR;
			USART2_write(c);
			//USART_write('\n');
			//USART_write('\r');
	}
}

void wifi_init(void)
{
delay_Ms(2000);
char buf[4]="AT\r\n";
for(int i=0;i<4;i++)
	  USART1_write(buf[i]);
delay_Ms(2000);

char buf2[8]="AT+RST\r\n";
for(int i=0;i<8;i++)
	  USART1_write(buf2[i]);
delay_Ms(2000);

char buf3[8]="AT+GMR\r\n";
for(int i=0;i<8;i++)
	  USART1_write(buf3[i]);
delay_Ms(2000);

char buf4[13]="AT+CWMODE=3\r\n";
for(int i=0;i<13;i++)
	  USART1_write(buf4[i]);
delay_Ms(1000);

char buf5[10]="AT+CWLAP\r\n";
for(int i=0;i<10;i++)
	  USART1_write(buf5[i]);
delay_Ms(3000);

char buf6[]="AT+CWJAP=\"\",\"\"\r\n"; //here your own wifi
for(int i=0;i<strlen(buf6);i++)
	  USART1_write(buf6[i]);
delay_Ms(5000);

char buf7[]="AT+CWJAP?\r\n";
for(int i=0;i<strlen(buf7);i++)
	  USART1_write(buf7[i]);
delay_Ms(5000);
/*
AT
AT+RST
AT+GMR
AT+CWMODE=3
AT+CWLAP
AT+CWJAP=”SSID”,”PASSWORD”
AT+CWJAP?
*/
}

void send_to_thigspeak_temperature(int temperature)
{
    float value=((float)temperature)/10;
    char bufx[5]="";
    gcvt(value, 4, bufx);

	delay_Ms(2000);
	char buf[]="AT+CIPMUX=0\r\n";
	for(int i=0;i<strlen(buf);i++)
		  USART1_write(buf[i]);
	delay_Ms(2000);

	char buf2[]="AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n";
	for(int i=0;i<strlen(buf2);i++)
		  USART1_write(buf2[i]);
	delay_Ms(2000);

	char buf3[]="AT+CIPSEND=51\r\n";
	for(int i=0;i<strlen(buf3);i++)
		  USART1_write(buf3[i]);
	delay_Ms(2000);

	char buf4[100]="";
	sprintf(buf4,"GET /update?api_key=Y5A7Y9NGAK0GH8P2&field1=%s\r\n",bufx);//Here your own thingspeak API Key
	for(int i=0;i<strlen(buf4);i++)
		  USART1_write(buf4[i]);
	delay_Ms(2000);

	char buf5[]="AT+CIPCLOSE\r\n";
	for(int i=0;i<strlen(buf5);i++)
		  USART1_write(buf5[i]);
	delay_Ms(15000);


	/*
	Thigspeak
	AT+CIPMUX=0
	AT+CIPSTART="TCP","api.thingspeak.com",80
	AT+CIPSEND=51
	GET /update?api_key=SN5GN44TC8YK2QPQ&field2=255
	AT+CIPCLOSE
	*/
}

void send_to_thigspeak_humidity(int humidity)
{
    float value=((float)humidity)/10;
    char bufx[5]="";
    gcvt(value, 4, bufx);

	delay_Ms(2000);
	char buf[]="AT+CIPMUX=0\r\n";
	for(int i=0;i<strlen(buf);i++)
		  USART1_write(buf[i]);
	delay_Ms(2000);

	char buf2[]="AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n";
	for(int i=0;i<strlen(buf2);i++)
		  USART1_write(buf2[i]);
	delay_Ms(2000);

	char buf3[]="AT+CIPSEND=51\r\n";
	for(int i=0;i<strlen(buf3);i++)
		  USART1_write(buf3[i]);
	delay_Ms(2000);

	char buf4[100]="";
	sprintf(buf4,"GET /update?api_key=Y5A7Y9NGAK0GH8P2&field2=%s\r\n",bufx);
	for(int i=0;i<strlen(buf4);i++)
		  USART1_write(buf4[i]);
	delay_Ms(2000);

	char buf5[]="AT+CIPCLOSE\r\n";
	for(int i=0;i<strlen(buf5);i++)
		  USART1_write(buf5[i]);
	delay_Ms(15000);


	/*
	Thigspeak
	AT+CIPMUX=0
	AT+CIPSTART="TCP","api.thingspeak.com",80
	AT+CIPSEND=51
	GET /update?api_key=SN5GN44TC8YK2QPQ&field2=255
	AT+CIPCLOSE
	*/
}

