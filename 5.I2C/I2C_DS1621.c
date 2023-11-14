/*This code tests NUCLEO-L152RE board transmitter UART communication by using
9600 BAUD and float print with sprintf
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
#define SLAVE_ADDRESS 0x48
/* Private variables */
/* Private function prototypes */
/* Private functions */
void delay_Ms(int delay);
void I2C1_init(void);
void I2C1_Write(uint8_t address, uint8_t command, int n, uint8_t* data);
void I2C1_ByteWrite(uint8_t address, uint8_t command);
void I2C1_Read(uint8_t address, uint8_t command, int n, uint8_t* data);
void USART2_Init(void);
void USART2_write(char data);
void DS1621_Init(void);
void DS1621_ReadTemperature(void);

/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void)
{
	USART2_Init();
	I2C1_init();
	/* Configure the system clock to 32 MHz and update SystemCoreClock */
  SetSysClock();
  SystemCoreClockUpdate();

  DS1621_Init();

  while (1)
  {
	  DS1621_ReadTemperature();
	  delay_Ms(1000);
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

	USART2->BRR = 0x00000D05;	//9600 BAUD and crystal 32MHz. p710, D05
	USART2->CR1 = 0x00000008;	//TE bit. p739-740. Enable transmit
	USART2->CR1 |= 0x00000004;	//RE bit. p739-740. Enable receiver
	USART2->CR1 |= 0x00002000;	//UE bit. p739-740. Uart enable
}

void USART2_write(char data)
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

void I2C1_init(void)
{
	RCC->AHBENR |= 2;			//Enable GPIOB clock PB8(D15)=SCL,PB9(D14)=SDA.
	RCC->APB1ENR |= (1<<21);	//Enable I2C1_EN clock

	//configures PB8,PB9 to I2C1_EN
	GPIOB->AFR[1] &= ~0x000000FF;	//PB8,PB9 I2C1 SCL, SDA. AFRH8 and AFRH9. clear
	GPIOB->AFR[1] |= 0x00000044;	//GPIOx_AFRL p.189,AF4=I2C1(0100 BIN) p.177
	GPIOB->MODER &= ~0x000F0000;	//PB8 and PB9 clear
	GPIOB->MODER |= 0x000A0000;		//Alternate function mode PB8,PB9
	GPIOB->OTYPER |= 0x00000300;	//output open-drain. p.184
	GPIOB->PUPDR &= ~0x000F0000;	//no internal pull-up resistors for PB8 and PB9 p.185

	I2C1->CR1 = 0x8000;				//software reset I2C1 SWRST p.682
	I2C1->CR1 &= ~0x8000;			//stop reset/ Normal operation
	I2C1->CR2 = 0x0020;				//peripheral clock 32 MHz

	/*how to calculate CCR
	TPCLK1=1/32MHz=31,25ns
	tI2C_bus=1/100kHz=10us=10000ns
	tI2C_bus_div2=10000ns/2=5000ns
	CCR value=tI2C_bus_div2/TPCLK1=5000ns/31,25ns=160
	p. 692*/
	I2C1->CCR = 160;

	//maximum rise time in sm mode = 1000ns. Equation 1000 ns/TPCK1
	I2C1->TRISE = 33;				//1000ns/31,25ns=32+1=33, p.693
	I2C1->CR1 |= 0x0001;			//peripheral enable (I2C1)
}


void I2C1_Write(uint8_t address, uint8_t command, int n, uint8_t* data)
{
	volatile int tmp;
	int i;

	while(I2C1->SR2 & 2){}			//wait until bus not busy

	I2C1->CR1 &= ~0x800;			//disable POS p.682
	I2C1->CR1 |= 0x100;				//generate start p.694
	while(!(I2C1->SR1&1)){}			//wait until start condition generated

	I2C1->DR=address << 1;			//transmit slave address
	while(!(I2C1->SR1 & 2)){}		//wait until end of address transmission p.690

	tmp=I2C1->SR2;					//Reading I2C_SR2 after reading I2C_SR1 clears the ADDR flag p691
	while(!(I2C1->SR1 & 0x80)){}	//wait until data register empty p.689

	I2C1->DR = command;				//send command

	for(i=0;i<n;i++)
	{
		while(!(I2C1->SR1 & 0x80)){}	//wait until data register empty p.689
		I2C1->DR=*data++;				//send command
	}

	while(!(I2C1->SR1 & 4)){}		//wait until byte transfer finished p.690
	I2C1->CR1 |= (1<<9);			//generate stop
}

void I2C1_ByteWrite(uint8_t address, uint8_t command)
{
	volatile int tmp;


	while(I2C1->SR2 & 2){}			//wait until bus not busy

	I2C1->CR1 &= ~0x800;			//Acknowledge clear p.682
	I2C1->CR1 |= 0x100;				//generate start p.694
	while(!(I2C1->SR1&1)){}			//wait until start condition generated

	I2C1->DR=address<<1;			//transmit slave address
	while (!(I2C1->SR1 & (1<<1))){}		//wait until end of address transmission p.690

	tmp=I2C1->SR2;					//Reading I2C_SR2 after reading I2C_SR1 clears the ADDR flag p691
	while(!(I2C1->SR1 & 0x80)){}	//wait until data register empty p.689

	I2C1->DR = command;				//send command

	while(!(I2C1->SR1 & 4)){}		//wait until byte transfer finished p.690
	I2C1->CR1 |= (1<<9);			//generate stop
}

void I2C1_Read(uint8_t address, uint8_t command, int n, uint8_t* data)
{
	volatile int tmp;

	while(I2C1->SR2 & 2){}			//wait until bus not busy
	I2C1->CR1 &= ~0x800;			//Acknowledge clear p.682

	I2C1->CR1 |= 0x100;				//generate start p.694
	while(!(I2C1->SR1&1)){}			//wait until start condition generated

	I2C1->DR=address << 1;			//transmit slave address
	while(!(I2C1->SR1 & 2)){}		//wait until end of address transmission p.690

	tmp=I2C1->SR2;					//Reading I2C_SR2 after reading I2C_SR1 clears the ADDR flag p691
	while(!(I2C1->SR1 & 0x80)){}	//wait until data register empty p.689

	I2C1->DR=command;				//send command
	while(!(I2C1->SR1 & 0x80)){}	//wait until data register empty p.689

	I2C1->CR1 |= 0x100;				//generate repeated start p.694
	while(!(I2C1->SR1&1)){}			//wait until start condition generated

	I2C1->DR=address << 1|1;		//transmit slave address
	while(!(I2C1->SR1 & 2)){}		//wait until end of address transmission p.690

	tmp=I2C1->SR2;					//Reading I2C_SR2 after reading I2C_SR1 clears the ADDR flag p691
	I2C1->CR1 |= (1<<10);			//Enable acknowledge p.683

	while(n > 0)					//read data from chip
	{
		while(!(I2C1->SR1 & 0x40)){}	//wait until RXNE flag is set
		(*data++) = I2C1->DR;			//read data from DR
		I2C1->CR1 |= 1<<10;  // Set the ACK bit to Acknowledge the data received
		n--;
	}
	I2C1->CR1 |= (1<<9);			//generate stop p.682
	I2C1->CR1 &= ~(1<<10);			//disable acknowledge p.682
}


void DS1621_Init(void)
{
    I2C1_ByteWrite(SLAVE_ADDRESS, 0xAC); //Access Configuration Register:Command Code: 0xAC, send command to the salve
    I2C1_ByteWrite(SLAVE_ADDRESS, 0x22);//continuous conversion mode
    I2C1_ByteWrite(SLAVE_ADDRESS, 0xEE);//start conversion
}

void DS1621_ReadTemperature(void)
{
	uint8_t data[2];
    I2C1_Read(SLAVE_ADDRESS, 0xAA, 2, data);	// Read Temperature:Command Code: 0xAA, resolution 0.5 degree
    //degree part
    int temp_degree = 0;
    if((data[0] >= 0) && (data[0] <= 125))
    {
    	temp_degree = data[0];
    }
    else if((data[0]>>7) == 1)
    {
    	temp_degree = data[0] - 256;
    }
    //decimal part
    int temp_decimal = 0;
    if(data[1] == 128)
    {
    	temp_decimal = 5;
    }

    char buf[100];
    sprintf(buf, "Temperature: %d.%d", temp_degree, temp_decimal);

    int len = 0;
    while (buf[len] != '\0')
    	len++;

    for (int i = 0; i < len; i++)
    {
       USART2_write(buf[i]);
    }
    USART2_write('\n');
    USART2_write('\r');
}
