#include <LPC17xx.h>				 //Always Put this header file on the top of the code else u will get errors
#include "LPC17xx_1.h"

#include "SPI.h"


char receive; 
void SPI_MasterInit(int clock)
  {
  PCONP |= (1<<8);
  PINSEL0 &= 0x3FFFFFFF;
  PINSEL0 |= 0xC0000000;
  PINSEL1 &= 0xFFFFFFC0;
  PINSEL1 |= 0x0000003C;
  FIO0DIR &= 0xFFF87FFF;
  FIO0DIR |= 0x00058000;
  S0SPCR |= (1<<5);
  S0SPCCR = clock;
  }

void SPI_SlaveInit()
 	{
	PCONP |= (1<<8);
  PINSEL0 &= 0x3FFFFFFF;
	PINSEL0 |= 0xC0000000;
  PINSEL1 &= 0xFFFFFFC0;
  PINSEL1 |= 0x0000003F;	
	FIO0DIR &= 0xFFF87FFF;
  FIO0DIR |= 0x00020000; 
	}		
	
void SPI_IntEnable()
	{
		S0SPCR |= (1<<7);
	}

unsigned short SPI_Communicate(char data)
	{
		S0SPDR=data;
		while(!(S0SPSR &(1<<7)));
		return S0SPDR;	
	}

