#include<LPC17xx.h>    //Always Put this header file on the top of the code else u will get errors

#include"LPC17xx_1.h"
#include"PLL.h"

#include"UART.h"
#include"PWM.h"

int PWM_prescalar;

void ConfigPWM(int channel,int prescalar,int frequency) 								//channel select 1-6
{
	PWM_prescalar=prescalar;
	PCONP |= 0x40;
	PCLKSEL0&=~(3<<12);
	PINSEL4 |= 0x01<<(2*(channel-1));
	PWM1MR0 = 30000000/frequency;

	PWM1MCR |= 0x02;
	switch (channel)	{

		case Pwm1:
			//PWM1TCR |= 0x09;
			PWM1PCR|=1<<9;
			break;
		case Pwm2:
			//PWM1TCR |= 0x09;
			PWM1PCR|=1<<10;
			break;
		case Pwm3:
			//PWM1TCR |= 0x09;
			PWM1PCR|=1<<11;
			break;
		case Pwm4:
			//PWM1TCR |= 0x09;
			PWM1PCR|=1<<12;
			break;
		case Pwm5:
			//PWM1TCR |= 0x09;
			PWM1PCR|=1<<13;
			break;
		case Pwm6:
			//PWM1TCR |= 0x09;
			PWM1PCR|=1<<14;
			break;
	}
}

void PWM(int channel,int value)
{
	switch (channel)	{

		case Pwm1:
			if(value)
				PWM1MR1 = (PWM1MR0*value)/(PWM_prescalar-1);
			else
				PWM1MR1 = 1;
			break;
		case Pwm2:
			if(value)
				PWM1MR2 = (PWM1MR0*value)/(PWM_prescalar-1);
			else
				PWM1MR2 = 1;
			break;
		case Pwm3:
			if(value)
				PWM1MR3 = (PWM1MR0*value)/(PWM_prescalar-1);
			else
				PWM1MR3 = 1;
			break;
		case Pwm4:
			if(value)
				PWM1MR4 = (PWM1MR0*value)/(PWM_prescalar-1);
			else
				PWM1MR4 = 1;
			break;
		case Pwm5:
			if(value)
				PWM1MR5 = (PWM1MR0*value)/(PWM_prescalar-1);
			else
				PWM1MR5 = 1;
			break;
		case Pwm6:
			if(value)
				PWM1MR6 = (PWM1MR0*value)/(PWM_prescalar-1);
			else
				PWM1MR6 = 1;
			break;
	}
	PWM1TCR &= ~0x08;
	PWM1TCR |= 0x09;

}
