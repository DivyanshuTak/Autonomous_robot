#include<LPC17xx.h>    //Always Put this header file on the top of the code else u will get errors

#include"PLL.h"
#include"NVIC.h"

#include"GPIO.h"
#include"QEI.h"

extern unsigned int encoderID=0;
unsigned int ENC1Vel,ENC2Vel;

void ConfigEncoder(unsigned short encoder,unsigned int timer,unsigned int capture,unsigned int timerMode)
{
	//-------------------------------
	InitPLL();
	encoderID&=~(0x0F<<(encoder<<2));					  //mask
	encoderID|=((0x01<<(timer-1))<<(encoder<<2));
	//-------------------------------			
	switch(timer)	{					

		case TIMER0:
			PCONP |= 0x00000002;						//power	
			T0TCR = 0x00000000;
			T0TC = 0x00000000;

			switch(capture)		{
				
				case CAP0:
				    PINSEL3|=0x00300000;
					switch(timerMode)	{
					
					 	case RISING:
							 T0CTCR=0x00000001;
					         break;
					 	case FALLING:
							 T0CTCR=0x00000002;
					         break;
					  	case BOTH:
							 T0CTCR=0x00000003;
					         break;
					}  
					break;
				case CAP1:
					PINSEL3|=0x00C00000;
					switch(timerMode)	{
					 	
						case RISING:
							T0CTCR=0x00000005;
					        break;
					  	case FALLING:
							T0CTCR=0x00000006;
					        break;
					  	case BOTH:
							T0CTCR=0x00000007;
					        break;
					}  
					break;
			}
			break;
		case TIMER1:
			PCONP |= 0x00000004;	//power	  
		    T1TCR = 0x00000000;
			T1TC = 0x00000000;
			
			switch(capture)	
			{
				case CAP0:
					PINSEL3|=0x00000030;
					
					switch(timerMode)
					{
					  case RISING:
							T1CTCR=0x00000001;
					              break;
					  case FALLING:
							T1CTCR=0x00000002;
					              break;
					  case BOTH:
							T1CTCR=0x00000003;
					              break;
					}  
					break;
				case CAP1:
					PINSEL3|=0x000000C0;
						switch(timerMode)
					{
					  case RISING:
							 T1CTCR=0x00000005;
					              break;
					  case FALLING:
							 T1CTCR=0x00000006;
					              break;
					  case BOTH:
							 T1CTCR=0x00000007;
					              break;
					}  
				
					break;
			}
			break;
		     
		case TIMER2:
			PCONP |= 0x00400000;	//power	  
    		T2TCR = 0x00000000;
			T2TC = 0x00000000;
			
			switch(capture)	
			{
				case CAP0:
					PINSEL0|=0x00000300;
					switch(timerMode)
					{
					  case RISING:
							 T2CTCR=0x00000001;
					              break;
					  case FALLING:
							 T2CTCR=0x00000002;
					              break;
					  case BOTH:
							 T2CTCR=0x00000003;
					              break;
					}  
					break;
				case CAP1:
					PINSEL0|=0x00000C00;
					switch(timerMode)
					{
					  case RISING:
							 T2CTCR=0x00000005;
					              break;
					  case FALLING:			 
							 T2CTCR=0x00000006;
					              break;
					  case BOTH:
							 T2CTCR=0x00000007;
					              break;
					}  
					break;
			}
			break;
		case TIMER3:
			PCONP |= 0x00800000;	//power	  
    		T3TCR = 0x00000000;
			T3TC = 0x00000000;
			
			switch(capture)	
			{
				case CAP0:
					PINSEL1|=0x0000C000;
					switch(timerMode)
					{
					  case RISING:
							 T3CTCR=0x00000001;
					              break;
					  case FALLING:
							 T3CTCR=0x00000002;
					              break;
					  case BOTH:
							 T3CTCR=0x00000003;
					              break;
					}  
					break;
				case CAP1:
					PINSEL1|=0x00030000;
					switch(timerMode)
					{
					  case RISING:
							 T3CTCR=0x00000005;
					              break;
					  case FALLING:
							 T3CTCR=0x00000006;
					              break;
					  case BOTH:
							 T3CTCR=0x00000007;
					              break;
					}  
					break;
			}
			break;
	}
	//-------------------------------
}

void StartEncoder(unsigned short encoder)	{

	unsigned int encoderCheck;
	encoderCheck=(encoderID&(0x0F<<(encoder<<2)))>>(encoder<<2);					  //mask
	switch(encoderCheck)
	{
		case 0x01:
			T0TCR |= 0x00000001;
			break;
		case 0x02:
	    	T1TCR |= 0x00000001;
			break;
		case 0x04:
			T2TCR |= 0x00000001;
			break;
		case 0x08:
		    T3TCR |= 0x00000001;
			break;	
	}
}

void StopEncoder(unsigned short encoder)	{

	unsigned int encoderCheck;
	encoderCheck=(encoderID&(0x0F<<(encoder<<2)))>>(encoder<<2);					  //mask
	switch(encoderCheck)
	{
		case 0x01:
			T0TCR &= ~0x00000001;
			break;
		case 0x02:
	    	T1TCR &= ~0x00000001;
			break;
		case 0x04:
			T2TCR &= ~0x00000001;
			break;
		case 0x08:
		    T3TCR &= ~0x00000001;
			break;	
	}
}

void ClearEncoder(unsigned short encoder)	{	  

	unsigned int encoderCheck;
	encoderCheck=(encoderID&(0x0F<<(encoder<<2)))>>(encoder<<2);					  //mask
	switch(encoderCheck)
	{
		case 0x01:
			T0TC = 0x00000000;
			break;
		case 0x02:
			T1TC = 0x00000000;
			break;
		case 0x04:
			T2TC = 0x00000000;
			break;
		case 0x08:
		    T3TC = 0x00000000;
			break;	
	}
	
}

unsigned int GetEncoderCount(unsigned short encoder){
	
	unsigned int encoderCheck;
	encoderCheck=(encoderID&(0x0F<<(encoder<<2)))>>(encoder<<2);					  //mask
	switch(encoderCheck)	{

		case 0x01:
			return T0TC;
		case 0x02:
			return T1TC;
		case 0x04:
			return T2TC;
		case 0x08:
			return T3TC;
	}
	return 0;
}

void calEncoderVeloity(void)	{
	
	static unsigned int preENC1Count,ENC1Count;
	static unsigned int preENC2Count,ENC2Count;

	ENC1Count = GetEncoderCount(ENC1);
	ENC1Vel = (ENC1Count - preENC1Count);
	preENC1Count = ENC1Count;

	ENC2Count = GetEncoderCount(ENC2);
	ENC2Vel = (ENC2Count - preENC2Count);
	preENC2Count = ENC2Count;
}

unsigned int GetEncoderVelocity(unsigned short encoder)	{

	unsigned int encoderCheck;
	encoderCheck=(encoderID&(0x0<<(encoder<<2)))>>(encoder<<2);					  //mask
	switch(encoderCheck)	{

		case 0x01:
			return ENC1Vel;
		case 0x02:
			return ENC2Vel;
	}
	return 0;	
}

void ConfigQEI(unsigned int dirstat,unsigned int capmode,unsigned int maxcount,unsigned int timeper)	{
	
	InitPLL();

	PCONP |= 0x00040000;
	PCLKSEL1 &= ~0x00000003;
	PCLKSEL1 |= 0x00000000;
	PINSEL3 |= 0x00004100;

	QEICON = 0x0000000C;
	QEICONF = (dirstat<<0)|(capmode<<2);
	QEIMAXPOS = maxcount;
	QEILOAD = timeper;

	FILTER = 0x00000190;
	PCONP &= ~0x00040000;
}

void StartQEI(void)	{

	PCONP |= 0x00040000;
}

void StopQEI(void)	{

	PCONP &= ~0x00040000;
}

void ClearQEI(void)	{

	QEICON |= 0x00000001;
}

unsigned int GetQEICount(void)	{
	
	return QEIPOS;
}

unsigned int GetQEIVelocity(void)	{
	

	return QEICAP;
}

unsigned int GetQEIDir(void)	{
	
	unsigned int dir = QEISTAT & 0x00000001;
	if(QEICONF&0x00000001)
		return ~dir;
	else
		return dir;
}
