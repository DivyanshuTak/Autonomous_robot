#include<LPC17xx.h>
#include"LPC17xx_1.h"

#include"PLL.h"
#include"NVIC.h"

#include"TIMER.h"
//-------------------------------------------------------------------- 
void ConfigTimer(int timerSelect,int prescale)	{
																		    
	InitPLL();
	switch(timerSelect)
	{
		case TIMER0:
			PCONP |= 0x00000002;	//power	  
		    T0TCR = 0x02;
			T0PR = prescale; 
			break;
		case TIMER1:
			PCONP |= 0x00000004;	//power	  
		    T1TCR = 0x02;
			T1PR = prescale;
			break; 
		case TIMER2:
			PCONP |= 0x00400000;	//power	  
    		T2TCR = 0x02;
			T2PR = prescale; 
			break;
		case TIMER3:
			PCONP |= 0x00800000;	//power	  
    		T3TCR = 0x02;
			T3PR = prescale;  
			break;
	}
           			           	 
}						   
//--------------------------------------------------------------------
void MRConfig(int MRSelect,int MCROption,int value)	{
	
	switch(MRSelect)	{
		//----------------------------
		case _T0MR0: 
				T0MCR &= 0xFFFFFFF8;
				T0MCR |= MCROption<<0;
				T0MR0 = value;
			break;	
		case _T0MR1:
				T0MCR &= 0xFFFFFFC7; 
				T0MCR |= MCROption<<3;
				T0MR1 = value;
			break;
		case _T0MR2: 
				T0MCR &= 0xFFFFF3CF;
				T0MCR |= MCROption<<6;
				T0MR2 = value;
			break;	
		case _T0MR3:
				T0MCR &= 0xFFFFF1FF; 
				T0MCR |= MCROption<<9;
				T0MR3 = value;
			break;
		//----------------------------		
		case _T1MR0: 
				T1MCR &= 0xFFFFFFF8;
				T1MCR |= MCROption<<0;
				T1MR0 = value;
			break;	
		case _T1MR1: 
				T1MCR &= 0xFFFFFFC7; 
				T1MCR |= MCROption<<3;
				T1MR1 = value;
			break;
		case _T1MR2:	
				T1MCR &= 0xFFFFF3CF;
				T1MCR |= MCROption<<6;
				T1MR2 = value;
			break;
		case _T1MR3:	
				T1MCR &= 0xFFFFF1FF;
				T1MCR |= MCROption<<9;
				T1MR3 = value;
			break;
		//----------------------------
		case _T2MR0: 
				T2MCR &= 0xFFFFFFF8;
				T2MCR |= MCROption<<0;
				T2MR0 = value;
			break;	
		case _T2MR1:
				T2MCR &= 0xFFFFFFC7;  
				T2MCR |= MCROption<<3;
				T2MR1 = value;
			break;
		case _T2MR2: 
				T2MCR &= 0xFFFFF3CF;
				T2MCR |= MCROption<<6;
				T2MR2 = value;
			break;	
		case _T2MR3:
				T2MCR &= 0xFFFFF1FF; 
				T2MCR |= MCROption<<9;
				T2MR3 = value;
			break;
		//----------------------------
		case _T3MR0: 
				T3MCR &= 0xFFFFFFF8;
				T3MCR |= MCROption<<0;
				T3MR0 = value;
			break;	
		case _T3MR1:
				T3MCR &= 0xFFFFFFC7;  
				T3MCR |= MCROption<<3;
				T3MR1 = value;
			break;
		case _T3MR2: 
				T3MCR &= 0xFFFFF3CF;
				T3MCR |= MCROption<<6;
				T3MR2 = value;
			break;	
		case _T3MR3:
				T3MCR &= 0xFFFFF1FF; 
				T3MCR |= MCROption<<9;
				T3MR3 = value;
			break;
		//----------------------------
	}  
}
//--------------------------------------------------------------------
void StartTimer(int timerSelect)	{

	switch(timerSelect)	{

	case TIMER0: 
			T0TCR = 0x00000001;	
		break;
	case TIMER1: 
			T1TCR = 0x00000001;
		break;	
	case TIMER2: 
			T2TCR = 0x00000001;
		break;	
	case TIMER3: 
			T3TCR = 0x00000001;
		break;	
	}
}
//--------------------------------------------------------------------
void StopTimer(int timerSelect)	{

	switch(timerSelect)	{

	case TIMER0: 
			T0TCR &= ~0x00000001;
		break;	
	case TIMER1: 
			T1TCR &= ~0x00000001;
		break;	
	case TIMER2: 
			T2TCR &= ~0x00000001;
		break;
	case TIMER3: 
			T3TCR &= ~0x00000001;
		break;	
	}
}
//--------------------------------------------------------------------
//--------------------------------------------------------------------
