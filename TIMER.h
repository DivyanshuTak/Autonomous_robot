#ifndef _TIMER_H
#define _TIMER_H

#include<LPC17xx.h>
#include "LPC17xx_1.h"

#define _Nothing		0x0000
#define _INT 			0x0001
#define _RESET	 		0x0002
#define _INT_RESET 		0x0003
#define _STOP 			0x0004
#define _INT_STOP		0x0005
#define _RESET_STOP		0x0006
#define _INT_RESET_STOP		0x0007

void ConfigTimer(int,int);
void MRConfig(int,int,int);
void StartTimer(int);
void StopTimer(int);

#endif
