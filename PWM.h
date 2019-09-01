#ifndef _PWM_H
#define _PWM_H

#define Pwm1 1
#define Pwm2 2
#define Pwm3 3
#define Pwm4 4
#define Pwm5 5
#define Pwm6 6

void ConfigPWM(int channel,int prescalar,int frequency); 	//clock divider=1/2/4/8
																	  		//channel select 1-6
void PWM(int channel,int value);

#endif

