#ifndef _QEI_H
#define _QEI_H

#define CAP0 0
#define CAP1 1
			  
#define ENC1	0
#define ENC2	1
#define ENC3	2
#define ENC4	3

#define CW_DIR 		0
#define ACW_DIR		1

#define MODE_2X		0
#define MODE_4X		1

extern unsigned int ENC1Vel,ENC2Vel;

void ConfigEncoder(unsigned short,unsigned int,unsigned int,unsigned int);
void StartEncoder(unsigned short encoder);
void StopEncoder(unsigned short encoder);
void ClearEncoder(unsigned short);
unsigned int GetEncoderCount(unsigned short encoder);

void ConfigQEI(unsigned int,unsigned int,unsigned int,unsigned int);
void ClearQEI(void);
unsigned int GetQEICount(void);
unsigned int GetQEIVelocity(void);
void StartQEI(void);
void StopQEI(void);

void calEncoderVeloity(void);
unsigned int getEncderVelocity(unsigned short);

#endif
