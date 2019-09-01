#include "common.h"

int main()
{	
	InitPLL();
	Enable(RIT);	
		
	ConfigPort(Port0,0x80797005);
	//	7:0		0000_0101	:			relay-snsr-snsr-Cap2.0(ENC2)-Adc6(3)-Tx0-Rx3(zone detect for auto)-Tx3
	//	15:8	0111_0000	:			Actuation-Reserved-Reserved-Reserved-Rx2_imu-snsr-snsr-relay
	//	23:16	0111_1001	:			Adc0(7)-Actuation-Actuation-Actuation-Actuation-Actuation-Actuation-Actuation
	//	31:24	1000_0000	:		 	Reserved-Bitwait2-Bitwait3-Bitwait1-Bitwait4-Adc3(4)-(Adc2(5)),Adc1(6)
	ConfigPort(Port1,0x3B6BF8EC);          //3b7838ec without driving indicator led (high lux)
	//	7:0		0000_0101	:			relay-snsr-snsr-Cap2.0(ENC2)-Adc6(3)-Tx0-Rx3(zone detect for auto)-Tx3
	//	15:8	0111_0000	:			Actuation-Reserved-Reserved-Reserved-Rx2_imu-snsr-snsr-relay
	//	23:16	0111_1001	:			Adc0(7)-Actuation-Actuation-Actuation-Actuation-Actuation-Actuation-Actuation
	//	31:24	1000_0000	:		 	Reserved-Bitwait2-Bitwait3-Bitwait1-Bitwait4-Adc3(4)-(Adc2(5)),Adc1(6)
	ConfigPort(Port2,0xFFFFC4C1);
	//	7:0		1100_0001	:			Actuation-Actuation-relay-relay-relay-snsr-snsr-input
	//	15:8	1100_0100	:			Reserved-Reserved-Eint3(ENC1)-Eint2(ENC3)-Eint1(ENC2)-ISP-Actuation-Actuation	
	ConfigPort(Port3,0xF9FFFFFF);	
	//	31:24	1111_1011	:			Reserved-Reserved-Reserved-Reserved-Reserved-Xtra(Driving)-PS2_data-Reserved	
	ConfigPort(Port4,0xCFFFFFFF);
	//	31:24	1100_1111	:			Reserved-Reserved-snsr-snsr-Reserved-Reserved-Reserved-Reserved

	ConfigPortMode(Port0,PULLUP);		  					//	Enable Pull down   (PULL UP AFTER TESTING)
	ConfigPortMode(Port1,PULLDOWN);								//	Enable Pull up
	ConfigPortMode(Port2,PULLDOWN);								//	Enable Pull up
	ConfigPortMode(Port3,PULLDOWN);								//	Enable Pull up
	ConfigPortMode(Port4,PULLDOWN);								//	Enable Pull up

	PINMODE0 |= 0x000FCCC0;         //enabling pull down resistor for pin 0-3,7,8,9 for sensor patti	\
	
	SPI_MasterInit(CLK_1_66);
	
	//*********** Timer(s) ******* TIMER3 in IMU values @ 500Hz *****  TIMER0 & TIMER1 in encoder values *** TIMER2 un-used ******
	ConfigTimer(TIMER3,256);    
	MRConfig(_T3MR0,_INT_RESET,293);           
	On(_T3MR0,get_rate);
	Enable(TIMER3);
	StartTimer(TIMER3);
	
	ConfigTimer(TIMER2,512);    
	MRConfig(_T2MR0,_INT_RESET,293);           
	On(_T2MR0,timer);
	Enable(TIMER2);
	StartTimer(TIMER2);
	
	// Config UARTS: COM1/2 for driving motors. COM0/3 unconfigured	
	ConfigUART(COM1,9600,_TX,WORD_LENGTH_8,STOP_BIT_1,PARITY_DISABLE);      // for sabertooth 25A 
	ConfigExtPrintbin(COM1,2);

	ConfigUART(COM2,9600,_TX,WORD_LENGTH_8,STOP_BIT_1,PARITY_DISABLE);
	ConfigExtPrintbin(COM2,2);
	
	ConfigUART(COM3,19200,_TX,WORD_LENGTH_8,STOP_BIT_1,PARITY_DISABLE);
	ConfigExtPrintbin(COM3,2);
	
	// Config Encoder: COM1/2 for driving motors. COM0/3 unconfigured 	 
	ConfigEncoder(ENC2,TIMER0,CAP0,BOTH);  		 
	ClearEncoder(ENC2);
	StartEncoder(ENC2);	

  
	
	ConfigLcd(Rs,E,Db4,Db5,Db6,Db7);
	
	waitms(5);
	ang=0;                       //Initial angle set to be zero
  
	left_dia=50.894;                           //encoder which gives x dis without rotating the bot whatsoever
  right_dia=left_dia;//50.507;               //encoder which gives y dis without rotating the bot whatsoever  	
	
	dis_per_count_left=(PI*left_dia)/2048;     //distance calculated per tick of encoder
	dis_per_count_right=(PI*right_dia)/2048;   //distance calculated per tick of encoder	
	
	acc=0.0006;                   //Initial value of accelaration
	lim11=80+10+10;               //these limits limit the speed of motors so that it doesn't beyond a range
	lim12=48-10-10;               //these limits limit the speed of motors so that it doesn't beyond a range
  lim21=208+10+10;              //these limits limit the speed of motors so that it doesn't beyond a range
	lim22=176-10-10;              //these limits limit the speed of motors so that it doesn't beyond a range
	
	kp_line_sense=8;               //Kp for line follower
	kd_line_sense=2.5;             //Kd for line follower
	ki_line_sense=0; 							 //Ki for line follower
	icontrol_line_sense=40;        //Maximum value for control of Line follow PID - in terms of maximum angle of motion allowed

	kp=0.8;												 //Kp for Omni
	kd=0; 												 //Kd for Omni
	ki=0;													 //Ki for omni
	icontrol=8;                    //Maximum value for control of Omni - in terms of maximum value sent to the motor
			
	zone=START_POINT;//ZONE_TWO_START;
		
	cls();
	lcd((char*)"START POINT");   
  
//_________________________________________DEBUGGING  SNIPPETS ARE PLACED HERE ___________________________________________________________________	
	

	while(0)                                                                     // FOR IMU AGNLE                   
	{
		cls();
	//	lcd(double(divyanshu_ka_chut)/(double)divyanshu);
	lcd(ang);
	}
	
	/*while(1)                                                                   //FOR TESTING PROXY FUNCTION 
	{
		test_decider();
		cls();
		
		if(zone==ZONE_ONE_START)
			lcd((char*)"ZONE ONE");
		else if(zone == ZONE_TWO_START)
			lcd((char*)"ZONE TWO");		
		else
			lcd((char*)"THREE");
	}*/
	
	/*while(1)                                                                   // FOR CHECKING SENSOR PATTI
	{
		sensor_val_update_v();
		cls();
		lcd(sum_sensor_v);
	}*/ 
	
   /*                                                                          // FOR CHECKING ENCODER
	   prvvaluel=0;
		 T0TC=0;
		 x_dis=0;
		 while(1)
		 { 
		   pos();
			 cls();
			 lcd(x_dis);
	   }
		 
	 */
	
//___________________________________________END OF DEBUGGING SNIPPETS______________________________________________________________________________ 	
	
//__________________________________________START OF MAIN RUNNING LOOPS_____________________________________________________________________________		
	bool restart=0;
	while(1)
	{
		control_pid_omni(4*PI,10);
		if  (!RACK_1_PROXY)
		{
			restart=0;
			break;
		}
		if (!NORMAL_BALL_PROXY)
		{
			restart=1;
			break;
		}
	}
	
	while((!RACK_1_PROXY)||(!NORMAL_BALL_PROXY))
		control_pid_omni(4*PI,10);
	
	if (!restart)
	{
		test_start_to_tz1();
		test_tz1_to_tz2();
	}
  else 	
		test_start_to_tz2();		
		
	while((zone == ZONE_THREE_SHOOT) || (zone == ZONE_TWO_START))
	{
		if(zone == ZONE_THREE_SHOOT)
		{
			field_error=1;
			test_tz2_to_tz3_shoot();
			tz3_return();
			test_decider();
		}
		else		
			test_tz1_to_tz2();		
	}
	
	
//__________________________________________END OF MAIN RUNNING LOOPS__________________________________________________________________________	
	
	
	while(1)
	{
		pos();
		control_pid_omni(4*PI,6);
	}
		
  /*while(1)
	{
		// left 2_7  0_19 0_21 0_22
		reset(Port2_4);
		reset(Port2_7);
		reset(Port0_19);
		reset(Port0_20);
		reset(Port0_21);
		reset(Port0_22);
		
		cls();
		lcd("2_4");
		set(Port2_4);		
		wait(1);
		
		reset(Port2_4);
		reset(Port2_7);
		reset(Port0_19);
		reset(Port0_20);
		reset(Port0_21);
		reset(Port0_22);
		
		cls();
		lcd("2_7");
		set(Port2_7);
		wait(1);
		
		reset(Port2_4);
		reset(Port2_7);
		reset(Port0_19);
		reset(Port0_20);
		reset(Port0_21);
		reset(Port0_22);
		
		cls();
		lcd("0_19");
		set(Port0_19);
		wait(1);
		
		reset(Port2_4);
		reset(Port2_7);
		reset(Port0_19);
		reset(Port0_20);
		reset(Port0_21);
		reset(Port0_22);
		
		cls();
		lcd("0_20");
		set(Port0_20);
		wait(1);
		
		reset(Port2_4);
		reset(Port2_7);
		reset(Port0_19);
		reset(Port0_20);
		reset(Port0_21);
		reset(Port0_22);
		
		cls();
		lcd("0_21");
		set(Port0_21);
		wait(1);
		
		reset(Port2_4);
		reset(Port2_7);
		reset(Port0_19);
		reset(Port0_20);
		reset(Port0_21);
		reset(Port0_22);
		
		cls();
		lcd("0_22");
		set(Port0_22);
		wait(1);
	}*/		
	//	
	while(proxy_check()!=START_PROXY);
	
	cls();
	lcd((char*)"NEXT");
	
	go_to_zone_one_start();	
	angle_90();
	
	zone = ZONE_TWO_START;
	
	while(1)
	{
		int proxy_op;
		proxy_op = proxy_check();
		switch(proxy_op)
		{
			case START_PROXY:
				                angle_0();
	                      //wait_pid(3000);
												go_to_zone_one_shoot();
			                  set(Port2_7);
	                      wait_pid(9000);
			                  reset(Port2_7);
			                  go_to_zone_one_start();
			                  //wait_pid(1000);
	                      angle_90();
				                break;
			case ZONE_TWO_PROXY:
												angle_0();
	                      //wait_pid(3000);
												go_to_zone_two_shoot();
			                  set(Port2_7);
	                      wait_pid(9000);
			                  reset(Port2_7);
			                  go_to_zone_two_start();
			                  //wait_pid(1000);
	                      angle_90();
												break;
			case ZONE_THREE_PROXY:
												angle_0();
	                      //wait_pid(3000);
												go_to_zone_three_shoot();
												set(Port2_7);
	                      wait_pid(9000);
												reset(Port2_7);
			                  go_to_zone_two_start();
			                  //wait_pid(1000);
	                      angle_90();
												break;
		}
	}
}
