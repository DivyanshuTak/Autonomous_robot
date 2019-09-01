#include"common.h"
//PS2 variables
bool ip[4][8],ps_up,ps_right,ps_down,ps_left,ps_square,ps_triangle,ps_circle,ps_cross,ps_select,ps_start,
	    ps_r1,ps_r2,ps_l1,ps_l2;
int ps2_val[4], one=0,two=0,three=0,four=0,turn_adc=0,turn_yes=0,golden_counter;
double temp_four=0;
double divyanshu=0,divyanshu_ka_chut=0;
bool enc_diff_1_2=0,restart_2=0;
//
//PID variables
double b_heading,hold_angle=0,value_1,value_2,value_3, proportional, integral,derivative,integrald,rate,
	     control,old_control,icontrol;
float kp=0,ki=0,kd=0,manual_kp=0,manual_ki=0,manual_kd=0,auto_kp=0,auto_ki=0,auto_kd=0;
char dummyl,dummyr,txt1,txt2,txt3;
//
// LIMITING factors
double lim11_lim,lim12_lim,lim21_lim,lim22_lim;
double lim11=80,lim12=48;             //limits for channel 1 -> with stop value 64    80,48
double lim21=208,lim22=176;           //limits for channel 2 -> with stop value 192   208,176
int control_speed=20,flag=0,safety,minor_adjustment_speed;
double acc=0;
//
//Flags/ misc.
int speed=0,count_cycle=0,mode=0,stop_count=0,count_cycle1=0,turn_speed_limit,turn_speed,count_cycle2=0,
    res[4]={0},ans=0,time=0,turn_final,field,pos_flag=1,sop_flag=0,p_ang=0,ang_time=0;;
double ang=0,x_hold=199,y_hold=6717,angle_change=0,difference,heading_2=0;
int plane_select=1, err_count1=0,err_count2=0,centre_point=0,time_limit,tz2_counter=0,heading_1=0;
int count=0,aniruddha=0;
bool field_error=0,tz2_ball_proxy=0;		
//
//DED recon - encoder
int b1,a1,previous,tickl,tickr,prvvaluel,prvvaluer,prevposition,templ=0,tempr=0;
double th=0;
double left_ddis,dis_per_count_left,right_ddis,dis_per_count_right,x_dis,y_dis,left_dia,right_dia;
//
// line sensor + line sensor PID
double proportional_line_sense, kp_line_sense, integral_line_sense, integrald_line_sense, ki_line_sense, rate_line_sense, prev_err, derivative_line_sense, kd_line_sense, 
	     control_line_sense, icontrol_line_sense;
int sum_sensor_v=0,sum_sensor_h=0;
bool sensor_value[8];
//
// THEME specific variables
  int zone,count_task=0;
  bool next=0,dis_flag,shoot_3,zone_complete=0,in_tz1=0,golden_proxy=0;
  int junction_count=0;
	double error_v,error_h, velocity, velocity_final;
  double x_co_arr[10]={0,0,0,0,0,0,0,0,0,0},y_co_arr[10]={0,0,0,0,0,0,0,0,0,0};
  bool flag_shoot_3_2= 0;
//
void test_shoot()
{	 
   int i=4000;          //15000
   count_cycle1=0;	
	 switch (zone)
	 {
		 case ZONE_ONE_START:
			 
	 while(count_cycle1<=80)                                                    // delay added before gripper open
	  {
		
	  	pos(); 
	  	control_pid_omni(4*PI,10);
	  }
		
	
	 ExtPrintbin(COM3,1,170);
	 i=4000;
	                                               // restore kp
	 reset(PISTON_3);
	 count_cycle1=0;
	 while(count_cycle1<=70)
	 {   
		    pos();
		    control_pid_omni(4*PI,10);
		  //control_pid_omni(0,7);
	 }
	 
			 set(PISTON_1);
		   i=5000;                                                 // delay added after actuation 
	     count_cycle1=0;
		   while(count_cycle1<=90)
			 {
	       pos();				
				 control_pid_omni(4*PI,10);
			 }
       break;      
		  
		 
		 
		 case ZONE_TWO_START:
	  i=5000;
	  while(i--)                                                    // delay added before gripper open
	 {
		control_pid_omni(4*PI,10);
	 }
	 ExtPrintbin(COM3,1,170);
	 i=4000;
	                                               // restore kp
	// reset(GRIPPER);
	 count_cycle1=0;
	 while(count_cycle1<=50)
	 {
		    control_pid_omni(4*PI,10);
		  //control_pid_omni(0,7);
	 }
	 
			 set(PISTON_1);
		   i=2000;                                                 // delay added after actuation 
		   while(i--)
				 control_pid_omni(4*PI,10);
       break;      
		 


			 
		 case	ZONE_THREE_SHOOT:
			 x_dis=0;
   	   y_dis=0;
	     prvvaluel=0;
	     prvvaluer=0;
	     T0TC=0;
	     T1TC=0;
	     enc_diff_1_2=0;
			 count_cycle1=0;
		    kp=0;
		    i=60000;
		    double diff=0,temp_tick=0;
		 if(!flag_shoot_3_2) 
		 {  
		 while(count_cycle1<=200)
				{
					 pos();
					 diff = temp_tick - y_dis ;
					 if ((diff < 1)&&(y_dis > 0))
					 {
						// break;
					 }
					 control_pid_omni(25*Degree_to_Rad,10 );
					 temp_tick = y_dis; 
				}
			}
 
			else
			{
			while(count_cycle1<=250)
				{
					 pos();
					 diff = temp_tick - y_dis ;
					 if ((diff < 1)&&(y_dis > 0))
					 {
						// break;
					 }
					 control_pid_omni(10*Degree_to_Rad,10);
					 temp_tick = y_dis; 
				}
			}
			
			
			   reset(PISTON_3);
			   ang=90;
				 ExtPrintbin(COM3,1,170);
				count_cycle1=0;
			  while(count_cycle1<=50)                                                    // delay added before gripper open
	      {
		      control_pid_omni(0*Degree_to_Rad,10);
	      }
		//		set(PISTON_2);
		//		set(PISTON_2);
				
	     ExtPrintbin(COM3,1,210);
	      set(PISTON_1);
			 while(count_cycle1<=200)                                                    // delay added before gripper open
	      {
		      control_pid_omni(0*Degree_to_Rad,10);
	      }	
				kp=2;
       break;			 
	 }	 
//_________________________restore everything to its normal condition________________________________________________________________________	 
	 //reset(PISTON_1);
	 reset(PISTON_1);
	 reset(PISTON_3);
	 ///reset(GRIPPER);
 }

//
void test_decider()
{	
	next=0;
	int rack_one_counter,rack_two_counter,ball_counter,golden_ball_counter;
	bool rack_two_flag=0,rack_one_flag=0;
	
	ball_counter=0;				  
	rack_one_flag=0;
	rack_two_flag=0;
	rack_one_counter=0;
	rack_two_counter=0;
	golden_ball_counter=0;
	
	while(!next)
	{
		control_pid_omni(4*PI,10);
		
		if(!RACK_1_PROXY)
			rack_one_counter++;
		
		if(!RACK_2_PROXY)
			rack_two_counter++;
	
		if(rack_one_counter>400)
		{
			rack_one_flag=1;
			rack_one_counter=401;
		}
		
		if(rack_two_counter>400)
		{
			rack_two_flag=1;
		  rack_one_counter=401;
		}
		
		if(rack_one_flag)// || rack_two_flag)
		  if(!NORMAL_BALL_PROXY)
			  ball_counter++;
		
		if(ball_counter>5000)
		{			
			CLOSE_GRIPPER;
			
			count_cycle1=0;
			while((!RACK_1_PROXY || !RACK_2_PROXY))// && rack_one_flag)
			{
				control_pid_omni(4*PI,10);
			}
			
			if(rack_one_flag)//|| rack_two_flag)                    //(rack_one_flag)
			{							
				next=1;
			}			
			else
			{
				ball_counter=0;				
			}
		}
		
		if(!GOLDEN_BALL_PROXY)
		{
			golden_ball_counter++;
		}
		
		if(golden_ball_counter>7000)
		{
			CLOSE_GRIPPER;
			count_cycle1=0;
			rack_one_flag=0;
			rack_two_flag=0;
			next=1;
		}
	}
	//CLOSE_GRIPPER;                                 // CLOSING CONDITION OF GRIPPER IS KEPT OPERATOR INDEPENDENT
	while((!RACK_1_PROXY || !RACK_2_PROXY) || (count_cycle1<75))
		control_pid_omni(4*PI,10);
	
	if(rack_one_flag && rack_two_flag)
		zone = ZONE_ONE_START;
	else if(rack_one_flag)
		zone = ZONE_TWO_START;
	else
		zone = ZONE_THREE_SHOOT;	
}
//
void test_tz1_to_tz2()
{	
	int counterv_90=0;
	if(!zone_complete)
	{
		CLOSE_GRIPPER;
		
		next=0;
		count_cycle1=0;
		sum_sensor_h=0;
		sum_sensor_v=0;
		x_dis=0;
		T0TC=0;
		prvvaluel=0;
		
		while(!next)
		{
			pos();
			sensor_val_update_h();
			sensor_val_update_v();
			
			if(count_cycle1<270)
				control_pid_omni((-155)*Degree_to_Rad,20);
			else
				control_pid_omni((-155)*Degree_to_Rad,3);
			
			if((sum_sensor_h>3 || sum_sensor_v>0) && count_cycle1>500)                               // backup 
				next=1;
			
			if(x_dis<-550)
				next=1;
		}
		
		int i=1600;	
		next=0;
		count_cycle1=0;
		icontrol=15;
		
		while(!next)
		{
			sensor_val_update_h();
			sensor_val_update_v();
			
			hold_angle+=0.01;                                              // 0.01
			if(hold_angle>75)
				hold_angle=75;
			
			if(count_cycle1<320)
				control_pid_omni((-70-ang)*Degree_to_Rad,13);                  // 50
			else
			 	control_pid_omni((-70-ang)*Degree_to_Rad,4);                  // 60
			
			if(((sum_sensor_v>1)||(sum_sensor_h>1)) && ang>30)
				next=1;
		}
		next=0;
		hold_angle=75;
		if (sum_sensor_v>1)
		{
			cls();
			lcd((char *)"if");
			next=0;
			while(sum_sensor_v>1)
			{
				error_v = sensor_val_update_v();
				control_pid_omni((-60-ang)*Degree_to_Rad,4);                  // 50
			}
			
			while(sum_sensor_v<1)
			{
				error_v = sensor_val_update_v();
				control_pid_omni((-60-ang)*Degree_to_Rad,4);                  // 50
			}		
			
			next=0;			
			while(!next)
			{
				control_pid_line_follow(FORWARD,5,error_v);
				error_v = sensor_val_update_v();
				if(!RIGHT_PROXY)
				{
					next=1;
				}
			}
		}
		else
		{
			cls();
			lcd((char *)"else");
			while(sum_sensor_v<1)
			{
				sensor_val_update_v();
				control_pid_omni((-90-ang)*Degree_to_Rad,4);                  // 50
			}
			next=0;			
			sum_sensor_h=0;
			while(!next)
			{
				sensor_val_update_h();
				control_pid_line_follow(FORWARD,5,error_v);
				error_v = sensor_val_update_v();
				if(sum_sensor_h>2)
				{
					next=1;
				}
			}
			while(!next)
			{
				sensor_val_update_h();
				control_pid_line_follow(FORWARD,5,error_v);
				error_v = sensor_val_update_v();
				if(sum_sensor_h<2)
				{
					next=1;
				}
			}
			
			while(!next)
			{
				sensor_val_update_h();
				control_pid_line_follow(FORWARD,5,error_v);
				error_v = sensor_val_update_v();
				if(!RIGHT_PROXY)
				{
					next=1;
				}
			}
			
		}
		
		/*while(!next)
		{
			
			sensor_val_update_h();
			error_v = sensor_val_update_v();
			
				control_pid_omni((-60-ang)*Degree_to_Rad,4);    //-50
			if(sum_sensor_h==0)
				next=1;
		}
		  next=0;
			while(!next)
		{
			sensor_val_update_h();
			error_v = sensor_val_update_v();
			
				control_pid_omni((-60-ang)*Degree_to_Rad,4);                 //-50
			if(sum_sensor_h>0) //&& ang>30)
				next=1;
		}
				
		next=0;
		count_cycle1=0;
		sum_sensor_v=0;
		hold_angle=ZONE_2_ANGLE+0.5;         //1.5
		
		i=1000;
		while(i--)
		{
			control_pid_omni((-90*Degree_to_Rad),4);
			error_v = sensor_val_update_v();
		}
		
		sum_sensor_v=0;	
		
		while(sum_sensor_v==0)
		{
			error_v = sensor_val_update_v();	
			control_pid_omni((-50-ang)*Degree_to_Rad,4);//(10*Degree_to_Rad,10);
		}
			
    i=500;
		while(i--)
		{
			control_pid_omni(10,8);          //0,5
			error_v = sensor_val_update_v();
		}
		
		while(!next)
		{
			error_v = sensor_val_update_v();
			if(error_v>5)
				find_line(BACKWARD,RIGHT,6);
			
			control_pid_line_follow(FORWARD,5,error_v);
					
			if(sum_sensor_v>0 && !Pin0_27)	
				next=1;
			else if(sum_sensor_v>3)
				next=1;
		}
		
		i=1500;
		while(i--)
		{
			error_v = sensor_val_update_v();
				control_pid_line_follow(BACKWARD,10,error_v);
			
		}
		*/
		i=1200;//1200;
		while(i--)
		{
			control_pid_omni(90*Degree_to_Rad,7);
		}
		
		while(RIGHT_PROXY)
		{
			error_v = sensor_val_update_v();
			control_pid_line_follow(BACKWARD,5,error_v);
		}
	
		test_shoot();//wait_pid(10000);       //we shoot here
		
		while(ang>0)
		{
			hold_angle-=0.01;    //0.005
			
			if(hold_angle<0)
				hold_angle=0;
			
			if(ang>10)
				control_pid_omni(4*PI,8);      //10
			else
				control_pid_omni(4*PI,2);     //5
		}	
		
		test_decider();  //decide where to go
		
		zone_complete=1;
 	}
	while(zone == ZONE_TWO_START)
	{
		CLOSE_GRIPPER;
		next=0;
	  count_cycle1=0;
	  icontrol=15;
		while(ang<70)
		{
			hold_angle+=0.005;
      control_pid_omni(4*PI,10);			
		}
		
		hold_angle=ZONE_2_ANGLE+3.8;                //2.8

/*		
		if (!field_error)
		hold_angle=ZONE_2_ANGLE;
		else 
			hold_angle=ZONE_2_ANGLE+2;
	*/		
		
		test_shoot();
		
		while(ang>0)
	  {
			hold_angle-=0.005;
			
			if(hold_angle<0)
				hold_angle=0;
			
			if(ang>10)
				control_pid_omni(4*PI,8);
			else
				control_pid_omni(4*PI,2);
			
			error_h=sensor_val_update_h();
	  }
		int i=0;
		i=8000;
		while(i--)
		{
			control_pid_omni(4*PI,5);
		}
		cls();
		lcd((char *)"mabkl");
		while(FORWARD_PROXY)//&&(sum_sensor_h<1))    //saamne wali proxy 
		{
		//	error_h=sensor_val_update_h();
		//	control_pid_line_follow(BACKWARD,5,error_h);
        control_pid_omni(160*Degree_to_Rad,6);			
		}

		cls();
		lcd((char *)"bypassed");
		test_decider();
	}
}

//
void test_tz2_to_tz3_shoot()
{
	bool safety=0;
	CLOSE_GRIPPER;
	int u=3000;
	while(u--)
	{
		control_pid_omni(4*PI,5);
	}
	/*
	while(ang<90)                                                // snippet for ctrm from wall to line  
	{
		sensor_val_update_h();
		if(ang<10)
		{
			icontrol=3;
		}
		else if (ang<60)
		{
		  icontrol = ang;
			if(icontrol>10)
				icontrol=10;
		}
		else 
		{
			icontrol=4;
		}
		control_pid_omni((27-ang)*Degree_to_Rad,10);     //210		
	}
	*/
	                                                 ////  main working snippet if the above one doesnt work uncomment it 
	while(ang<90)
	{
		hold_angle+=0.01;                   // 0.005
			
		icontrol = ((90-ang)/15) + 5;          //4
				
		if(hold_angle>87)                   //87
			hold_angle=90;
		
		//if(ang<50)
			control_pid_omni(4*PI,10);
		//else
			//control_pid_omni(4*PI,2);
	}
	icontrol=8;                              // restore icontrol to its normal value
	hold_angle=90;
	next=0;
	x_dis=0;
	T0TC=0;
	prvvaluel=0;
	int i= 2000,counterv_90=0;            // 1600
	count_cycle1=0;
	sum_sensor_v=0;
	while(!next)
	{
		sensor_val_update_v();
		
		if(sum_sensor_v==4)
		{
			//next=0;
			while(1)
			{
				sensor_val_update_v();
				error_h=sensor_val_update_v();
		   // control_pid_line_follow(BACKWARD,12,error_h);
				control_pid_omni((-83*Degree_to_Rad),20);
				if(sum_sensor_v<=2)
				{
					counterv_90++;
					break;
				}
			}
		
		}
		
		pos();
		
		if(x_dis>-1000)
			control_pid_omni((-66+(90-ang))*Degree_to_Rad,25);           //20
		else if(x_dis>-2500) //3000
			control_pid_omni((-83+(90-ang))*Degree_to_Rad,18);      //15          //18
		else if(x_dis>-3000)             //3200
			control_pid_omni((-79+(90-ang))*Degree_to_Rad,4);  //5               //5
		else
		  control_pid_omni((-79)*Degree_to_Rad,2);  //81
		
		if(!Pin0_27 && x_dis<-3000)			// 3300
		{
			safety=1;
			next=1;
		}
//___________________________________EMERGENCY STOP CONDITION IF ENCODER FAILS________________________________________________________-_____		
	
		if (count_cycle1>535)
		{
			i=0;//5000;
			while(i--)
			{
					control_pid_omni(70*Degree_to_Rad,25);
			}
			//next=1;
		}
//______________________________________________________________________________________________________________________________________________		
	}
	cls();
	lcd(counterv_90);
	
	
//____________________________________________CHANGE IN CONDITION FOR SHOOT IN TZ3________________________________________________________________
	i=2000;
	while(i--)
	{
		control_pid_omni(65*Degree_to_Rad,8);
	}
	
	
	while(Pin0_27)
	{
		control_pid_omni((70)*Degree_to_Rad,6);          //5
	}
	i=500;
	kp=0;
	while(i--)
	{		
		pos();		
		ang=90;
		control_pid_omni(-10*Degree_to_Rad,8);
	}
	
	kp=0;
	shoot_3=1;	
	test_shoot();
	CLOSE_GRIPPER;
	kp=0;		
	x_dis=0;
	T0TC=0;
	prvvaluel=0;
	
	while(x_dis>-270)
	{
		pos();
		if(x_dis>-200)
			control_pid_omni(-70*Degree_to_Rad,5);
		else
			control_pid_omni(-70*Degree_to_Rad,4);
	}
	
	i= 500;
	while(i--)
	{
		control_pid_omni(60*Degree_to_Rad,10);
	}
	
	shoot_3=0;
	test_shoot();
	
//___________________________________________________S H O O T  # 2 PATH CHANGE	__________________________________________________________________________
	/*
	while(Pin0_27)
	{
		control_pid_omni((-70)*Degree_to_Rad,5);
	}
	
	shoot_3=0;	
	
	kp=0;		
	x_dis=0;
	T0TC=0;
	prvvaluel=0;
	
	while(x_dis>320)
	{
		pos();
		if(x_dis>300)
			control_pid_omni(70*Degree_to_Rad,5);
		else
			control_pid_omni(70*Degree_to_Rad,4);
	}
	
	i= 500;
	while(i--)
	{
		control_pid_omni(-60*Degree_to_Rad,10);
	}
	
	shoot_3=0;
	test_shoot();
	*/
//------------------------------------------------------------FUNCTON ENDS-------------------------------------------------------------------------------------	
	
}
//
/*
DESCRIPTION : Bot first moves towards tz1 (start_to_load)
              then it waits for ball proxy (test_decider_2)
              then it shoots in tz1(tz1_shoot)
   
*/
void test_start_to_tz1()
{
	 start_to_load();
	if(!restart_2)
	{
    while(NORMAL_BALL_PROXY)
	   {
		   control_pid_omni(4*PI,5);
	   }
		 int	i=0;
		while(i--)
		{
			control_pid_omni(4*PI,5);
		}
	  TZ1_NEW();//tz1_shoot();
	}		 
}
//
int proxy_check()
{	
	int i_local;
	
	while(1)
	{
		control_pid_omni(4*PI,20);
		
		if(Start_proxy==0)
		{	
			i_local=100;
  		while(i_local--)
				control_pid_omni(4*PI,20);
			
			if(Start_proxy==0)
		  {
				i_local=100;
  		  while(i_local--)
					control_pid_omni(4*PI,20);
			  if(Start_proxy==0)
		    {					
					while(Start_proxy==0)
							 control_pid_omni(4*PI,20);
						 
	 				//while(Start_proxy==1)
					//		control_pid_omni(4*PI,20);
					cls();
          lcd((char*)"Start proxy");					
					return START_PROXY;
  			}
			}	  		
		}
		else if(Zone_two_proxy==0)
		{
			i_local=100;
  		while(i_local--)
				control_pid_omni(4*PI,20);
			
			if(Zone_two_proxy==0)
		  {
				i_local=100;
  		  while(i_local--)
					control_pid_omni(4*PI,20);
			  if(Zone_two_proxy==0)
		    {					
					while(Zone_two_proxy==0)
							 control_pid_omni(4*PI,20);
						 
	 				//while(Start_proxy==1)
					//		control_pid_omni(4*PI,20);
					cls();
          lcd((char*)"Two proxy");
					return ZONE_TWO_PROXY;
				}
			}
		}
		else if(Zone_three_proxy==0)
		{
			i_local=100;
  		while(i_local--)
				control_pid_omni(4*PI,20);
			
			if(Zone_three_proxy==0)
		  {
				i_local=100;
  		  while(i_local--)
					control_pid_omni(4*PI,20);
			  if(Zone_three_proxy==0)
		    {					
					while(Zone_three_proxy==0)
							 control_pid_omni(4*PI,20);
						 
	 				//while(Start_proxy==1)
					//		control_pid_omni(4*PI,20);
					cls();
          lcd((char*)"Three proxy");
					
					return ZONE_THREE_PROXY;
				}
			}
		}
	}	
}
//
void go_to_zone_two_start()
{
	if(zone == ZONE_THREE_SHOOT)
	{
		      next=0;
					time_limit=200;
					velocity=12;
					velocity_final=15;
					acc=0.0007;	
          junction_count=0;
          dis_flag=0;	
          x_dis=0;
					T0TC=0;
					prvvaluel=0;
					
					while(!next)
					{
						static int actual_line_control=3;
						
						error_h=sensor_val_update_h();
																		
						if(sum_sensor_h==4 && time_limit>50)
						{
							junction_count++;							
							
							#if DEBUG
								cls();
								lcd(junction_count);
							#endif
							
							int i=2;
							while(i--)
							{
								while(sum_sensor_h>3)
								{
									sensor_val_update_h();
									
									if(!actual_line_control)
									{
										control_pid_line_follow(LEFT,velocity,0);
										actual_line_control=3;
									}
								}
								
								control_pid_line_follow(LEFT,velocity,0);
								sensor_val_update_h();								
							}
							
							if(junction_count==3)
							{									
								velocity_final = velocity = 16;
							}
							else if(junction_count==5)
							{
								velocity = velocity_final = 7;
								x_dis=0;
					      T0TC=0;
					      prvvaluel=0;
								dis_flag=1;
							}		
              else if(junction_count==6)
                next=1;
							
              time_limit=0;							
						}		        
												
						if(dis_flag==1)
						{							
							pos();
							if(x_dis<-750)
							{
								lowerline();
								lcd(x_dis);
								next=1;
							}
						}
						
						if(velocity<velocity_final && acc>0)
							velocity+=acc;
						else if(velocity>velocity_final && acc<0)
							velocity+=acc;
						else
							velocity = velocity_final;
												
						if(actual_line_control==0)
						{
							
							control_pid_line_follow(LEFT,velocity,error_h);
							actual_line_control=3;
						}
						
						actual_line_control--;
					} 
	}
	else if(zone==ZONE_TWO_SHOOT)
	{
		      next=0;
					time_limit=100;
					velocity=12;
					velocity_final=12;
					acc=0.0007;	
          junction_count=0;
          dis_flag=0;		
					
					while(!next)
					{
						error_h=sensor_val_update_h();
																		
						if(sum_sensor_h==4  && time_limit>70)
						{
							junction_count++;							
							
							#if DEBUG
								cls();
								lcd(junction_count);
							#endif
							
							int i=2;
							while(i--)
							{
								while(sum_sensor_h>3)
								{
									sensor_val_update_h();
									control_pid_line_follow(LEFT,velocity,0);
								}
								sensor_val_update_h();
								control_pid_line_follow(LEFT,velocity,0);
							}
							
							if(junction_count==2)
							{
								velocity = velocity_final = 7;
								x_dis=0;
					      T0TC=0;
					      prvvaluel=0;
								dis_flag=1;
							}							
              time_limit=0;							
						}		        
						
						pos();
						
						if(dis_flag==1)
						{
							if(x_dis<-750)
								next=1;
						}
						
						if(velocity<velocity_final && acc>0)
							velocity+=acc;
						else if(velocity>velocity_final && acc<0)
							velocity+=acc;
						else
							velocity = velocity_final;
						
						control_pid_line_follow(LEFT,velocity,error_h);
					}
	}	
	zone = ZONE_TWO_START;
	return;	
}
//
void go_to_zone_one_start()
{
	if(zone==START_POINT)
	{
		     velocity = 10;
				 next=0;
				 time_limit = 200;
		     junction_count=0;
				 				 
				 find_line(LEFT, BACKWARD, 10);
				 //find_line(LEFT, BACKWARD, 7);                       //After moving out of start zone try to find the line. I know that it should be RIGHT,BACKWARD but the code works
																														 //fine this way!!				 				 
				 time_limit=0;
				 junction_count=0;
				 velocity=13;
				 velocity_final=22;
				 acc=0.0007;
				 next=0;
				 kp_line_sense=6;
				 kd_line_sense=2;
				 
				 while(!next)
				 {
						error_v=sensor_val_update_v();
						
						if(sum_sensor_v==4  && time_limit>50)
						{
							junction_count++;
							time_limit=0;
							#if DEBUG
								cls();
								lcd(junction_count);
							#endif
							int i=2;
							while(i--)
							{
								while(sum_sensor_v>2)
								{
									sensor_val_update_v();
									control_pid_line_follow(BACKWARD,velocity,0);
								}
								sensor_val_update_v();
								control_pid_line_follow(BACKWARD,velocity,0);
							}
							
							if(junction_count==1)
							{
								next=1;
								junction_count=0;
							}
						}
						
						if(velocity==velocity_final)
						{
							acc=-10;//-0.008;
							velocity_final=9;
						}
						
						if(velocity<velocity_final && acc>0)
							velocity+=acc;
						else if(velocity>velocity_final && acc<0)
							velocity+=acc;
						else
							velocity = velocity_final;
						
						control_pid_line_follow(BACKWARD,velocity,error_v);
					}
					
					next=0;
					time_limit=0;
					velocity=12;
					velocity_final=15;
					acc=0.0007;
					
					//Encoder values zeroed.
					x_dis=0;
					T0TC=0;
					prvvaluel=0;					
					
					find_line(FORWARD, RIGHT, 10);                                //I know that it should be (FORWARD,RIGHT,10) but the placement of sensors is like this!
				  
					while(!next)
					{
						error_h=sensor_val_update_h();
												
						pos();
						
						cls();
						lcd(x_dis);
						
						if(x_dis>800 && velocity>7)
							velocity=7;
						
            if(x_dis > 1000)
              next=1;						
             
            if(sum_sensor_h==0 && error_h>0)
								control_pid_omni(-(50*Degree_to_Rad),7);
						else
						    control_pid_line_follow(RIGHT,velocity,error_h);
					}
					
					/*if(sum_sensor_h==0)
					{
						while(!sum_sensor_h)
						{
							sensor_val_update_h();
							control_pid_omni(0,7);
						}
					}*/
	}
	else if(zone==ZONE_ONE_SHOOT)
	{
		      next=0;
		      junction_count=0;
					time_limit=0;
					velocity=11;
					velocity_final=15;
					acc=0.0007;
		      dis_flag=0;
		      while(!next)
					{
						error_h=sensor_val_update_h();
						
						if(sum_sensor_h==4  && time_limit>50)
						{
							junction_count++;							
							#if DEBUG
								cls();
								lcd(junction_count);
							#endif
							int i=2;
							while(i--)
							{
								while(sum_sensor_h>2)
								{
									sensor_val_update_h();
									control_pid_line_follow(LEFT,velocity,0);
								}
								sensor_val_update_h();
								control_pid_line_follow(LEFT,velocity,0);
							}
							
							if(junction_count==2)
							{
								//resetting the encoder
								x_dis=0;
								T0TC=0;
								prvvaluel=0;		
								//--------------------
								
								//-Slowing down the robot-
                velocity = 8;
								velocity_final = 8;								
								acc=0;
								//------------------------
								
								junction_count=0;
								dis_flag=1;
							}
							time_limit=0;
						}
						
						pos();
						
						if(dis_flag && x_dis<-650)
            {
							next=1;
							dis_flag=0;							
						}
								
												
						if(velocity<velocity_final && acc>0)
							velocity+=acc;						
						else
							velocity = velocity_final;
						
						control_pid_line_follow(LEFT,velocity,error_h);
					}
	}
	
	zone= ZONE_ONE_START;
	return;
}
//
void go_to_zone_three_shoot()
{
	 if(zone == ZONE_TWO_START)
	 {
					next=0;
					time_limit=200;
					velocity=12;
					velocity_final=25;
		      junction_count=0;
					acc=0.00075;					
					
					while(!next)
					{
						error_h=sensor_val_update_h();
																		
						if(sum_sensor_h==4  && time_limit>50)
						{
							junction_count++;							
							
							#if DEBUG
								cls();
								lcd(junction_count);
							#endif
							
							int i=2;
							while(i--)
							{
								while(sum_sensor_h>=3)
								{
									sensor_val_update_h();
									control_pid_line_follow(RIGHT,velocity,0);
								}
								sensor_val_update_h();
								control_pid_line_follow(RIGHT,velocity,0);
							}
							
							if(junction_count==3)
							{									
								velocity_final = velocity=16;
							}
							else if(junction_count==4)
							{
								velocity = velocity_final = 5;
							}
							else if(junction_count==5)
							{
								next=1;
								junction_count=0;
							}
							
              time_limit=0;							
						}		        
						
						if(velocity<velocity_final && acc>0)
							velocity+=acc;
						else if(velocity>velocity_final && acc<0)
							velocity+=acc;
						else
							velocity = velocity_final;
						
						control_pid_line_follow(RIGHT,velocity,error_h);
					}								
	 }
	 else if(zone == ZONE_ONE_START)
	 {
		      velocity=10;
		      next=0;
		 
		      while(!next)
					{
						error_h=sensor_val_update_h();
						error_v=sensor_val_update_v();
						
						if(abs(error_v)==3 && sum_sensor_v==2)
							next=1;
						
						if(sum_sensor_h==4  && time_limit>70)
						{
							junction_count++;							
							
							#if DEBUG
								cls();
								lcd(junction_count);
							#endif
							
							int i=2;
							while(i--)
							{
								while(sum_sensor_h>3)
								{
									sensor_val_update_h();
									control_pid_line_follow(LEFT,velocity,0);
								}
								sensor_val_update_h();
								control_pid_line_follow(LEFT,velocity,0);
							}
							
							if(junction_count==1)
							{
								next=1;
								junction_count=0;
							}							
							time_limit=0;
						}
						
						control_pid_line_follow(LEFT,velocity,error_h);
					}
					
					find_line(LEFT,BACKWARD,7);
           
         	velocity=10;
					velocity_final=16;
					acc=0.0007;
		      next=0;
					junction_count=0;
					time_limit=0;
		 
		      while(!next)
					{						
						error_v=sensor_val_update_v();
												
						if(sum_sensor_v==4  && time_limit>200)
						{
							junction_count++;							
							
							#if DEBUG
								cls();
								lcd(junction_count);
							#endif
							
							int i=2;
							while(i--)
							{
								while(sum_sensor_v>3)
								{
									sensor_val_update_v();
									control_pid_line_follow(BACKWARD,velocity,0);
								}
								sensor_val_update_v();
								control_pid_line_follow(BACKWARD,velocity,0);
							}
							
							if(junction_count==1)
							{
								next=1;
								junction_count=0;
							}							
							time_limit=0;
						}
						
						if(velocity==velocity_final)
						{
							acc=-0.01;
							velocity_final=7;
						}
						
						if(velocity<velocity_final && acc>0)
							velocity+=acc;
						else if(velocity>velocity_final && acc<0)
							velocity+=acc;
						else
							velocity = velocity_final;
						
						control_pid_line_follow(BACKWARD,velocity,error_v);
					}		

          velocity=13;
					velocity_final=26;
					next=0;
					acc=0.0007;
					find_line(FORWARD,RIGHT,7);                   //I know that it should be (backward,right,7)  but the configuration of sensor array forced me to use this
					
					while(!next)
					{
						error_h=sensor_val_update_h();
						
						if(sum_sensor_h==4  && time_limit>70)
						{
							junction_count++;							
							
							#if DEBUG
								cls();
								lcd(junction_count);
							#endif
							
							int i=2;
							while(i--)
							{
								while(sum_sensor_h>3)
								{
									sensor_val_update_h();
									control_pid_line_follow(RIGHT,velocity,0);
								}
								sensor_val_update_h();
								control_pid_line_follow(RIGHT,velocity,0);
							}
							
							if(junction_count==3)
							{
								velocity=velocity_final=14;
							}
							else if(junction_count==4)
							{
								velocity=velocity_final=7;
							}
							else if(junction_count==5)
							{				
								next=1;
								junction_count=0;
							}
							time_limit=0;
						}
						
						if(velocity<velocity_final && acc>0)
							velocity+=acc;
						else if(velocity>velocity_final && acc<0)
							velocity+=acc;
						else
							velocity = velocity_final;
						
						control_pid_line_follow(RIGHT,velocity,error_h);
					}								
					
	 }
	 zone=ZONE_THREE_SHOOT;
	 return;
}
//
void go_to_zone_two_shoot()
{		 
	 if(zone == ZONE_TWO_START)
	 {
					next=0;
					time_limit=200;
					velocity=12;
					velocity_final=15;
					acc=0.0007;			
          junction_count=0;		 
					
					while(!next)
					{
						error_h=sensor_val_update_h();
																		
						if(sum_sensor_h==4  && time_limit>70)
						{
							junction_count++;							
							
							#if DEBUG
								cls();
								lcd(junction_count);
							#endif
							
							int i=2;
							while(i--)
							{
								while(sum_sensor_h>3)
								{
									sensor_val_update_h();
									control_pid_line_follow(RIGHT,velocity,0);
								}
								sensor_val_update_h();
								control_pid_line_follow(RIGHT,velocity,0);
							}
							
							if(junction_count==1)
							{									
								velocity_final = velocity=7;
							}
							else if(junction_count==2)
							{
								next=1;
								junction_count=0;
							}
							
              time_limit=0;							
						}		        
						
						if(velocity<velocity_final && acc>0)
							velocity+=acc;
						else if(velocity>velocity_final && acc<0)
							velocity+=acc;
						else
							velocity = velocity_final;
						
						control_pid_line_follow(RIGHT,velocity,error_h);
					}								
	 }
	 else if(zone == ZONE_ONE_START)
	 {
		      velocity=11;
					velocity_final=11;
		      next=0;	 
		      
		      while(!next)
					{
						error_h=sensor_val_update_h();
						error_v=sensor_val_update_v();
						
						if(abs(error_v)==3 && sum_sensor_v==2)
							next=1;
						
						if(sum_sensor_h==4  && time_limit>70)
						{
							junction_count++;							
							
							#if DEBUG
								cls();
								lcd(junction_count);
							#endif
							
							int i=2;
							while(i--)
							{
								while(sum_sensor_h>3)
								{
									sensor_val_update_h();
									control_pid_line_follow(LEFT,velocity,0);
								}
								sensor_val_update_h();
								control_pid_line_follow(LEFT,velocity,0);
							}
							
							if(junction_count==1)
							{
								next=1;
								junction_count=0;
							}							
							time_limit=0;
						}
						
						control_pid_line_follow(LEFT,velocity,error_h);
					}
					
					find_line(LEFT,BACKWARD,7);
           
         	velocity=10;
					velocity_final=18;
					acc=0.0007;
		      next=0;
					junction_count=0;
					time_limit=0;
		 
		      while(!next)
					{						
						error_v=sensor_val_update_v();
												
						if(sum_sensor_v==4  && time_limit>200)
						{
							junction_count++;							
							
							#if DEBUG
								cls();
								lcd(junction_count);
							#endif
							
							int i=2;
							while(i--)
							{
								while(sum_sensor_v>3)
								{
									sensor_val_update_v();
									control_pid_line_follow(BACKWARD,velocity,0);
								}
								sensor_val_update_v();
								control_pid_line_follow(BACKWARD,velocity,0);
							}
							
							if(junction_count==1)
							{
								next=1;
								junction_count=0;
							}							
							time_limit=0;
						}
						
						if(velocity==velocity_final)
						{
							acc=-0.01;
							velocity_final=7;
						}
						
						if(velocity<velocity_final && acc>0)
							velocity+=acc;
						else if(velocity>velocity_final && acc<0)
							velocity+=acc;
						else
							velocity = velocity_final;
						
						control_pid_line_follow(BACKWARD,velocity,error_v);
					}		

          velocity=15;
					next=0;
					find_line(FORWARD,RIGHT,7);                   //I know that it should be (backward,right,7)  but the configuration of sensor array forced me to use this
					
					while(!next)
					{
						error_h=sensor_val_update_h();
						
						if(sum_sensor_h==4  && time_limit>70)
						{
							junction_count++;							
							
							#if DEBUG
								cls();
								lcd(junction_count);
							#endif
							
							int i=2;
							while(i--)
							{
								while(sum_sensor_h>3)
								{
									sensor_val_update_h();
									control_pid_line_follow(RIGHT,velocity,0);
								}
								sensor_val_update_h();
								control_pid_line_follow(RIGHT,velocity,0);
							}
							
							if(junction_count==1)
								velocity=7;
							if(junction_count==2)
							{				
								next=1;
								junction_count=0;
							}
							time_limit=0;
						}
						
						control_pid_line_follow(RIGHT,velocity,error_h);
					}								
					
	 }
	 zone= ZONE_TWO_SHOOT;
	 return;
}
//
void go_to_zone_one_shoot()
{	
	 if(zone == ZONE_TWO_START)
	 {
					next=0;
					time_limit=0;
					velocity=8;
					velocity_final=8;
					acc=0.0007;
          junction_count=0;		 
					
					while(!next)
					{
						error_h=sensor_val_update_h();
						error_v=sensor_val_update_v();
						
						if(sum_sensor_v!=0)//if(abs(error_v)==3 && sum_sensor_v==2)
							next=1;
						
						if(sum_sensor_h==4  && time_limit>70)
						{
							junction_count++;							
							
							#if DEBUG
								cls();
								lcd(junction_count);
							#endif
							
							int i=2;
							while(i--)
							{
								while(sum_sensor_h>3)
								{
									sensor_val_update_h();
									control_pid_line_follow(LEFT,velocity,0);
								}
								sensor_val_update_h();
								control_pid_line_follow(LEFT,velocity,0);
							}
							
							if(junction_count==1)
							{				
								//next=1;
								junction_count=0;
							}
							
              time_limit=0;							
						}		        
						
						control_pid_line_follow(LEFT,velocity,error_h);
					}
					
					find_line(LEFT,FORWARD,7);
			 
		     velocity=10;
				 velocity_final=19;
				 acc=0.00065;//0.0007;
				 next=0;
				 time_limit=200;
				 junction_count=0;
				 bool temporary_flag=0;
				 
				 while(!next)
				 {
						error_v=sensor_val_update_v();
					  error_h=sensor_val_update_h();
						
					  if(sum_sensor_h>1 && junction_count>0)
						{
							temporary_flag=1;
							next=1;
						}
					 
						if(sum_sensor_v==4  && time_limit>50)
						{
							junction_count++;
							
							#if DEBUG
								cls();
								lcd(junction_count);
							#endif
							int i=2;
							while(i--)
							{
								while(sum_sensor_v>2)
								{
									sensor_val_update_v();
									control_pid_line_follow(FORWARD,velocity,0);
								}
								sensor_val_update_v();
								control_pid_line_follow(FORWARD,velocity,0);
							}
							
							if(junction_count==2)
							{
								next=1;						
								junction_count=0;
							}
							
							time_limit=0;
						}
						
						if(velocity==velocity_final)
						{
							acc=-0.008;
							velocity_final=7;
						}
						
						if(velocity<velocity_final && acc>0)
							velocity+=acc;
						else if(velocity>velocity_final && acc<0)
							velocity+=acc;
						else
							velocity = velocity_final;
						
						control_pid_line_follow(FORWARD,velocity,error_v);
					}
					
					sensor_val_update_v();
					
					if(!temporary_flag)
						while(sum_sensor_v>0)
						{
							find_line(FORWARD,RIGHT,7);
						}
					
					next=0;
					junction_count=0;
					velocity=15;
					
					while(!next)
					{
						error_h=sensor_val_update_h();
						
						if(sum_sensor_h==4  && time_limit>70)
						{
							junction_count++;							
							
							#if DEBUG
								cls();
								lcd(junction_count);
							#endif
							
							int i=2;
							while(i--)
							{
								while(sum_sensor_h>3)
								{
									sensor_val_update_h();
									control_pid_line_follow(RIGHT,velocity,0);
								}
								sensor_val_update_h();
								control_pid_line_follow(RIGHT,velocity,0);
							}
							
							if(junction_count==1)
								velocity=7;
							if(junction_count==2)
							{				
								next=1;
								junction_count=0;
							}
							time_limit=0;
						}
						
						control_pid_line_follow(RIGHT,velocity,error_h);
					}			
	 }
	 else if(zone == ZONE_ONE_START)
	 {
		      velocity=9;
		      velocity_final=15;
		      acc=0.007;
		      next=0;
		      while(!next)
					{
						error_h=sensor_val_update_h();
						
						if(sum_sensor_h==4  && time_limit>70)
						{
							junction_count++;							
							
							#if DEBUG
								cls();
								lcd(junction_count);
							#endif
							
							int i=2;
							while(i--)
							{
								while(sum_sensor_h>3)
								{
									sensor_val_update_h();
									control_pid_line_follow(RIGHT,velocity,0);
								}
								sensor_val_update_h();
								control_pid_line_follow(RIGHT,velocity,0);
							}
							
							if(junction_count==1)
							{
								velocity=velocity_final=7;
							}
							else if(junction_count==2)
							{				
								next=1;
								junction_count=0;								
							}
							time_limit=0;
						}
						
						if(velocity<velocity_final && acc>0)
							velocity+=acc;
						else if(velocity>velocity_final && acc<0)
							velocity+=acc;
						else
							velocity = velocity_final;
						
						control_pid_line_follow(RIGHT,velocity,error_h);
					}
	 }
	 zone = ZONE_ONE_SHOOT;
	 return;
}
//
void wait_pid(int value)
{	
	while(value--)
	{
		control_pid_omni(4*PI,20);
	}
	return;
}
//
void angle_90()
{
	next=0;
	while(!next)
	{
		hold_angle-=0.01;
		
		if(hold_angle<=-90 && ang<=-90)
		{
			next=1;
			hold_angle=-90;
		}
				
		control_pid_omni(4*PI,20);
	}
	return;
}
//
void angle_0()
{
	next=0;
	while(!next)
	{
		hold_angle+=0.01;
		if(hold_angle>=0 && ang>=0)
		{
			hold_angle=0;
	    next=1;
		}				
		control_pid_omni(4*PI,20);
	}	
	return;
}
//
void find_line(char Direction_fin1,char Direction_fin2, int max_speed)
{
	/* --------------------FUNCTION FIND_LINE()-------------------------
	Parameters: Direction_fin1 - direction given in direction of previous
                               motion
	            Direction_fin2 - direction which we want to move finally
	            Max_speed      - speed of omni
	Return:     void
	
	Notes:      give direction which is opposite to previous direction of 
	            motion in Direction_fin1 and the direction in which the 
	            robot should move finally should be given in the feild 
	            marked Direction_fin2.
	*/
	bool in_loop=1;
	while(in_loop)
	{		
		pos();
    sensor_val_update_v();
	  sensor_val_update_h();
		
		/*cls();
		lcd(sum_sensor_v);
		lowerline();
		lcd(sum_sensor_h);
		*/
		switch(Direction_fin1)
		{
			case BACKWARD:  switch(Direction_fin2)
			               {
				                case FORWARD:   control_pid_omni(0*Degree_to_Rad,max_speed);
			                                  if(sum_sensor_v>1)
											                    in_loop=0;
													              break;
											  case BACKWARD:  cls();
																				lcd((char*)"err in parameters");
																				control_pid_omni(360*Degree_to_Rad,max_speed);                                 //stop condition
																				break;
											  case LEFT:      control_pid_omni(45*Degree_to_Rad,max_speed);
			                                  if(sum_sensor_v>1)
											                    in_loop=0;
																				break;
											  case RIGHT:     control_pid_omni(-45*Degree_to_Rad,max_speed);
			                                  if(sum_sensor_v>1)
											                    in_loop=0;
																				break;
										 }
				             break;
			case FORWARD: switch(Direction_fin2)
			               {
				                case BACKWARD:   control_pid_omni(180*Degree_to_Rad,max_speed);
			                                  if(sum_sensor_v>1)
											                    in_loop=0;
													              break;
											  case FORWARD:   cls();
																				lcd((char*)"err in parameters");
																				control_pid_omni(360*Degree_to_Rad,max_speed);                                 //stop condition
																				break;
											  case LEFT:      control_pid_omni(125*Degree_to_Rad,max_speed);
			                                  if(sum_sensor_v>1)
											                    in_loop=0;
																				break;
											  case RIGHT:     control_pid_omni(-125*Degree_to_Rad,max_speed);
			                                  if(sum_sensor_v>1)
											                    in_loop=0;
																				break;
										 }
				             break;
			case RIGHT:     switch(Direction_fin2)
			               {
				                case FORWARD:   control_pid_omni(45*Degree_to_Rad,max_speed);
			                                  if(sum_sensor_h>1)
											                    in_loop=0;
													              break;
											  case BACKWARD:  control_pid_omni(130*Degree_to_Rad,max_speed);
																				if(sum_sensor_h>1)
											                    in_loop=0;
																				break;
											  case LEFT:      control_pid_omni(90*Degree_to_Rad,max_speed);
			                                  if(sum_sensor_h>1)
											                    in_loop=0;
																				break;
											  case RIGHT:     cls();
																				lcd((char*)"err in parameters");
																				control_pid_omni(360*Degree_to_Rad,max_speed);			                                  
																				break;
										 }
				             break;
			case LEFT:    switch(Direction_fin2)
			               {
				                case FORWARD:   control_pid_omni(-35*Degree_to_Rad,max_speed);
			                                  if(sum_sensor_h>1)
											                    in_loop=0;
													              break;
											  case BACKWARD:  control_pid_omni(-130*Degree_to_Rad,max_speed);
																				if(sum_sensor_h>1)
											                    in_loop=0;
																				break;
											  case LEFT:      cls();
																				lcd((char*)"err in parameters");
																				control_pid_omni(360*Degree_to_Rad,max_speed);			                                  
																				break;
											  case RIGHT:     control_pid_omni(-90*Degree_to_Rad,max_speed);			                                  
																				if(sum_sensor_h>1)
											                    in_loop=0;
																				break;
										 }
				             break;
		}
	}
  return;	
}
//
void timer()
{
	/*_____________________________TIMER()_________________________________
	Uses/Method:      can be used to update number of variables when called
										by ISR of timer. Works like a RTC.
	Returns :         void
	Scope:            universal                                          */
	
	
	if(time_limit<400)      //Just to avoid overflow
	  time_limit++;
	
	count_cycle++;
	count_cycle1++;
	count_cycle2++;	
}
	


//
float sensor_val_update_h()
{	
	/*_______________________SENSOR_VAL_UPDATE_V()_____________________________
	Uses/Method:      updates the sensor values taken from the line sensor.
                    Vertical direction only!
	                  Calculates the error and sum total of all the sensors
                    currently detecting the line.
	Returns :         void
	Scope:            universal                                          */
	
	static double net_err=0;
	
	//sensor_value[0] = Pin0_7;
	//sensor_value[1] = Pin0_8;
	/*lcd(Pin4_28); //left
		lcd(Pin0_5);
		lcd(Pin2_2);
		lcd(Pin4_29); //right*/
	sensor_value[2] = Pin4_28;
	sensor_value[3] = Pin0_5;
	sensor_value[4] = Pin2_2;
	sensor_value[5] = Pin4_29;
	//sensor_value[6] = Pin1_30;
	//sensor_value[7] = Pin0_3;
	
	sum_sensor_h = 0;
	//cls();
	//lowerline();
	for(int i=2;i<6;i++)
	{
		sum_sensor_h += sensor_value[i];	
		//lcd(sensor_value[i]);
	}
	if(sum_sensor_h!=0)
	{
		if(sum_sensor_h==1)
		{
			if(sensor_value[2])
			   net_err = 4;
			else if(sensor_value[3])
			   net_err = 1.5;
			else if(sensor_value[4])
			   net_err = -1.5;
			else if(sensor_value[5])
			   net_err = -4;
		}
		else if(sum_sensor_h==2)
		{
			if(sensor_value[2] && sensor_value[3])
				 net_err = 3;
			else if(sensor_value[3] && sensor_value[4])
				 net_err = 0;
			else if(sensor_value[4] && sensor_value[5])
				 net_err = -3;
		}
		else if(sum_sensor_h==3)
		{
			if(sensor_value[2] && sensor_value[3] && sensor_value[4])
				 net_err = 1.5;
			else if(sensor_value[3] && sensor_value[4] && sensor_value[5])
				 net_err = -1.5;
		}
	}	
	else
	{
		if(net_err<0)
			net_err=-5;
		else if(net_err>0)
			net_err=5;
	}
	return net_err;
}
//
float sensor_val_update_v()
{	
	/*_______________________SENSOR_VAL_UPDATE_H()_____________________________
	Uses/Method:      updates the sensor values taken from the line sensor.
                    Horizontal direction only!
	                  Calculates the error and sum total of all the sensors
                    currently detecting the line.
	Returns :         void
	Scope:            universal                                          */
	
	static double net_err=0;	
	
	/*
	lcd(Pin1_23); //up
		lcd(Pin0_9);
		lcd(Pin2_5);
		lcd(Pin1_20);  //down
	*/
	
	sensor_value[0] = Pin0_7;
	sensor_value[1] = Pin0_8;
	sensor_value[2] = Pin1_23;
	sensor_value[3] = Pin0_9;
	sensor_value[4] = Pin2_5;
	sensor_value[5] = Pin1_20;
	sensor_value[6] = Pin1_30;
	sensor_value[7] = Pin0_3;
	
	sum_sensor_v = 0;
	//cls();
	for(int i=2;i<6;i++)
	{
		sum_sensor_v += sensor_value[i];	
		//lcd(sensor_value[i]);
	}
	if(sum_sensor_v!=0)
	{
		if(sum_sensor_v==1)
		{
			if(sensor_value[2])
			   net_err = 4;
			else if(sensor_value[3])
			   net_err = 1.5;
			else if(sensor_value[4])
			   net_err = -1.5;
			else if(sensor_value[5])
			   net_err = -4;
		}
		else if(sum_sensor_v==2)
		{
			if(sensor_value[2] && sensor_value[3])
				 net_err = 3;
			else if(sensor_value[3] && sensor_value[4])
				 net_err = 0;
			else if(sensor_value[4] && sensor_value[5])
				 net_err = -3;
		}
		else if(sum_sensor_v==3)
		{
			if(sensor_value[2] && sensor_value[3] && sensor_value[4])
				 net_err = 1.5;
			else if(sensor_value[3] && sensor_value[4] && sensor_value[5])
				 net_err = -1.5;
		}     		
	}	
	else
	{
		if(net_err<0)
			net_err=-5;
		else if(net_err>0)
			net_err=5;		
	}
	return net_err;
}
//
void control_pid_omni(double dr_angle,double upperlimit)
{
	/*__________________________CONTROL_PID_OMNI()___________________________
	Uses/Method:      when provided with direction of motion and the maximum 
                    speed it drives the omni wheel drive while staying in limits
                    set by lim11, lim12, lim21, lim22 and the value of icontrol
                    and keeps the value given to motor in check.
	Returns :         void
	Scope:            universal                                          */
	
	/*
	Parameters:-	dr_angle is the angle the robot has to follow;
	              upperlimit  is the max speed with which the motor can move.

	Use:- used to drive omni robot which holds its orientation using pid which utilises the value of kp,ki & kd;

	Notes:-
	-lim11,lim12,lim21,lim22 are limits which control speed of motor. these limits are slowly changed to accelerate the bot slowly.
	 change is made in drive() function.
	-if constant axis is to be maintained(i.e the bot will move towards a given point without effect of orientation), then
	 instead of passing angle which is to be followed as the parameter, pass (angle_to_be_followed - current_angle_of_robot)
	 as parameter.
	-icontrol is the parameter which limits the PID control value. If set too high then drastic oscillations will be produced.
	 if set too low then the robot can't maintain constant angle.
	-value_1, value_2, value_3 can be calculated and derived using simple trigonometric functions.
	-wheels are numbered along with their respective variables in the order shown below:
	-if angle greater than 2PI is sent, it'll hold its angle without moving(this is for user's convenience, that is if the user
	 want's robot to stop but still wants application of PID
	                                              3
	                                             ___
	                                           /     \
	                                          /       \
	                                         /         \
	                                         \         /
																			   2  \_______/  1

	                                         ( operator )

																				   COM0- motor 3
																		 COM1 channel 1 - motor 2
																		 COM1 channel 2 - motor 1
	*/
  heading_1=(int)(dr_angle*57.32);                                      // added by DT
	dummyl = upperlimit;
	dummyr = upperlimit;
	
	static double kp_copy;
	kp_copy=kp;              //Keep a copy of Kp as Kp is changed in the below program for better action
	
	difference=hold_angle-ang;
	
	if(dr_angle<(2*PI))      //Driving in a particular direction not asked to hold angle
	{
	  value_1=upperlimit * ((cos(dr_angle)*0.866) - (sin(dr_angle)*0.5));
	  value_2=upperlimit * ((cos(dr_angle)*0.866) + (sin(dr_angle)*0.5));
	  value_3=upperlimit * (sin(dr_angle));		
	}
	else //Hold angle command
	{
		value_1=0;
		value_2=0;
		value_3=0;
		dr_angle=0;
    /*if(difference<0.7 && difference>-0.7)
		   kp=5;
		else //if(difference<-0.1)		
			kp=0.9;//(tan(difference*Degree_to_Rad)*12.7);*/
		
		/*if(kp<0.9)
			kp=0.9;
		
		cls();
		lcd(kp);*/
    /*if(difference>-0.6 && difference<0.6)
        kp=6;
    else
        kp=0.9;			*/
	}
	
	//convention of motor changed later.
	value_1=0-value_1;
	value_2=0-value_2;
	value_3=0-value_3;

//-------------------------PID Algorithm-----------------------------------
	if(difference!= 0)	{
		//-----------------Proportional------------------------
		proportional = difference * kp;
		//-------------------Integral--------------------------
		integral += difference;
		integrald = integral * ki;
		//------------------Derivative-------------------------
		rate = prevposition - difference;
		derivative = rate * kd;
		//--------------------Control--------------------------
		control = proportional+derivative+integrald;
		integral /= 1.3;
		//--------------------PID Ends-------------------------

		//limit on control parameter: so that if angle changes more than a limit, the motors don't go hay-wire.
		if(control>icontrol)
			control=icontrol;
		else if(control<(0-icontrol))
				control=(0-icontrol);
		///////////////////////////////////////////////////////////////////////////////////////////////////////////

		control=0-control;     // Just inversion of values. Was getting inverted output. :p

		// LOCAL VARIABLES FOR USE IN THE FUNCTION ITSELF
		static double t_lim11,t_lim12,t_lim22,t_lim21;
		t_lim11=lim11;
		t_lim12=lim12;
		t_lim22=lim22;
		t_lim21=lim21;

		//----  LIMIT THE MAXIMUM VALUES BEING SENT TO THE DRIVER
		if((lim11+control)>128)
				 lim11=128-control;
		else if((lim11+control)<0)
				 lim11=0-control;

		if((lim12+control)>128)
				 lim12=128-control;
		else if((lim12+control)<0)
				 lim12=0-control;

		if((lim21-control)>255)
				 lim21=255+control;
		else if((lim21-control)<129)
				 lim21=129+control;

		if((lim22-control)>255)
				 lim22=255+control;
		else if((lim22-control)<129)
				 lim22=129+control;
		//________________________________________________ Check for limits on th output values _____________________________________________________
		txt1=(((192-value_1-control)<(lim22-control))?(lim22-control):(((192-value_1-control)>(lim21-control))?(lim21-control):(192-value_1-control)));
		txt2=(((64-value_2+control)<(lim12+control))?(lim12+control):(((64-value_2+control)>(lim11+control))?(lim11+control):(64-value_2+control)));
		txt3=(((64+value_3+control)>(lim11+control))?(lim11+control):(((64+value_3+control)<(lim12+control))?(lim12+control):(64+value_3+control)));

		//Still check if limits will not hamper the motion of other mototrs
		if((int)txt1<129)
			txt1=129;
		else if((int)txt1>255)
			txt1=255;

		if((int)txt2>127)
			txt2=127;
		else if((int)txt2<1)
			txt2=1;

		if((int)txt3>127)
			txt3=127;
		else if((int)txt3<1)
			txt3=1;

		//___ Send the Values____
		ExtPrintbin(COM1,1,txt2);    
		ExtPrintbin(COM2,1,txt3);
		ExtPrintbin(COM1,2,txt1);
	  
	 
		//RESTORE LIMIT VALUES BACK TO PREVIOUS ONES
    lim11=t_lim11;
		lim12=t_lim12;
		lim22=t_lim22;
		lim21=t_lim21;
		/////////////////////////////////////////////
	}
	else if(difference == 0)
	{
		//last argument is simply for keeping the value in range of 0-128 and 128-255.. using ternary operator
		ExtPrintbin(COM1,1,(((64-value_2)>lim11)?lim11:(((64-value_2)<lim12)?lim12:(64-value_2))));
    ExtPrintbin(COM2,1,(((64+value_3)>lim11)?lim11:(((64+value_3)<lim12)?lim12:(64+value_3))));
    ExtPrintbin(COM1,2,(((192-value_1)>lim21)?lim21:(((192-value_1)<lim22)?lim22:(192-value_1))));	  
	}
	prevposition = difference;             //Store error for use with Kd term (i.e. Rate of change of error)
	kp=kp_copy;
	
	return;
}
//
void drive()
{
/*
Function:   drive()
Return:     void
Parameters: void
Notes:      This function drives 3 wheel omni robot using adc values from ps2. Put this in a loop and the result will be
            achieved.
            Caution, set high enough value of icontrol and set values of average values of adc's o get apt results!
*/

	get_angle();                       // gets angle of joystick in radians

  //static double current_hold_angle = 0;
  //double temp_hold_angle;
  // Bot direction correction
  //	static bool hold_angle_flag=0;
  //	static double aniruddha=0;
	/*
	if(temp_four == 90)
	{
		if(hold_angle_flag==0)
		{
			aniruddha=hold_angle;
	 	  hold_angle = 5.2521+hold_angle;// + current_hold_angle; //0.1871  4.8521
		}
		hold_angle_flag=1;
	}
	else if(temp_four == -90)
	{
		if(hold_angle_flag==0)
		{
			aniruddha=hold_angle;
 		  hold_angle = -5.8521+hold_angle;// + current_hold_angle; //-3.4658  -4.8521
		}
		hold_angle_flag=1;
	}
	else if(hold_angle_flag==1) //temp_four == 400    &&
	{
		hold_angle = aniruddha;//temp_hold_angle;//0;
		hold_angle_flag=0;
	}
		*/
   
	/*
	1. Change, but, only as fast as society can accept!
	2. You should not follow a leader who can't lead!
	3. You should not initiate a war, if u can't sustain it!
	4. Do not change things whose outcome you dont know; if u dare change it, make sure u have a backup plan!
	5. if a mistake happens, somebody must be responsible!
	                           - 5 Commandments of Aniruddha

	*/
   
   
   
	static int flag_speed=0;//,flg;

	//static double tmp_x,tmp_y,tmp_heading;

	if(temp_four==400 || count_cycle2 < 1000)//(((adc_r_h<(adc_r_h_avg+400))&&(adc_r_h>(adc_r_h_avg-400)))&&((adc_r_v<(adc_r_v_avg+400))&&(adc_r_v>(adc_r_v_avg-400))))//2042,2013,2077,2042
	{
		 if(flag_speed>0)
         flag_speed--;

		   txt1=64;
			txt2=64;
			txt3=192;

		   lim11=80,lim12=48;             //limits for channel 1 -> with stop value 64    80,48
	      lim21=208,lim22=176;           //limits for channel 2 -> with stop value 192   208,176

			// replace temp_hold_angle with hold_angle
			if(turn_final==1)
         {
				if(abs(ang-hold_angle)<10)
		        hold_angle += 0.2;//hold_angle+=ang_increment; // hold_angle=

			   turn_yes=1;
		      // if(hold_angle>=359)
				// hold_angle=hold_angle-359;
	      }
	      else if(turn_final==-1)
	      {
				if(abs(ang-hold_angle)<10)
		      hold_angle -= 0.2;//hold_angle-=ang_increment;
				// current_hold_angle = hold_angle;

				turn_yes=1;

				//if(hold_angle<=-359)
			   //  hold_angle=hold_angle+359;
	      }
			else if(turn_yes>=1)
			{
				turn_yes++;
				if(turn_yes>4)
					turn_yes=0;
				hold_angle=ang;
			}
         
         //cls();
         //lcd(abs(hold_angle-ang));
         //lowerline();
         //lcd(err_count1);

			if(abs(hold_angle-ang)>1)
			{
				err_count2=0;
				err_count1=0;
			}
         static double multiplier=0;
			//        ------------       turn according to command condition      ------------
			if((ang>hold_angle) && (err_count1<550))// && abs(hold_angle-ang)>.5)
			{           
           if(abs(hold_angle-ang)<1)
           {
              err_count1++;              
           }
           
           if(abs(hold_angle-ang)>1)
           {
              multiplier=2;
           }          
           else if(abs(hold_angle-ang)>0.7)
           {
              multiplier=5;
           }
           else if(abs(hold_angle-ang)>0.3)
           {
              multiplier=10;
           }
           /*else if(abs(hold_angle-ang)>.1)
           {
              multiplier=30;
           }           
           else if(abs(hold_angle-ang)>.05)
           {
              multiplier=40;
           }*/
           
           
           //else
           //err_count1--;
			  //if(err_count2>1000)
			  //err_count2=0;
			  //last argument is simply for keeping the value in range of 0-128 and 128-255.. using ternary operator
        ExtPrintbin(COM0,1,192);
			  ExtPrintbin(COM1,1,(((64-(multiplier*(hold_angle-ang)))>(64+turn_speed_limit))?(64+turn_speed_limit):(64-(multiplier*(hold_angle-ang)))));
			  ExtPrintbin(COM0,1,(((64-(multiplier*(hold_angle-ang)))>(64+turn_speed_limit))?(64+turn_speed_limit):(64-(multiplier*(hold_angle-ang)))));
			  ExtPrintbin(COM1,1,(((192+(multiplier*(hold_angle-ang)))<(192-turn_speed_limit))?(192-turn_speed_limit):(192+(multiplier*(hold_angle-ang)))));
			}
			else if((ang<hold_angle) && (err_count1<50))// && abs(hold_angle-ang)>.5)
			{
            if(abs(hold_angle-ang)<1)
           {
              err_count1++;              
           }
           
           if(abs(hold_angle-ang)>1)
           {
              multiplier=2;
           }          
           else if(abs(hold_angle-ang)>0.7)
           {
              multiplier=5;
           }
           else if(abs(hold_angle-ang)>0.3)
           {
              multiplier=10;
           }
           /*
              
           else if(abs(hold_angle-ang)>.1)
           {
              multiplier=30;
           }
           
           else if(abs(hold_angle-ang)>.05)
           {
              multiplier=40;
           }
           */
           
           //else
           //err_count1--;
			  //err_count1++;
			  //if(err_count1>1000)
			  //err_count1=0;
			  //last argument is simply for keeping the value in range of 0-128 and 128-255.. using ternary operator to save space
           ExtPrintbin(COM0,1,192);
			  ExtPrintbin(COM1,1,(((64-(multiplier*(hold_angle-ang)))<(64-turn_speed_limit))?(64-turn_speed_limit):(64-(multiplier*(hold_angle-ang)))));
			  ExtPrintbin(COM0,1,(((64-(multiplier*(hold_angle-ang)))<(64-turn_speed_limit))?(64-turn_speed_limit):(64-(multiplier*(hold_angle-ang)))));
			  ExtPrintbin(COM1,1,(((192+(multiplier*(hold_angle-ang)))>(192+turn_speed_limit))?(192+turn_speed_limit):(192+(multiplier*(hold_angle-ang)))));
			}
			else
			{
			  ExtPrintbin(COM1,1,txt2);
			  ExtPrintbin(COM0,1,txt1);
			  ExtPrintbin(COM1,2,txt3);
        ExtPrintbin(COM0,1,192);
			}
   }
	else if(count_cycle2>1000)                // if anything except stop condition
	{
		err_count1=0;
		err_count2=0;
     //flg=1;
		 if(turn_final==1)
	    {
		     hold_angle+=0.01;//ang_increment;
				 turn_yes=1;
		     if(hold_angle>=360)
				   hold_angle=hold_angle-360;
	    }
	    else if(turn_final==-1)
	    {
		      hold_angle-=0.01;//ang_increment;
				  turn_yes=1;
		      if(hold_angle<=-360)
				    hold_angle=hold_angle+360;
	    }
		 else if(turn_yes)
		 {
				turn_yes=0;
				hold_angle=ang;
		 }

       
       lim11+=acc;
		 if(lim11>lim11_lim)
				 lim11=lim11_lim;

		 lim12-=acc;
		 if(lim12<lim12_lim)
			   lim12=lim12_lim;

				//limits for channel 2 -> with stop value 192   208,176
	    lim21+=acc;
		 if(lim21>lim21_lim)
				lim21=lim21_lim;

		  lim22-=acc;
		  if(lim22<lim22_lim)
				lim22=lim22_lim;

						
	    control_pid_omni(b_heading-th/*-((hold_angle*PI)/180)*/,61);	    // to keep axis const.. b_heading-hold_angle
		}
}
//
void get_angle()
{
	static double check,check2;
	/*
	  waitms(1);
	  adc_r_h=((unsigned int)((AD0DR0&0x80000000)?((AD0DR0&0x0000FFF0)>>4):adc_r_h_avg));
    waitms(1);
	  adc_l_v=((unsigned int)((AD0DR6&0x80000000)?((AD0DR4&0x0000FFF0)>>4):adc_l_v_avg));
	  waitms(1);
	  adc_r_v=((unsigned int)((AD0DR5&0x80000000)?((AD0DR5&0x0000FFF0)>>4):adc_r_v_avg));
	  waitms(1);
	  adc_l_h=((unsigned int)((AD0DR4&0x80000000)?((AD0DR6&0x0000FFF0)>>4):adc_l_h_avg));
	  //b_heading=-(atan2(((double)adc_r_v-adc_r_v_avg),-((double)adc_r_h-adc_r_h_avg)));  //2026,2062
	*/
	  if((temp_four!=360)&&(temp_four!=400))
			b_heading = temp_four;
		else
		{
			b_heading = 4*PI;
			if(temp_four==400)
			{
				err_count2=0;
				//err_count1=0;
			}
		}

		// snippet for stopping jerky ride!!
		if(abs(check2-b_heading)>1.5)
		{
			lim11=80,lim12=48;             //limits for channel 1 -> with stop value 64    80,48
	    lim21=208,lim22=176;           //limits for channel 2 -> with stop value 192   208,176
		}
		check2=check;
		check=b_heading;

		/*
	  if(abs(b_heading)<0.2618)
			b_heading=0;
		else if(b_heading>0.2618  &&  b_heading<0.7854)
			b_heading=0.5235;
		else if(b_heading>0.7854  &&  b_heading<1.309)
			b_heading=1.0472;
		else if(b_heading>1.309  &&  b_heading<1.8326)
			b_heading=1.5708;
		else if(b_heading>1.8326  &&  b_heading<2.3562)
			b_heading=2.0944;
		else if(b_heading>2.3562  &&  b_heading<2.8798)
			b_heading=2.618;
		else if(abs(b_heading)>2.8798)
			b_heading=3.1416;
		else if(b_heading<-0.7854  &&  b_heading>-1.309)
			b_heading=-1.0472;
		else if(b_heading<-1.309  &&  b_heading>-1.8326)
			b_heading=-1.5708;
		else if(b_heading<-1.8326  &&  b_heading>-2.3562)
			b_heading=-2.0944;
		else if(b_heading<-2.3562  &&  b_heading>-2.8798)
			b_heading=-2.618;
		*/
}

//----------------------------------------------------- IMU VALUES----------------------------------------------------------------------
void get_rate()
{
	reset(Port0_16);
	res[0]=SPI_Communicate(0x80);
	res[1]=SPI_Communicate(0x00);
	res[2]=SPI_Communicate(0x00);
	res[3]=SPI_Communicate(0x00);
	set(Port0_16);
	ans=(res[3]|(res[2]<<8)|(res[1]<<16)|(res[0]<<24));
	ans=((ans & 0x001FFFE0)>>5);
	
	if(ans>32767)
		ans=ans-(65536);                               
	
	static double ans1=0; 
  int diff_ang=0;
	
	ang_time++;
	divyanshu++;
  divyanshu_ka_chut+=ans;
//-------------------------------------------------O R I G I N A L --------------------------------------------------------------------------------	
	ans1 = (double)ans-41.00;//40.48;         //41.25;//41
//------------------------------------------------------------------------------------------------------------------------------------------------	

	if((ans1<=20)&&(ans1>=-20))
	{
		ans1=ans1/800;
	}
	else
	{
		ans1=ans1/80;
	}		
	
	ang+=0.0025002*ans1;
  
	/*
	ans1 = ans-43.62;																																																																																		//review-I ke liye -86;
	
  
	if((ans<=20)&&(ans>=-20))
	{
		ans1=(ans1)/800;
	}
	else
	{
		ans1=(ans1)/80;
	}
		
	ang+=(ans1*0.0025002);
	*/
	p_ang=ans;
	return;
}
//-------------------------------------------------FUNCTIONS FOR DEAD RECON--------------------------------------------------------------
void pos()            // this tiny one does all the heavy work!! whole dead recon is based on this
{
	a1=T1TC;
	b1=T0TC;

	if(Pin2_13==1)
	{tickl=b1-prvvaluel;}
	else
	{tickl=prvvaluel-b1;}

	

	if(Pin2_12==1)
	{tickr=a1-prvvaluer;}
	else
	{tickr=prvvaluer-a1;}
  
	
	
	//   |_|

	//templ+=tickl;
	//tempr+=tickr;
	
	//_______________________________________________________________________________
	//---------------COMMENTED SECTION: NOT MEANT TO BE UNCOMMENTED------------------
	//-------------------------------------------------------------------------------
	//dth=(tickl-tickr)*onetick;     //onetick = (PI*wheel_dia)/(1024*bot_dia);
	//radiansPerCount= 2*Pi*(wheelDiameter/trackWidth)/countsPerRevolution
	//deltaHeading= (rightCounts - leftCounts)* radiansPerCount/2

	//ddis=(((tickl*left_dia)+(tickr*right_dia))*factor);

	//ddis=(tickl+tickr)*factor;     //factor= (PI*wheel_dia)/2048;
	//distancePerCount = Pi * diameterWheel /countsPerRevolution
	//deltaD=istance = (leftCounts + rightCounts)/2*distancePerCount

	//dis+=ddis;
	
	//---------------COMMENTED SECTION: NOT MEANT TO BE UNCOMMENTED------------------
	//_______________________________________________________________________________
	
	//th=(ang*Degree_to_Rad);
	//th+=dth;

	//left here is used for the encoder which points towards wheel 3; right is for encoder which is horizontal..
	left_ddis=tickl*dis_per_count_left;
	right_ddis=tickr*dis_per_count_right;
  if (!enc_diff_1_2)
	{
	   x_dis+=left_ddis;
     y_dis+=right_ddis;
	}
	else 
	{
		 th=ang*Degree_to_Rad;
     x_dis+=((left_ddis*cos(th))+(right_ddis*sin(th)));//((ddis)*(cos(th)));
     y_dis+=((right_ddis*cos(th))-(left_ddis*sin(th)));//((ddis)*(sin(th)));
	}


	prvvaluel=b1;
	prvvaluer=a1;
	return;
}

//
void set_speed_limit()
{
	//FUNCTION TO SET SPEED IN MANUAL/AUTO MODE. PROVIDES FIVE DIFFERENT SPEED RANGES FOR NORMAL MOTION WHILE THREE SPEEDS FOR ROTATION
	if(ps_r1)
		{
			ps_r1=0;
			if(speed==0)
			{
			  lim11_lim=91; //80
		    lim12_lim=35;   //48
		    lim21_lim=219; //208
		    lim22_lim=165; //176
				speed++;
				////Update_display();
				//icontrol=18;
		  }
			else if(speed==1)
			{
			  lim11_lim=102; //80
		    lim12_lim=24;   //48
		    lim21_lim=230; //208
		    lim22_lim=154; //176
				speed++;
				//Update_display();
			  //icontrol=25;
		  }
			else if(speed==2)
			{
			  lim11_lim=113; //80
		    lim12_lim=13;   //48
		    lim21_lim=241; //208
		    lim22_lim=143; //176
				speed++;
				//Update_display();
			  //icontrol=30;
		  }
			else if(speed==3)
			{
			  lim11_lim=126; //80
		    lim12_lim=2;   //48
		    lim21_lim=253; //208
		    lim22_lim=130; //176
				speed++;
				//Update_display();
			  //icontrol=30;
		  }
		}
	else if(ps_r2)
	{
		ps_r2=0;
			if(speed==1)
			{
			  lim11_lim=75; //80
		    lim12_lim=53;   //48
		    lim21_lim=203; //208
		    lim22_lim=181; //176
				speed--;
				//Update_display();
			  //icontrol=18;
		  }
			else if(speed==2)
			{
			  lim11_lim=91; //80
		    lim12_lim=35;   //48
		    lim21_lim=219; //208
		    lim22_lim=165; //176
				speed--;
				//Update_display();
				//icontrol=18;
		  }
			else if(speed==3)
			{
			  lim11_lim=102; //80
		    lim12_lim=24;   //48
		    lim21_lim=230; //208
		    lim22_lim=154; //176
				speed--;
				//Update_display();
				//icontrol=25;
		  }
			else if(speed==4)
			{
			  lim11_lim=113; //80
		    lim12_lim=13;   //48
		    lim21_lim=241; //208
		    lim22_lim=143; //176
				speed--;
				//Update_display();
				//icontrol=30;
		  }
		}
}
//
void Ps2_val_update()
{
	/*
	updates the variables named after ps2 buttons, call it just as is to update all values.
	put Bluetooth UART in COM2
	QUE:- is interrupt to ATmega better for speed?
	*/
	  ps_up=ps_right=ps_left=ps_down=ps_square=ps_triangle=ps_cross=ps_select=ps_start=ps_circle=ps_r1=ps_r2=ps_l1=ps_l2=0;

	  int temp_four1=0;
		static int temp_four2,temp_one,temp_one2,temp_two,temp_two2,temp_three,temp_three2,temp_turn;
		static int go=1,go_one=1,go_two=1,go_three=1;
	  //___________________________________________________________________________________________________________________________
	  //----------------------------------- INPUT FROM COM2 STORED IN BYTES  ----------------------------------------------------------
	  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		ps2_val[0]=Inputbin(COM2);
		ps2_val[1]=Inputbin(COM2);
		ps2_val[2]=Inputbin(COM2);
	  ps2_val[3]=Inputbin(COM2);
    //___________________________________________________________________________________________________________________________
	  //------------------------ IF COMMUNICATION HAS STOPPED, THEN STOP THE BOT FROM MOVING   ----------------------------------------------------------
	  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	  while(safety!=0)                     // SAFETY BECOMES ZERO IN PRINTBIN() COMMAND -> SAFETY INCREASES IN TIMER-2 INTERRUPT
		{
			Inputbin(COM2);
			ExtPrintbin(COM1,1,64);
			ExtPrintbin(COM0,1,64);
			ExtPrintbin(COM1,1,192);
			ExtPrintbin(COM0,1,192);
		}

    //___________________________________________________________________________________________________________________________
    //-----------------------Byte decoding section; numbering of bytes, seeking actually pressed buttons, etc ---------------------
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    for(int u=0;u<4;u++)
    {
			 for(int i=0;i<8;i++)
			 {
				 ip[u][i]=ps2_val[u]%2;
				 ps2_val[u]=ps2_val[u]/2;
			 }
		}

		if((ip[0][1]==0)&&(ip[0][0]==0))
			one=0;
		else if((ip[0][1]==0)&&(ip[0][0]==1))
		  two=0;
		else if((ip[0][1]==1)&&(ip[0][0]==0))
		  three=0;
		else if((ip[0][1]==1)&&(ip[0][0]==1))
		  four=0;


		if((ip[1][1]==0)&&(ip[1][0]==0))
			one=1;
		else if((ip[1][1]==0)&&(ip[1][0]==1))
		  two=1;
		else if((ip[1][1]==1)&&(ip[1][0]==0))
		  three=1;
		else if((ip[1][1]==1)&&(ip[1][0]==1))
		  four=1;


		if((ip[2][1]==0)&&(ip[2][0]==0))
			one=2;
		else if((ip[2][1]==0)&&(ip[2][0]==1))
		  two=2;
		else if((ip[2][1]==1)&&(ip[2][0]==0))
		  three=2;
		else if((ip[2][1]==1)&&(ip[2][0]==1))
		  four=2;


		if((ip[3][1]==0)&&(ip[3][0]==0))
			one=3;
		else if((ip[3][1]==0)&&(ip[3][0]==1))
		  two=3;
		else if((ip[3][1]==1)&&(ip[3][0]==0))
		  three=3;
		else if((ip[3][1]==1)&&(ip[3][0]==1))
		  four=3;


		//temp_four=0;
		temp_one= 0;
		temp_two= 0;
		temp_three= 0;

		// to get adc values for driving
		for(int i=7;i>=4;i--){
			  temp_four1= (temp_four1*2) + ip[four][i];
		}

		// to get turn command

		if((ip[four][2]==1)&&(ip[four][3]==0))
			turn_adc=-1;
		else if((ip[four][2]==0)&&(ip[four][3]==1))
		  turn_adc=1;
    else
			turn_adc=0;

		if(temp_turn==turn_adc)
			turn_final=turn_adc;


		//___________________________________________________________________________________________________________________________
		//---------------------------Byte verification section, comparision of two bytes to check elligibality of data--------------
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~`
		temp_turn=turn_adc;

		go_one=0;
		go_two=0;
		go_three=0;
		for(int i=7;i>=0;i--)
		{
			temp_one= (temp_one*2) + ip[one][i];
			temp_two= (temp_two*2) + ip[two][i];
			temp_three= (temp_three*2) + ip[three][i];
		}

		if(temp_one2==temp_one)//&&(temp_four3==temp_four1))
		{
   			go_one=1;
		}
      
		if(temp_two2==temp_two)//&&(temp_four3==temp_four1))
		{
   			go_two=1;
		}
      
		if(temp_three2==temp_three)//&&(temp_four3==temp_four1))
		{
   			go_three=1;
		}
		go=0;
		//static int temp_four3=0;
		if(temp_four2==temp_four1)//)&&(temp_four3==temp_four1))
		{
   			go=1;
		}

		//temp_four3=temp_four2;
		temp_three2=temp_three;
		temp_two2=temp_two;
		temp_four2=temp_four1;
		temp_one2=temp_one;
      #if DED_RECON
         pos();
      #endif

		if(go==1)
		switch(temp_four1)
		{	                                                    //final values
			case 0: temp_four=((180*PI)/180); // 0	180   0         180
		            break;
         case 1: temp_four=((-120*PI)/180); // 30 	 -150   60    -120
		            break;
			case 2: temp_four=((-105*PI)/180); 	// 60  -120   75     -105
		            break;
			case 3: temp_four=(((-90-3.7)*PI)/180); 	 //90  -90     90    -90
		            break;
			case 4: temp_four=((-75*PI)/180); 	 //120  -60   105    -75
		            break;
			case 5: temp_four=((-60*PI)/180); 	 //150  -30    120    -60
		            break;
			case 6: temp_four=((0*PI)/180); 	 //180  0      180       0
		            break;
			case 7: temp_four=((120*PI)/180); 	 //-30  150   -60      120
		            break;
			case 8: temp_four=((105*PI)/180); 	 //-60  120    -75      105
		            break;
			case 9: temp_four=(((90-2)*PI)/180); 	 //-90  90       -90      90
		            break;
			case 10: temp_four=((75*PI)/180); 	 //-120  60    -105     75
		            break;
         case 11: temp_four=((60*PI)/180); 	 //-150  30    -120      60
                     break;
         case 12: temp_four=360;
                     break;
         case 13: temp_four=360;
                     break;
         case 14: temp_four=360;
                     break;
         case 15: temp_four=400;
                     break;
		}

		// Portion to stop jerky motion due to sudden change in joystick direction.
		static double temp_four_,temp_1four,temp_2four,temp_3four,temp_4four,temp_5four,temp_6four,temp_7four,temp_8four,temp_9four,temp_10four;
		
		if(((temp_10four>0  &&  temp_four<0) ||  (temp_10four<0  &&  temp_four>0)) && temp_10four!=400 && temp_four!=400)// && temp_4four!=400
			                    //&& temp_3four!=400 && temp_2four!=400 && temp_1four!=400 && temp_four_!=400)// && temp_four!=400)
		{
			count_cycle2=0;
		}
		temp_10four=temp_9four;
		temp_9four=temp_8four;
		temp_8four=temp_7four;
		temp_7four=temp_6four;
		temp_6four=temp_5four;
		temp_5four=temp_4four;
		temp_4four=temp_3four;
		temp_3four=temp_2four;
		temp_2four=temp_1four;
		temp_1four=temp_four_;
    temp_four_=temp_four;
    // the jerky motion section ends
		
		
		if(go_two>0)
		{
		if(ip[two][2]==0)
			ps_up=1;		 //lcd("UP");
		if(ip[two][3]==0)
			ps_right=1;		// lcd("RIGHT");
		if(ip[two][4]==0)
			ps_down=1;		 //lcd("DOWN");
		if(ip[two][5]==0)
			ps_left=1;		 //lcd("LEFT");
    	}

		if(go_three==1)
		{
		if(ip[three][2]==0)
			ps_cross=1;		 //lcd("Cross");
		if(ip[three][3]==0)
			ps_square=1;		 //lcd("Square");
		if(ip[three][4]==0)
			ps_select=1;		 //lcd("SELECT");
		if(ip[three][7]==0)
		  ps_start=1;     //lcd("START");
	   }
		if(go_one==1)
		{
		if(ip[one][2]==0)
		  ps_l2=1;        //lcd("L2");
		if(ip[one][3]==0)
		  ps_r2=1;        //lcd("R2");
		if(ip[one][4]==0)
		  ps_l1=1;        //lcd("L1");
		if(ip[one][5]==0)
		  ps_r1=1;        //lcd("R1");
		if(ip[one][6]==0)
		  ps_triangle=1;  //lcd("Triangle");
		if(ip[one][7]==0)
		  ps_circle=1;    //lcd("Circle");
	  }

	}
//
void control_pid_line_follow(char Direction, int max_speed, double error_line)
{
	if(error_line!=0)
	{		
		//-----------------Proportional------------------------
		proportional_line_sense = kp_line_sense * error_line;
		//-------------------Integral--------------------------
		integral_line_sense += error_line;
		integrald_line_sense = integral_line_sense * ki_line_sense;
		//------------------Derivative-------------------------
		rate_line_sense = prev_err - error_line;
		derivative_line_sense = rate_line_sense * kd_line_sense;
		//--------------------Control--------------------------
		control_line_sense = proportional_line_sense+derivative_line_sense+integrald_line_sense;
		integral_line_sense /= 1.3;
		//--------------------PID Ends-------------------------		
		
		if(control_line_sense>icontrol_line_sense)
			  control_line_sense=icontrol_line_sense;
		else if(control_line_sense<(0-icontrol_line_sense))
				control_line_sense=(0-icontrol_line_sense);
				
		/*sum_sensor_h = Pin2_3+Pin0_9+Pin2_5+Pin2_4;
		if(sum_sensor_h==4)
			return;*/
				
		switch(Direction)
		{
			case FORWARD:   control_pid_omni((control_line_sense-ang)*Degree_to_Rad,max_speed);                //FORWARD depicts heading=0 degree
			                break;
			case BACKWARD:  control_pid_omni(PI - ((control_line_sense+ang)*Degree_to_Rad),max_speed);        //FORWARD depicts heading=0 degree
			                break;			
			case RIGHT:     control_pid_omni(-PI/2+((control_line_sense-ang)*Degree_to_Rad),max_speed);				 //RIGHT depicts heading= -90 degree
			                break;
			case LEFT:      control_pid_omni(PI/2-((control_line_sense+ang)*Degree_to_Rad),max_speed);         //LEFT depicts heading=90 degree
			                break;			
		}
    return;		
	}
	else
	{
		switch(Direction)
		{
			case FORWARD: control_pid_omni(0-ang*Degree_to_Rad,max_speed);						//FORWARD depicts heading=0 degree
			              break;
			case BACKWARD:control_pid_omni(PI-ang*Degree_to_Rad,max_speed);						//FORWARD depicts heading=0 degree
			              break;
			case RIGHT:   control_pid_omni(-PI/2-ang*Degree_to_Rad,max_speed); 				//RIGHT depicts heading= -90 degree
			              break;
			case LEFT:    control_pid_omni(PI/2-ang*Degree_to_Rad,max_speed); 				//LEFT depicts heading=90 degree
			              break;
		}
		
		return;
		//After this, refer to control_pid_omni() for more detail
	}	
}
//

//________________________________________________TZ3 RETURN_________________________________________________________________
/*
small change is done in control pid drive function the drive angle is copied in another global variable->heading_1 which is used here
*/
void tz3_return()
{
//______________--------------restore kp ------------___________________	
	kp=0.95;

	
//________________________encoder initialise____________________________	
	x_dis=0;
	T0TC=0;
	prvvaluel=0;
//______________________--local variables--______________________________
	int counterh_90=0,i=0,local_time=0;	
	hold_angle=ang=90;
	
//_______________________________________________________________________	
	/*while(x_dis<=1)  //30
	{
		pos();
		control_pid_omni(120*Degree_to_Rad,15);
	}*/
	
	sum_sensor_v=0;
	sum_sensor_h=0;
	icontrol=17;
	while(ang>0)                                                // snippet for ctrm from wall to line  
	{
		sensor_val_update_h();
		if (ang>71)
		{
		hold_angle-=0.025;
		}
		else 
		{
			hold_angle-=0.025;
		}
		control_pid_omni((275-ang)*Degree_to_Rad,10);     // 270        ||   //215 for normal path
		if (hold_angle<0)
		{
			hold_angle=0;
		}	
	}
	icontrol=8;
	sum_sensor_v=0;
	sum_sensor_h=0;	
	
	while(sum_sensor_v<4)//(sum_sensor_v<2 && sum_sensor_h<1)
	{
		cls();
		lcd((char *)"in");
		sensor_val_update_v();
		sensor_val_update_h();
		control_pid_omni((-90*Degree_to_Rad),4);
	}
	
/*	
	sensor_val_update_h();
	sensor_val_update_v();
	
	if (sum_sensor_v==0)
	{
		if (sum_sensor_h==0)
		{
			while(sum_sensor_v<4)//(sum_sensor_v<2 && sum_sensor_h<1)
	     {
	     	cls();
		    lcd((char *)"in");
		    sensor_val_update_v();
		    sensor_val_update_h();
		    control_pid_omni((-90*Degree_to_Rad),4);
	     }
		}
		else 
		{
				while(sum_sensor_v<4)//(sum_sensor_v<2 && sum_sensor_h<1)
	     {
	     	cls();
		    lcd((char *)"in");
		    sensor_val_update_v();
		    sensor_val_update_h();
		    control_pid_omni((90*Degree_to_Rad),4);
	     }
		}
	}

	*/
	while(sum_sensor_h<1)//(sum_sensor_v<2 && sum_sensor_h<1)
	{
		cls();
		lcd((char *)"in");
		sensor_val_update_v();
		sensor_val_update_h();
		control_pid_omni((-90*Degree_to_Rad),4);
	}
	
	i=1000;           // 1500
	while(i--)                                                     // responsible for making the bot stick to line 
	{                                                              // the first time it catches the line(without overshoot)  
		control_pid_omni(90*Degree_to_Rad,25); //-230
	}
	
	/*sum_sensor_h=0;
	cls();
	lcd((char *)"this shit ");
	while(sum_sensor_h<2)
	{		
		sensor_val_update_h();
		control_pid_omni(-230*Degree_to_Rad,10); //-230
	}
		
	i=100;
	while(i--)
	{
			control_pid_omni(180*Degree_to_Rad,14); //-230
	}*/
	
	cls();
	
	kp_line_sense=8;
	kd_line_sense=2.5;
	
  kp=0.9;
	
	next=0;
		count_cycle1=500;
	sum_sensor_h=0;
	while(!next)                                               // line follower in backward direction
	{
		error_h=sensor_val_update_h();
		if (counterh_90<1)
		control_pid_line_follow(BACKWARD,12,error_h);
		else if ((counterh_90>=1)&&(counterh_90<4))                                   // 4
			control_pid_line_follow(BACKWARD,23,error_h);            //25
		else if (counterh_90==4)                                                       // 4
			control_pid_line_follow(BACKWARD,4,error_h);             //5
	/*	
		if(error_h>4)
			find_line(FORWARD,RIGHT,12);
		else if(error_h<-4)
			find_line(FORWARD,LEFT,12);
	*/	
		if(sum_sensor_h==4 || sum_sensor_v>1)
		{
			next=0;
			while(1)
			{
				sensor_val_update_h();
				error_h=sensor_val_update_h();
		    control_pid_line_follow(BACKWARD,12,error_h);
				//control_pid_omni(180*Degree_to_Rad,20);
				if(sum_sensor_h<=2 && sum_sensor_v<1)
				{
					if (count_cycle1>=50)
					{
					counterh_90++;
					}
					count_cycle1=0;
					break;
				}
			}
			sum_sensor_h=0;
		}
		
		//______________________________________________BACKUP FOR UNSTABLE CONDITION________(LINE NOT FOLLOWED)___________________________________________________________		
	/*	
		if (sum_sensor_h==0)
			local_time++;
		else 
			local_time--;
		
		if(local_time<0)
			local_time=0;
		
		if (local_time>=1000)
		{
			if (heading_1<0)
			{
				while(sum_sensor_h<=1)
					
				{
					cls();
					lcd((char *)"dir1");
					lowerline();
					lcd(heading_1);
					sensor_val_update_h();
					control_pid_omni(-90*Degree_to_Rad,10);
				}
				local_time=0;
			}
			else 
			{
				while(sum_sensor_h<=1)
				{
					cls();
					lcd((char *)"dir2");
					lowerline();
					lcd(heading_1);
					sensor_val_update_h();
					control_pid_omni(90*Degree_to_Rad,10);
				}
				local_time=0;
			}
		}
		*/
//_________________________________________________________________________________________________________________________________________________		
		
		
		if(counterh_90==5)                                 // 5 for normal path
		{
			next=1;
		}			
	}
	while(FORWARD_PROXY)
	{
		error_h=sensor_val_update_h();
		control_pid_line_follow(BACKWARD,5,error_h);
	}
	i=2000;
	while(i--)                                                           // reverse braking for proxy 
	{
    control_pid_omni(0*Degree_to_Rad,15);
	}
	while(FORWARD_PROXY)
	{
    error_h=sensor_val_update_h();
		control_pid_line_follow(FORWARD,5,error_h);    
	}
	kp_line_sense=8;                                                    // restore kp and kd (of line follower) to its original value
	kd_line_sense=2.5;
}

//________________________________________________________END FUNCTION_________________________________________________________________

//=====================

void test_start_to_tz2()
{	
	
	count_cycle1=0;
//	CLOSE_GRIPPER;
	zone_complete=1;
	while(x_dis>-3720 && (sum_sensor_v<4 || count_cycle1<700))
	{
		pos();		
		sensor_val_update_v();
		if(x_dis>-1000)
		  control_pid_omni(-80*Degree_to_Rad,28);
		else
			control_pid_omni(-80*Degree_to_Rad,33);		
	}

  sum_sensor_h=0;	
	
	while(sum_sensor_h<4)
	{
		pos();
		sensor_val_update_h();
		sensor_val_update_v();
		control_pid_omni(-10*Degree_to_Rad,13);
	}
	
	sum_sensor_v=0;
	
	while(sum_sensor_v<4)
	{
		pos();
		sensor_val_update_v();
		control_pid_omni(-60*Degree_to_Rad,3);  
	}
	
	sum_sensor_v=0;
	sum_sensor_h=0;
	while(sum_sensor_h<2)
	{
		sensor_val_update_h();
		control_pid_omni(90*Degree_to_Rad,6);
	}
	/*
	while(sum_sensor_h<2)                //hazardous
	{
		pos();
		sensor_val_update_h();
		control_pid_omni(-60*Degree_to_Rad,3); 
	}
	*/
	sum_sensor_h=0;
	sum_sensor_v=0;
	int counterh_90=0;
	next=0;
	while(!next)                                               // line follower in backward direction
	{
		error_h=sensor_val_update_h();
		if (counterh_90<1)
		control_pid_line_follow(FORWARD,12,error_h);
		else if (counterh_90>=1)
			control_pid_line_follow(FORWARD,8,error_h);
		
	/*	
		if(error_h>4)
			find_line(FORWARD,RIGHT,12);
		else if(error_h<-4)
			find_line(FORWARD,LEFT,12);
	*/	
		if(sum_sensor_h==4)
		{
			next=0;
			while(1)
			{
				sensor_val_update_h();
				error_h=sensor_val_update_h();
		    //control_pid_line_follow(BACKWARD,10,error_h);
				control_pid_omni(90*Degree_to_Rad,10);
				if(sum_sensor_h<=2)
				{
					counterh_90++;
					break;
				}
			}
			sum_sensor_h=0;
		}		
		if((counterh_90==1)&&(!FORWARD_PROXY))
		{
			next=1;
		}			
	}	
	 int o=1500;
	// i=1500;
	while(o--)
	{
		control_pid_omni(PI,15);
	}
	
	while(FORWARD_PROXY)
	{
    error_h=sensor_val_update_h();
		control_pid_line_follow(BACKWARD,5,error_h);    
	}
	zone_complete=1;
	/*
	//int l=3000;
	while(1)//(l--)
	{
		control_pid_omni(90*Degree_to_Rad,10);
	}
	
	while(1)
	{
		cls();
		lcd((char *)"fuckyou A,M");
		control_pid_omni(4*PI,5);
	}
	
	
	sum_sensor_h=0;
	while(sum_sensor_h<2)
	{
		sensor_val_update_h();
		control_pid_omni(90*Degree_to_Rad,5);
	}
	
		int l=0;
	l=2000;
	while(l--)
	{
		control_pid_omni(90*Degree_to_Rad,15);
	}
	
	
	
	while(1)
	{
		cls();
		lcd((char *)"fuckyou A,M");
		control_pid_omni(4*PI,5);
	}
//---------------------------------------the snippet added above is all mine	
	while(sum_sensor_v>2)
	{
		pos();
		sensor_val_update_v();
		control_pid_omni(-40*Degree_to_Rad,8);  ///4 tha
	}
  
	sum_sensor_h=0;
	
  while(sum_sensor_v<3)
	{
		pos();
		sensor_val_update_v();
		sensor_val_update_h();
		control_pid_omni(-40*Degree_to_Rad,6);  //4 tha
	}
	
	sum_sensor_h=0;
	
	while(sum_sensor_h<1)
	{
		pos();
		sensor_val_update_h();
		control_pid_omni(-40*Degree_to_Rad,3);
	}
	
	count_cycle1=0;
	sum_sensor_h=0;
	next=0;
	
	while(sum_sensor_h<4)
	{
		error_h=sensor_val_update_h();
	  
		if(count_cycle1>300)
			control_pid_line_follow(FORWARD,4,error_h);
		else
			control_pid_line_follow(FORWARD,8,error_h);			
		
		//if((sum_sensor_h>0) && (!FORWARD_PROXY))
			//break;		
	}
	
	while(sum_sensor_h>=3)
	{
		error_h=sensor_val_update_h();
	  
		if(count_cycle1>300)
			control_pid_line_follow(FORWARD,4,error_h);
		else
			control_pid_line_follow(FORWARD,8,error_h);			
		
		//if((sum_sensor_h>0) && (!FORWARD_PROXY))
			//break;		
	}
	
	while(sum_sensor_h<4)
	{
		error_h=sensor_val_update_h();
	  
		if(count_cycle1>300)
			control_pid_line_follow(FORWARD,4,error_h);
		else
			control_pid_line_follow(FORWARD,8,error_h);			
		
		if((sum_sensor_h>0) && (!FORWARD_PROXY))
			break;		
	}
	sum_sensor_h=0;
	while(sum_sensor_h<3)
	{
		sensor_val_update_h();
		control_pid_omni(90*Degree_to_Rad,5);
	}
	next=0;
	
	while(!next)
	{
		error_h=sensor_val_update_h();
		control_pid_line_follow(BACKWARD,5,error_h);
		if((!FORWARD_PROXY && sum_sensor_h>=1))// || sum_sensor_h>3)
			next=1;
	}
	while(1)
	{
		cls();
		lcd((char *)"fuckyou A,M");
		control_pid_omni(4*PI,5);
	}
	
	
	sum_sensor_h=0;
	
	while(sum_sensor_h<=1)
	{
		pos();
		sensor_val_update_h();
		control_pid_omni(-45*Degree_to_Rad,16);
	}
	
	sum_sensor_v=0;
	sum_sensor_h=0;
	
	while(sum_sensor_h>0)
	{
		pos();
		sensor_val_update_v();
		error_h=sensor_val_update_h();
		control_pid_omni(-40*Degree_to_Rad,4);
	}
	
	while(sum_sensor_v<1)
	{
		pos();
		sensor_val_update_v();
		error_h=sensor_val_update_h();
		if(sum_sensor_h>0 && x_dis<-4400)
			break;
		if(x_dis>-4200)
		  control_pid_omni(-40*Degree_to_Rad,3);
		else
			control_pid_omni(-30*Degree_to_Rad,3);
	}
	
	while(sum_sensor_h<1 && FORWARD_PROXY)
	{
		pos();
		sensor_val_update_v();
		error_h=sensor_val_update_h();
		if(sum_sensor_h>0 && x_dis<-4400)
			break;
		if(x_dis>-4200)
		  control_pid_omni(-40*Degree_to_Rad,3);
		else
			control_pid_omni(-30*Degree_to_Rad,3);
	}
	
	//find_line(LEFT,FORWARD,3);
			
	sum_sensor_h=0;
	error_h=sensor_val_update_h();
	
*/	
	
	
	
	
/*	
	while(sum_sensor_h<=3)
	{
		error_h=sensor_val_update_h();
		if(error_h>4)
			find_line(RIGHT,FORWARD,7);
		control_pid_line_follow(FORWARD,8,error_h);//control_pid_omni(4*PI,15);
		
		if(sum_sensor_h>0 && !FORWARD_PROXY)
			break;
	}
	
*/	
	
//___________________________________unnecessary condition________IDK WHY THIS SNIPPET IS WRITTEN ____________________________________________	
	/*
	i=1500;
	while(i--)
	{
		control_pid_omni(PI,15);
	}
	
	while(FORWARD_PROXY)
	{
    error_h=sensor_val_update_h();
		control_pid_line_follow(BACKWARD,5,error_h);    
	}
	*/
//________________________________________________________________________________________________________________________________________________	
	/*
	test_decider();	
	
	while(zone == ZONE_TWO_START)
	{
		CLOSE_GRIPPER;
	  //wait till proxy is detected		
		while(ang<ZONE_2_ANGLE)
		{
			hold_angle+=0.01;
			
			if(hold_angle>ZONE_2_ANGLE)
				hold_angle=ZONE_2_ANGLE+4;
			
			if(ang<50)
				control_pid_omni(4*PI,10);
			else
				control_pid_omni(4*PI,5);
		}
		
		test_shoot();
		
		while(ang>0)
		{
			hold_angle-=0.005;
			
			if(hold_angle<0)
				hold_angle=0;
			
			if(ang>10)
				control_pid_omni(4*PI,10);
			else
				control_pid_omni(4*PI,5);
		}
		
		while(FORWARD_PROXY)
		{
			error_h=sensor_val_update_h();
			control_pid_line_follow(BACKWARD,5,error_h);    
		}
		
		test_decider();
 }
	*/
}


//=====================
void tz3_modified_DT()
{
  //______________--------------restore kp ------------___________________	
	kp=0;
	
//________________________encoder initialise____________________________	
	x_dis=0;
	T0TC=0;
	prvvaluel=0;
//______________________--local variables--______________________________
	int counterh_90=0,i=0,local_time=0;	
	hold_angle=ang=90;
	sum_sensor_h=0;
	sum_sensor_v=0;
	while(1)
	{
		cls();
		lcd(x_dis);
		sensor_val_update_v();
		pos();
		control_pid_omni(70*Degree_to_Rad,10);
		
		if(sum_sensor_v==4)
		{
			next=0;
			while(1)
			{
				sensor_val_update_v();
				error_h=sensor_val_update_v();
		    control_pid_omni(70*Degree_to_Rad,10);
				if(sum_sensor_h<=2)
				{
					counterh_90++;
					break;
				}
			}
			sum_sensor_h=0;
		}
		if ((counterh_90==1)&&(x_dis>300))
		{
			while(1)
			{
				kp=0.95;
				control_pid_omni(4*PI,5);
			}
			break;
		}
	}
	
	while(RIGHT_PROXY)
	{
		control_pid_omni(80*Degree_to_Rad,5);
	}
	while(1)
	{
		control_pid_omni(4*PI,5);
	}
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
//_______________________________________________________________________	
	while(x_dis<=1)  //30
	{
		pos();
		control_pid_omni(120*Degree_to_Rad,20);
	}
	sum_sensor_v=0;
	sum_sensor_h=0;
	while(ang>0)                                                // snippet for ctrm from wall to line  
	{
		sensor_val_update_h();
		if (ang>71)
		{
		hold_angle-=0.03;
		}
		else 
		{
			hold_angle-=0.03;
		}
		control_pid_omni((215-ang)*Degree_to_Rad,19);     //210
		if (hold_angle<0)
		{
			hold_angle=0;
		}
	
	}
	sum_sensor_v=0;
	sum_sensor_h=0;
	
	
	
	while(sum_sensor_h<3)//(sum_sensor_v<2 && sum_sensor_h<1)
	{
		cls();
		lcd((char *)"in");
		sensor_val_update_h();
		control_pid_omni((-160*Degree_to_Rad),4);
	}
	
	i=1500;           // 1500
	while(i--)                                                     // responsible for making the bot stick to line 
	{                                                              // the first time it catches the line(without overshoot)  
		control_pid_omni(-230*Degree_to_Rad,25); //-230
	}
	sum_sensor_h=0;
	while(sum_sensor_h<2)
	{
		cls();
		lcd((char *)"this shit ");
		sensor_val_update_h();
		control_pid_omni(-230*Degree_to_Rad,10); //-230
	}
	i=100;
	while(i--)
	{
			control_pid_omni(180*Degree_to_Rad,14); //-230
	}
	
	kp_line_sense=8;
	kd_line_sense=2.5;
	
  kp=0.9;
	
	next=0;
	sum_sensor_h=0;
	while(!next)                                               // line follower in backward direction
	{

		error_h=sensor_val_update_h();
		control_pid_line_follow(BACKWARD,12,error_h);
	/*	
		if(error_h>4)
			find_line(FORWARD,RIGHT,12);
		else if(error_h<-4)
			find_line(FORWARD,LEFT,12);
	*/	
		if(sum_sensor_h==4)
		{
			next=0;
			while(1)
			{
				sensor_val_update_h();
				error_h=sensor_val_update_h();
		    control_pid_line_follow(BACKWARD,12,error_h);
				//control_pid_omni(180*Degree_to_Rad,20);
				if(sum_sensor_h<=2)
				{
					counterh_90++;
					break;
				}
			}
			sum_sensor_h=0;
		}
		
		//______________________________________________BACKUP FOR UNSTABLE CONDITION________(LINE NOT FOLLOWED)___________________________________________________________		
	/*	
		if (sum_sensor_h==0)
			local_time++;
		else 
			local_time--;
		
		if(local_time<0)
			local_time=0;
		
		if (local_time>=1000)
		{
			if (heading_1<0)
			{
				while(sum_sensor_h<=1)
					
				{
					cls();
					lcd((char *)"dir1");
					lowerline();
					lcd(heading_1);
					sensor_val_update_h();
					control_pid_omni(-90*Degree_to_Rad,10);
				}
				local_time=0;
			}
			else 
			{
				while(sum_sensor_h<=1)
				{
					cls();
					lcd((char *)"dir2");
					lowerline();
					lcd(heading_1);
					sensor_val_update_h();
					control_pid_omni(90*Degree_to_Rad,10);
				}
				local_time=0;
			}
		}
		*/
//_________________________________________________________________________________________________________________________________________________		
		
		
		if(counterh_90==4)
		{
			next=1;
		}			
	}
	while(FORWARD_PROXY)
	{
		error_h=sensor_val_update_h();
		control_pid_line_follow(BACKWARD,5,error_h);
	}
	i=3500;
	while(i--)                                                           // reverse braking for proxy 
	{
    control_pid_omni(0*Degree_to_Rad,15);
	}
	while(FORWARD_PROXY)
	{
    error_h=sensor_val_update_h();
		control_pid_line_follow(FORWARD,3,error_h);    
	}
	kp_line_sense=8;                                                    // restore kp and kd (of line follower) to its original value
	kd_line_sense=2.5;
}




//----------------------------------------------------------------------------------------------------------------------------------

 void tz2_modified()
{
	
	tz2_fast_3();
/*	
		ang=0;
	 double heading_1=0,distance=0,factor=0;
	  int base_value=0,checkpost=0,x_rec=0,y_rec=0,i=0;
	enc_diff_1_2=1;
	int counterv_90=0;
//	if(!zone_complete)
//	{
		CLOSE_GRIPPER;
		
		next=0;
		count_cycle1=0;
		sum_sensor_h=0;
		sum_sensor_v=0;
		x_dis=0;
	  y_dis=0;
		T0TC=0;
	  T1TC=0;
		prvvaluel=0;
	  prvvaluer=0;
		 
		while(!next)
		{
			if(y_dis>-100)
				base_value=15;
			else
			  base_value=27;
			pos();
			sensor_val_update_h();
			sensor_val_update_v();
			
			if(count_cycle1<270)                 //
				control_pid_omni((-155)*Degree_to_Rad,base_value);                //23
			else
				control_pid_omni((-155)*Degree_to_Rad,3);
			
			if((sum_sensor_h>3 || sum_sensor_v>0) && count_cycle1>500)                               // backup 
				next=1;
			
			
			if (y_dis<-570)
				next=1;
		//	if(x_dis<-400)                 //
			//	next=1;
		}
		
		i=1600;	
		next=0;
		count_cycle1=0;
		icontrol=17;
		
		while(!next)
		{
			pos();
			sensor_val_update_h();
			sensor_val_update_v();
			
			hold_angle+=0.020; //0.017        //0.015                                      // 0.01
			if(hold_angle>77)				//75
			{
				hold_angle=77;
        next=1;				
			}
			if(count_cycle1<320)
				control_pid_omni((-96)*Degree_to_Rad,19); //16   // 105              // 13
			else
			 	control_pid_omni((-96)*Degree_to_Rad,13); //10  //105                  // 4
			
			//if(((sum_sensor_v>1)||(sum_sensor_h>1)) && ang>30)
//				next=1;
		//	if(ang>88)
				//next=1;
		}
		
		 hold_angle=77.3;	
		 next=0;
		 i=1000;
		 while(i--)
		 {
			 pos();
			 control_pid_omni((-30)*Degree_to_Rad,20);
		 }
		 factor = sqrt((double)(X1_TZ2_2 - x_dis)*(X1_TZ2_2 - x_dis) + (double)(Y1_TZ2_2 - y_dis)*(Y1_TZ2_2 - y_dis));
		 count_cycle=0;
		 while(!next) 
		{
			  distance = sqrt(((X1_TZ2_2-x_dis)*(X1_TZ2_2-x_dis))+((Y1_TZ2_2-y_dis)*(Y1_TZ2_2-y_dis)));
			  pos();
//*********************************************************			
			  base_value = 25*exp((-2)*(((factor-distance)/factor)));         // 2        // 30 
        if (base_value<8)
				{
					base_value=6;
				}					
//*********************************************************		
        
			  heading_1 = atan2((Y1_TZ2_2-y_dis),(X1_TZ2_2-x_dis));
				heading_1*=(-1);
			  heading_1 += 1.57;
				heading_1 -= ang*Degree_to_Rad;
//*********************************************************	
        if(count_cycle>150)//(distance<0.9*factor)	
				{
					reset(GRIPPER);
				}
				if (count_cycle>250)//(y_dis< Y1_TZ2_2 + 10)
				{
					set(PISTON_2);
				}
			  control_pid_omni(heading_1,base_value);
			  if (((x_dis<X1_TZ2_2+20)&&(x_dis>X1_TZ2_2-20))&&((y_dis<Y1_TZ2_2+20)&&(y_dis>Y1_TZ2_2-20)))//(y_dis<Y_TZ1_2+5)//((x_dis>X_TZ1_2+5)&&(y_dis<Y_TZ1_2+5))
			   {
				   next=1;
		 	   }
		}
		
	  
	//	count_cycle=0;
	//	while(count_cycle<50)
	//	{
	//		control_pid_omni(4*PI,5);
	//	}
	 zone = ZONE_TWO_START;
		set(PISTON_2);
		count_cycle=0;
		while(count_cycle<100)
		{
			control_pid_omni(4*PI,5);
		}
//  test_shoot		
		
*/	
	 
//----------------------------------------------------------------------------------------------------------------------------------------	 
//_---------------------------------------R E T U R N -------------------------------------------------------------------------------------	 
//------------------------------------------------------------------------------------------------------------------------------------------
	 //______________--------------restore kp ------------___________________	
	kp=0.95;
	
//________________________encoder initialise____________________________	
	x_dis=0;
	T0TC=0;
	prvvaluel=0;
//______________________--local variables--______________________________
	int counterh_90=0,i2=0,local_time=0;	
	//hold_angle=ang=90;
	
//_______________________________________________________________________	
	/*while(x_dis<=1)  //30
	{
		pos();
		control_pid_omni(120*Degree_to_Rad,15);
	}*/
	
	sum_sensor_v=0;
	sum_sensor_h=0;
	icontrol=10;
	hold_angle=0;
	while(ang>0)
	{
		control_pid_omni(4*PI,5);
	}
	reset(PISTON_2);
	/*
	while(ang>0)                                                // snippet for ctrm from wall to line  
	{
		sensor_val_update_h();
		if (ang>71)
		{
		hold_angle-=0.025;
		}
		else 
		{
			hold_angle-=0.025;
		}
		control_pid_omni((220-ang)*Degree_to_Rad,10);     //215
		if (hold_angle<0)
		{
			hold_angle=0;
		}	
	}
	*/
	
	sum_sensor_v=0;
	sum_sensor_h=0;	
	
	while(sum_sensor_v<4)//(sum_sensor_v<2 && sum_sensor_h<1)
	{
		cls();
		lcd((char *)"in");
		sensor_val_update_v();
	//	sensor_val_update_h();
		control_pid_omni((-130*Degree_to_Rad),7);
	}
	
	while(sum_sensor_h<1)//(sum_sensor_v<2 && sum_sensor_h<1)
	{
		cls();
		lcd((char *)"in");
		sensor_val_update_v();
		sensor_val_update_h();
		control_pid_omni((-90*Degree_to_Rad),4);
	}
	
	 int i=1000;           // 1500
	while(i--)                                                     // responsible for making the bot stick to line 
	{                                                              // the first time it catches the line(without overshoot)  
		control_pid_omni(90*Degree_to_Rad,25); //-230
	}
	
	/*sum_sensor_h=0;
	cls();
	lcd((char *)"this shit ");
	while(sum_sensor_h<2)
	{		
		sensor_val_update_h();
		control_pid_omni(-230*Degree_to_Rad,10); //-230
	}
		
	i=100;
	while(i--)
	{
			control_pid_omni(180*Degree_to_Rad,14); //-230
	}*/
	
	cls();
	
	kp_line_sense=8;
	kd_line_sense=2.5;
	
  kp=0.9;
	
	next=0;
	sum_sensor_h=0;
//_______________________ENCODER SETTING________________________________
  		x_dis=0;
	    y_dis=0;
		  T0TC=0;
	    T1TC=0;
		  prvvaluel=0;
	    prvvaluer=0;
      enc_diff_1_2=0;
//_______________________________________________________________________
	
	while(!next)                                               // line follower in backward direction
	{
		pos();
		error_h=sensor_val_update_h();
	//	if (counterh_90<=1)
		control_pid_line_follow(BACKWARD,4,error_h);
		//else if ((counterh_90>=1)&&(counterh_90<4))
		//	control_pid_line_follow(BACKWARD,23,error_h);            //25
		//else if (counterh_90==4)
		//	control_pid_line_follow(BACKWARD,4,error_h);             //5
	/*	
		if(error_h>4)
			find_line(FORWARD,RIGHT,12);
		else if(error_h<-4)
			find_line(FORWARD,LEFT,12);
	*/	
		if(sum_sensor_h==4)
		{
			next=0;
			while(1)
			{
				sensor_val_update_h();
				error_h=sensor_val_update_h();
		    control_pid_line_follow(BACKWARD,5,error_h);
				//control_pid_omni(180*Degree_to_Rad,20);
				if(sum_sensor_h<=2)
				{
					counterh_90++;
					break;
				}
			}
			sum_sensor_h=0;
		}
		
		//______________________________________________BACKUP FOR UNSTABLE CONDITION________(LINE NOT FOLLOWED)___________________________________________________________		
	/*	
		if (sum_sensor_h==0)
			local_time++;
		else 
			local_time--;
		
		if(local_time<0)
			local_time=0;
		
		if (local_time>=1000)
		{
			if (heading_1<0)
			{
				while(sum_sensor_h<=1)
					
				{
					cls();
					lcd((char *)"dir1");
					lowerline();
					lcd(heading_1);
					sensor_val_update_h();
					control_pid_omni(-90*Degree_to_Rad,10);
				}
				local_time=0;
			}
			else 
			{
				while(sum_sensor_h<=1)
				{
					cls();
					lcd((char *)"dir2");
					lowerline();
					lcd(heading_1);
					sensor_val_update_h();
					control_pid_omni(90*Degree_to_Rad,10);
				}
				local_time=0;
			}
		}
		*/
//_________________________________________________________________________________________________________________________________________________		
		
		if ((!FORWARD_PROXY)&&(y_dis<(-180)))
		 next=1;
				
	}
	
/*	
	i=1000;
	while(i--)
	{
		control_pid_omni(PI,10);
	}
	while(FORWARD_PROXY)
	{
		error_h=sensor_val_update_h();
		control_pid_line_follow(BACKWARD,5,error_h);         //5
	}
	*/
	
	
	/*
	i=2000;
	while(i--)                                                           // reverse braking for proxy 
	{
    control_pid_omni(0*Degree_to_Rad,10);
	}
	while(FORWARD_PROXY)
	{
    error_h=sensor_val_update_h();
		control_pid_line_follow(FORWARD,5,error_h);    
	}
	*/
	kp_line_sense=8;                                                    // restore kp and kd (of line follower) to its original value
	kd_line_sense=2.5;

//	zone_complete=1; 
	 
	 
	 

		
	//	test_decider();  //decide where to go
		
		zone_complete=1;
		
		
		
 	
	while(0)//(zone == ZONE_TWO_START)
	{
		tz2_counter++;
		//CLOSE_GRIPPER;
		
		next=0;
	  count_cycle1=0;
	  icontrol=15;
		while(ang<70)
		{
			hold_angle+=0.005;                      // 0.005
      control_pid_omni(4*PI,10);			
		}
		if(tz2_counter<5)	
	  	hold_angle=ZONE_2_ANGLE+2.7; //4               //2.8
		else 
			hold_angle=ZONE_2_ANGLE+1.25;
//------------------------------------------------delay added due to bad fucking mechanism of auto ------------------------------------
//---------------------------------------------------bad and inaccurate mechanical structure and gripper------------------------------- 		
	count_cycle1=0;
	while(count_cycle1<=250)
		control_pid_omni(4*PI,5);
		
//----------------------------------------------------------------------------------------------------------------------------------		
		
/*		
		if (!field_error)
		hold_angle=ZONE_2_ANGLE;
		else 
			hold_angle=ZONE_2_ANGLE+2;
	*/		
		
	  ExtPrintbin(COM3,1,170);
	 reset(GRIPPER);
	 count_cycle1=0;
	 while(count_cycle1<=80)
	 {
		    control_pid_omni(4*PI,10);
		  //control_pid_omni(0,7);
	 }
	 
		 set(PISTON_1);
	   int k=0;
	   k=6000;                                                 // delay added after actuation 
		 while(k--)
			 control_pid_omni(4*PI,10);
	   reset(PISTON_1);
	//	test_shoot();
		
		while(ang>0)
	  {
			hold_angle-=0.004;
			
			if(hold_angle<0)
				hold_angle=0;
			
			if(ang>10)
				control_pid_omni(4*PI,8);
			else
				control_pid_omni(4*PI,2);
			
			error_h=sensor_val_update_h();
	  }
		int i=0;
		i=8000;
		while(i--)
		{
			control_pid_omni(4*PI,5);
		}
		cls();
		lcd((char *)"mabkl");
		
		while((FORWARD_PROXY) || (sum_sensor_h<1))    //saamne wali proxy 
		{
			error_h=sensor_val_update_h();
			
			if((FORWARD_PROXY) && (sum_sensor_h<1))
				control_pid_omni(135*Degree_to_Rad,5);
			else if(FORWARD_PROXY)
				control_pid_line_follow(BACKWARD,5,error_h);
			else
				control_pid_omni(90*Degree_to_Rad,5);		
		}

		cls();
		lcd((char *)"bypassed");
		test_decider();
	}
}
//________________________________________________________________________________________________________________________________________________


void tz3_modified()
{
	static int shoot_count=0;	
	bool safety=0;
	CLOSE_GRIPPER;
	/*
	while(ang<90)                                                // snippet for ctrm from wall to line  
	{
		sensor_val_update_h();
		if(ang<10)
		{
			icontrol=3;
		}
		else if (ang<60)
		{
		  icontrol = ang;
			if(icontrol>10)
				icontrol=10;
		}
		else 
		{
			icontrol=4;
		}
		control_pid_omni((27-ang)*Degree_to_Rad,10);     //210		
	}
	*/
	                                                 ////  main working snippet if the above one doesnt work uncomment it 
	while(ang<77)              //90
	{
		hold_angle+=0.01;                   // 0.005
			
		icontrol = ((90-ang)/15) + 6;          //5
		
    if(ang>hold_angle)
			hold_angle=ang+3;
		
		if(hold_angle>87)                   //87
			hold_angle=90;
	
		//if(ang<50)
			control_pid_omni(4*PI,10);
		//else
			//control_pid_omni(4*PI,2);
	}
	icontrol=8;                              // restore icontrol to its normal value
	hold_angle=90;
	next=0;
	x_dis=0;
	T0TC=0;
	prvvaluel=0;
	int i= 2000,counterv_90=0;            // 1600
	count_cycle1=0;
	sum_sensor_v=0;
		
	while(!next)
	{
		sensor_val_update_v();
		
		if(sum_sensor_v==4)
		{
			//next=0;
			while(1)
			{
				sensor_val_update_v();
				error_h=sensor_val_update_v();
		   // control_pid_line_follow(BACKWARD,12,error_h);
				control_pid_omni((-83*Degree_to_Rad),20);
				if(sum_sensor_v<=2)
				{
					counterv_90++;
					break;
				}
			}
		
		}
		
		pos();
		
		if(x_dis>-1000)
			control_pid_omni((-76+(90-ang))*Degree_to_Rad,25); //66          //20
		else if(x_dis>-2050)                         //2850                                    //3000
			control_pid_omni((-83+(90-ang))*Degree_to_Rad,24);        // 18      //15          //18
		else if(x_dis>-2850)                                                             //3000
			control_pid_omni((-80+(90-ang))*Degree_to_Rad,15);    // 20  //83 
		else if(x_dis>-3100)                                                             //3200
			control_pid_omni((-79+(90-ang))*Degree_to_Rad,4);  //4               //5
		else
		  control_pid_omni((-79)*Degree_to_Rad,2);  //81
		
//_____________________________________-----slow the momentum of the bot-----_______________________________________________________		
		
		if ((x_dis>(-3100))&&(x_dis<(-3050)))
		   control_pid_omni((-83+(90-ang))*Degree_to_Rad,25);
		
//_____________________________________________________________________________________________________________________________________		
		
		if(((!Pin0_27)||(sum_sensor_v>=3)) && x_dis<-3200)			// 3300
		{
			safety=1;
			next=1;
		}
		
	
//___________________________________EMERGENCY STOP CONDITION IF ENCODER FAILS________________________________________________________-_____		
	/*
		if (count_cycle1>535)
		{
			i=500;
			while(i--)
			{
					control_pid_omni(70*Degree_to_Rad,25);
			}
			next=1;
		}
		*/
//______________________________________________________________________________________________________________________________________________		
	}
	
//____________________________if bot stops due to sensor patti the position still remains accurate__________________________________________		
		
 
	
		
//___________________________________________________________________________________________________________________________________________		
	
	
	
	cls();
	lcd(counterv_90);
	
	
//____________________________________________CHANGE IN CONDITION FOR SHOOT IN TZ3________________________________________________________________
	
	x_dis=0;
	T0TC=0;
	prvvaluel=0;
	shoot_3=1;
	
	
	i=1000;
	while(i--)
	{
		pos();
		control_pid_omni(65*Degree_to_Rad,25);           //20
	}
	
	
	while(x_dis>-220)
	{
		pos();
		if(x_dis>-200)
			control_pid_omni(-70*Degree_to_Rad,6);
		else
			control_pid_omni(-70*Degree_to_Rad,5);
	}
	
	/*
	while(Pin0_27)
	{
		control_pid_omni((70)*Degree_to_Rad,6);          //5
	}
	*/
	
	i=500;
	kp=0;
	while(i--)
	{		
		pos();		
		ang=90;
		control_pid_omni(20*Degree_to_Rad,8);
	}
	
	while(x_dis<-220)
	{
		pos();
		control_pid_omni(70*Degree_to_Rad,5);
	}
	
	kp=0;
	shoot_3=0;
  if(shoot_count<4)
	{		
		test_shoot();
		shoot_count++;
	}
	CLOSE_GRIPPER;
	kp=0;		
	x_dis=0;
	T0TC=0;
	prvvaluel=0;
	
	while(x_dis<350)                              // 380
	{
		pos();
		if(x_dis<100)                                       // 200
			control_pid_omni(70*Degree_to_Rad,12);                //5
		else
			control_pid_omni(70*Degree_to_Rad,4);
	}
	
	i= 500;
	while(i--)
	{
		control_pid_omni(-60*Degree_to_Rad,10);
	}
	
	shoot_3=1;
	test_shoot();
	
	shoot_count++;	
//___________________________________________________S H O O T  # 2 PATH CHANGE	__________________________________________________________________________
	/*
	while(Pin0_27)
	{
		control_pid_omni((-70)*Degree_to_Rad,5);
	}
	
	shoot_3=0;	
	
	kp=0;		
	x_dis=0;
	T0TC=0;
	prvvaluel=0;
	
	while(x_dis>320)
	{
		pos();
		if(x_dis>300)
			control_pid_omni(70*Degree_to_Rad,5);
		else
			control_pid_omni(70*Degree_to_Rad,4);
	}
	
	i= 500;
	while(i--)
	{
		control_pid_omni(-60*Degree_to_Rad,10);
	}
	
	shoot_3=0;
	test_shoot();
	*/
//------------------------------------------------------------FUNCTON ENDS-------------------------------------------------------------------------------------	
	
}

//__________________________________________________TZ3 F A S T E S T___________________________________________________________________________

void tz3_fastest()
{
	
		bool safety=0;
	CLOSE_GRIPPER;
	/*
	while(ang<90)                                                // snippet for ctrm from wall to line  
	{
		sensor_val_update_h();
		if(ang<10)
		{
			icontrol=3;
		}
		else if (ang<60)
		{
		  icontrol = ang;
			if(icontrol>10)
				icontrol=10;
		}
		else 
		{
			icontrol=4;
		}
		control_pid_omni((27-ang)*Degree_to_Rad,10);     //210		
	}
	*/
	/*                                                 ////  main working snippet if the above one doesnt work uncomment it 
	while(ang<90)
	{
		hold_angle+=0.01;                   // 0.005
			
		icontrol = ((90-ang)/15) + 6;          //5
				
		if(hold_angle>87)                   //87
			hold_angle=90;
		
		//if(ang<50)
			control_pid_omni(4*PI,10);
		//else
			//control_pid_omni(4*PI,2);
	}
	
	*/
	icontrol=8;                              // restore icontrol to its normal value
//	hold_angle=90;
	next=0;
	x_dis=0;
	T0TC=0;
	prvvaluel=0;
	int i= 2000,counterv_90=0;            // 1600
	count_cycle1=0;
	sum_sensor_v=0;
	int subtractor=0,difference=0,acc=0;
	while(!next)
	{
		
		
		
		
		
		
		pos();
		difference=x_dis-subtractor;
		if (difference<-100)
		{
			subtractor-=100;
			difference=0;
			acc++;
		}
		
		
		if (x_dis<-2000)
		control_pid_omni((-90*Degree_to_Rad),acc+10);
		else 
		control_pid_omni((-90*Degree_to_Rad),27-acc);
		
		
		//sensor_val_update_v();
	/*	
		if(sum_sensor_v==4)
		{
			//next=0;
			while(1)
			{
				sensor_val_update_v();
				error_h=sensor_val_update_v();
		   // control_pid_line_follow(BACKWARD,12,error_h);
				control_pid_omni((-83*Degree_to_Rad),20);
				if(sum_sensor_v<=2)
				{
					counterv_90++;
					break;
				}
			}
		
		}
		*/
		
		
		
		
		/*
		if(x_dis>-1000)
			control_pid_omni((-76+(90-ang))*Degree_to_Rad,25); //66          //20
		else if(x_dis>-2050)                         //2850                                    //3000
			control_pid_omni((-83+(90-ang))*Degree_to_Rad,25);        // 18      //15          //18
		else if(x_dis>-2850)                                                             //3000
			control_pid_omni((-80+(90-ang))*Degree_to_Rad,15);    // 20  //83 
		else if(x_dis>-3100)                                                             //3200
			control_pid_omni((-79+(90-ang))*Degree_to_Rad,4);  //4               //5
		else
		  control_pid_omni((-79)*Degree_to_Rad,2);  //81
		
		*/
		
		
//_____________________________________-----slow the momentum of the bot-----_______________________________________________________		
		
	//	if ((x_dis>(-3100))&&(x_dis<(-3050)))
	//	   control_pid_omni((-83+(90-ang))*Degree_to_Rad,25);
		
//_____________________________________________________________________________________________________________________________________		
		
		if(x_dis<-3500)//(((!Pin0_27)||(sum_sensor_v>=3)) && x_dis<-3200)			// 3300
		{
			safety=1;
			next=1;
		}
		
	
//___________________________________EMERGENCY STOP CONDITION IF ENCODER FAILS________________________________________________________-_____		
	/*
		if (count_cycle1>535)
		{
			i=500;
			while(i--)
			{
					control_pid_omni(70*Degree_to_Rad,25);
			}
			next=1;
		}
		*/
//______________________________________________________________________________________________________________________________________________		
	}
	
//____________________________if bot stops due to sensor patti the position still remains accurate__________________________________________		
		
   while(1)
	 {
		 control_pid_omni(4*PI,4);
	 }
	
		
//___________________________________________________________________________________________________________________________________________		
	
	
	
	cls();
	lcd(counterv_90);
	
	
//____________________________________________CHANGE IN CONDITION FOR SHOOT IN TZ3________________________________________________________________
	
	x_dis=0;
	T0TC=0;
	prvvaluel=0;
	shoot_3=1;
	
	
	i=1000;
	while(i--)
	{
		pos();
		control_pid_omni(65*Degree_to_Rad,25);           //20
	}
	
	
	while(x_dis>-220)
	{
		pos();
		if(x_dis>-200)
			control_pid_omni(-70*Degree_to_Rad,4);
		else
			control_pid_omni(-70*Degree_to_Rad,3);
	}
	
	/*
	while(Pin0_27)
	{
		control_pid_omni((70)*Degree_to_Rad,6);          //5
	}
	*/
	
	i=500;
	kp=0;
	while(i--)
	{		
		pos();		
		ang=90;
		control_pid_omni(20*Degree_to_Rad,8);
	}
	
	while(x_dis<-220)
	{
		pos();
		control_pid_omni(70*Degree_to_Rad,5);
	}
	
	kp=0;
	shoot_3=0;	
	test_shoot();
	CLOSE_GRIPPER;
	kp=0;		
	x_dis=0;
	T0TC=0;
	prvvaluel=0;
	
	while(x_dis<340)                              // 380
	{
		pos();
		if(x_dis<100)                                       // 200
			control_pid_omni(70*Degree_to_Rad,12);                //5
		else
			control_pid_omni(70*Degree_to_Rad,4);
	}
	
	i= 500;
	while(i--)
	{
		control_pid_omni(-60*Degree_to_Rad,10);
	}
	
	shoot_3=1;
	test_shoot();
	
//___________________________________________________S H O O T  # 2 PATH CHANGE	__________________________________________________________________________
	/*
	while(Pin0_27)
	{
		control_pid_omni((-70)*Degree_to_Rad,5);
	}
	
	shoot_3=0;	
	
	kp=0;		
	x_dis=0;
	T0TC=0;
	prvvaluel=0;
	
	while(x_dis>320)
	{
		pos();
		if(x_dis>300)
			control_pid_omni(70*Degree_to_Rad,5);
		else
			control_pid_omni(70*Degree_to_Rad,4);
	}
	
	i= 500;
	while(i--)
	{
		control_pid_omni(-60*Degree_to_Rad,10);
	}
	
	shoot_3=0;
	test_shoot();
	*/
//------------------------------------------------------------FUNCTON ENDS-------------------------------------------------------------------------------------	
	
}


//________________________________________TZ1_2 FAST FUNTION ___________________________________________________________________________________

void tz1_2_fast()
{
	kp=0.95;
	int i=500;
	while(i--)
	{
		 pos();
		 control_pid_omni(3.14,15);
	}
	
	bool local_junc=0;
	next=0;
	while(!next)
	{
		sensor_val_update_h();
	  pos();
	  heading_2 = atan(y_dis/(RADIUS+x_dis));
	 
	  if (heading_2<0)
	   {
			 heading_2*=1.5; 
		  //heading_2=1.57+heading_2;
		  heading_2 = heading_2*(-1);
		  heading_2+=1.57;
 	   }
		 
		 if ((y_dis>=(RADIUS-50))&&(local_junc==0))
			 local_junc=1;
			 
			 
	  heading_1=int(heading_2);
	  control_pid_omni(-(3.14-heading_2),15);
    if ((sum_sensor_h>=3)&&(local_junc==1))
			next=1;
		
	}
	
	i=1000;
	while(i--)
	{
		pos();
		 control_pid_omni((3.14-heading_2),15);
	}
	
	
	
	
	
	
	
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||	
	icontrol=17;
	while(ang<ZONE_2_ANGLE)
	{
		hold_angle+=0.025;
		control_pid_omni((0-ang)*Degree_to_Rad,10);     // 270        ||   //215 for normal path
	}
	icontrol=8;
	
	while(1)
	{
		control_pid_omni(4*PI,10);
	}
		
	
}





//_______________________________________________________________________________________________________________________________________________

void tz3_fast()
{
	CLOSE_GRIPPER;
	static int shoot_count=0;	
	int u=3000;
	bool rot=0;
	while(u--)
	{
		control_pid_omni(4*PI,5);
	}
	next=0;
	sum_sensor_v=0;
	icontrol=17;
	while(!next)
	{
	 sensor_val_update_v();
	 hold_angle+=0.019;                //0.017 //0.025;
	 control_pid_omni((10-ang)*Degree_to_Rad,30);    // 15  //20     // 270        ||   //215 for normal path
	 if (hold_angle>90)
	  {
		  hold_angle=90;
		  rot=1;	
	  }	
	 if ((sum_sensor_v>=3)&&(rot==1))
	 {
		 next=1;
	 }		 	
  }
	icontrol=8;
	u=1;
	while(u--)
	{
		 pos();
		 control_pid_omni(-((11-ang)*Degree_to_Rad),20);
	}
	
	
//______________________________________________________ENCODER SETTING___________________________________________________________________	
	T0TC=0;
	T1TC=0;
	x_dis=0;
	y_dis=0;
	prvvaluel=0;
	prvvaluer=0;
//{________________________________________________________________________________________________________________________________________
 sum_sensor_v=0;
 double distance=0;	
 int base_value=20;	
 bool local_flag=0;	
 next=0;
 while(!next)	
 {
	 sensor_val_update_v();
	 pos();
/*	 
//==================================================================================================	 
//------------------------------DEAD REKONING USING TWO ENCODES-----------------------------------	 
   distance = sqrt(((X_TZ3+x_dis)*(X_TZ3+x_dis))+((Y_TZ3+y_dis)*(Y_TZ3+y_dis))); 
	 heading_2 = atan2((((-1)*Y_TZ3)-y_dis),(((-1)*X_TZ3)-x_dis));
	 heading_2+=1.50;//1.57;
	// heading_2*=(-1);
   base_value = 40*exp((-2)*(((3429-distance)/3429)));         // 2        // 30 
	 control_pid_omni(heading_2,base_value);
//====================================================================================================
*/

//=====================================================================================================
//------------------------------- DEAD REKONING USING SINGLE HORIZONTAL ENCODER------------------------
  if (base_value>=15)//(x_dis<=(-2500))
	{
	 heading_2 = -90*Degree_to_Rad;
	}
	else 
	{
		 heading_2 = -75*Degree_to_Rad;               //70
	}
  base_value = 40*exp((2)*(((x_dis)/3418)));        // 2        // 30 	 
	control_pid_omni(heading_2,base_value);
	  
//=====================================================================================================	 
	 if (base_value<10)
	 {
		 base_value=10;
	 }
	 
	 if ((base_value<=12)||(x_dis<(-2800)))
	 {
		  local_flag=1;
	 }
	 
	 if((sum_sensor_v>=3)&&(local_flag==1))
	 {
	   next=1;
	 }
	 
	 
	 if((((-1)*(Y_TZ3-y_dis)<5))||(((-1)*(X_TZ3-x_dis)<5)))
	 {
		// next=1;
	 }
	 
	 
	 
 }
 
 	
//____________________________________________CHANGE IN CONDITION FOR SHOOT IN TZ3________________________________________________________________
  int i=0;
	x_dis=0;
  y_dis=0;
	T0TC=0;
  T1TC=0;
	prvvaluel=0;
  prvvaluer=0;
	shoot_3=1;
	
	
	i=1000;
	while(i--)
	{
		pos();
		control_pid_omni(65*Degree_to_Rad,20);           //25
	}
	
	
	while(x_dis>-220)
	{
		pos();
		if(x_dis>-200)
			control_pid_omni(-60*Degree_to_Rad,6);   //70
		else
			control_pid_omni(-60*Degree_to_Rad,5);
	}
	

	/*
	while(Pin0_27)
	{
		control_pid_omni((70)*Degree_to_Rad,6);          //5
	}
	*/
	
	i=500;
	//kp=0;
	while(i--)
	{		
		pos();		
		ang=90;
		control_pid_omni(20*Degree_to_Rad,8);
	}
	
  while(x_dis<-220)
	{
		pos();
		control_pid_omni(60*Degree_to_Rad,10);   //5  //70
	}
	
	kp=0;
	shoot_3=0;
  if(shoot_count<4)
	{			
		test_shoot();
		shoot_count++;
	}
	CLOSE_GRIPPER;
	kp=0;		
	x_dis=0;
	T0TC=0;
	prvvaluel=0;
	
	while(x_dis<350)                              // 380
	{
		pos();
		if(x_dis<100)                                       // 200
			control_pid_omni(70*Degree_to_Rad,8);                //5
		else
			control_pid_omni(70*Degree_to_Rad,4);
	}
	
	i= 500;
	while(i--)
	{
		control_pid_omni(-60*Degree_to_Rad,10);
	}
	
	shoot_3=1;
	test_shoot();
	
	shoot_count++;	
	
	
}

//_______________________________________________________________________________________________________________________________________________

//===============================================================================================================================================
//--------------------------------------------------START TO LOAD FUNCTION ----------------------------------------------------------------------

void start_to_load()
{
	 
	  enc_diff_1_2=0;
//==================take the encoder ticks the simple way=============
	  int i=0,temp=0;
	  double multiplier_ded_rekon=1;
//_________________________ENCODER SETTINGS____________________________ 	
		T0TC=0;
	  T1TC=0;
	  prvvaluel=0;
	  prvvaluer=0;
	  x_dis=0;
	  y_dis=0;
//_____________________________________________________________________
   
    next=0;
	  double heading_1=0,distance=0,factor=0;
	  int base_value=0,checkpost=0,x_rec=0,y_rec=0;
	  bool subtask_done = 0;
   
//===================first time coordinate allcation ===================	
		x_rec = (checkpost+1)*(X_TZ1/10);
		y_rec = (checkpost+1)*(Y_TZ1/10);
	  factor = sqrt((double)(X_TZ1)*(X_TZ1) + (double)(Y_TZ1)*(Y_TZ1));
    while(!next)
		{
		  //	cls();
		  //	lcd(heading_1*57.32);
			/*
			cls();
			lcd(x_dis);
			lowerline();
			lcd(y_dis);
			*/
			  pos();
			  sensor_val_update_h();
			  distance = sqrt(((X_TZ1-x_dis)*(X_TZ1-x_dis))+((Y_TZ1-y_dis)*(Y_TZ1-y_dis))); 
				heading_1 = atan2((Y_TZ1-y_dis),((X_TZ1)-x_dis));
				heading_1*=(-1);
				
//---------------------------setting for inintial slip of the bot -----------------------------------				
				if (x_dis > (-1)*1000)
				{
					base_value=35;
				  heading_1 += 1.8;
				}
				/*
				else if (x_dis > (-1)*500)
				{
					base_value=30;
				  heading_1 = ((-1)*1.22);
				}
				*/
//----------------------------------------------------------------------------------------------------				
				else 
				{ 
					base_value = 40*exp((-2)*(((factor-distance)/factor)));         // 2        // 30 
					heading_1 += 1.57;
				}
				/*
				if (x_dis<-1000)
				{
				  heading_1 += 1.57;    //57
				}
				else 
				{
					 heading_1 += 1.57;     // 1.67
 				}
				*/
			
				
			/*
			if (subtask_done==0)
			{
				checkpost++;
				x_rec = (checkpost+1)*(X_TZ1/10);
		    y_rec = (checkpost+1)*(Y_TZ1/10);
				subtask_done=1;
			}
		  heading_1 = (-1)*(atan2((y_rec+y_dis),(x_rec+x_dis)));   	
			distance = sqrt(((X_TZ1+x_dis)*(X_TZ1+x_dis))+((Y_TZ1+y_dis)*(Y_TZ1+y_dis))); 
      base_value = 15*exp((-2)*(((4139-distance)/4139)));         // 2        // 30 
			
			if (((-x_dis) > (x_rec - 7))||((-x_dis) > (x_rec - 5)))
			{
				subtask_done=0;
			}
			*/
			if (base_value<8)
			{
				base_value=6;
			}
	    control_pid_omni(heading_1,base_value);
			
			if (x_dis<(X_TZ1+400)&&(sum_sensor_h>1))//&&(y_dis>(Y_TZ1-5)))//(((-x_dis)>X_TZ1-5)&&((-y_dis)>Y_TZ1-5))
			{
				next=1;
			}
		
		}
				
		while(0)
		{
			pos();
			cls();
			lcd(y_dis);
			control_pid_omni(4*PI,5);
		}
		
		next=0;
		i=2000;//1000;
		base_value=30;
		while(i--)
		{
			pos();
			control_pid_omni(90*Degree_to_Rad, base_value);
		}
		sum_sensor_h=0;
		while(sum_sensor_h<2)
		{
			sensor_val_update_h();
			control_pid_omni(90*Degree_to_Rad, 6);
		}
		next=1;
		sum_sensor_h=0;
		while(!next)
		{
			sensor_val_update_h();
			control_pid_omni(90*Degree_to_Rad, 5);
			if (sum_sensor_h>=1)
				next=1;
		}	
		
//=====DEBUG SEQUENCE==============		
		while(0)
		{
			pos();
			cls();
			lcd(y_dis);
			lowerline();
			lcd(x_dis);
			control_pid_omni(4*PI,5);
		}
		
//===================	
    bool proxy_allowed=0;		
		sum_sensor_h=0;
		next=1;
		factor = sqrt((double)(X_TZ1_2 - x_dis)*(X_TZ1_2 - x_dis) + (double)(Y_TZ1_2 - y_dis)*(Y_TZ1_2 - y_dis));
		while(!next)
		{
		  	pos();
       // sensor_val_update_h(); 			
//*********************************************************			
			  base_value = 40*exp((-3)*(((factor-distance)/factor)));    //949     // 2        // 30 
			  distance = sqrt(((X_TZ1_2-x_dis)*(X_TZ1_2-x_dis))+((Y_TZ1_2-y_dis)*(Y_TZ1_2-y_dis)));
        if (base_value<8)
				{
					base_value=8;
				}					
				if (base_value>20)
				{
					base_value=20;
				}
//**************SENSOR AND PROXY CONDITION*******************************************
        /*
				if (sum_sensor_h==4)
				{
					proxy_allowed=1;
				}
				if ((proxy_allowed==1)&&(!FORWARD_PROXY))
				{
					next=1;
				}
				*/
//*********************************************************				
			  heading_1 = atan2((Y_TZ1_2-y_dis),(X_TZ1_2-x_dis));
				heading_1*=(-1);
			  heading_1*=multiplier_ded_rekon;
			  heading_1 += 1.57;
				//if (distance<0.9*factor)
				//{
				//	multiplier_ded_rekon=1.5;
				//}
			  control_pid_omni(heading_1,base_value);
				
				if (y_dis>(Y_TZ1_2-7))//(((x_dis<(X_TZ1_2+7))&&(x_dis>(X_TZ1_2-7)))&&((y_dis>(Y_TZ1_2-7))&&(y_dis<(Y_TZ1_2+7))))//(((-x_dis)>X_TZ1-5)&&((-y_dis)>Y_TZ1-5))
			   {
				   next=1;
			   }
				
        if ((distance<500)&&(!FORWARD_PROXY))
				{
				//	next=1;
				}					
			   
		}
		base_value=15;
		i=0;//7000;
		while(i--)
		{
			pos();
			control_pid_omni(4*PI,5);
			//control_pid_omni( ((-1)*150*Degree_to_Rad), base_value);
		}
//============================================================================================		
//=============================================================================================		
//=================FINAL ALIGNMENT OF THE BOT USING TWO PROXY ================================= 	
//_________________________ENCODER SETTINGS____________________________ 	
		T0TC=0;
	  T1TC=0;
	  prvvaluel=0;
	  prvvaluer=0;
	  x_dis=0;
	  y_dis=0;
//_____________________________________________________________________			
		next=0;
		bool backup=1;
		while((FORWARD_PROXY)&&(!next))
		{
			pos();
			if (y_dis>750)
			{
				backup=0;
				break;
			}
			error_h = sensor_val_update_h();
			control_pid_line_follow(FORWARD,6,error_h);
			//control_pid_omni(5*Degree_to_Rad,6);
		}
	
		/*
		sum_sensor_h=0;
		while(sum_sensor_h<1)
		{
			sensor_val_update_h();
			control_pid_omni((90*Degree_to_Rad),4);
		}
		*/
		i=1000;
		while(i--)
		{
			control_pid_omni(180*Degree_to_Rad,20);
		}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++		
		
		
		
//============================================================================================================		
//---------------------------------BACKUP IF PROXY FAILS------------------------------------------------------		
//=============================================================================================================		
	
		next=0;
		sum_sensor_v=0;
		while ((!backup)&&(!next))
		{
			sensor_val_update_v();
			if(sum_sensor_v>1)
			{
				next=1;
			}
			error_h = sensor_val_update_h();
			control_pid_line_follow(BACKWARD,8,error_h);
		}
		
		if (!backup)
		{
//_________________________ENCODER SETTINGS____________________________ 	
		T0TC=0;
	  T1TC=0;
	  prvvaluel=0;
	  prvvaluer=0;
	  x_dis=0;
	  y_dis=0;
//_____________________________________________________________________			
		
	  	
		while(!backup)
		{
			if (y_dis < (-345))
			{
				backup=1;
			}
			pos();
			error_h = sensor_val_update_h();
			control_pid_line_follow(BACKWARD,5,error_h);
		}
		backup=0;
	}
//=============================================================================================================		
//---------------------------------BACKUP ENDS ------------------------------------------------------		
//=============================================================================================================





//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	
		while((FORWARD_PROXY)&&(backup))
		{
			control_pid_omni(180*Degree_to_Rad,5);
		}
	
		if (backup)
		{
//_________________________ENCODER SETTINGS____________________________ 	
		T0TC=0;
	  T1TC=0;
	  prvvaluel=0;
	  prvvaluer=0;
	  x_dis=0;
	  y_dis=0;
//_____________________________________________________________________						
		}
		while(backup)
		{
			pos();
			if (y_dis<(-105))
			{
				backup=0;
			}
			error_h = sensor_val_update_h();
			control_pid_line_follow(BACKWARD,6,error_h);
		}	
		
//===============================================================================================		
		while(0)
		{
			
			pos();
			cls();
			lcd(y_dis);
			lowerline();
			lcd(x_dis);
			control_pid_omni(4*PI,5);
		}
		
}




//-----------------------------------------------------FUNCTION END ----------------------------------------------------------------------------- 
//===============================================================================================================================================



///=========================================FUNCTION FOR TZ1 TO TZ2 ====================================================================
//=-------------------------------------------------------------------------------------------------------------------------------------


void tz2_fast()
{
  CLOSE_GRIPPER;
//=================================================================================================================
//*
//*                                       GOING FOR FIRST CO-ORDINATE      
//*
//*
//==================================================================================================================	
//___________________________________________ENCODER SETTING ____________________________________________
	  x_dis=0;
   	y_dis=0;
	  prvvaluel=0;
	  prvvaluer=0;
	  T0TC=0;
	  T1TC=0;
//_______________________________________________________________________________________________________	
	  next=0;
    icontrol=10;	
    enc_diff_1_2=1;		
	  double heading_1=0,distance=0,factor=0;
	  int base_value=0,checkpost=0,x_rec=0,y_rec=0,i=0;
	  factor = sqrt((double)(X_TZ2)*(X_TZ2) + (double)(Y_TZ2)*(Y_TZ2));
    icontrol = 40;		
    while(!next) 
		{
			  distance = sqrt(((X_TZ2-x_dis)*(X_TZ2-x_dis))+((Y_TZ2-y_dis)*(Y_TZ2-y_dis)));
			
			  if (hold_angle < 0)
				{
					hold_angle = 0;
				}
				 
			  pos();
//*********************************************************			
			  base_value = 33*exp((-2)*(((factor-distance)/factor)));         // 2        // 30 
        if (base_value<15)
				{
					base_value=15;
				}					
//*********************************************************				
			  heading_1 = atan2((Y_TZ2-y_dis),(X_TZ2-x_dis));
				heading_1*=(-1);
			  heading_1 += 1.57;
				heading_1 -= ang*Degree_to_Rad;
//*********************************************************				
			  control_pid_omni(heading_1,base_value);
			  if ((x_dis<X_TZ2+5)&&(y_dis<Y_TZ2+5))//(y_dis<Y_TZ1_2+5)//((x_dis>X_TZ1_2+5)&&(y_dis<Y_TZ1_2+5))
			   {
				   next=1;
		 	   }
		}
		
		
		
		
		
	/*	
    i=500;
		base_value=10;
    while(i--)
		{
			pos();
			control_pid_omni((-1)*heading_1,base_value);
		}
   		
		while(1)
		{
			cls();
			lcd(x_dis);
			lowerline();
			lcd(y_dis);
			control_pid_omni(4*PI,5);
		}
	*/
	
    
//=================================================================================================================
//*
//*                                    GOING FOR SECOND CO-ORDINATE      
//*
//*
//==================================================================================================================
//___________________________________________ENCODER SETTING ____________________________________________	
	  x_dis=0;
   	y_dis=0;
	  prvvaluel=0;
	  prvvaluer=0;
	  T0TC=0;
	  T1TC=0;
//_______________________________________________________________________________________________________		 
	  factor = sqrt((double)(X_TZ2_2 - x_dis)*(X_TZ2_2 - x_dis) + (double)(Y_TZ2_2 - y_dis)*(Y_TZ2_2 - y_dis));
	  next=0;
		enc_diff_1_2=1;
	//	hold_angle = 70;
    while(!next)
	  {
		    pos();
//*********************************************************			
			  base_value = 30*exp((-2.5)*(((factor-distance)/factor)));         // 2        // 30 
			  distance = sqrt(((X_TZ2_2-x_dis)*(X_TZ2_2-x_dis))+((Y_TZ2_2-y_dis)*(Y_TZ2_2-y_dis)));
        if (base_value<15)
				{
					base_value=15;
				}					
//*********************************************************				
			  heading_1 = atan2((Y_TZ2_2-y_dis),(X_TZ2_2-x_dis));
				heading_1 *= (-1);
			  heading_1 += 1.57;
				heading_1 -= ang*Degree_to_Rad;
//*********************************************************				
			  control_pid_omni(heading_1,base_value);
			  if ((x_dis<X_TZ2_2+5)&&(y_dis>Y_TZ2_2-5))//(y_dis<Y_TZ1_2+5)//((x_dis>X_TZ1_2+5)&&(y_dis<Y_TZ1_2+5))
			   {
				    next=1;
		 	   }  
	  }		 
		
	   
	
		
		
//=================================================================================================================
//*
//*                                   GOING FOR THIRD CO-ORDINATE      
//*
//*
//==================================================================================================================
//___________________________________________ENCODER SETTING ____________________________________________	
	  x_dis=0;
   	y_dis=0;
	  prvvaluel=0;
	  prvvaluer=0;
	  T0TC=0;
	  T1TC=0;
//_______________________________________________________________________________________________________		 
	  distance=0;
	  next=0;
		icontrol=17;
		factor = sqrt((double)(X_TZ2_3 - x_dis)*(X_TZ2_3 - x_dis) + (double)(Y_TZ2_3 - y_dis)*(Y_TZ2_3 - y_dis));
    while(!next)
	  {   
			
				if (distance > 0.20*factor)
				 {
	 		       hold_angle = ZONE_2_ANGLE*(1 - distance/factor);  
             hold_angle*=3;
				     if (ang>hold_angle)
				      {
                hold_angle=ang;
				      }
					}
				
        if (hold_angle>70)
				 {
					 icontrol = 10;
					 hold_angle=70;//ZONE_2_ANGLE;
				 }
		
			
			
		   	pos();
//*********************************************************			
			  base_value = 40*exp((-3)*(((factor-distance)/factor)));         // 2        // 30 
			  distance = sqrt(((X_TZ2_3-x_dis)*(X_TZ2_3-x_dis))+((Y_TZ2_3-y_dis)*(Y_TZ2_3-y_dis)));
        if (base_value<8)
				{
					base_value=8;
				}					
//*********************************************************				
			  heading_1 = atan2((Y_TZ2_3-y_dis),(X_TZ2_3-x_dis));
				heading_1 *= (-1);
			  heading_1 += 1.57;
			  heading_1 -= ang*Degree_to_Rad;
//*********************************************************	
        if (hold_angle!=70)
				{					
				control_pid_omni(heading_1,20);
				}
        else
				{
				control_pid_omni(heading_1,8);	
				}					
			  if (((y_dis>Y_TZ2_3-5)&&(y_dis<Y_TZ2_3+5))&&((x_dis > X_TZ2_3-5)&&(x_dis < X_TZ2_3+5)))//&&(x_dis<X_TZ1_2+5))//(y_dis<Y_TZ1_2+5)//((x_dis>X_TZ1_2+5)&&(y_dis<Y_TZ1_2+5))
			   {
					 reset(GRIPPER);
				   next=1;
		 	   } 
        
	  }		

		
    count_cycle=0;
		while(count_cycle<100)
		{
			control_pid_omni(4*PI,5);
		}
		set(PISTON_2);		
		
		
			while(1)
		{
			pos();
			cls();
			lcd(x_dis);
			lowerline();
			lcd(y_dis);
			control_pid_omni(4*PI,5);
		}	
		
			
		
		next=0;
		icontrol=8;
		hold_angle = ZONE_2_ANGLE;
//================================================================================================================
    while(!next)
		{
			cls();
			lcd(heading_1*57.32);
			pos();
			base_value = 20*exp((-2)*(((1166-distance)/1166)));         // 2        // 30 
			distance = sqrt(((X_TZ2_3-x_dis)*(X_TZ2_3-x_dis))+((Y_TZ2_3-y_dis)*(Y_TZ2_3-y_dis)));
			heading_1 = atan2((Y_TZ2_3-y_dis),(X_TZ2_3-x_dis));
			heading_1 += 1.57;
			control_pid_omni(heading_1,10);
			
			
		}










//=================================================================================================================		
		
		i=0;//500;
		base_value=18;
		 while(i--)
		{
			pos();
			control_pid_omni((-1)*heading_1,base_value);
		}
  
}







//-------------------------------------------FUNCTION ENDS ------------------------------------------------------------------------------
//=======================================================================================================================================


void test_decider_2()
{
	next=0;
	bool abrupt=0;
	while(!next)
	{
		control_pid_omni(4*PI,5);
		if(in_tz1==0)
		{
		  if(!NORMAL_BALL_PROXY)
		   {
				// next=1;
			   count_cycle=0;
			   while(count_cycle<1)
			    {
			    	control_pid_omni(4*PI,5);
					}
					next=1;
	/*				
				    if (NORMAL_BALL_PROXY)
					    abrupt=1;
			    }
			   if (abrupt)
			    {
				    abrupt=0;
			    } 
			   else 
		    	{
				    count_task++;
				    next=1;
			    }
				*/
		    }
			 else 
			 {
				 if (!NORMAL_BALL_PROXY)
				 {
					 count_cycle=0;
					 while(count_cycle<1)
					 {
						 control_pid_omni(4*PI,5);
					 }
					 next=1;
				 }
			 }
	}
		if (count_task==1)
		  zone = ZONE_ONE_START;
		else if (count_task==2)
			zone = ZONE_TWO_START;
		else if (count_task==3)
			zone = ZONE_THREE_SHOOT;
		
	}
}
//=============================================FUNCTION FOR TZ2 TO TZ3========================================================================================
//-------------------------------------------------------------------------------------------------------------------------------------
void tz3_fast_2()
{
	  enc_diff_1_2=1;
//==================take the encoder ticks the simple way=============
	  int i=0;
 //___________________________________________ENCODER SETTING ____________________________________________
	  x_dis=0;
   	y_dis=0;
	  prvvaluel=0;
	  prvvaluer=0;
	  T0TC=0;
	  T1TC=0;
//_______________________________________________________________________________________________________	
	  next=0;
    icontrol=10;	
    enc_diff_1_2=1;		
	  bool acc=0;
	  double heading_1=0,distance=0,factor=0;
	  int base_value=0;//checkpost=0,x_rec=0,y_rec=0;
	  factor = sqrt((double)(X_TZ3_2)*(X_TZ3_2) + (double)(Y_TZ3_2)*(Y_TZ3_2));
    icontrol = 50;
    sum_sensor_v=0;	
    while(!next) 
		{
			  sensor_val_update_v();
			  distance = sqrt(((X_TZ3_2-x_dis)*(X_TZ3_2-x_dis))+((Y_TZ3_2-y_dis)*(Y_TZ3_2-y_dis)));
			
			  if (hold_angle < 0)
				{
					hold_angle = 0;
				}
				
				if (distance > 0.2*factor)
				 {
	 		       hold_angle = ZONE_2_ANGLE*(1 - distance/factor);  
             hold_angle*=5;
				     if (ang>hold_angle)
				      {
                hold_angle=ang;
				      }
					}
				
        if (ang>90)
				 {
					 acc=1;
					 icontrol = 10;
					 hold_angle=90;//ZONE_2_ANGLE;
				   icontrol=5;			
				 }
				 
			  pos();
//*********************************************************
        if (acc)
				{			
        		
			   base_value = 30*exp((-3)*(((factor-distance)/factor)));         // 2        // 30 
				}
				else 
				{
					base_value = 30;
				}
        if (base_value<10)
				{
					base_value=6;
				}					
//*********************************************************				
			  heading_1 = atan2((Y_TZ3_2-y_dis),(X_TZ3_2-x_dis));
				heading_1*=(-1);
			  heading_1 += 1.57;
				heading_1 -= ang*Degree_to_Rad;
//*********************************************************		
			  if ((y_dis>(Y_TZ3_2-8))&&(y_dis<(Y_TZ3_2+8)))//(((x_dis<(X_TZ3_2+5))&&(x_dis>(X_TZ3_2-5)))&&((y_dis>(Y_TZ3_2-8))&&(y_dis<(Y_TZ3_2+8))))//((x_dis>(X_TZ3_2-8))&&(y_dis>3000))//(Y_TZ3_2-5)))//((x_dis<X_TZ2+5)&&(y_dis<Y_TZ2+5))//(y_dis<Y_TZ1_2+5)//((x_dis>X_TZ1_2+5)&&(y_dis<Y_TZ1_2+5))
			   {
				    //next=1;
		 	   }
//=== TEMPORARY CONDITION FOR STOPPING ==							 
				if (y_dis > 3500)
				{
					//control_pid_omni(90*Degree_to_Rad,15);
					
					if (sum_sensor_v>=3)
					{
						//next=1;
					}
				}
        //else
				{
					control_pid_omni(heading_1,base_value);
				}					
//========================================				 
				//if((y_dis>3600)&&(!FORWARD_PROXY)) 
					//next=1;
				 
				 
				// if((!RIGHT_PROXY)&&(heading_1>0)&&(hold_angle==90))
				 // next=1;
		}

		
//___________________________________________ENCODER SETTING ____________________________________________
	  x_dis=0;
   	y_dis=0;
	  prvvaluel=0;
	  prvvaluer=0;
	  T0TC=0;
	  T1TC=0;
//_______________________________________________________________________________________________________		
	
		
		
		
		
//=======================================		
		i=1500;
		while(i--)
		{
			 pos();
			 control_pid_omni(80*Degree_to_Rad,23);				
		}
		while(0)
		{
			pos();
			control_pid_omni(4*PI,5);
			cls();
			lcd(x_dis);
		}
		/*
		while(x_dis<70)
		{
			pos();
			if (x_dis<50)
			 control_pid_omni(75*Degree_to_Rad,8);
			else 
				 control_pid_omni(75*Degree_to_Rad,4);
		}
		*/
		
		next=1;
		while(!next)
		{
			if (!FORWARD_PROXY)
				next=1;
			control_pid_omni((-1)*(70*Degree_to_Rad),8);
		}
		
		test_shoot();
}

//------------------------------------------------FUNCTION ENDS------------------------------------------------------------------------------------
//=====================================================================================================================================

void tz2_fast_2()
{
	  CLOSE_GRIPPER;
//=================================================================================================================
//*
//*                                       GOING FOR FIRST CO-ORDINATE      
//*
//*
//==================================================================================================================	
//___________________________________________ENCODER SETTING ____________________________________________
	  x_dis=0;
   	y_dis=0;
	  prvvaluel=0;
	  prvvaluer=0;
	  T0TC=0;
	  T1TC=0;
//_______________________________________________________________________________________________________	
	  next=0;
	  kp=0.95;
    icontrol=10;	
    enc_diff_1_2=1;		
	  double heading_1=0,distance=0,factor=0;
	  int base_value=0,checkpost=0,x_rec=0,y_rec=0,i=0;
	  factor = sqrt((double)(X1_TZ2)*(X1_TZ2) + (double)(Y1_TZ2)*(Y1_TZ2));
    icontrol = 40;		
    while(!next) 
		{
			  distance = sqrt(((X1_TZ2-x_dis)*(X1_TZ2-x_dis))+((Y1_TZ2-y_dis)*(Y1_TZ2-y_dis)));
			
			  if (hold_angle < 0)
				{
					hold_angle = 0;
				}
				 
			  pos();
//*********************************************************	
        if(distance<0.2*factor)	
				{					
	  		 	base_value = 20;
				}
				else if (distance<0.5*factor)
				{
						base_value = 30;
				}
				else
				{
					 base_value = 30*exp((-2)*(((factor-distance)/factor)));         // 2        // 30 
				
				}
        if (base_value<15)
				{
					base_value=15;
				}					
//*********************************************************				
			  heading_1 = atan2((Y1_TZ2-y_dis),(X1_TZ2-x_dis));
				heading_1*=(-1);
			  heading_1 += 1.57;
				heading_1 -= ang*Degree_to_Rad;
//*********************************************************	
     //   control_pid_omni(-168*Degree_to_Rad,30); 				
			  control_pid_omni(heading_1,base_value);
			  if ((x_dis<X1_TZ2+5)&&(y_dis<Y1_TZ2+5))//(y_dis<Y_TZ1_2+5)//((x_dis>X_TZ1_2+5)&&(y_dis<Y_TZ1_2+5))
			   {
				   next=1;
		 	   }
		}
		
		next=0;
		count_cycle1=0;
		icontrol=17;
		
		while(!next)
		{
			
			pos();
			//sensor_val_update_h();
			//sensor_val_update_v();
			
			hold_angle+=0.017;        //0.015                                      // 0.01
			if(hold_angle>77.3)				//75
			{
				next=1;
				hold_angle=77.3;				
			}
			if(count_cycle1<320)
				control_pid_omni((-98)*Degree_to_Rad,16);   //16      //105          // 13
			else
			 	control_pid_omni((-70)*Degree_to_Rad,12); //10             //105    // 4
			
			//if(((sum_sensor_v>1)||(sum_sensor_h>1)) && ang>30)
//				next=1;
			//if(ang>88)
				//next=1;
		}
		hold_angle=77.3;	
		next=0;
		 factor = sqrt((double)(X1_TZ2_2 - x_dis)*(X1_TZ2_2 - x_dis) + (double)(Y1_TZ2_2 - y_dis)*(Y1_TZ2_2 - y_dis));
		 while(!next) 
		{
			  distance = sqrt(((X1_TZ2_2-x_dis)*(X1_TZ2_2-x_dis))+((Y1_TZ2_2-y_dis)*(Y1_TZ2_2-y_dis)));
			
			 
				 
			  pos();
//*********************************************************			
			  base_value = 25*exp((-3)*(((factor-distance)/factor)));         // 2        // 30 
        if (base_value<6)
				{
					base_value=6;
				}					
//*********************************************************		
        
			  heading_1 = atan2((Y1_TZ2_2-y_dis),(X1_TZ2_2-x_dis));
				heading_1*=(-1);
			  heading_1 += 1.57;
				heading_1 -= ang*Degree_to_Rad;
//*********************************************************				
			  control_pid_omni(heading_1,base_value);
			  if (((x_dis<X1_TZ2_2+5)&&(x_dis>X1_TZ2_2-5))&&((y_dis<Y1_TZ2_2+5)&&(y_dis>Y1_TZ2_2-5)))//(y_dis<Y_TZ1_2+5)//((x_dis>X_TZ1_2+5)&&(y_dis<Y_TZ1_2+5))
			   {
				   next=1;
		 	   }
		}
		
	  reset(GRIPPER);	
		count_cycle=0;
		while(count_cycle<100)
		{
			control_pid_omni(4*PI,5);
		}
	 zone = ZONE_TWO_START;
		test_shoot();
		
}
//================================================================================================================================
//                                                TZ3_FAST_3
//================================================================================================================================ 
 void tz3_fast_3()
 { 	 
	  enc_diff_1_2=1;
	 zone  = ZONE_THREE_SHOOT;
//==================take the encoder ticks the simple way=============
	  int i=0;
 //___________________________________________ENCODER SETTING ____________________________________________
	  x_dis=0;
   	y_dis=0;
	  prvvaluel=0;
	  prvvaluer=0;
	  T0TC=0;
	  T1TC=0;
//____________________________________________VARIABLES INITIALISE_______________________________________________________________________________	
	  next=0;
    icontrol=10;	
    enc_diff_1_2=1;		
	  bool acc=0,turned=0;
	  double heading_1=0,distance=0,factor=0;
	  int base_value=25,counter_v=0;//checkpost=0,x_rec=0,y_rec=0;
	  factor = sqrt((double)(X_TZ3_3_2)*(X_TZ3_3_2) + (double)(Y_TZ3_3)*(Y_TZ3_3));
    icontrol = 60;
    sum_sensor_h=0;	
//===============================================================================================================================================		
//_______________________________________GO TOWARDS FIRST JUNCTION USING CTRM WITHOUT FEEDBACK____________________________________________________
//===============================================================================================================================================
    while(!next) 
		{
			  sensor_val_update_v();
			  heading_1 = hold_angle*Degree_to_Rad;
			  heading_1 *= (-1);
			  heading_1+=(-1)*(2*Degree_to_Rad);
			  hold_angle += 0.015;
			 
/*				
				if (distance > 0.1*factor)
				 {
	 		       hold_angle = ZONE_2_ANGLE*(1 - distance/factor);  
             hold_angle*=5.2;
				     if (ang>hold_angle)
				      {
                hold_angle=ang;
				      }
					}
*/				
			
			  
			
			  if (ang>hold_angle)
				  {
             hold_angle=ang;
				  }
			
        if ((hold_angle>90)||(ang>90))
				 {
					 acc=1;
					 icontrol = 40;
					 hold_angle=90;//ZONE_2_ANGLE;
					 turned=1;
				 }
			
				if ((sum_sensor_v>=2)&&(turned==1))
				 {
				 	 next=1;
				 }
	         
				if (turned==1)
				 {
					 icontrol=40;
					 base_value=50;
					 heading_1 = (-89)*Degree_to_Rad;
				 }
				control_pid_omni(heading_1,base_value);
		}
		hold_angle=90;
//=================================================================================================================================================		
//______________________________________GO TOWARDS FINAL POINT WITH DECCELERATION WITH ENCODER FEEDBACK___________________________________________		
//================================================================================================================================================= 
//___________________________________________ENCODER SETTING ____________________________________________
	  x_dis=0;
   	y_dis=0;
	  prvvaluel=0;
	  prvvaluer=0;
	  T0TC=0;
	  T1TC=0;
		enc_diff_1_2=0;
//_______________________________________________________________________________________________________
			
	next=0;
	sum_sensor_h=0;
	bool reverse_over=0;	
  factor = sqrt((double)(X1_TZ3)*(X1_TZ3) + (double)(Y1_TZ3)*(Y1_TZ3));	
  double p_x_dis=0,p_y_dis=0,velocity_x=0,velocity_y=0;
  double diff_tick_enc=0,diff_max=0,target_speed=55,temp_tick_enc=0,factor_enc=0;
  base_value = 30;
  count_cycle2=0;
  bool gripper_set=0,non_return_tz3=0;	
  while(!next)
	{  
		 pos();
		 diff_tick_enc = ((-1)*x_dis) - temp_tick_enc;
		 if ((count_cycle2 > 20)&&(gripper_set==0))
		 {
			reset(PISTON_3);
      gripper_set=1;			 
		 }
		 else if ((count_cycle2 > 50)&&(gripper_set==1))
		 {
			 CLOSE_GRIPPER;
		 }
		 sensor_val_update_h();
		
		 
//==================================================	
/*    
		 if (x_dis < (-2500))	
		 {
			 target_speed = 40;
		 }			 
	*/	
/*		 
//---------------------------------------------------	
		 heading_1 = atan2((Y1_TZ3-y_dis),(X1_TZ3-x_dis));
		 heading_1*=(-1);
		 heading_1 += 1.57;
		 //heading_1 -= ang*Degree_to_Rad;
//---------------------------------------------------	
		 */
     /*
		 base_value = 40*exp((-3.1)*(((-1)*x_dis)/3200));       // base_value = 25*exp((-2)*(((factor-distance)/factor)));         // 2      // 30      
     distance = sqrt(((X1_TZ3-x_dis)*(X1_TZ3-x_dis))+((Y1_TZ3-y_dis)*(Y1_TZ3-y_dis)));
		 if (base_value<10)
			 base_value=10;
		 if (base_value>30)
			 base_value=30;
		 
		 if (base_value>20)
		 {
			 icontrol=30;
			 heading_1 = (-90)*Degree_to_Rad;
		 }
		 else 
		 {
			  icontrol=20;
			  heading_1 = (-90)*Degree_to_Rad;
		 }
		 */
		

		 
		
		 
	//=============================================================	 
		 if(non_return_tz3==0)
		 {
		 if (x_dis > (-1180))
		 {
			 heading_1 = (-87)*Degree_to_Rad;
			 base_value = 40;
		 }
		 else if (x_dis > (-2670))
		 {
			 heading_1 = (-76)*Degree_to_Rad;
			 base_value=6;
		 }
		 else 
		 {
			 base_value=4;
			 heading_1 = 85*Degree_to_Rad;
			 reverse_over=1;
		 }
		 
		 if (diff_tick_enc <= 0)
		 {
			 non_return_tz3=1;
			 reverse_over=1;
			  heading_1 = (-82)*Degree_to_Rad;
			   base_value=8;
		 }
			
	 }
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 /*
		 if (reverse_over==1)
		 {
			   heading_1 = (-80)*Degree_to_Rad;
			   base_value = 6; 
		 }
*/
	

		 
		 control_pid_omni(heading_1,base_value); 
		if (x_dis<(-2300))//(distance<0.25*factor)
		 {
			 if (sum_sensor_h>=2)
			 {
				 kp=0.9;
				 next=1;
			 }
		 }
		 p_x_dis = x_dis;
     p_y_dis = y_dis;		
     temp_tick_enc = ((-1)*x_dis);		 
	}
//=============================================================================================================================================	
//                     SNIPPET FOR COMPENSATING THE OVERSHOOT AND BRINGING THE BOT TO FINAL POSITION
//=============================================================================================================================================
		i=1500;
	  if (1)
		{
		while(i--)
		{
			 pos();
			 control_pid_omni(81*Degree_to_Rad,28);				
		}
	}
		sum_sensor_h=0;
		
//___________________________________________ENCODER SETTING ____________________________________________
	  x_dis=0;
   	y_dis=0;
	  prvvaluel=0;
	  prvvaluer=0;
	  T0TC=0;
	  T1TC=0;
		enc_diff_1_2=0;
		kp=0.9;
//_____________________________________________
   while(sum_sensor_h < 2)
 {
	 sensor_val_update_h();
	 control_pid_omni((88)*Degree_to_Rad,5);
 }	 
  
//________________________use only if bot is to be stopped on sensor patti______________________________________	 
		next=1;
		while(!next)
		{
			sensor_val_update_h();
			control_pid_omni(80*Degree_to_Rad,6);
			if (sum_sensor_h>=2)
				next=1;
		}

		/*
		while(x_dis<70)
		{
			pos();
			if (x_dis<50)
			 control_pid_omni(75*Degree_to_Rad,8);
			else 
				 control_pid_omni(75*Degree_to_Rad,4);
		}
		*/
		
		next=1;
		while(!next)
		{
			if (!FORWARD_PROXY)
				next=1;
			control_pid_omni((-1)*(70*Degree_to_Rad),8);
		}
		zone = ZONE_THREE_SHOOT;
		test_shoot();
 }

//---------------------------------FUNCTION ENDS-----------------------------------------------------------------------------------
//================================================================================================================================
 //==================================TZ1_SHOOT============================================================
 //-------------------------------------------------------------------------------------------------------
  void tz1_shoot()
	{
//========================================================================================================
//_________________________________VARIABLES INITIALISE___________________________________________________
//========================================================================================================		
		set(PISTON_3);
		REMOVE_BRAKE;
		next=0;
		double heading_1=0,distance=0,factor=0;
	  int base_value=0,counter_v=0;//checkpost=0,x_rec=0,y_rec=0;
		in_tz1=1;
//___________________________________________ENCODER SETTING ____________________________________________
	  x_dis=0;
   	y_dis=10;  //20
	  prvvaluel=0;
	  prvvaluer=0;
	  T0TC=0;
	  T1TC=0;
		enc_diff_1_2=1;
//_______________________________________________________________________________________________________
		icontrol=60;
		factor = sqrt((double)(X_TZ1_CTRM)*(X_TZ1_CTRM) + (double)(Y_TZ1_CTRM)*(Y_TZ1_CTRM));	
//========================================================================================================
//_________________________________GO FOR SHOOTING(DED REKON)PREFOMING CTRM_______________________________
//========================================================================================================		
		while(!next)
		{
		 pos();
//---------------------------------------------------	
		 heading_1 = atan2((Y_TZ1_CTRM-y_dis),(X_TZ1_CTRM-x_dis));
		 heading_1*=(-1);
		 heading_1 += 1.57;
		 heading_1 -= ang*Degree_to_Rad;
//---------------------------------------------------
        
			if (y_dis>30)//(distance < 0.9*factor)
			{
	 		   hold_angle = ZONE_1_ANGLE*(1 - distance/factor);  
         hold_angle*=2;
				 if (ang>hold_angle)
				  {
            hold_angle=ang;
				  }
			}  
      if (ang>ZONE_1_ANGLE)
			{
				hold_angle = ZONE_1_ANGLE;
				next=1;
			}
			
			distance = sqrt(((X_TZ1_CTRM-x_dis)*(X_TZ1_CTRM-x_dis))+((Y_TZ1_CTRM-y_dis)*(Y_TZ1_CTRM-y_dis)));
			control_pid_omni(heading_1,18);
			
		}
		hold_angle = ZONE_1_ANGLE;
		icontrol=6;
	
		
		zone = ZONE_ONE_START;
		REMOVE_BRAKE;
		test_shoot();
			
//========================================================================================================
//_________________________________RETURN TO THE INITIAL CO-ORDINATES_____________________________________
//========================================================================================================
   base_value=18;    //18
	 icontrol=13;	
		while(1)
	{
		pos();
		control_pid_omni(135*Degree_to_Rad,base_value);
		hold_angle-=0.40;//43    //0.05
		if(ang<20)
		{
			icontrol=8;
			base_value=10;
		}
		if (ang<0)
			break;
	}
  hold_angle=0;










//=========================================================================================================
		next=0;
		icontrol=60;
		distance=0;
		bool local_flag=0;
	  double temp_enc=0,diff_enc=0;
		factor = sqrt((double)(X_TZ1_CTRM)*(X_TZ1_CTRM) + (double)(Y_TZ1_CTRM)*(Y_TZ1_CTRM));	
		while(!next)
		{
		 sensor_val_update_h();	
		 pos();
		 diff_enc = y_dis - temp_enc;
//---------------------------------------------------	
		 heading_1 = atan2((Y_TZ1_2_CTRM-y_dis),(X_TZ1_2_CTRM-x_dis));
		 heading_1*=(-1);
		 heading_1 += 1.57;
		 heading_1 -= ang*Degree_to_Rad;
//---------------------------------------------------
 /*
			 hold_angle-=0.037;
			 if (ang<hold_angle)
			 {
					hold_angle = ang;
			 }
      if (hold_angle<0)
			{
				hold_angle = 0;
			}				
      if (ang<0)
			{
				hold_angle = 0;
				icontrol=8;
				//next=1;
			}
			if(hold_angle>20)
			{
				base_value=15;
			}
			else 
			{
				base_value=11;
			}
			
			if ((hold_angle==0)||(ang==0))
			{
				base_value = 6;
			}
			
			
			*/
			base_value=7;  //10
			
			
			
			
			if(((y_dis<(40))&&(y_dis>30))&&((x_dis < 10)&&(x_dis > (-10)))&&(diff_enc>0))
			    next=1;
//-------------------UNECESSARY CONDITION---------------------------------------			
	
//--------------------------------------------------------------------------------------------------			
			distance = sqrt((Y_TZ1_CTRM-y_dis)*(Y_TZ1_CTRM-y_dis));//(((X_TZ1_CTRM-x_dis)*(X_TZ1_CTRM-x_dis))+((Y_TZ1_CTRM-y_dis)*(Y_TZ1_CTRM-y_dis)));
			control_pid_omni(heading_1,base_value);
			temp_enc = y_dis;
		}
//___________________________________________ENCODER SETTING ____________________________________________
	  x_dis=0;
   	y_dis=20;
	  prvvaluel=0;
	  prvvaluer=0;
	  T0TC=0;
	  T1TC=0;
		enc_diff_1_2=0;
//_______________________________________________________________________________________________________
		BRAKE;
		count_cycle1=0;
		while(count_cycle1<100)
		{
			control_pid_omni(4*PI,5);
		}
		REMOVE_BRAKE;
		
//====================================================================		
//													REMOVED BEACUSE OF BREAK		          
//===================================================================== 
    /*
    if (diff_enc > 0)
		{
			if (!NORMAL_BALL_PROXY)
				tz2_ball_proxy=1;
		 int i=400;
		 while(i--)
		 {
		   pos();
			 control_pid_omni(180*Degree_to_Rad,21);	 
		 }
		 i=500;
		 while(i--)
		 {
		   pos();
			 control_pid_omni(180*Degree_to_Rad,6);	 
		 }
		
		}
		else if (diff_enc < 0)
		{
			if (!NORMAL_BALL_PROXY)
				tz2_ball_proxy=1;
			int i=130;
			 while(i--)
		 {
		   pos();
			 control_pid_omni(180*Degree_to_Rad,21);	 
		 }
		 while(y_dis < (-30)){
			  pos();
			 control_pid_omni(180*Degree_to_Rad,6);	
		 }
		 cls();
		 lcd((char *)"down");
		
	  }
		*/
//==========================================================

//==========================================================	
		/*
		while(y_dis>10)
		{		  
		  pos();
			//---------------------------------------------------	
			heading_1 = atan2((Y_TZ1_2_CTRM-y_dis),(X_TZ1_2_CTRM-x_dis));
			heading_1*=(-1);
			heading_1 += 1.57;
			heading_1 -= ang*Degree_to_Rad;
			base_value=11;
			control_pid_omni(heading_1,base_value);
		}
		
		
		while(y_dis<0)
		{		  
		  pos();
			//---------------------------------------------------	
			heading_1 = atan2((Y_TZ1_2_CTRM-y_dis),(X_TZ1_2_CTRM-x_dis));
			heading_1*=(-1);
			heading_1 += 1.57;
			heading_1 -= ang*Degree_to_Rad;
			base_value=11;
			control_pid_omni(heading_1,base_value);
		}
	*/
	

	  
		base_value=15;
		
		while(0)//(y_dis>(-30))
		{		  
		  pos();
			//---------------------------------------------------	
			heading_1 = atan2((((-1)*100)-y_dis),(0-x_dis));
			heading_1*=(-1);
			heading_1 += 1.57;
			heading_1 -= ang*Degree_to_Rad;
			control_pid_omni(heading_1,6);
		}
	}

 //-------------------------------------------------------------------------------------------------------
 //===========================FUNCTION ENDS===============================================================
 
//=--------------------------------------------------------------------------------------------------------	
//=========================================================================================================	
//                                      TZ2_FAST_3
//=========================================================================================================	
	void tz2_fast_3()
	{	
//--------------------------------VARIABLES INITIALISE ----------------------------------------------------------		
	  ang=0;
	  double heading_1=0,distance=0,factor=0;
	  int base_value=0,checkpost=0,x_rec=0,y_rec=0,i=0;
	  enc_diff_1_2=1;
	  int counterv_90=0;
  	set(PISTON_3);
		CLOSE_GRIPPER;
		next=0;
		count_cycle1=0;
		sum_sensor_h=0;
		sum_sensor_v=0;
		
		x_dis=0;
	  y_dis=0;              ///0
		T0TC=0;
	  T1TC=0;
		prvvaluel=0;
	  prvvaluer=0;
		count_cycle=0;
		
//-----------------------------------VERTICALLY GO TOWARDS TZ2--------------------------------------------------------		 
		while(!next)
		{
		 
			if(count_cycle1 > 85)
			{
				set(PISTON_3);
			}
			else if (count_cycle1 > 60)
			{
				reset(PISTON_3);
			}
			if(x_dis>-70)  //
				base_value=15;
			else
			  base_value=35;
			pos();
			sensor_val_update_h();
			sensor_val_update_v();
			
			if(count_cycle1<270)                 //
				control_pid_omni((-110)*Degree_to_Rad,base_value);      //105          //23
			else
				control_pid_omni((-110)*Degree_to_Rad,3);
			
			if((sum_sensor_h>3 || sum_sensor_v>0) && count_cycle1>500)                               // backup 
				next=1;
			
			if (x_dis<-530)
				next=1;
		}
		i=1600;	
		next=0;
		count_cycle=0;
		icontrol=60;
		count_cycle=0;
		kp=1.2;
		bool actuate_now=0;
//----------------------------------TURN THE BOT TO THE SPECIFIC ANGLE------------------------------------------------------		
		while(!next)
		{
			pos();
			
			pos();
			sensor_val_update_h();
			sensor_val_update_v();
			if (hold_angle<35)
				hold_angle+=0.90;
			else
			  hold_angle+=0.90;
			if(hold_angle>79)				//75
			{
				if(restart_2==0)
				{
				hold_angle=81;
				}
				else 
				{
					hold_angle=79;
				}
        next=1;				
			}
			if(count_cycle1<320)
				control_pid_omni((-37)*Degree_to_Rad,25);      //96//16   // 105              // 13
			else
			 	control_pid_omni((-37)*Degree_to_Rad,20); //96//10  //105                  // 4
		}
	
		 hold_angle=80;	
		 icontrol=7;
		 factor = sqrt((double)(X1_TZ2_2 - x_dis)*(X1_TZ2_2 - x_dis) + (double)(Y1_TZ2_2 - y_dis)*(Y1_TZ2_2 - y_dis));
		 count_cycle1=0;
		 next=0;
		 actuate_now=0;
		 kp=0.8;
		 bool light=0;
//----------------------------TURNING OVER NOW DED REKON AND SHOOT ON TIME BASIS -------------------------------------------		 	 
		 while(!next) 
		{
			/*
			if ((count_cycle1>350)&&(actuate_now==0))
			{
				count_cycle1=0;
				actuate_now=1;
				reset(PISTON_3);
			}
			
			if((actuate_now==1)&&(count_cycle1>30))
			{
				set(PISTON_2);
			}
			*/
			distance = sqrt(((X1_TZ2_2-x_dis)*(X1_TZ2_2-x_dis))+((Y1_TZ2_2-y_dis)*(Y1_TZ2_2-y_dis)));
			pos();
//*********************************************************			
			base_value = 10*exp((-2)*(((factor-distance)/factor)));         // 2        // 30 101
      if (base_value<8)
			 {
				 base_value=6;
			 }					
//*********************************************************		
			  heading_1 = atan2((Y1_TZ2_2-y_dis),(X1_TZ2_2-x_dis));
				heading_1*=(-1);
			  heading_1 += 1.57;
				heading_1 -= ang*Degree_to_Rad;
//*********************************************************	
        if(count_cycle1>295)//(distance<0.9*factor)	//255
				{
					reset(PISTON_3);
				}
				if ((count_cycle1>265)&&(light==0))
				{
					 ExtPrintbin(COM3,1,170);
					light=1;
				}
				if (count_cycle1>320)//(y_dis< Y1_TZ2_2 + 10)
				{
					set(PISTON_1);
				}
				if (count_cycle1 > 390)
				{
					next=1;
				}
			  control_pid_omni(heading_1,10);
			  if (((x_dis<X1_TZ2_2+5)&&(x_dis>X1_TZ2_2-5))&&((y_dis<Y1_TZ2_2+5)&&(y_dis>Y1_TZ2_2-5)))//(y_dis<Y_TZ1_2+5)//((x_dis>X_TZ1_2+5)&&(y_dis<Y_TZ1_2+5))
			   {
				   next=1;
		 	   }
		}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++		
//=================================GO TO THE POSITION FOR GOLDEN LOAD(DED REKON)=================================================================================	
		reset(PISTON_1);	
		next=0;
		icontrol=40;
		distance=0;
		double diff_enc=0,temp_enc=0;
		bool local_flag=0;
		int proxy_pass=0;
		count_cycle1=0;
		factor = sqrt((double)(X_TZ1_CTRM)*(X_TZ1_CTRM) + (double)(Y_TZ1_CTRM)*(Y_TZ1_CTRM));	
		while(!next)
		{
		 sensor_val_update_h();	
		 pos();
		 diff_enc = y_dis - temp_enc;	
//---------------------------------------------------	
		 heading_1 = atan2((50-y_dis),(((-1)*1500)-x_dis));
		 heading_1*=(-1);
		 heading_1 += 1.57;
		 heading_1 -= ang*Degree_to_Rad;
//---------------------------------------------------			
		 hold_angle-=0.040;
		 if (ang<hold_angle)
			{
				hold_angle = ang;
			}
				
			
      if (hold_angle<0)
			{
				hold_angle = 0;
			}				
      if (ang<0)
			{
				hold_angle = 0;
				icontrol=8;
				//next=1;
			}
			if(hold_angle==0)
			{
				base_value=8;
			}
			else 
			{
				base_value=19;
			}
			
			if((!FORWARD_PROXY)&&(hold_angle==0)&&(count_cycle1>280))                  //(y_dis<35)
			    next=1;
			
			if(((y_dis<(49))&&(y_dis>30))&&(diff_enc>0))
			    //next=1;//(((y_dis<55)&&(y_dis>(-45)))&&((x_dis<(-1495))&&(x_dis>(-1505))))//((y_dis=)&&(x_dis==0))//(((y_dis<3)&&(y_dis>(-3)))&&((x_dis<3)&&(x_dis>(-3))))
			
				
				
				
				/*
				int i=0;
				if ((heading_1<0.785)&&(heading_1>(-0.785)))
				{
					if (heading_1>0)
				 	 {
						 heading_1 = -0.75-heading_1;
					 }
					else
					 {
						 heading_1 = 0.75+ ((-1)*heading_1);
					 }
				}
				else if ((heading_1>0.785)||(heading_1<(-0.785)))
				{
					if (heading_1>0)
					 {
						 heading_1 = heading_1-0.785;
						 heading_1*=(-1);
					 }
					else
					 { 
						  heading_1 = heading_1 + 0.785;
						  heading_1 *= (-1);
				   }
				}
				while(i--)
				{
					control_pid_omni(0,20);
				}
				count_cycle1=0;
				local_flag=1;
				count_cycle1=0;
			//	next=1;
				
				
				*/
			
			
			if ((local_flag==1)&&(count_cycle>80))
			{
			//	next=1;
			}
			
			distance = sqrt((Y_TZ1_CTRM-y_dis)*(Y_TZ1_CTRM-y_dis));//(((X_TZ1_CTRM-x_dis)*(X_TZ1_CTRM-x_dis))+((Y_TZ1_CTRM-y_dis)*(Y_TZ1_CTRM-y_dis)));
			control_pid_omni(heading_1,base_value);
			temp_enc = y_dis;
		}
		
		
		
		
		
		
		
		
		x_dis=0;
	  y_dis=0;
		T0TC=0;
	  T1TC=0;
		prvvaluel=0;
	  prvvaluer=0;  
		/*
		while(!next)
		{
		 sensor_val_update_h();	
		 pos();
		 diff_enc = y_dis - temp_enc;
//---------------------------------------------------	
		 heading_1 = atan2(((-210)-y_dis),(0-x_dis));
		 heading_1*=(-1);
		 heading_1 += 1.57;
		 heading_1 -= ang*Degree_to_Rad;
//---------------------------------------------------
 
			base_value=8;
	    cls();
			lcd(y_dis);
			
			if (y_dis < (-210))
			{
				next=1;
			}
			
			//if(((y_dis<(40))&&(y_dis>30))&&((x_dis < 10)&&(x_dis > (-10)))&&(diff_enc>0))
			   // next=1;
//-------------------UNECESSARY CONDITION---------------------------------------			
	
//--------------------------------------------------------------------------------------------------			
			distance = sqrt((Y_TZ1_CTRM-y_dis)*(Y_TZ1_CTRM-y_dis));//(((X_TZ1_CTRM-x_dis)*(X_TZ1_CTRM-x_dis))+((Y_TZ1_CTRM-y_dis)*(Y_TZ1_CTRM-y_dis)));
			control_pid_omni(heading_1,base_value);
			temp_enc = y_dis;
		}
		*/
		
		i=1000;
		 reset(PISTON_2);
		if (!golden_proxy)
		{			
		while(i--)
		{
			pos();
			control_pid_omni(0,20);
		}
	
		x_dis=0;
	  y_dis=0;
		T0TC=0;
	  T1TC=0;
		prvvaluel=0;
	  prvvaluer=0;  
		
		
		while(y_dis > (-66))
		{
			pos();
			control_pid_omni(180*Degree_to_Rad,6);
		}
		x_dis=0;
	  y_dis=0;
		T0TC=0;
	  T1TC=0;
		prvvaluel=0;
	  prvvaluer=0; 
		while(y_dis < 0)
		{
			pos();
			control_pid_omni(0*Degree_to_Rad,5);
		}
		i=2300;
		
		x_dis=0;
	  y_dis=0;
		T0TC=0;
	  T1TC=0;
		prvvaluel=0;
	  prvvaluer=0; 
		enc_diff_1_2 =0 ;
		while(x_dis < 160)
		{
			pos();
			control_pid_omni(90*Degree_to_Rad,5);
		}
		
	} 	
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
	//if condition arises to revert back	
		 /*
		
    	while(y_dis < 35)
		{		  
		  pos();
			//---------------------------------------------------	
			heading_1 = atan2((50-y_dis),(((-1)*1500)-x_dis));
			heading_1*=(-1);
			heading_1 += 1.57;
			heading_1 -= ang*Degree_to_Rad;
			control_pid_omni(heading_1,6);
		}
		
		
		while(FORWARD_PROXY)
		{
			control_pid_omni(0,5);
		}
		
		x_dis=0;
	  y_dis=0;
		T0TC=0;
	  T1TC=0;
		prvvaluel=0;
	  prvvaluer=0;
		
		while(y_dis > (-80))
		{
			pos();
			control_pid_omni(180*Degree_to_Rad,6);
		}
		
		x_dis=0;
	  y_dis=0;
		T0TC=0;
	  T1TC=0;
		prvvaluel=0;
	  prvvaluer=0;
		
		
		
		while(x_dis < 10)
		{
			pos();
			control_pid_omni(90*Degree_to_Rad,6);
		}
	   
		*/
		
		
		
	
	
   /* 

//========================================================================================================================================
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++		
//----------------------------------------------------------------------------------------------------------------------------------------	 
//_---------------------------------------R E T U R N -------------------------------------------------------------------------------------	 
//------------------------------------------------------------------------------------------------------------------------------------------
	 //______________--------------restore kp ------------___________________	
	kp=0.95;
	
//________________________encoder initialise____________________________	
	x_dis=0;
	T0TC=0;
	prvvaluel=0;
//______________________--local variables--______________________________
	int counterh_90=0,i2=0,local_time=0;	
	//hold_angle=ang=90;
	
//_______________________________________________________________________	
	
	
	sum_sensor_v=0;
	sum_sensor_h=0;
	icontrol=10;
	hold_angle=0;
	while(ang>0)
	{
		control_pid_omni(4*PI,5);
	}
	reset(PISTON_2);
	
	
	sum_sensor_v=0;
	sum_sensor_h=0;	
	
	while(sum_sensor_v<4)//(sum_sensor_v<2 && sum_sensor_h<1)
	{
		cls();
		lcd((char *)"in");
		sensor_val_update_v();
	//	sensor_val_update_h();
		control_pid_omni((-130*Degree_to_Rad),7);
	}
	
	while(sum_sensor_h<1)//(sum_sensor_v<2 && sum_sensor_h<1)
	{
		cls();
		lcd((char *)"in");
		sensor_val_update_v();
		sensor_val_update_h();
		control_pid_omni((-90*Degree_to_Rad),4);
	}
	
	 i=1000;           // 1500
	while(i--)                                                     // responsible for making the bot stick to line 
	{                                                              // the first time it catches the line(without overshoot)  
		control_pid_omni(90*Degree_to_Rad,25); //-230
	}
	

	
	cls();
	
	kp_line_sense=8;
	kd_line_sense=2.5;
	
  kp=0.9;
	
	next=0;
	sum_sensor_h=0;
//_______________________ENCODER SETTING________________________________
  		x_dis=0;
	    y_dis=0;
		  T0TC=0;
	    T1TC=0;
		  prvvaluel=0;
	    prvvaluer=0;
      enc_diff_1_2=0;
//_______________________________________________________________________
	
	while(!next)                                               // line follower in backward direction
	{
		
		pos();
		error_h=sensor_val_update_h();
	//	if (counterh_90<=1)
		control_pid_line_follow(BACKWARD,8,error_h);
		//else if ((counterh_90>=1)&&(counterh_90<4))
		//	control_pid_line_follow(BACKWARD,23,error_h);            //25
		//else if (counterh_90==4)
		//	control_pid_line_follow(BACKWARD,4,error_h);             //5

		if(sum_sensor_h==4)
		{
			next=0;
			while(1)
			{
				sensor_val_update_h();
			//	error_h=sensor_val_update_h();
		   // control_pid_line_follow(BACKWARD,5,error_h);
				control_pid_omni(180*Degree_to_Rad,4);
				if(sum_sensor_h<=2)
				{
					counterh_90++;
					break;
				}
			}
//_______________________ENCODER SETTING________________________________
  		x_dis=0;
	    y_dis=0;
		  T0TC=0;
	    T1TC=0;
		  prvvaluel=0;
	    prvvaluer=0;
      enc_diff_1_2=0;
//_______________________________________________________________________
	
			sum_sensor_h=0;
		}
		
		//______________________________________________BACKUP FOR UNSTABLE CONDITION________(LINE NOT FOLLOWED)___________________________________________________________		

//_________________________________________________________________________________________________________________________________________________		
		
		if (((!FORWARD_PROXY)&&(counterh_90>=1))||((!FORWARD_PROXY)&&(y_dis < (-400))))
		 next=1;
			
	}
	

	
	cls();
	i=2000;
	while(i--)                                                           // reverse braking for proxy 
	{
    control_pid_omni(0*Degree_to_Rad,10);
	}
	while(FORWARD_PROXY)
	{
    error_h=sensor_val_update_h();
		control_pid_line_follow(FORWARD,5,error_h);    
	}
	
	kp_line_sense=8;                                                    // restore kp and kd (of line follower) to its original value
	kd_line_sense=2.5;
//==================================================================================================
//------------------------------ BRINGING AUTO AT GOLDEN LOAD POSITION------------------------------	
//_______________________ENCODER SETTING________________________________
  		x_dis=0;
	    y_dis=0;
		  T0TC=0;
	    T1TC=0;
		  prvvaluel=0;
	    prvvaluer=0;
      enc_diff_1_2=0;
//_______________________________________________________________________
      next=0;
    	while(!next)
			{
				pos();
//*********************************************************		        
			  heading_1 = atan2((0-y_dis),(350-x_dis));
				heading_1*=(-1);
			  heading_1 += 1.57;
//*********************************************************	
        control_pid_omni(heading_1,6);
        if (x_dis>290)
				{
					next=1;
				}
			}
	
//====================================================================================================	
//	zone_complete=1; 
	 
	 
	 
*/
		
		
		zone_complete=1;	
	}
	
	
	
	
//-------------------------------------------------------------------------------------------------------
 //===========================FUNCTION ENDS===============================================================	
	
void tz3_fast_4()
{
	 CLOSE_GRIPPER;
	 zone = ZONE_THREE_SHOOT;
	 enc_diff_1_2=1;
//==================take the encoder ticks the simple way=============
	  int i=0;
 //___________________________________________ENCODER SETTING ____________________________________________
	  x_dis=0;
   	y_dis=0;
	  prvvaluel=0;
	  prvvaluer=0;
	  T0TC=0;
	  T1TC=0;
	  enc_diff_1_2=1;
//____________________________________________VARIABLES INITIALISE_______________________________________________________________________________	
	  next=0;
    icontrol=10;	
    enc_diff_1_2=1;		
	  bool acc=0,turned=0;
	  double heading_1=0,distance=0,factor=0;
	  int base_value=38,counter_v=0,gripper_set=0;//checkpost=0,x_rec=0,y_rec=0;
	  factor = sqrt((double)(X_TZ3_3_2)*(X_TZ3_3_2) + (double)(Y_TZ3_3)*(Y_TZ3_3));
    icontrol = 60;
    sum_sensor_v=0;	
//===============================================================================================================================================		
//_______________________________________GO TOWARDS FIRST JUNCTION USING CTRM WITHOUT FEEDBACK____________________________________________________
//===============================================================================================================================================
    count_cycle1=0;
		while(y_dis < 200)   //300
		{
			pos();
			/*
		  if ((count_cycle1 > 25)&&(gripper_set==0))
			{
				 reset(PISTON_3);
				gripper_set=1;
			}
			if ((count_cycle1 > 45)&&(gripper_set==1))
			{
				CLOSE_GRIPPER;
				gripper_set=2;
			}
			*/
			if (y_dis < 80)
			control_pid_omni(0*Degree_to_Rad,20);
			else 
			control_pid_omni(0*Degree_to_Rad,40);	
			
		}
	
		bool non_return=0;
		double temp_tick=0,diff_tick=0;
		while(!next) 
		{
			  pos();
			  diff_tick = abs(x_dis) - temp_tick;
			  sensor_val_update_v();
			  sensor_val_update_h();
			  heading_1 = hold_angle*Degree_to_Rad;
			  heading_1 *= (-1);
			  if (ang < 30)
				{
			  heading_1+=(8*Degree_to_Rad);    //6
				}
				else if(ang >= 30)
				{
					heading_1+=(8*Degree_to_Rad);    //6
				}
			  hold_angle += 0.0199;//0.0190;
				
/*				
				if (distance > 0.1*factor)
				 {
	 		       hold_angle = ZONE_2_ANGLE*(1 - distance/factor);  
             hold_angle*=5.2;
				     if (ang>hold_angle)
				      {
                hold_angle=ang;
				      }
					}
*/				
			
			  
			
			  if (ang>hold_angle)
				  {
             hold_angle=ang;
				  }
			
        if ((hold_angle>88)||(ang>88))
				 {
					 acc=1;
					 icontrol = 40;
					 hold_angle=88;//ZONE_2_ANGLE;
					 turned=1;
				 }
				 
			
				if ((sum_sensor_v>=2)&&(turned==1))
				 {
				 	next=1;       //previous
				 }
	      	
				if (turned==1)
				 {
										 
					 icontrol=40;
					 if(golden_counter==0)
					 {
					  base_value=5;  //9                                     // 5    same as below
					 }
					 else
					 {
						 base_value=5;    //10
					 }						 
					 
					 if(golden_counter==0)
					 {
					  heading_1 = (-54)*Degree_to_Rad;              // 64 change to this if the current one doesnt work					 //68
					 }
					 else 
					 {
						 heading_1 = (-44)*Degree_to_Rad; 
					 }
				 }
				 if ((turned==1)&&(non_return==0))
				 {
					 count_cycle1=0;
					 non_return=1;
				 }
				 if (non_return==1)
				 {
					 if (count_cycle1 > 95)
					 {
						 CLOSE_GRIPPER;
					 }
					 else if(count_cycle1 > 45)
					 {
						 reset(PISTON_3);
					 }
				 }
				 
				 if ((diff_tick == 0)&&(golden_counter>=1)&&(turned==1))
				 {
					 base_value=6;
				 }
				 
				 if (x_dis < (-800))
				 {
					// base_value=5;
				 }
				control_pid_omni(heading_1,base_value);
				temp_tick = abs(x_dis); 
		}
		hold_angle=88;
		
		
	//----------------	
	
	
		
//___________________________________________ENCODER SETTING ____________________________________________
	  x_dis=0;
   	y_dis=0;
	  prvvaluel=0;
	  prvvaluer=0;
	  T0TC=0;
	  T1TC=0;
		enc_diff_1_2=0;
//_____________________________________________		
		sum_sensor_v=0;
		
		BRAKE;
		count_cycle1=0;
		sum_sensor_h=0;
		bool offset=0;
		while(count_cycle1 < 90)
		{
			sensor_val_update_h();
			if(sum_sensor_h > 1)
			{
				offset=1;
			}
			control_pid_omni(4*PI,5);
		}
		REMOVE_BRAKE;
		if(offset==1)
		{
	  while(sum_sensor_v < 2)
		{
			sensor_val_update_v();
			control_pid_omni((70)*Degree_to_Rad,5);
		}
	}
		test_shoot();
		count_cycle1=0;
		while(count_cycle1<50)
		{
			control_pid_omni(4*PI,5);
		}
		
		

//=======================================================================
//												REMOVE IF BRAKES WORK
//========================================================================		
		/*
		i=2500;
		while(i--)
		{
			 pos();
			 control_pid_omni(78*Degree_to_Rad,28);				
		}
		sum_sensor_h=0;
		
		
	  while(sum_sensor_h < 2)
		{
			if (x_dis > 30)
			{
				break;
			}
		
		   pos();
			sensor_val_update_h();
			 control_pid_omni((70*Degree_to_Rad),6);				
		}
		
		
  zone = ZONE_THREE_SHOOT;
  test_shoot();
		
		*/
}	
//=======================================================================================================




void tz1_shoot_2()
{
	//========================================================================================================
//_________________________________VARIABLES INITIALISE___________________________________________________
//========================================================================================================		
		set(PISTON_3);
		next=0;
		double heading_1=0,distance=0,factor=0;
	  int base_value=0,counter_v=0;//checkpost=0,x_rec=0,y_rec=0;
		in_tz1=1;
//___________________________________________ENCODER SETTING ____________________________________________
	  x_dis=0;
   	y_dis=0;
	  prvvaluel=0;
	  prvvaluer=0;
	  T0TC=0;
	  T1TC=0;
		enc_diff_1_2=1;
//_______________________________________________________________________________________________________
		icontrol=60;
		factor = sqrt((double)(X_TZ1_CTRM)*(X_TZ1_CTRM) + (double)(Y_TZ1_CTRM)*(Y_TZ1_CTRM));	
//========================================================================================================
//_________________________________GO FOR SHOOTING(DED REKON)PREFOMING CTRM_______________________________
//========================================================================================================		
		while(!next)
		{
		 pos();
//---------------------------------------------------	
		 heading_1 = atan2((Y_TZ1_CTRM-y_dis),(X_TZ1_CTRM-x_dis));
		 heading_1*=(-1);
		 heading_1 += 1.57;
		 heading_1 -= ang*Degree_to_Rad;
//---------------------------------------------------
        
			if (y_dis>30)//(distance < 0.9*factor)
			{
	 		   hold_angle = ZONE_1_ANGLE*(1 - distance/factor);  
         hold_angle*=2;
				 if (ang>hold_angle)
				  {
            hold_angle=ang;
				  }
			}  
      if (ang>ZONE_1_ANGLE)
			{
				hold_angle = ZONE_1_ANGLE;
				next=1;
			}
			
			distance = sqrt(((X_TZ1_CTRM-x_dis)*(X_TZ1_CTRM-x_dis))+((Y_TZ1_CTRM-y_dis)*(Y_TZ1_CTRM-y_dis)));
			control_pid_omni(heading_1,18);
		}
		hold_angle = ZONE_1_ANGLE;
		icontrol=6;
	
		
		zone = ZONE_ONE_START;
		test_shoot();
			
//========================================================================================================
//_________________________________RETURN TO THE INITIAL CO-ORDINATES_____________________________________
//========================================================================================================
   
		hold_angle=0;
    next=0;
		icontrol = 40;
    while(!next)
		{
			pos();
			control_pid_omni(4*PI,5);
			if (ang < 60)
				icontrol = 6;
		  if(ang < 0)
        next=1;				
		}
    factor = sqrt((double)(X_TZ1_CTRM)*(X_TZ1_CTRM) + (double)(Y_TZ1_CTRM)*(Y_TZ1_CTRM));
    next=0;
    base_value=20;
    icontrol = 30;
		while(!next)
		{
		 pos();
//-----------------------------			
		 distance = sqrt(((X_TZ1_CTRM-x_dis)*(X_TZ1_CTRM-x_dis))+((Y_TZ1_CTRM-y_dis)*(Y_TZ1_CTRM-y_dis)));
     if (distance > factor){
		     distance = factor;}			 
	   heading_1 = atan2(((Y_TZ1_2_CTRM)-y_dis),(( X_TZ1_2_CTRM)-x_dis));
	   heading_1*=(-1);
//-------------------------------			
     base_value = 25*exp((-2)*(((factor-distance)/factor))); 		
     if (base_value < 4){
		     base_value=4;}
			
		 if (base_value > 25){
		     base_value=25;}		 
			
    control_pid_omni(heading_1,base_value);		
				 
		}
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
//===================================================================================================================		
		
		base_value=18;    //18
	 icontrol=13;	
		while(1)
	{
		pos();
		control_pid_omni(135*Degree_to_Rad,base_value);
		hold_angle-=0.40;//43    //0.05
		if(ang<20)
		{
			icontrol=8;
			base_value=10;
		}
		if (ang<0)
			break;
	}
  hold_angle=0;










//=========================================================================================================
		next=0;
		icontrol=60;
		distance=0;
		bool local_flag=0;
	  double temp_enc=0,diff_enc=0;
		factor = sqrt((double)(X_TZ1_CTRM)*(X_TZ1_CTRM) + (double)(Y_TZ1_CTRM)*(Y_TZ1_CTRM));	
		while(!next)
		{
		 sensor_val_update_h();	
		 pos();
		 diff_enc = y_dis - temp_enc;
//---------------------------------------------------	
		 heading_1 = atan2((Y_TZ1_2_CTRM-y_dis),(X_TZ1_2_CTRM-x_dis));
		 heading_1*=(-1);
		 heading_1 += 1.57;
		 heading_1 -= ang*Degree_to_Rad;
//---------------------------------------------------
 
			base_value=7;  //10
			
			
			
			
			if(((y_dis<(40))&&(y_dis>30))&&((x_dis < 10)&&(x_dis > (-10)))&&(diff_enc>0))
			    next=1;
//-------------------UNECESSARY CONDITION---------------------------------------			
	
//--------------------------------------------------------------------------------------------------			
			distance = sqrt((Y_TZ1_CTRM-y_dis)*(Y_TZ1_CTRM-y_dis));//(((X_TZ1_CTRM-x_dis)*(X_TZ1_CTRM-x_dis))+((Y_TZ1_CTRM-y_dis)*(Y_TZ1_CTRM-y_dis)));
			control_pid_omni(heading_1,base_value);
			temp_enc = y_dis;
		}
   
    
    if (diff_enc > 0)
		{
			if (!NORMAL_BALL_PROXY)
				tz2_ball_proxy=1;
		 int i=400;
		 while(i--)
		 {
		   pos();
			 control_pid_omni(180*Degree_to_Rad,21);	 
		 }
		 i=1300;
		 while(i--)
		 {
		   pos();
			 control_pid_omni(180*Degree_to_Rad,6);	 
		 }
		
		}
		else if (diff_enc < 0)
		{
			if (!NORMAL_BALL_PROXY)
				tz2_ball_proxy=1;
			int i=130;
			 while(i--)
		 {
		   pos();
			 control_pid_omni(180*Degree_to_Rad,21);	 
		 }
		 while(y_dis < (-30)){
			  pos();
			 control_pid_omni(180*Degree_to_Rad,6);	
		 }
		 cls();
		 lcd((char *)"down");
		
	  }
	
	

	  
		base_value=15;
		
		while(0)//(y_dis>(-30))
		{		  
		  pos();
			//---------------------------------------------------	
			heading_1 = atan2((((-1)*100)-y_dis),(0-x_dis));
			heading_1*=(-1);
			heading_1 += 1.57;
			heading_1 -= ang*Degree_to_Rad;
			control_pid_omni(heading_1,6);
		}
}

//---------------------------------------------------------------

void tz3_fast_5()
{ 
	  next=0;
		icontrol=40;
	  hold_angle=90;
    while(!next)
		{		
			control_pid_omni(4*PI,5);
			if (ang > 30)
				icontrol=5;
			if (ang > 89)
				next=1;
		}//
    int i=1500;
    while(i--){
      control_pid_omni((-100)*Degree_to_Rad,35);	}		
		icontrol=40;
	  x_dis=0;
   	y_dis=0;
	  prvvaluel=0;
	  prvvaluer=0;
	  T0TC=0;
	  T1TC=0;
	  enc_diff_1_2=0; 
		next=0;
		double distance=0;
		int base_value=40;
		double factor = sqrt((double)(4200)*(4200) + (double)(0)*(0));
		while(!next)
		{
			 sensor_val_update_v();
			 pos();
			 if (x_dis > (-200)){
					heading_1 = (-90)*Degree_to_Rad;}
			 else {
				  heading_1 = (-87)*Degree_to_Rad;}
		   distance = sqrt(((4200+x_dis)*(4200+x_dis))+((0-y_dis)*(0-y_dis))); 
		 	 base_value = 40*exp((-2)*(((factor-distance)/factor)));  
			 if (distance > factor){
				 distance = factor;}
			 if (base_value > 40){
				 base_value=40;}
			 if (base_value < 4){
				 base_value = 4;}
			 if((x_dis < (-2500))&&(sum_sensor_v > 2)){
				 next=1;}
			 if (x_dis > (-2200))
			 {
				 base_value=40;
			 }
			 else if (x_dis > (-2700))
			 {
				 base_value=6;
		 	 }
							 
		
				 
			 control_pid_omni(heading_1,base_value);
		}
	   
		
	     i=2500;
		while(i--)
		{
			 control_pid_omni(85*Degree_to_Rad,28);				
		}
			sum_sensor_v=0;
		
		
	  while(sum_sensor_v < 2)
		{
			pos();
			sensor_val_update_v();
			 control_pid_omni((80*Degree_to_Rad),4);				
		}
		
		
		test_shoot();
	

while(1)
{
	control_pid_omni(4*PI,5);
}
	
	
}

//==========================================================================================

void tz3_fast_6()
{
	enc_diff_1_2=1;
	 zone  = ZONE_THREE_SHOOT;
//==================take the encoder ticks the simple way=============
	  int i=0;
 //___________________________________________ENCODER SETTING ____________________________________________
	  x_dis=0;
   	y_dis=0;
	  prvvaluel=0;
	  prvvaluer=0;
	  T0TC=0;
	  T1TC=0;
//____________________________________________VARIABLES INITIALISE_______________________________________________________________________________	
	  next=0;
    icontrol=10;	
    enc_diff_1_2=1;		
	  bool acc=0,turned=0;
	  double heading_1=0,distance=0,factor=0;
	  int base_value=25,counter_v=0;//checkpost=0,x_rec=0,y_rec=0;
	  factor = sqrt((double)(X_TZ3_3_2)*(X_TZ3_3_2) + (double)(Y_TZ3_3)*(Y_TZ3_3));
    icontrol = 60;
    sum_sensor_h=0;	
//===============================================================================================================================================		
//_______________________________________GO TOWARDS FIRST JUNCTION USING CTRM WITHOUT FEEDBACK____________________________________________________
//===============================================================================================================================================
    while(!next) 
		{
			  sensor_val_update_v();
			  heading_1 = hold_angle*Degree_to_Rad;
			  heading_1 *= (-1);
			  heading_1+=(-1)*(2*Degree_to_Rad);
			  hold_angle += 0.015;
			 
/*				
				if (distance > 0.1*factor)
				 {
	 		       hold_angle = ZONE_2_ANGLE*(1 - distance/factor);  
             hold_angle*=5.2;
				     if (ang>hold_angle)
				      {
                hold_angle=ang;
				      }
					}
*/				
			
			  
			
			  if (ang>hold_angle)
				  {
             hold_angle=ang;
				  }
			
        if ((hold_angle>90)||(ang>90))
				 {
					 acc=1;
					 icontrol = 40;
					 hold_angle=90;//ZONE_2_ANGLE;
					 turned=1;
				 }
			
				if ((sum_sensor_v>=2)&&(turned==1))
				 {
				 	 next=1;
				 }
	         
				if (turned==1)
				 {
					 icontrol=40;
					 base_value=50;
					 heading_1 = (-90)*Degree_to_Rad;
				 }
				control_pid_omni(heading_1,base_value);
		}
		hold_angle=90;
//=================================================================================================================================================		
//______________________________________GO TOWARDS FINAL POINT WITH DECCELERATION WITH ENCODER FEEDBACK___________________________________________		
//================================================================================================================================================= 
//___________________________________________ENCODER SETTING ____________________________________________
	  x_dis=0;
   	y_dis=0;
	  prvvaluel=0;
	  prvvaluer=0;
	  T0TC=0;
	  T1TC=0;
		enc_diff_1_2=0;
//_______________________________________________________________________________________________________
			
	next=0;
	sum_sensor_h=0;
	bool reverse_over=0;	
  factor = sqrt((double)(X1_TZ3)*(X1_TZ3) + (double)(Y1_TZ3)*(Y1_TZ3));	
  double p_x_dis=0,p_y_dis=0,velocity_x=0,velocity_y=0;
  double diff_tick_enc=0,diff_max=0,target_speed=55,temp_tick_enc=0,factor_enc=0;
  base_value = 40;
	heading_1 = (-90)*Degree_to_Rad;
	int non_return_tz3=0;
  count_cycle2=0;
  bool gripper_set=0;
	
  while(!next)
	{  
		 pos();
		 diff_tick_enc = ((-1)*x_dis) - temp_tick_enc;
		 if ((count_cycle2 > 20)&&(gripper_set==0))
		 {
			reset(PISTON_3);
      gripper_set=1;			 
		 }
		 else if ((count_cycle2 > 50)&&(gripper_set==1))
		 {
			 CLOSE_GRIPPER;
		 }
		 sensor_val_update_h();
		
		 
//==================================================	
/*    
		 if (x_dis < (-2500))	
		 {
			 target_speed = 40;
		 }			 
	*/	
/*		 
//---------------------------------------------------	
		 heading_1 = atan2((Y1_TZ3-y_dis),(X1_TZ3-x_dis));
		 heading_1*=(-1);
		 heading_1 += 1.57;
		 //heading_1 -= ang*Degree_to_Rad;
//---------------------------------------------------	
		 */
     /*
		 base_value = 40*exp((-3.1)*(((-1)*x_dis)/3200));       // base_value = 25*exp((-2)*(((factor-distance)/factor)));         // 2      // 30      
     distance = sqrt(((X1_TZ3-x_dis)*(X1_TZ3-x_dis))+((Y1_TZ3-y_dis)*(Y1_TZ3-y_dis)));
		 if (base_value<10)
			 base_value=10;
		 if (base_value>30)
			 base_value=30;
		 
		 if (base_value>20)
		 {
			 icontrol=30;
			 heading_1 = (-90)*Degree_to_Rad;
		 }
		 else 
		 {
			  icontrol=20;
			  heading_1 = (-90)*Degree_to_Rad;
		 }
		 */
		

		 
		
		 
	//=============================================================	 
		
		 
		 if ((x_dis < (-920))&&((non_return_tz3==0)||(non_return_tz3==1)))
		 {
			 base_value=28;
			 heading_1 = 86*Degree_to_Rad;
			 non_return_tz3=1;
		 }
		 if ((non_return_tz3==1)&&(diff_tick_enc <= 0))
		 {
			 base_value=8;
			 heading_1 = (-89)*Degree_to_Rad;
			 non_return_tz3=2;
		 }
		 
		 if (y_dis > 5)
		 {
			 if (heading_1 > 0)
			 {
				 heading_1 += 0.12;
			 }
			 else if (heading_2 < 0)
			 {
				  heading_1 -= 0.12;
			 }
		 }
		 
		  if (y_dis < (-20))
		 {
			 if (heading_1 > 0)
			 {
				 heading_1 -= 0.12;
			 }
			 else if (heading_2 < 0)
			 {
				  heading_1 += 0.12;
			 }
		 }
		 
		 if (x_dis < (-2950))
		 {
			 icontrol=10;
			 base_value=3;
		 }
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 /*
		 if (reverse_over==1)
		 {
			   heading_1 = (-80)*Degree_to_Rad;
			   base_value = 6; 
		 }
*/
	

		 
		 control_pid_omni(heading_1,base_value); 
		if (x_dis<(-2300))//(distance<0.25*factor)
		 {
			 if (sum_sensor_h>=2)
			 {
				 kp=0.9;
				 next=1;
			 }
		 }
		 p_x_dis = x_dis;
     p_y_dis = y_dis;		
     temp_tick_enc = ((-1)*x_dis);		 
	}
//=============================================================================================================================================	
//                     SNIPPET FOR COMPENSATING THE OVERSHOOT AND BRINGING THE BOT TO FINAL POSITION
//=============================================================================================================================================
		i=2500;
	  if (1)
		{
		while(i--)
		{
			 pos();
			 control_pid_omni(81*Degree_to_Rad,28);				
		}
	}
		sum_sensor_h=0;
		
//___________________________________________ENCODER SETTING ____________________________________________
	  x_dis=0;
   	y_dis=0;
	  prvvaluel=0;
	  prvvaluer=0;
	  T0TC=0;
	  T1TC=0;
		enc_diff_1_2=0;
		kp=0.9;
//_____________________________________________
   while(sum_sensor_h < 2)
 {
	 sensor_val_update_h();
	 control_pid_omni((88)*Degree_to_Rad,5);
 }	 
  
//________________________use only if bot is to be stopped on sensor patti______________________________________	 
		next=1;
		while(!next)
		{
			sensor_val_update_h();
			control_pid_omni(80*Degree_to_Rad,6);
			if (sum_sensor_h>=2)
				next=1;
		}

		/*
		while(x_dis<70)
		{
			pos();
			if (x_dis<50)
			 control_pid_omni(75*Degree_to_Rad,8);
			else 
				 control_pid_omni(75*Degree_to_Rad,4);
		}
		*/
		
		next=1;
		while(!next)
		{
			if (!FORWARD_PROXY)
				next=1;
			control_pid_omni((-1)*(70*Degree_to_Rad),8);
		}
		zone = ZONE_THREE_SHOOT;
		test_shoot();
}
///===================================================================================================================

//        																	TZ3 RETURN FUNCTION                                     
  

//=====================================================================================================================

void tz3_fast_return()
{
	double heading_1 = (70)*Degree_to_Rad,base_value=30;
	next=0;
	x_dis=0;
  y_dis=0;
	prvvaluel=0;
	prvvaluer=0;
	T0TC=0;
	T1TC=0;
  enc_diff_1_2=0;
  kp=0.9;
	while(!next)
	{
		pos();
		control_pid_omni(heading_1,base_value);
		if(x_dis > (2200))
		{
			heading_1 = (70)*Degree_to_Rad;
			base_value = 6;
		}
		if (x_dis > (3500))
		{
			next=1;
		}
	}
	int i=1200;
	while(!next)
	{
		control_pid_omni((-80)*Degree_to_Rad,20);
	}
	next=0;
	hold_angle=0;
	icontrol = 20;
  while(!next)
	 {
			pos();
			control_pid_omni(4*PI,5);
			if (ang < 60)
				icontrol = 6;
		  if(ang < 0)
        next=1;				
	 }
  next=0;
	 	x_dis=0;
  y_dis=0;
	prvvaluel=0;
	prvvaluer=0;
	T0TC=0;
	T1TC=0;
  enc_diff_1_2=0;
 while(!next)
 {
	 pos();
	 if((!FORWARD_PROXY)&&(y_dis<(-100)))
	 {
		 next=1;
	 }
	 if(y_dis < (-1300))
	 {
		 next=1;
	 }
	 control_pid_omni(177*Degree_to_Rad,8);
 }

 next=0;
 x_dis=0;
 y_dis=0;
 prvvaluel=0;
 prvvaluer=0;
 T0TC=0;
 T1TC=0;
 enc_diff_1_2=0; 
 if(golden_counter==0)
 {
 while(y_dis > (-98))
 {
	  pos();
	  control_pid_omni(180*Degree_to_Rad,6);
 }
 }
 else if(golden_counter==1)
 {
	  while(y_dis > (-78))
 {
	  pos();
	  control_pid_omni(180*Degree_to_Rad,6);
 }
 }
 else  
 {
	 while(y_dis > (-88))
 {
	  pos();
	  control_pid_omni(180*Degree_to_Rad,6);
 }
 }
 BRAKE;
 i=2000;
 while(i--)
 {
	 control_pid_omni(4*PI,5);
 }
 REMOVE_BRAKE;
 next=0;
 x_dis=0;
 y_dis=0;
 prvvaluel=0;
 prvvaluer=0;
 T0TC=0;
 T1TC=0;
 enc_diff_1_2=0;
 while(x_dis > (-5))
 {
	  pos();
	  control_pid_omni((-90)*Degree_to_Rad,8);
 }
 
	
}

void restart_tz3()
{
		 
	  enc_diff_1_2=0;
//==================take the encoder ticks the simple way=============
	  int i=0;
	  double multiplier_ded_rekon=1;
//_________________________ENCODER SETTINGS____________________________ 	
		T0TC=0;
	  T1TC=0;
	  prvvaluel=0;
	  prvvaluer=0;
	  x_dis=0;
	  y_dis=0;
//_____________________________________________________________________
   
    next=0;
	  double heading_1=0,distance=0,factor=0;
	  int base_value=0,checkpost=0,x_rec=0,y_rec=0;
	  bool subtask_done = 0;
   
//===================first time coordinate allcation ===================	
		x_rec = (checkpost+1)*(X_TZ1/10);
		y_rec = (checkpost+1)*(Y_TZ1/10);
	  factor = sqrt((double)(RST_TZ3_X)*(RST_TZ3_X) + (double)(RST_TZ3_Y)*(RST_TZ3_Y));
    while(!next)
		{
		  //	cls();
		  //	lcd(heading_1*57.32);
			/*
			cls();
			lcd(x_dis);
			lowerline();
			lcd(y_dis);
			*/
			  pos();
			  distance = sqrt(((RST_TZ3_X-x_dis)*(RST_TZ3_X-x_dis))+((RST_TZ3_Y-y_dis)*(RST_TZ3_Y-y_dis))); 
				heading_1 = atan2((RST_TZ3_Y-y_dis),((RST_TZ3_X)-x_dis));
				heading_1*=(-1);
				
//---------------------------setting for inintial slip of the bot -----------------------------------				
				if (x_dis > (-1)*1000)
				{
					base_value=27;
				  heading_1 += 1.57;
				}
				/*
				else if (x_dis > (-1)*500)
				{
					base_value=30;
				  heading_1 = ((-1)*1.22);
				}
				*/
//----------------------------------------------------------------------------------------------------				
				else 
				{ 
					base_value = 40*exp((-2)*(((factor-distance)/factor)));         // 2        // 30 
					heading_1 += 1.57;
				}
				/*
				if (x_dis<-1000)
				{
				  heading_1 += 1.57;    //57
				}
				else 
				{
					 heading_1 += 1.57;     // 1.67
 				}
				*/
			
				
			/*
			if (subtask_done==0)
			{
				checkpost++;
				x_rec = (checkpost+1)*(X_TZ1/10);
		    y_rec = (checkpost+1)*(Y_TZ1/10);
				subtask_done=1;
			}
		  heading_1 = (-1)*(atan2((y_rec+y_dis),(x_rec+x_dis)));   	
			distance = sqrt(((X_TZ1+x_dis)*(X_TZ1+x_dis))+((Y_TZ1+y_dis)*(Y_TZ1+y_dis))); 
      base_value = 15*exp((-2)*(((4139-distance)/4139)));         // 2        // 30 
			
			if (((-x_dis) > (x_rec - 7))||((-x_dis) > (x_rec - 5)))
			{
				subtask_done=0;
			}
			*/
			if (base_value<8)
			{
				base_value=8;
			}
	    control_pid_omni(heading_1,base_value);
			
			if ((x_dis<(RST_TZ3_X+10)&&(x_dis>(RST_TZ3_X-10)))&&((y_dis>(RST_TZ3_Y-10))&&((y_dis<(RST_TZ3_Y+10)))))//(((-x_dis)>X_TZ1-5)&&((-y_dis)>Y_TZ1-5))
			{
				BRAKE;
				next=1;
			}
		
		}
		i=1000;
		while(i--)
		{
			control_pid_omni(4*PI,5);
		}
		REMOVE_BRAKE;
			T0TC=0;
	  T1TC=0;
	  prvvaluel=0;
	  prvvaluer=0;
	  x_dis=0;
	  y_dis=0;
		while(y_dis < 80)
		{
			pos();
			control_pid_omni(0,6);
		}
		/*
		//_________________________ENCODER SETTINGS____________________________ 	
		T0TC=0;
	  T1TC=0;
	  prvvaluel=0;
	  prvvaluer=0;
	  x_dis=0;
	  y_dis=0;
//_____________________________________________________________________
   

		while(y_dis > (-50))
		{
			pos();
			control_pid_omni(180*Degree_to_Rad,6);
		}
			
    		*/
	
}

//====================================================================
// 										tz2 repetative                  
//====================================================================

void tz2_repetative()
{
	//========================================================================================================
//_________________________________VARIABLES INITIALISE___________________________________________________
//========================================================================================================		
		set(PISTON_3);
		REMOVE_BRAKE;
		next=0;
		double heading_1=0,distance=0,factor=0;
	  int base_value=0,counter_v=0;//checkpost=0,x_rec=0,y_rec=0;
		in_tz1=1;
//___________________________________________ENCODER SETTING ____________________________________________
	  x_dis=0;
   	y_dis=10;  //20
	  prvvaluel=0;
	  prvvaluer=0;
	  T0TC=0;
	  T1TC=0;
		enc_diff_1_2=1;
//_______________________________________________________________________________________________________
		icontrol=60;
		factor = sqrt((double)(X_TZ1_CTRM)*(X_TZ1_CTRM) + (double)(Y_TZ1_CTRM)*(Y_TZ1_CTRM));	
//========================================================================================================
//_________________________________GO FOR SHOOTING(DED REKON)PREFOMING CTRM_______________________________
//========================================================================================================		
		while(!next)
		{
		 pos();
//---------------------------------------------------	
		 heading_1 = atan2((Y_TZ1_CTRM-y_dis),(X_TZ1_CTRM-x_dis));
		 heading_1*=(-1);
		 heading_1 += 1.57;
		 heading_1 -= ang*Degree_to_Rad;
//---------------------------------------------------
        
			if (y_dis>30)//(distance < 0.9*factor)
			{
	 		   hold_angle = ZONE_2_ANGLE*(1 - distance/factor);  
         hold_angle*=2;
				 if (ang>hold_angle)
				  {
            hold_angle=ang;
				  }
			}  
      if (ang>ZONE_2_ANGLE)
			{
				hold_angle = ZONE_2_ANGLE;
				next=1;
			}
			
			distance = sqrt(((X_TZ1_CTRM-x_dis)*(X_TZ1_CTRM-x_dis))+((Y_TZ1_CTRM-y_dis)*(Y_TZ1_CTRM-y_dis)));
			control_pid_omni(heading_1,18);
			
		}
		hold_angle = ZONE_2_ANGLE;
		icontrol=6;
	
		
		zone = ZONE_ONE_START;
		REMOVE_BRAKE;
		test_shoot();
			
//========================================================================================================
//_________________________________RETURN TO THE INITIAL CO-ORDINATES_____________________________________
//========================================================================================================
   base_value=18;    //18
	 icontrol=13;	
		while(1)
	{
		pos();
		control_pid_omni(135*Degree_to_Rad,base_value);
		hold_angle-=0.40;//43    //0.05
		if(ang<20)
		{
			icontrol=8;
			base_value=10;
		}
		if (ang<0)
			break;
	}
  hold_angle=0;










//=========================================================================================================
		next=0;
		icontrol=60;
		distance=0;
		bool local_flag=0;
	  double temp_enc=0,diff_enc=0;
		factor = sqrt((double)(X_TZ1_CTRM)*(X_TZ1_CTRM) + (double)(Y_TZ1_CTRM)*(Y_TZ1_CTRM));	
		while(!next)
		{
		 sensor_val_update_h();	
		 pos();
		 diff_enc = y_dis - temp_enc;
//---------------------------------------------------	
		 heading_1 = atan2((Y_TZ1_2_CTRM-y_dis),(X_TZ1_2_CTRM-x_dis));
		 heading_1*=(-1);
		 heading_1 += 1.57;
		 heading_1 -= ang*Degree_to_Rad;
//---------------------------------------------------
 /*
			 hold_angle-=0.037;
			 if (ang<hold_angle)
			 {
					hold_angle = ang;
			 }
      if (hold_angle<0)
			{
				hold_angle = 0;
			}				
      if (ang<0)
			{
				hold_angle = 0;
				icontrol=8;
				//next=1;
			}
			if(hold_angle>20)
			{
				base_value=15;
			}
			else 
			{
				base_value=11;
			}
			
			if ((hold_angle==0)||(ang==0))
			{
				base_value = 6;
			}
			
			
			*/
			base_value=7;  //10
			
			
			
			
			if(((y_dis<(40))&&(y_dis>30))&&((x_dis < 10)&&(x_dis > (-10)))&&(diff_enc>0))
			    next=1;
//-------------------UNECESSARY CONDITION---------------------------------------			
	
//--------------------------------------------------------------------------------------------------			
			distance = sqrt((Y_TZ1_CTRM-y_dis)*(Y_TZ1_CTRM-y_dis));//(((X_TZ1_CTRM-x_dis)*(X_TZ1_CTRM-x_dis))+((Y_TZ1_CTRM-y_dis)*(Y_TZ1_CTRM-y_dis)));
			control_pid_omni(heading_1,base_value);
			temp_enc = y_dis;
		}
		while(sum_sensor_h < 2)
		{
			sensor_val_update_h();
			control_pid_omni(90*Degree_to_Rad,5);
		}
//___________________________________________ENCODER SETTING ____________________________________________
	  x_dis=0;
   	y_dis=20;
	  prvvaluel=0;
	  prvvaluer=0;
	  T0TC=0;
	  T1TC=0;
		enc_diff_1_2=0;
//_______________________________________________________________________________________________________
		BRAKE;
		count_cycle1=0;
		while(count_cycle1<100)
		{
			control_pid_omni(4*PI,5);
		}
		REMOVE_BRAKE;
		
//====================================================================		
//													REMOVED BEACUSE OF BREAK		          
//===================================================================== 
    /*
    if (diff_enc > 0)
		{
			if (!NORMAL_BALL_PROXY)
				tz2_ball_proxy=1;
		 int i=400;
		 while(i--)
		 {
		   pos();
			 control_pid_omni(180*Degree_to_Rad,21);	 
		 }
		 i=500;
		 while(i--)
		 {
		   pos();
			 control_pid_omni(180*Degree_to_Rad,6);	 
		 }
		
		}
		else if (diff_enc < 0)
		{
			if (!NORMAL_BALL_PROXY)
				tz2_ball_proxy=1;
			int i=130;
			 while(i--)
		 {
		   pos();
			 control_pid_omni(180*Degree_to_Rad,21);	 
		 }
		 while(y_dis < (-30)){
			  pos();
			 control_pid_omni(180*Degree_to_Rad,6);	
		 }
		 cls();
		 lcd((char *)"down");
		
	  }
		*/
//==========================================================

//==========================================================	
		/*
		while(y_dis>10)
		{		  
		  pos();
			//---------------------------------------------------	
			heading_1 = atan2((Y_TZ1_2_CTRM-y_dis),(X_TZ1_2_CTRM-x_dis));
			heading_1*=(-1);
			heading_1 += 1.57;
			heading_1 -= ang*Degree_to_Rad;
			base_value=11;
			control_pid_omni(heading_1,base_value);
		}
		
		
		while(y_dis<0)
		{		  
		  pos();
			//---------------------------------------------------	
			heading_1 = atan2((Y_TZ1_2_CTRM-y_dis),(X_TZ1_2_CTRM-x_dis));
			heading_1*=(-1);
			heading_1 += 1.57;
			heading_1 -= ang*Degree_to_Rad;
			base_value=11;
			control_pid_omni(heading_1,base_value);
		}
	*/
	

	  
		base_value=15;
		
		while(0)//(y_dis>(-30))
		{		  
		  pos();
			//---------------------------------------------------	
			heading_1 = atan2((((-1)*100)-y_dis),(0-x_dis));
			heading_1*=(-1);
			heading_1 += 1.57;
			heading_1 -= ang*Degree_to_Rad;
			control_pid_omni(heading_1,6);
		}
}



void TZ1_NEW()
{
	//========================================================================================================
//_________________________________VARIABLES INITIALISE___________________________________________________
//========================================================================================================		
		set(PISTON_3);
		REMOVE_BRAKE;
		next=0;
		double heading_1=0,distance=0,factor=0;
	  int base_value=0,counter_v=0;//checkpost=0,x_rec=0,y_rec=0;
		in_tz1=1;
//___________________________________________ENCODER SETTING ____________________________________________
	  x_dis=0;
   	y_dis=10;  //20
	  prvvaluel=0;
	  prvvaluer=0;
	  T0TC=0;
	  T1TC=0;
		enc_diff_1_2=1;
	  count_cycle1=0;
//_______________________________________________________________________________________________________
		icontrol=60;
		factor = sqrt((double)(X_TZ1_CTRM)*(X_TZ1_CTRM) + (double)(Y_TZ1_CTRM)*(Y_TZ1_CTRM));	
//========================================================================================================
//_________________________________GO FOR SHOOTING(DED REKON)PREFOMING CTRM_______________________________
//========================================================================================================		
		while(!next)
		{
		 pos();
//---------------------------------------------------	
		 heading_1 = atan2((Y_TZ1_CTRM-y_dis),(X_TZ1_CTRM-x_dis));
		 heading_1*=(-1);
		 heading_1 += 1.57;
		 heading_1 -= ang*Degree_to_Rad;
//---------------------------------------------------
      if (count_cycle1 > 80)
			{
				set(PISTON_3);
			}
      else if (count_cycle1 > 60)
			{
				reset(PISTON_3);
			}				
     			
			if (y_dis>30)//(distance < 0.9*factor)
			{
	 		   hold_angle = ZONE_1_ANGLE*(1 - distance/factor);  
         hold_angle*=2;
				 if (ang>hold_angle)
				  {
            hold_angle=ang;
				  }
			}  
      if (ang>ZONE_1_ANGLE)
			{
				hold_angle = ZONE_1_ANGLE;
				next=1;
			}
			
			distance = sqrt(((X_TZ1_CTRM-x_dis)*(X_TZ1_CTRM-x_dis))+((Y_TZ1_CTRM-y_dis)*(Y_TZ1_CTRM-y_dis)));
			control_pid_omni(heading_1,18);
			
		}
		hold_angle = ZONE_1_ANGLE;
		icontrol=6;
	
		
		zone = ZONE_ONE_START;
		REMOVE_BRAKE;
		test_shoot();
		
		base_value=18;
		icontrol=30;
		factor = sqrt((double)(X_TZ1_2_CTRM)*(X_TZ1_2_CTRM) + (double)(Y_TZ1_2_CTRM)*(Y_TZ1_2_CTRM));
		next=0;
		while(!next)
		{
			if(ang>30)
			 hold_angle-=0.035;  //25
			else
				hold_angle-=0.025;
		 pos();
//---------------------------------------------------	
		 heading_1 = atan2((Y_TZ1_2_CTRM-y_dis),(X_TZ1_2_CTRM-x_dis));
		 heading_1*=(-1);
		 heading_1 += 1.57;
		 heading_1 -= ang*Degree_to_Rad;
//---------------------------------------------------
    
		 		
			
			
      if (ang<1)
			{
				hold_angle = 0;
				base_value=5;
				//next=1;
			}
			if (((y_dis<(58))&&(y_dis>45))&&((x_dis < 30)&&(x_dis > (-30))))
			    next=1;
			
			if ((hold_angle==0)&&((heading_1>1.39)&&(heading_1 < 1.744)))
			{
				next=1;
			}
			
			distance = sqrt(((X_TZ1_2_CTRM-x_dis)*(X_TZ1_2_CTRM-x_dis))+((Y_TZ1_2_CTRM-y_dis)*(Y_TZ1_2_CTRM-y_dis)));
			control_pid_omni(heading_1,base_value);
			
		}
		hold_angle = 0;
		icontrol=6;
			BRAKE;
		count_cycle1=0;
		while(count_cycle1<100)
		{
			control_pid_omni(4*PI,5);
		}
		REMOVE_BRAKE;
		hold_angle=0;

}





















