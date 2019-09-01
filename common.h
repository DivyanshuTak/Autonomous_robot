#ifndef _COMMON_H
#define _COMMON_H

#include<LPC17xx.h>
#include"lpc/LPC17xx_1.h"
#include<math.h>
#include"lpc/GPIO.h"
#include"lpc/lcd.h"
#include"lpc/NVIC.h"
#include"lpc/PLL.h"
#include"lpc/SPI.h"
#include"lpc/TIMER.h"
#include"lpc/UART.h"
#include"lpc/QEI.h"
#include"lpc/DAC.h"

#define PI 3.1415926535
#define Degree_to_Rad 0.0174532925
#define DIA	56.25
#define CNTPERREV 2048.0

#define DED_RECON 0
#define DED_RECON_SEMI_AUTO 0

//-----------------LCD Aliases-------------------
#define Rs 			Port1_19							// Rsest Pin
#define E 			Port1_21							// Enable Pin 
#define Db4			Port1_28							// Databit 4
#define Db5			Port1_25							// Databit 5
#define Db6			Port1_27							// Databit 6
#define Db7			Port1_29							// Databit 7
//
//-------------10 Pin FRC aliases-----------------
#define R1_1	  Pin4_28     // MAT2.0  TX3
#define R1_2 	  Pin4_29			// 
#define R1_3 	  Pin2_2			// PWM1_2
#define R1_4 	  Pin0_5			// MAT2.0
#define R1_5    Pin0_6			// CAP2.1
#define R1_6    Pin2_1			// PWM1_3

#define R2_1   	Port0_9      // MAT2.3
#define R2_2   	Port0_7   		// MAT2.1
#define R2_3   	Port2_5      // PWM1_6
#define R2_4   	Port2_4			// PWM1_5
#define R2_5   	Port2_3			// PWM1_4
#define R2_6   	Port0_8			// MAT2.2

#define P1_1	Port0_6
#define P1_2	Port0_5
#define P1_3	Port4_28 
#define P1_4	Port4_29
#define P1_5	Port2_2
#define P1_6	Port2_1

#define P2_1   	Port2_4			// PWM1_5
#define P2_2   	Port2_3			// PWM1_4
#define P2_3   	Port0_9      // MAT2.3
#define P2_4   	Port0_7   		// MAT2.1
#define P2_5   	Port0_8			// MAT2.2
#define P2_6   	Port2_5			// PWM1_4

#define P3_1    Port0_18 
#define P3_2    Port0_17
#define P3_3    Port0_15
#define P3_4    Port0_16
#define P3_5    Port2_9
#define P3_6    Port2_8

#define P4_1    Port0_22 
#define P4_2    Port0_21
#define P4_3    Port0_20
#define P4_4    Port0_19
#define P4_5    Port2_7
#define P4_6    Port2_6

#define R3_1		Port0_15	//SCK/TX1
#define R3_2    Port0_16	//RX1
#define R3_3    Port2_8
#define R3_4    Port0_18 
#define R3_5    Port0_17
#define R3_6    Port2_9

#define R4_1    Port0_20
#define R4_2    Port0_19
#define R4_3    Port2_4
#define R4_4    Port0_22
#define R4_5    Port0_21
#define R4_6    Port2_7
//

//------Debug and direction aliases------
#define I2Cinit 0
#define DEBUG 1

#define FORWARD 1
#define BACKWARD 2
#define RIGHT 3
#define LEFT 4

#define START_POINT 0
#define ZONE_ONE_START 1
#define ZONE_ONE_SHOOT 2
#define ZONE_TWO_START 3
#define ZONE_TWO_SHOOT 4
#define ZONE_THREE_SHOOT 5

#define START_PROXY 0
#define DIRECTION_PROXY 1
#define ZONE_ONE_PROXY 2
#define ZONE_TWO_PROXY 3
#define ZONE_THREE_PROXY 4

#define Start_proxy Pin0_25
#define Zone_two_proxy Pin0_24
#define Zone_three_proxy Pin0_26

#define PISTON_1 Port1_14
#define PISTON_2 Port1_15
#define PISTON_3 Port1_16
#define GRIPPER Port1_17

#define RACK_1_PROXY Pin0_23
#define RACK_2_PROXY Pin0_24
#define NORMAL_BALL_PROXY Pin0_25
#define GOLDEN_BALL_PROXY Pin0_25

#define FORWARD_PROXY Pin0_29
#define RIGHT_PROXY Pin0_27

#define ZONE_1_ANGLE 80             //70.5             // 74           //71
#define ZONE_2_ANGLE 86                     //78.8
#define RADIUS 990
#define BRAKE set(PISTON_2);
#define REMOVE_BRAKE reset(PISTON_2);

//--------------------DED REKON CO-ORDINATES ---------------------------------------------
//---------CURRENT WORKING CO-ORDINATES FOR TZ1-------------------------------------------
#define X_TZ3   3418
#define Y_TZ3   278                                                                         // 278
#define X_TZ3_2 160                                                                         //300//450//550//600//(-1)*3477//  3418
#define Y_TZ3_2 4977                                                                        //3677//3477 //1299 //278               // 278
//---------CURRENT WORKING CO-ORDINATES FOR TZ1----------------------------------------------
#define X_TZ1   (-1)*4505                                                    //  (feild error)  4630                                                                    //3848//3748//(-1)*4462//4662//(-1)*4687//4162//4266
#define Y_TZ1   1350                                                                      //950 //1254  // 950//814//1033
#define X_TZ1_2 (-1)*4505                                                                  //4530//4490//4839//(-1)*4662//4250
#define Y_TZ1_2 1430   // for inital path                            //2061      //2361//2631//2731//2141//2000 //2200
#define RST_TZ3_X (-1)*6050
#define RST_TZ3_Y 1390
//--------------------------------------------------------------------------------------------
#define Y_TZ2   (-1)*950                                                                      //1087//1025
#define X_TZ2   (-1)*453                                                                      //1190//1310
#define X_TZ2_2 (-1)*800                                                                      //1000//1713//1483//1750//1850       //2156
#define Y_TZ2_2  8                                                                            //10//(-1)*1240//503//1336
#define X_TZ2_3  (-1)*270                                                                     //-10//1225  ////(-1)*1725                        //(-1)*2156
#define Y_TZ2_3  930                                                                           //850 //1200//(-1)*15//142//223//1137                        //170
//-----------CURRENT WORKING CO-ORDINATES FOR TZ2-----------------------------------------------
#define Y1_TZ2   (-1)*750                                                                       //1087//1025
#define X1_TZ2   (-1)*353                                                                       //1190//1310
#define Y1_TZ2_2  1095///999                                     //120																																			// (-1)*120//750//1087//1025
#define X1_TZ2_2  (-1)*1850                                     //1688																																			//453//1190//1310
//------------NEW CO-ORDINATES FOR NEW FUNCTION FOR TZ3------------------------------------------
#define X_TZ3_3 (-1)*5
#define X_TZ3_3_2 (-1)*50
#define Y_TZ3_3 3977
#define X1_TZ3 (-1)*4400
#define Y1_TZ3 30//220
//-------------NEW CO-ORDINATES FOR TZ1 SHOOT WHILE DOING CTRM----------------------------------
#define X_TZ1_CTRM   0//(-1)*30                                                     //  (feild error)  4630                                                                    //3848//3748//(-1)*4462//4662//(-1)*4687//4162//4266
#define Y_TZ1_CTRM   440     
#define X_TZ1_2_CTRM 0//(-1)*30                                                    //  (feild error)  4630                                                                    //3848//3748//(-1)*4462//4662//(-1)*4687//4162//4266
#define Y_TZ1_2_CTRM 40///40//(-1)*500  
//----------------------------------------------------------------------------------------------
#define goldenproxy 1

extern double divyanshu,divyanshu_ka_chut;
extern bool enc_diff_1_2,golden_proxy,restart_2;
//-------------------------------------VARIABLES USED THROUGHT THE CODE------------------------------------------
//PS2 variables
extern bool ip[4][8],ps_up,ps_right,ps_down,ps_left,ps_square,ps_triangle,ps_circle,ps_cross,ps_select,ps_start,
	    ps_r1,ps_r2,ps_l1,ps_l2;
extern int ps2_val[4], one,two,three,four,turn_adc,turn_yes,golden_counter;
extern double temp_four;
//

//PID variables
extern double b_heading,hold_angle,value_1,value_2,value_3, proportional, integral,derivative,integrald,rate,
	     control,old_control,icontrol;
extern float kp,ki,kd,manual_kp,manual_ki,manual_kd,auto_kp,auto_ki,auto_kd;
extern char dummyl,dummyr,txt1,txt2,txt3;
//

// LIMITING factors
extern double lim11_lim,lim12_lim,lim21_lim,lim22_lim;
extern double lim11,lim12;             //limits for channel 1 -> with stop value 64    80,48
extern double lim21,lim22;           //limits for channel 2 -> with stop value 192   208,176
extern int control_speed,flag,safety,minor_adjustment_speed;
extern double acc;
//

//Flags/ misc.
extern int speed ,count_cycle,mode,stop_count,count_cycle1,turn_speed_limit,turn_speed,count_cycle2 ,
    res[4],ans,time,turn_final,field,pos_flag,sop_flag,p_ang,ang_time;
extern double ang,x_hold ,y_hold,angle_change,difference,heading_2;
extern int plane_select,err_count1,err_count2,centre_point, count,aniruddha,time_limit,tz2_counter,heading_1;
extern bool field_error,tz2_ball_proxy;
//

//DED recon - encoder
extern int b1,a1,previous,tickl,tickr,prvvaluel,prvvaluer,prevposition,templ ,tempr ;
extern double th;
extern double left_ddis,dis_per_count_left,right_ddis,dis_per_count_right,x_dis,y_dis,left_dia,right_dia;
//

//line sensor - line sensor PID
extern double proportional_line_sense, kp_line_sense, integral_line_sense, integrald_line_sense, ki_line_sense, rate_line_sense, prev_err, derivative_line_sense, kd_line_sense, 
	     control_line_sense, icontrol_line_sense;
extern int sum_sensor_v,sum_sensor_h;
extern bool sensor_value[8];
//

//Theme specific
extern int zone,count_task;
extern bool next,dis_flag,shoot_3,zone_complete,in_tz1;
extern int junction_count;
extern double error_v,error_h, velocity,velocity_final;
extern double x_co_arr[10],y_co_arr[10];
extern  bool flag_shoot_3_2;
//

//-------------FUNCTION DECLARATION FOR USE IN WHOLE PROJECT---------

#define CLOSE_GRIPPER set(PISTON_3);

float sensor_val_update_v();
float sensor_val_update_h();
void find_line(char Direction_fin1, char Direction_fin2, int max_speed);
//

//
void go_to_zone_one_shoot();
void go_to_zone_two_shoot();
void go_to_zone_three_shoot();
void go_to_zone_one_start();
void go_to_zone_two_start();
int proxy_check();
//
void TZ1_NEW();
void test_start_to_tz1();
void test_tz2_to_tz3_shoot();
void test_tz1_to_tz2();
void test_decider();
void test_start_to_tz2();
void tz3_modified_DT();
void tz2_modified();
void tz3_modified();
void tz3_fastest();	
//
void set_speed_limit(void);		
void get_rate(void);
void get_angle(void);
void pos();
void angle_90();
void angle_0();
void wait_pid(int wait_period);
void drive(void);
void timer();
void control_pid_omni(double heading,double speed_limit);
void Ps2_val_update(void);
void control_pid_line_follow(char Direction, int max_speed, double error_line);
void tz1_2_fast();
void tz3_fast();
void start_to_load();
void tz2_fast();
void test_decider_2();
void tz3_fast_2();
void tz2_fast_2();
void tz3_fast_3();
void tz1_shoot();
void tz2_fast_3();
void tz3_fast_4();
void tz3_fast_5();
void tz1_shoot_2();
void tz3_fast_6();
void tz3_fast_return();
void restart_tz3();
void tz2_repetative();

//

//                           DIVYANSHU TAK'S FUNCTIONS 
void test_shoot();
void tz3_return();
//

#endif
