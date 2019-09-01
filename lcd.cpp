//Note	: Here hign and low status of pin are inverted because of hardware

#include<LPC17xx.h>    //Always Put this header file on the top of the code else u will get errors
#include<math.h>

#include"LPC17xx_1.h"

#include"PLL.h"
#include"GPIO.h"

#include"lcd.h"
						   
unsigned short E;
unsigned short Rs;
unsigned short Db4;
unsigned short Db5;
unsigned short Db6;
unsigned short Db7;

void LCD_enable()
{
    set(E); 
	waitms(1);
    reset(E);
	waitms(1);
}

void LCD_command(unsigned char command)
{
    reset(Rs);
	  unsigned short int Db = 0;
    Db = ((command>>4) & 0x000F);
	if( Db &0x0001 ) set(Db4); else reset(Db4);
	if( Db &0x0002 ) set(Db5); else reset(Db5);
	if( Db &0x0004 ) set(Db6); else reset(Db6);
	if( Db &0x0008 ) set(Db7); else reset(Db7);
	  
	LCD_enable();
	Db = 0;
    Db = (command & 0x000F);
	if( Db &0x0001 ) set(Db4); else reset(Db4);
	if( Db &0x0002 ) set(Db5); else reset(Db5);
	if( Db &0x0004 ) set(Db6); else reset(Db6);
	if( Db &0x0008 ) set(Db7); else reset(Db7);

	LCD_enable();
	//waitms(1);
}

void LCD_putc(unsigned char ascii)
{
    set(Rs); 
    unsigned short int Db = 0;
    Db = ((ascii>>4) & 0x000F);
	if( Db &0x0001 ) set(Db4); else reset(Db4);
	if( Db &0x0002 ) set(Db5); else reset(Db5);
	if( Db &0x0004 ) set(Db6); else reset(Db6);
	if( Db &0x0008 ) set(Db7); else reset(Db7);

    LCD_enable();
    Db = 0;
    Db = (ascii & 0x000F);
	if( Db &0x0001 ) set(Db4); else reset(Db4);
	if( Db &0x0002 ) set(Db5); else reset(Db5);
	if( Db &0x0004 ) set(Db6); else reset(Db6);
	if( Db &0x0008 ) set(Db7); else reset(Db7);

	LCD_enable();
}
//------------------------------------
void lcd(char *lcd_string)
{
	while (*lcd_string)
		LCD_putc(*lcd_string++);
}

void lcd(signed int lcd_num)
{
	int tlcd_num;
	short len,sign = 0;
	char *lcd_string,num[11];
	lcd_string = num;
	//--------------------------------
	if(lcd_num == 0)
		LCD_putc('0');		
	else if(lcd_num < 0)	{
		sign = 1;
		*lcd_string++ = '-';
		lcd_num = -lcd_num;
	}
	//--------------------------------
	tlcd_num = lcd_num;							  
	for(len=0;tlcd_num>0;len++,tlcd_num/=10);
	lcd_string += len;
	*lcd_string-- = 0;
	for(;len>0;len--)	{
		*lcd_string-- = (lcd_num%10) + 48;
		lcd_num /= 10;
	}
	if(sign == 1)
		lcd_string--;
	lcd_string++;
	//--------------------------------
	while (*lcd_string) 
		LCD_putc(*lcd_string++);
}

void lcd(unsigned int lcd_num)
{
	int tlcd_num;
	short len;
	char *lcd_string,num[11];
	lcd_string = num;
	//--------------------------------
	if(lcd_num == 0)
		LCD_putc('0');
	//--------------------------------
	tlcd_num = lcd_num;
	for(len=0;tlcd_num>0;len++,tlcd_num/=10);
	lcd_string += len;
	*lcd_string-- = 0;
	for(;len>0;len--)	{
		*lcd_string-- = lcd_num%10 + 48;
		lcd_num /= 10;	
	}
	lcd_string++;
	//--------------------------------
	while (*lcd_string)
		LCD_putc(*lcd_string++);
}

void lcd(float lcd_num,unsigned short fdigit)
{
	int floating;
	int natural = (int)lcd_num;
	float tfloating;
	if(lcd_num >= 0) {	 
		tfloating = (lcd_num-(float)natural);
	}
	else {		
		tfloating = -(lcd_num-(float)natural);
	}			

	//--------------------------------
	if(lcd_num<0 && natural==0)
	{
		LCD_putc('-');
 	}
	//--------------------------------

	lcd(natural);
	LCD_putc('.');
	
	tfloating *= pow((float)10,fdigit);
	floating = (int)tfloating;
	//--------------------------------
	while(fdigit && floating)	{
		
		--fdigit;
		floating /= 10;
	}

	while(fdigit)	{

		--fdigit;
		LCD_putc('0');
	}
	//--------------------------------
	floating = (int)tfloating;
	lcd(floating);
}

void lcd(double lcd_num,unsigned short fdigit)
{
	int floating;
	int natural = (int)lcd_num;
	double tfloating;
    if(lcd_num >= 0) {	 
		tfloating = (lcd_num-(double)natural);
	}
	else {		
		tfloating = -(lcd_num-(double)natural);
	}			

	//--------------------------------
	if(lcd_num<0 && natural==0)
	{
		LCD_putc('-');
 	}
	//--------------------------------

	lcd(natural);
	LCD_putc('.');

	tfloating *= pow((double)10,fdigit);
	floating = (int)tfloating;
	//--------------------------------
	while(fdigit && floating)	{
		
		--fdigit;
		floating /= 10;
	}

	while(fdigit)	{

		--fdigit;
		LCD_putc('0');
	}
	//--------------------------------
	floating = (int)tfloating;
	lcd(floating);
}
//------------------------------------
void ConfigLcd(unsigned short tRs,unsigned short tE,unsigned short tDb4,
				unsigned short tDb5,unsigned short tDb6,unsigned short tDb7)
{
	Rs 	= tRs;
	E 	= tE;
	Db4 = tDb4;
	Db5 = tDb5;
	Db6 = tDb6;
	Db7 = tDb7;

    set(E);
    reset(Rs);
	   
    LCD_command(0x33);
    LCD_command(0x32);
    LCD_command(0x28);
    LCD_command(0x0C);
    LCD_command(0x06);
    LCD_command(0x01); /* Clear */
    waitms(256);
}
