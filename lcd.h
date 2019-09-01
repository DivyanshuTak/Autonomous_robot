#ifndef _LCD_H
#define _LCD_H

#define cls() LCD_command(0x1)						/* Clear display LCD */
#define origin() LCD_command(0x2)					/* Set to origin LCD */
#define upperline() LCD_command(0x80)				/* Begin at Line 1 */
#define lowerline() LCD_command(0xC0)   			/* Begin at Line 2 */

void LCD_command(unsigned char command);
void LCD_putc(unsigned char ascii);

void lcd(char *);
void lcd(signed int);
void lcd(unsigned int);
void lcd(unsigned long);
void lcd(float,unsigned short fdigit=6);
void lcd(double,unsigned short fdigit=6);

void ConfigLcd(unsigned short,unsigned short,unsigned short,unsigned short,unsigned short,unsigned short);

#endif
