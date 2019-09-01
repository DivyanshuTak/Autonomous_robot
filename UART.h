#ifndef _UART_H
#define _UART_H
//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
#define STOP_BIT_1	      0
#define STOP_BIT_2		  1

#define _TX				  0
#define _RX				  1
#define _TX_RX			  2

#define WORD_LENGTH_5	  0
#define WORD_LENGTH_6	  1
#define WORD_LENGTH_7	  2
#define WORD_LENGTH_8	  3

#define PARITY_DISABLE    0
#define PARITY_ENABLE     1

#define ODD_PARITY        0
#define EVEN_PARITY       1
#define FORCED_1_PARITY   2
#define FORCED_0_PARITY   3

#define BREAK_TX_DISABLE  0
#define BREAK_TX_ENABLE   1

#define FIFO_DISABLE  	  0
#define FIFO_ENABLE		  1

#define COM0              0
#define COM1              1
#define COM2              2
#define COM3              3
#define COM4              4
//-----------------------------------------------------------------------------------------------
void ConfigUART(unsigned short channel,unsigned int baud_rate=9600,unsigned short function=_TX,unsigned short word_length=WORD_LENGTH_8,
					unsigned short stop_bit=STOP_BIT_1,unsigned short parity_enable=PARITY_DISABLE,
					unsigned short parity_select=ODD_PARITY,unsigned short break_control=BREAK_TX_DISABLE);
//void ConfigUART(unsigned short channel,unsigned int baud_rate=9600,unsigned short word_length=WORD_LENGTH_8,
//					unsigned short stop_bit=STOP_BIT_1,unsigned short parity_enable=PARITY_DISABLE,
//					unsigned short parity_select=ODD_PARITY,unsigned short break_control=BREAK_TX_DISABLE);
void ConfigExtPrintbin(unsigned short channel,unsigned short variable);
void Printbin(unsigned short channel,unsigned short value);
void ExtPrintbin(unsigned short channel,unsigned short variable,unsigned short value);
void ConfigUARTFIFO(unsigned short channel,unsigned int status);
unsigned short Inputbin(unsigned short channel);
unsigned short Inputbin2(unsigned short channel);
//-----------------------------------------------------------------------------------------------

#endif
