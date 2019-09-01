//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
#include<LPC17xx.h>    //Always Put this header file on the top of the code else u will get errors

#include"LPC17xx_1.h"
#include"PLL.h"
#include"NVIC.h"
#include"GPIO.h"
#include"UART.h"

unsigned short uart0tx[4],uart1tx[4],uart2tx[4],uart3tx[4];
unsigned short uart0txdone[4],uart1txdone[4],uart2txdone[4],uart3txdone[4];
//-----------------------------------------------------------------------------------------------

void ConfigUART(unsigned short channel,unsigned int baud_rate,unsigned short function,unsigned short word_length,unsigned short stop_bit,
	       unsigned short parity_enable,unsigned short parity_select,unsigned short break_control)
{
	int dlest;
	InitPLL();
	dlest=(double)30000000/((double)baud_rate*16);

  	switch (channel)
  	{
	case COM0:
			PCONP = PCONP|0x00000008;					//PCUART0 Enable
        	
			if(function	== _TX)	{
				PINSEL0 &= 0xFFFFFFCF;
				PINSEL0 |= 0x00000010;
			}
			else if(function == _RX)	{
				PINSEL0 &= 0xFFFFFF3F;
				PINSEL0 |= 0x00000040;
			}
			else if(function == _TX_RX)	{
				PINSEL0 &= 0xFFFFFF0F;
				PINSEL0 |= 0x00000050;
			}
			
			
			U0LCR=(word_length)|(stop_bit)<<2|parity_enable<<3|parity_select<<4|break_control<<6|1<<7;
	        U0DLL=dlest%256;	
	        U0DLM=dlest/256;	 
			U0LCR=(word_length)|(stop_bit)<<2|parity_enable<<3|parity_select<<4|break_control<<6|0<<7;
            break;

	case COM1:	       
			PCONP = PCONP|0x00000010;					//PCUART1 Enable
			//PINSEL0 &= 0x3FFFFFFF;
			//PINSEL0 |= 0x40000000;
			if(function	== _TX)	{
				PINSEL4 &= 0xFFFFFFFC;
				PINSEL4 |= 0x00000002;
			}
			else if(function == _RX)	{
				PINSEL4 &= 0xFFFFFFF3;
				PINSEL4 |= 0x00000008;
			}
			else if(function == _TX_RX)	{
				PINSEL4 &= 0xFFFFFFF0;
				PINSEL4 |= 0x0000000A;
			}
			
			U1LCR=(word_length)|(stop_bit)<<2|parity_enable<<3|parity_select<<4|break_control<<6|1<<7;
	        U1DLL=dlest%256;	
	        U1DLM=dlest/256;
	        U1LCR=(word_length)|(stop_bit)<<2|parity_enable<<3|parity_select<<4|break_control<<6|0<<7;
            break;
  
   case COM2:
        	PCONP = PCONP|0x01000000;					//PCUART2 Enable
			if(function	== _TX)	{
				PINSEL0 &= 0xFFCFFFFF;
				PINSEL0 |= 0x00100000;			
			}
			else if(function == _RX)	{
				PINSEL0 &= 0xFF3FFFFF;
				PINSEL0 |= 0x00400000;			
        	}
			else if(function == _TX_RX)	{
				PINSEL0 &= 0xFF0FFFFF;
				PINSEL0 |= 0x00500000;			
        	}
			
			U2LCR=(word_length)|(stop_bit)<<2|parity_enable<<3|parity_select<<4|break_control<<6|1<<7;
	        U2DLL=dlest%256;	
	        U2DLM=dlest/256;
			U2LCR=(word_length)|(stop_bit)<<2|parity_enable<<3|parity_select<<4|break_control<<6|0<<7;
            break;
  
   case COM3:  
   			PCONP = PCONP|0x02000000;					//PCUART3 Enable
        	//PINSEL4 &= 0xFFFDFFFF;
			//PINSEL4 |= 0x00020000;
        	if(function	== _TX)	{
				PINSEL0 &= 0xFFFFFFFC;
				PINSEL0 |= 0x00000002;		
			}
			else if(function == _RX)	{
				PINSEL0 &= 0xFFFFFFF3;
				PINSEL0 |= 0x00000008;		
        	}
			/*else if(function == _TX_RX)	{
				PINSEL0 &= 0xFFFFFFF0;
				PINSEL0 |= 0x0000000A;			
        	}*/
			else if(function == _TX_RX)	{                 //for uart to be used in line sensor
				PINSEL9 &= 0xF0FFFFFF;
				PINSEL9 |= 0x0F000000;			
			    }
			
			U3LCR=(word_length)|(stop_bit)<<2|parity_enable<<3|parity_select<<4|break_control<<6|1<<7;
	        U3DLL=dlest%256;	
	        U3DLM=dlest/256;
			U3LCR=(word_length)|(stop_bit)<<2|parity_enable<<3|parity_select<<4|break_control<<6|0<<7;
            break;
  }
}

/*
// old uart without _tx_rx
void ConfigUART(unsigned short channel,unsigned int baud_rate,unsigned short word_length,unsigned short stop_bit,
	       unsigned short parity_enable,unsigned short parity_select,unsigned short break_control)
{
	int dlest;
	InitPLL();
	dlest=(double)30000000/((double)baud_rate*16);

  	switch (channel)
  	{
	case COM0:
			PCONP = PCONP|0x00000008;					//PCUART0 Enable
        	//IOCON_P0_2=0x001;
        	PINSEL0 &= 0xFFFFFFEF;
			PINSEL0 |= 0x00000010;
			
			U0LCR=(word_length)|(stop_bit)<<2|parity_enable<<3|parity_select<<4|break_control<<6|1<<7;
	        U0DLL=dlest%256;	
	        U0DLM=dlest/256;	 
			U0LCR=(word_length)|(stop_bit)<<2|parity_enable<<3|parity_select<<4|break_control<<6|0<<7;
            break;

	case COM1:	       
			PCONP = PCONP|0x00000010;					//PCUART1 Enable
        	//IOCON_P2_1=0x001;
			//PINSEL0 &= 0x3FFFFFFF;
			//PINSEL0 |= 0x40000000;
			PINSEL4 &= 0xFFFFFFFD;
			PINSEL4 |= 0x00000002;
        	
			U1LCR=(word_length)|(stop_bit)<<2|parity_enable<<3|parity_select<<4|break_control<<6|1<<7;
	        U1DLL=dlest%256;	
	        U1DLM=dlest/256;
	        U1LCR=(word_length)|(stop_bit)<<2|parity_enable<<3|parity_select<<4|break_control<<6|0<<7;
            break;
  
   case COM2:
        	PCONP = PCONP|0x01000000;					//PCUART2 Enable
			//IOCON_P0_10=0x001;	
			PINSEL0 &= 0xFFEFFFFF;
			PINSEL0 |= 0x00100000;			
        	
			U2LCR=(word_length)|(stop_bit)<<2|parity_enable<<3|parity_select<<4|break_control<<6|1<<7;
	        U2DLL=dlest%256;	
	        U2DLM=dlest/256;
			U2LCR=(word_length)|(stop_bit)<<2|parity_enable<<3|parity_select<<4|break_control<<6|0<<7;
            break;
  
   case COM3:  
   			PCONP = PCONP|0x02000000;					//PCUART3 Enable
        	//IOCON_P2_8=0x010;
			//PINSEL4 &= 0xFFFDFFFF;
			//PINSEL4 |= 0x00020000;
        	PINSEL0 &= 0xFFFDFFFD;
			PINSEL0 |= 0x00000002;
			
			U3LCR=(word_length)|(stop_bit)<<2|parity_enable<<3|parity_select<<4|break_control<<6|1<<7;
	        U3DLL=dlest%256;	
	        U3DLM=dlest/256;
			U3LCR=(word_length)|(stop_bit)<<2|parity_enable<<3|parity_select<<4|break_control<<6|0<<7;
            break;
  }
}
*/

//-----------------------------------------------------------------------------------------------
void ExtPrintbin(unsigned short channel,unsigned short variable,unsigned short value)	{
	
	switch(channel)	{
		//-----------------------------------
		case COM0:
			switch(variable)	{
				//--------------------------------
				case 1:
					if((U0LSR & 0x60) ==0x60)	{
 						U0THR=value;
					}
					else	{
						uart0tx[0] = value;
						uart0txdone[0] = 1;
					}
				break;
				//--------------------------------
				case 2:
					if((U0LSR & 0x60) ==0x60)	{
 						U0THR=value;
					}
					else	{
						uart0tx[1] = value;
						uart0txdone[1] = 1;
					}
				break;
				//--------------------------------
				case 3:
					if((U0LSR & 0x60) ==0x60)	{
 						U0THR=value;
					}
					else	{
						uart0tx[2] = value;
						uart0txdone[2] = 1;
					}
				break;
				//--------------------------------
				case 4:
					if((U0LSR & 0x60) ==0x60)	{
 						U0THR=value;
					}
					else	{
						uart0tx[3] = value;
						uart0txdone[3] = 1;
					}
				break;
				//--------------------------------
			}
		break;
		//-----------------------------------
		case COM1:
			switch(variable)	{
				//--------------------------------
				case 1:
					if((U1LSR & 0x60) ==0x60)	{
 						U1THR=value;
					}
					else	{
						uart1tx[0] = value;
						uart1txdone[0] = 1;
					}
				break;
				//--------------------------------
				case 2:
					if((U1LSR & 0x60) ==0x60)	{
 						U1THR=value;
					}
					else	{
						uart1tx[1] = value;
						uart1txdone[1] = 1;
					}
				break;
				//--------------------------------
				case 3:
					if((U1LSR & 0x60) ==0x60)	{
 						U1THR=value;
					}
					else	{
						uart1tx[2] = value;
						uart1txdone[2] = 1;
					}
				break;
				//--------------------------------
				case 4:
					if((U1LSR & 0x60) ==0x60)	{
 						U1THR=value;
					}
					else	{
						uart1tx[3] = value;
						uart1txdone[3] = 1;
					}
				break;
				//--------------------------------
			}
		break;
		//-----------------------------------
		case COM2:
			switch(variable)	{
				//--------------------------------
				case 1:
					if((U2LSR & 0x60) ==0x60)	{
 						U2THR=value;
					}
					else	{
						uart2tx[0] = value;
						uart2txdone[0] = 1;
					}
				break;
				//--------------------------------
				case 2:
					if((U2LSR & 0x60) ==0x60)	{
 						U2THR=value;
					}
					else	{
						uart2tx[1] = value;
						uart2txdone[1] = 1;
					}
				break;
				//--------------------------------
				case 3:
					if((U2LSR & 0x60) ==0x60)	{
 						U2THR=value;
					}
					else	{
						uart2tx[2] = value;
						uart2txdone[2] = 1;
					}
				break;
				//--------------------------------
				case 4:
					if((U2LSR & 0x60) ==0x60)	{
 						U2THR=value;
					}
					else	{
						uart2tx[3] = value;
						uart2txdone[3] = 1;
					}
				break;
				//--------------------------------
			}
		break;
		//-----------------------------------
		case COM3:
			switch(variable)	{
				//--------------------------------
				case 1:
					if((U3LSR & 0x60) ==0x60)	{
 						U3THR=value;
					}
					else	{
						uart3tx[0] = value;
						uart3txdone[0] = 1;
					}
				break;
				//--------------------------------
				case 2:
					if((U3LSR & 0x60) ==0x60)	{
 						U3THR=value;
					}
					else	{
						uart3tx[1] = value;
						uart3txdone[1] = 1;
					}
				break;
				//--------------------------------
				case 3:
					if((U3LSR & 0x60) ==0x60)	{
 						U3THR=value;
					}
					else	{
						uart3tx[2] = value;
						uart3txdone[2] = 1;
					}
				break;
				//--------------------------------
				case 4:
					if((U3LSR & 0x60) ==0x60)	{
 						U3THR=value;
					}
					else	{
						uart3tx[3] = value;
						uart3txdone[3] = 1;
					}
				break;
				//--------------------------------
			}
		break;
		//-----------------------------------
	}
}

void uart0ExtPrintbin(void)	{

	static short txcount = 0;
	unsigned short done = 1,count = 0;
	while(done && (count < 4))	{
		if(txcount == 0)	{
			txcount = 1;
			count++;
			if(uart0txdone[0] == 1)	{
				
				if( (U0LSR & 0x60) !=0x60 )	{
 					U0THR = uart0tx[0];
					uart0txdone[0] = 0;
				}
				done = 0;
			}
		}
		else if(txcount == 1)	{
			
			txcount = 2;
			count++;
			if(uart0txdone[1] == 1)	{
		
				if( (U0LSR & 0x60) !=0x60 )	{
 					U0THR = uart0tx[1];
					uart0txdone[1] = 0;
				}
				done = 0;
			}
		}
		else if(txcount == 2)	{
			
			txcount = 3;
			count++;
			if(uart0txdone[2] == 1)	{
		
				if( (U0LSR & 0x60) !=0x60 )	{
 					U0THR = uart0tx[2];
					uart0txdone[2] = 0;
				}
				done = 0;
			}
		}
		else if(txcount == 3)	{
			
			txcount = 0;
			count++;
			if(uart0txdone[3] == 1)	{
		
				if( (U0LSR & 0x60) !=0x60 )	{
 					U0THR = uart0tx[3];
					uart0txdone[3] = 0;
				}
				done = 0;
			}
		}	
	}
}

void uart1ExtPrintbin(void)	{

	static short txcount = 0;
	unsigned short done = 1,count = 0;
	while(done && (count < 4))	{

		if(txcount == 0)	{

			txcount = 1;
			count++;
			if(uart1txdone[0] == 1)	{
		
				if( (U1LSR & 0x60) !=0x60 )	{
 					U1THR = uart1tx[0];
					uart1txdone[0] = 0;
				}
				done = 0;
			}
		}
		else if(txcount == 1)	{
			
			txcount = 2;
			count++;
			if(uart1txdone[1] == 1)	{
		
				if( (U1LSR & 0x60) !=0x60 )	{
 					U1THR = uart1tx[1];
					uart1txdone[1] = 0;
				}
				done = 0;
			}
		}
		else if(txcount == 2)	{
			
			txcount = 3;
			count++;
			if(uart1txdone[2] == 1)	{
		
				if( (U1LSR & 0x60) !=0x60 )	{
 					U1THR = uart1tx[2];
					uart1txdone[2] = 0;
				}
				done = 0;
			}
		}
		else if(txcount == 3)	{
			
			txcount = 0;
			count++;
			if(uart1txdone[3] == 1)	{
		
				if( (U1LSR & 0x60) !=0x60 )	{
 					U1THR = uart1tx[3];
					uart1txdone[3] = 0;
				}
				done = 0;
			}
		}	
	}
}

void uart2ExtPrintbin(void)	{
	
	static short txcount = 0;
	unsigned short done = 1,count = 0;
	while(done && (count < 4))	{

		if(txcount == 0)	{

			txcount = 1;
			count++;
			if(uart2txdone[0] == 1)	{
		
				if( (U2LSR & 0x60) !=0x60 )	{
 					U2THR = uart2tx[0];
					uart2txdone[0] = 0;
				}
				done = 0;
			}
		}
		else if(txcount == 1)	{
			
			txcount = 2;
			count++;
			if(uart2txdone[1] == 1)	{
		
				if( (U2LSR & 0x60) !=0x60 )	{
 					U2THR = uart2tx[1];
					uart2txdone[1] = 0;
				}
				done = 0;
			}
		}
		else if(txcount == 2)	{
			
			txcount = 3;
			count++;
			if(uart2txdone[2] == 1)	{
		
				if( (U2LSR & 0x60) !=0x60 )	{
 					U2THR = uart2tx[2];
					uart2txdone[2] = 0;
				}
				done = 0;
			}
		}
		else if(txcount == 3)	{
			
			txcount = 0;
			count++;
			if(uart2txdone[3] == 1)	{
		
				if( (U2LSR & 0x60) !=0x60 )	{
 					U2THR = uart2tx[3];
					uart2txdone[3] = 0;
				}
				done = 0;
			}
		}	
	}
}
void uart3ExtPrintbin(void)	{
	
	static short txcount = 0;
	unsigned short done = 1,count = 0;
	while(done && (count < 4))	{

		if(txcount == 0)	{

			txcount = 1;
			count++;
			if(uart3txdone[0] == 1)	{
		
				if( (U3LSR & 0x60) !=0x60 )	{
 					U3THR = uart3tx[0];
					uart3txdone[0] = 0;
				}
				done = 0;
			}
		}
		else if(txcount == 1)	{
			
			txcount = 2;
			count++;
			if(uart3txdone[1] == 1)	{
		
				if( (U3LSR & 0x60) !=0x60 )	{
 					U3THR = uart3tx[1];
					uart3txdone[1] = 0;
				}
				done = 0;
			}
		}
		else if(txcount == 2)	{
			
			txcount = 3;
			count++;
			if(uart3txdone[2] == 1)	{
		
				if( (U3LSR & 0x60) !=0x60 )	{
 					U3THR = uart3tx[2];
					uart3txdone[2] = 0;
				}
				done = 0;
			}
		}
		else if(txcount == 3)	{
			
			txcount = 0;
			count++;
			if(uart3txdone[3] == 1)	{
		
				if( (U3LSR & 0x60) !=0x60 )	{
 					U3THR = uart3tx[3];
					uart3txdone[3] = 0;
				}
				done = 0;
			}
		}	
	}
}
//-----------------------------------------------------------------------------------------------
void ConfigExtPrintbin(unsigned short channel,unsigned short variable)	
{
	switch (channel)
  	{
  		case COM0:
				U0IER |= 0x00000002;
				Enable(UART0);
				On(_U0THRE,uart0ExtPrintbin);
			break;
		case COM1:
				U1IER |= 0x00000002;
				Enable(UART1);
				On(_U1THRE,uart1ExtPrintbin);
			break;
		case COM2:
				U2IER |= 0x00000002;
				Enable(UART2);
				On(_U2THRE,uart2ExtPrintbin);
			break;
		case COM3:
				U3IER |= 0x00000002;
				Enable(UART3);
				On(_U3THRE,uart3ExtPrintbin);
			break;
	}
}
//-----------------------------------------------------------------------------------------------
void ConfigUARTFIFO(unsigned short channel,unsigned int status)	
{
	switch (channel)
	{
		case COM0:
			if(status)
				U0FCR |= 0x00000007;
			break;
		case COM1:
			if(status)
				U1FCR |= 0x00000007;
			break;
		case COM2:
			if(status)
				U2FCR |= 0x00000007;
			break;
		case COM3:
			if(status)
				U3FCR |= 0x00000007;
			break;
	}
}
//-----------------------------------------------------------------------------------------------
void Printbin(unsigned short channel,unsigned short value)
{
 	switch (channel)
  	{
  		case COM0:
    		while((U0LSR & 0x60) !=0x60);
 			U0THR=value;
			break;

  		case COM1:	
			while((U1LSR & 0x60) != 0x60);
 			U1THR=value;
			break;

		case COM2:	
			while((U2LSR & 0x60) != 0x60);
 			U2THR=value;
			break;

  		case COM3:
			while((U3LSR & 0x60) != 0x60);
 			U3THR=value;
			break;
	}
}
//-----------------------------------------------------------------------------------------------
unsigned short Inputbin(unsigned short channel)
{
	unsigned short value = 0xFFFF;
 	switch (channel)
  	{
  		case COM0:
    		while((U0LSR & 0x01) != 0x01);
 			value = U0RBR;
			break;

  		case COM1:	
			while((U1LSR & 0x01) != 0x01);
 			value = U1RBR;
			break;

		case COM2:	
			while((U2LSR & 0x01) != 0x01)
			{
				if( (IO2IntStatR&0x00002000) == 0x00002000)
				{
					enc1_status=1;
					enc1_count1-=T0TC;
					T0TC=0x00000000; 
					IO2IntStatR &= ~(0x00002000);
				}
				else if( (IO2IntStatF&0x00002000)== 0x00002000)
				{
					enc1_status=2; 			
					enc1_count1+=T0TC;
					T0TC=0x00000000; 			
					IO2IntStatF &= ~(0x00002000);
				}
				if( (IO2IntStatR&0x00001000) == 0x00001000)
				{
					enc3_status=1;
					enc3_count1-=T1TC;
					T1TC=0x00000000; 			
					IO2IntStatR &= ~(0x00001000);
				}
				else if( (IO2IntStatF&0x00001000)== 0x00001000)
				{
					enc3_status=2; 			
					enc3_count1+=T1TC;
					T1TC=0x00000000; 			
					IO2IntStatF &= ~(0x00001000);
				}				
			}
 			value = U2RBR;
			break;

  		case COM3:
			while((U3LSR & 0x01) != 0x01);
 			value = U3RBR;
			break;
	}
	
	return value;
}



unsigned short Inputbin2(unsigned short channel)
{
	unsigned short value = 0xFFFF;
 	switch (channel)
  	{
  		case COM0:
    		if((U0LSR & 0x01) == 0x01)
					value = U0RBR;
			break;

  		case COM1:	
				if((U1LSR & 0x01) == 0x01)
					value = U1RBR;
			break;

		case COM2:	
				if((U2LSR & 0x01) == 0x01)
				value = U2RBR;
			break;

  		case COM3:
				if((U3LSR & 0x01) == 0x01)
					value = U3RBR;
			break;
	}
	return value;
}

//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
