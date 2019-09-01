/*
#include"IAP.h"

unsigned long command[5];
unsigned long result[5];

typedef void (*IAP)(unsigned int [],unsigned int[]);
IAP IapEntry;

void EnableIAP(void)	{

	IapEntry = (IAP) IAP_LOCATION;
}
*/
/*
 * IAP In-System Application Programming Demo
 */

#include <LPC17xx.h>
#include"lpc17xx_1.h"
#include"pll.h"
#include"IAP.h"
// Clock Frequency

#define XTAL   12000000                      // Oscillator Frequency

#ifdef BYPASS_IAP
#define CPUCLK  XTAL                         // CPU Clock without PLL
#else
#define CPUCLK  (XTAL*10)                     // CPU Clock with PLL
#endif

#define CCLK   (XTAL / 1000)                 // CPU Clock without PLL in kHz


// Phase Locked Loop (PLL) definitions
#define        PLL_BASE        0x400FC080  // PLL Base Address 
#define        PLLCON_OFS      0x00        // PLL Control Offset
#define        PLLSTAT_OFS     0x08        // PLL Status Offset 
#define        PLLFEED_OFS     0x0C        // PLL Feed Offset 
#define        PLLCON_PLLE     0x01		   // PLL Enable 
#define 	   PLLCON_PLLD	   0x00		   // PLL Disable
#define        PLLCON_PLLC     0x03		   // PLL Connect(0x02) | PLL Enable
#define        PLLSTAT_PLOCK   0x0400	   //1<<10 // PLL Lock Status



  unsigned long command[5];
  
/* Default Interrupt Function: may be called when interrupts are disabled */
void def_isr (void) __irq  {
 ;
}
void StopPLL(void)  
{
	 PLL0FEED=0xAA;
	 PLL0FEED=0x55;
	 PLL0CON=0x00;
	 PLL0FEED=0xAA;
	 PLL0FEED=0x55;

}
											
									 

/*
 * Convert 'addr' to sector number
 */
unsigned int Getsecnum (void *addr)  {
  unsigned int n;
  if((unsigned int)addr <=0x00FFF)
  {
     n=0;
  }
  else if((unsigned int)addr <=0x01FFF)
  {
     n=1;
  }
  else if((unsigned int)addr <=0x02FFF)
  {
     n=2;
  }
  else if((unsigned int)addr <=0x03FFF)
  {
     n=3;
  }
  else if((unsigned int)addr <=0x04FFF)
  {
     n=4;
  }
  else if((unsigned int)addr <=0x05FFF)
  {
     n=5;
  }
  else if((unsigned int)addr <=0x06FFF)
  {
     n=6;
  }
  else if((unsigned int)addr <=0x07FFF)
  {
     n=7;
  }
  else if((unsigned int)addr <=0x08FFF)
  {
     n=8;
  }
  else if((unsigned int)addr <=0x09FFF)
  {
     n=9;
  }
  else if((unsigned int)addr <=0x0AFFF)
  {
     n=10;
  }
  else if((unsigned int)addr <=0x0BFFF)
  {
     n=11;
  }
  else if((unsigned int)addr <=0x0CFFF)
  {
     n=12;
  }
  
  else if((unsigned int)addr <=0x0DFFF)
  {
     n=13;
  }
  else if((unsigned int)addr <=0x0EFFF)
  {
     n=14;
  }
  else if((unsigned int)addr <=0x0FFFF)
  {
     n=15;
  }
  else if((unsigned int)addr <=0x17FFF)
  {
     n=16;
  }
  else if((unsigned int)addr <=0x1FFFF)
  {
     n=17;
  }
  else if((unsigned int)addr <=0x27FFF)
  {
     n=18;
  }
  else if((unsigned int)addr <=0x2FFFF)
  {
     n=19;
  }
  else if((unsigned int)addr <=0x37FFF)
  {
     n=20;
  }
  else if((unsigned int)addr <=0x3FFFF)
  {
     n=21;
  }
  else if((unsigned int)addr <=0x47FFF)
  {
     n=22;
  }
  else if((unsigned int)addr <=0x4FFFF)
  {
     n=23;
  }
  else if((unsigned int)addr <=0x57FFF)
  {
     n=24;
  }
  else if((unsigned int)addr <=0x5FFFF)
  {
     n=25;
  }		 
  else if((unsigned int)addr <=0x67FFF)
  {	   
     n=26;
  }
  else if((unsigned int)addr <=0x6FFFF)
  {	   
     n=27;
  }
  else if((unsigned int)addr <=0x77FFF)
  {
     n=28;
  }
  else if((unsigned int)addr <=0x7FFFF)
  {
     n=29;
  }

  return (n);                                   // sector number
}


/*
 * Erase Sector between 'start' and 'end'
 * Return:  IAP error code (0 when OK)
 * NOTES:  start needs to be a 256 byte boundary
 *         size should be 256, 512, 1024 or 4089
 */						  



 unsigned int Erase(void* start, void* end)  {
 
 
  unsigned long result[16];                 // IAP results
  unsigned int save_ISER0,save_ISER1;                // for saving of interrupt enable register

	IAP iap_entry;
 	iap_entry = (IAP) IAP_LOCATION;

  save_ISER0 = ISER0;              // save interrupt enable status
  save_ISER1 = ISER1;
  ICER0=0xFFFFFFFF;                // disable all interrupts
  ICER1=0xFFFFFFFF;


  StopPLL();                              // IAP requires to run without PLL


  command[0] = 50;                            // IAP Command: Prepare Sectors for Write
  command[1] = Getsecnum (start);         // start sector
  command[2] = Getsecnum (end);           // end sector
  iap_entry (command, result);                // call IAP function
  if (result[0])  goto exit;               // an error occured?

  command[0] = 52;                            // IAP command: Erase Flash
  command[1] = Getsecnum (start);         // start sector
  command[2] = Getsecnum (end);           // end sector
  command[3] = CCLK;                       // CPU clock
  iap_entry (command, result);                // call IAP function
  exit:
  InitPLL();                             // start PLL


  ISER0 = save_ISER0;              // enable interrupts
  ISER1 = save_ISER1;
  return (result[0]);
}


/*
 * Program *data to flash_addr. number of bytes specified by size
 * Return:  IAP error code (0 when OK)
 * Note: 
 */
unsigned int Program (void *flash_addr, void *data, unsigned int size)  {
  unsigned long result[16];                 // IAP results
  unsigned int save_ISER0,save_ISER1;                // for saving of interrupt enable register

   IAP iap_entry;
 	iap_entry = (IAP) IAP_LOCATION;

  save_ISER0 = ISER0;              // save interrupt enable status
  save_ISER1 = ISER1;
  ICER0=0xFFFFFFFF;                // disable all interrupts
  ICER1=0xFFFFFFFF;

#ifdef BYPASS_IAP
  StopPLL();                              // IAP requires to run without PLL
#endif

   command[0] = 50;                            // IAP Command: Prepare Sectors for Write
   command[1] = Getsecnum (flash_addr);    // start sector
   command[2] = command[1];                 // end Sektor
  iap_entry (command, result);                // call IAP function
  if (result[0])  goto exit;               // an error occured?

  command[0] = 51;                            // IAP Command: Copy RAM to Flash
  command[1] = (unsigned int) flash_addr;  // destination-addr
  command[2] = (unsigned int) data;        // source-addr
  command[3] = size;                       // number of bytes
  command[4] = CCLK;                       // CPU clock
  iap_entry (command, result);                // call IAP function

exit:

#ifdef BYPASS_IAP
  InitPLL();                             // start PLL
#endif


  ISER0 = save_ISER0;              // enable interrupts
  ISER1 = save_ISER1;
  return (result[0]);
}


					
unsigned char vals[512];

//KILL!!!!!!!!!!!!!!!!!!!!!!!! ;)  CRP BY ERASING MEMORY LOCATION 0x000002FC
 

/*
void main (void)  {
  unsigned int i;

  unsigned int volatile start;

  for (start = 0; start < 1000000; start++) {
    ;    // wait for debugger connection (about 0.3 sec)
  }

  VICDefVectAddr = (unsigned int) def_isr;       // for spurious interrupts

  for (i = 0; i < sizeof (vals); i++)  {
    vals[i] = (unsigned char) i;
  }

  program (0x30000, vals, sizeof (vals));
  program (0x31000, vals, sizeof (vals));
  program (0x32000, vals, sizeof (vals));
  erase   (0x30000, 0x31FFF);
  erase   (0x32000, 0x33FFF);

  while (1);
}

  */
