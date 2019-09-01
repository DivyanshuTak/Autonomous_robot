#ifndef _IAP_H
#define _IAP_H

#include "LPC17xx_1.h"

#define IAP_LOCATION 0x1FFF1FF1

typedef void (*IAP) (unsigned long command[], unsigned long result[]);

unsigned int Getsecnum (void *addr);
unsigned int Erase (void* start, void* end);
unsigned int Program (void *flash_addr, void *data, unsigned int size);
#endif
