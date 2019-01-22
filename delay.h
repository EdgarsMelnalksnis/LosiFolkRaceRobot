#ifndef __DELAY_H__
#define __DELAY_H__

#include <stdint.h>
//#include "stm32f4xx_exti.h"


typedef volatile unsigned int uint; 

void del(uint);
void ms_delay(uint32_t);


#endif