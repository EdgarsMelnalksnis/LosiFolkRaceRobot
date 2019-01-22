#ifndef __INIT_H__
#define __INIT_H__

#include <stdint.h>
#include "stm32f4xx_exti.h"
#include <stm32f4xx.h>

void RCC_init(void);//this is important!!!
void GPIOA_init(void);
void GPIOB_init(void);
void GPIOC_init(void);
void TIM3_init(void);
void ADC1_init(void);
void DMA2_init(void);
void Interrupt_init(void);
#endif