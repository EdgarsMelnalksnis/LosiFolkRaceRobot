#ifndef __LOSI2_math_H__
#define __LOSI2_math_H__

#include <stdint.h>
#include "delay.h"
#include "LOSI2.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"

#define FRONT_LOW  //value to move back



void update_steering(void);
void update_distance_sensors(void);
uint8_t check_hill(void);
uint16_t middle_of_3(uint16_t, uint16_t, uint16_t);
void array_set(float *,uint8_t,uint8_t);
#endif