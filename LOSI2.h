#ifndef __LOSI2_H__
#define __LOSI2_H__

#include <stdint.h>
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_tim.h"
#include "delay.h"
#include "stm32f4xx_usart.h"
#include "misc.h"
#include "tm_stm32f4_mpu6050.h"

#define LED1_OFF GPIO_SetBits(GPIOB, GPIO_Pin_5)
#define LED1_ON GPIO_ResetBits(GPIOB, GPIO_Pin_5)
#define LED1_TOGGLE GPIO_ToggleBits(GPIOB, GPIO_Pin_5)

#define FRONT_GREEN_ON GPIO_SetBits(GPIOB, GPIO_Pin_15)
#define FRONT_GREEN_OFF GPIO_ResetBits(GPIOB, GPIO_Pin_15)
#define FRONT_GREEN_TOGGLE GPIO_ToggleBits(GPIOB, GPIO_Pin_15)
#define LEFT_GREEN_ON GPIO_SetBits(GPIOB, GPIO_Pin_7)
#define LEFT_GREEN_OFF GPIO_ResetBits(GPIOB, GPIO_Pin_7)
#define LEFT_GREEN_TOGGLE GPIO_ToggleBits(GPIOB, GPIO_Pin_7)
#define RIGHT_GREEN_ON GPIO_SetBits(GPIOC, GPIO_Pin_14)
#define RIGHT_GREEN_OFF GPIO_ResetBits(GPIOC, GPIO_Pin_14)
#define RIGHT_GREEN_TOGGLE GPIO_ToggleBits(GPIOC, GPIO_Pin_14)
#define FRONT_RED_ON GPIO_SetBits(GPIOC, GPIO_Pin_0)
#define FRONT_RED_OFF GPIO_ResetBits(GPIOC, GPIO_Pin_0)
#define FRONT_RED_TOGGLE GPIO_ToggleBits(GPIOC, GPIO_Pin_0)
#define LEFT_RED_ON GPIO_SetBits(GPIOC, GPIO_Pin_1)
#define LEFT_RED_OFF GPIO_ResetBits(GPIOC, GPIO_Pin_1)
#define LEFT_RED_TOGGLE GPIO_ToggleBits(GPIOC, GPIO_Pin_1)
#define RIGHT_RED_ON GPIO_SetBits(GPIOC, GPIO_Pin_10)
#define RIGHT_RED_OFF GPIO_ResetBits(GPIOC, GPIO_Pin_10)
#define RIGHT_RED_TOGGLE GPIO_ToggleBits(GPIOC, GPIO_Pin_10)

#define BUTTON_STATE GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8)
#define   TURN_LEFT (*((uint32_t *)(0x40000400u+0x40u))) //stuure pa kreisi
#define   TURN_RIGHT (*((uint32_t *)(0x40000400u+0x3cu))) //stuure pa labi
#define   MOVE_FORWARD (*((uint32_t *)(0x40000400u+0x38u))) //uz prieksu
#define   MOVE_BACK (*((uint32_t *)(0x40000400u+0x34u))) //atpakalj
#define LEFT 1
#define RIGHT 0
#define STRAIGHT 2
#define MAX 1
#define MIN 0
#define ANGLE adc1[2]/2 
#define BRAKE if(sec>sec_del){break;}
#define MAX_STRLEN 4 // this is the maximum string length of our string in characters
#define DRIVE 1
#define STOP 0
#define SPEED1 2
#define SPEED2 3
#define HILL 4
#define PAUSE 5
#define TURN_180 6
#define TURN_90_LEFT 7
#define TURN_90_RIGHT 8
#define BACK 1
#define FRONT 0
#define CURRENT adc1[1]
#define BRAKE if(sec>sec_del){break;} //get out of while if sec>sec_del
void init_losi2(void);//specific to losi2
void init_GPIOB(void);
void init_GPIOA(void);
void init_GPIOC(void);
void init_Interrupts(void);
void init_DMA2(void);
void init_ADC1(void);
void init_TIM3(void);

void set_angle(uint8_t,uint16_t);
void toggle_led_sec(uint32_t);

void set_direction(uint8_t, uint16_t);
void trn_lft(void);
void trn_rght(void);
void init_USART1(uint32_t);
void  update_IMU(void);
void set_speed(uint8_t,uint8_t);
void calibrate_IMU(void);
#endif