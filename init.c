#include "init.h"
#include "delay.h"


extern uint16_t adc1[9];//ADC measurments
void RCC_init(void)
{
(*((uint *)(0x40023800u+0x30u))) |= ((1<<0)|(1<<1)|(1<<2)|1<<22);//GPIOA, GPIOC, DMA2 enable
(*((uint *)(0x40023800u+0x40u))) |= (1<<1);//tim3 enable
(*((uint *)(0x40023800u+0x44u))) |= (1<<8);//ADC1 enable

}
void GPIOA_init(void)
{
 (*((uint *)(0x40020000u))) |= (1<<0|1<<22|1<<20|1<<24|1<<8|1<<9|1<<10|1<<11);//analog mode PA4,PA5
(*((uint *)(0x40020000u+0x04u))) |= (1<<0|1<<11|1<<10|1<<12);//open drain PA0
}

void GPIOB_init(void)
{
 (*((uint *)(0x40020400u))) |= (1<<10|1<<2|1<<3|1<<18|1<<24);//PB5,PB9,PB12 output, PB1 analog
 (*((uint *)(0x40020400u+0x0cu))) |= (1<<14|1<<16);//PB7 and PB8 pull up
 (*((uint *)(0x40020400u+0x04u))) |= (1<<5);//open drain PB5
}
void GPIOC_init(void)
{//base 0x40020800
  //pc6 tim3_ch1
  //pc7 tim3_ch2
  //pc8 tim3_ch3
  //pc9 tim3_ch4
  //pc0 ADC1_10
  //pc1 ADC1_11
  //pc2 ADC1_12
  (*((uint *)(0x40020800u))) |= (1<<20|1<<22|1<<24|1<<26);//PC10,PC11,PC12,PC13 output
  (*((uint *)(0x40020800u+0x24u))) |= (1<<1|1<<5);//AF02 for C8,C9
(*((uint *)(0x40020800u+0x20u))) |= (1<<25|1<<29);//AF02 for C8,C9
(*((uint *)(0x40020800u))) |= (1<<0|1<<1|1<<2|1<<3|1<<4|1<<5|1<<8|1<<9|1<<10|1<<11|1<<13|1<<15|1<<17|1<<19);//analog mode c0,c1,c2,c4,c5 alternate functions for ports C6,C7,C8,C9


}

void TIM3_init(void)
{
    //base 0x40000400
  //(*((uint *)(0x40000400u+0x28u))) |= 1000;//TIM3 prescaler
  (*((uint *)(0x40000400u+0x14u))) |= (1<<1|1<<2|1<<3|1<<4);//can modify ch1..ch4
  (*((uint *)(0x40000400u+0x18u))) |= (1<<11|1<<13|1<<14|1<<3|1<<5|1<<6);//OC2PE, OC2M, OC1PE, 
  (*((uint *)(0x40000400u+0x1cu))) |= (1<<11|1<<13|1<<14|1<<3|1<<5|1<<6);//OC3PE, OC3M, OC4PE, OC4M.
  (*((uint *)(0x40000400u+0x20u))) |= (1<<0|1<<4|1<<8|1<<12);//capture/compare output enable
  (*((uint *)(0x40000400u+0x00u))) |= 1<<7; //ARPE enable
  (*((uint *)(0x40000400u+0x2cu))) = 100;//pwm frekvence
    (*((uint *)(0x40000400u+0x28u))) = 10;//timer prescaler 10
  
/*
  //(*((uint *)(0x40000400u+0x28u))) |= 1000;//TIM3 prescaler
  (*((uint *)(0x40000400u+0x14u))) |= (1<<3);//modify ch3
  (*((uint *)(0x40000400u+0x1cu))) |=(1<<3|1<<5|1<<6);//CCMR2
  (*((uint *)(0x40000400u+0x20u))) |= (1<<8);
  (*((uint *)(0x40000400u+0x00u))) |= 1<<7; //ARPE enable
 */

}

void ADC1_init(void)
{//4c offset for 16bit DR
(*((uint *)(0x40012000u+0x8u))) |= 1<<0;//ADC enable, DMA enable
//(*((uint *)(0x40012000u+0x304u))) |=(1<<16);//ADC clock prescaler 6
(*((uint *)(0x40012000u+0x8u))) |= (1<<1|1<<8|1<<9);//CONTINUOUS mode, DMA enable,DDS set
(*((uint *)(0x40012000u+0x04u))) |= 1<<8;//scan mode
(*((uint *)(0x40012000u+0x0cu))) |= (1<<2|1<<5|1<<8|1<<11|1<<14|1<<17);//84cycle uz kanalu..nezinu kam to vajag :D
(*((uint *)(0x40012000u+0x10u))) |=(1<<2|1<<5|1<<8|1<<11|1<<14|1<<17);
(*((uint *)(0x40012000u+0x2cu))) |= 0x700000;//6 convertions   
//(*((uint *)(0x40012000u+0x34u))) |= (1<<2|1<<7|1<<5|1<<13|1<<11|1<<10|1<<18|1<17|1<<24|1<<23|1<<22|1<<29|1<<28|1<<27|1<<26|1<<25);//ch4,ch5,ch11,ch12,ch14,ch15
(*((uint *)(0x40012000u+0x34u))) |=0x0AE62C89;// 0x1EE62CA4;
(*((uint *)(0x40012000u+0x30u))) |=0x8F;//ADC1_ch9 7 in sequence
}

void DMA2_init(void)
{
 (*((uint *)(0x40026400u+0x18u))) = (uint)0x4001204cu; //set ADC1_DR port address DMA_SxPAR
 (*((uint *)(0x40026400u+0x1cu))) = (uint)&adc1[0] ;// SRAM adrese..vajag aspkatiities, vai stack kkur tur neparaskta
 (*((uint *)(0x40026400u+0x14u))) = 8;//number of data to be transferred 
 (*((uint *)(0x40026400u+0x10u))) |=(1<<8|1<<10|1<<11|1<<13); //(1<<8|1<<10|1<<12|1<<14);//circular mode,memory pointer is incremented,32 bit peripheral and 32 bit memory 
(*((uint *)(0x40026400u+0x10u))) |= 1<<0;//starts DMA
  
}

void Interrupt_init()
{
  //base 0x40013C00
  //(*((uint *)(0x40013C00u+0x0cu))) |= (1<<4|1<<5);//falling edge enable
  EXTI_InitTypeDef EXTI_InitStruct;
  NVIC_InitTypeDef NVIC_InitStruct;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource7);
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource8);
     
  /* PA  is connected to EXTI_Line */
    EXTI_InitStruct.EXTI_Line = EXTI_Line7 + EXTI_Line8;
    /* Enable interrupt */
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    /* Interrupt mode */
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    /* Triggers on rising and falling edge */
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
    /* Add to EXTI */
    EXTI_Init(&EXTI_InitStruct);
  


      NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn ;//PB7 and PB8
    /* Set priority */
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
    /* Set sub priority */
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
    /* Enable interrupt */
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    /* Add to NVIC */
    NVIC_Init(&NVIC_InitStruct);
  

    
    //sysTick config
     SysTick_Config(SystemCoreClock / 1000);//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
}
