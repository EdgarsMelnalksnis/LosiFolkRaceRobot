#include "LOSI2.h"




uint16_t adc1[9]={0,0,0,0,0,0,0,0,0};//make new file for global variables
extern uint32_t sec;
static uint8_t cnt = 0;
volatile uint16_t received_string[MAX_STRLEN+1]; // this will hold the recieved string
volatile char update=0;//toupdate usart
extern TM_MPU6050_t MPU6050_Data;

extern int16_t accel_x;
extern int16_t accel_y;
extern int16_t accel_z;
extern int16_t gyro_x;
extern int16_t gyro_y;
extern int16_t gyro_z;
extern int16_t temp;
extern int16_t gyro_y_avg[];
extern int16_t accel_z_average;
extern int16_t accel_z_avg[];
extern int16_t accel_x_avg[];
extern int16_t accel_y_avg[];
extern uint8_t mode;
extern int16_t offset;
int16_t offset_gyro_y=0,offset_gyro_z=0,offset_gyro_x=0;

void init_losi2(void)
{
 init_GPIOB(); 
 init_GPIOA();  
 init_GPIOC();
init_Interrupts();
init_ADC1(); 
init_DMA2();
init_TIM3();
init_USART1(9600); // initialize USART1 @ 9600 baud
}
void init_GPIOB(void)
{
    //configure GPIOB
  GPIO_InitTypeDef GPIO_InitDef;//structure
   GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
  NVIC_InitTypeDef NVIC_InitStruct;
  
  //enable clock
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
  

  GPIO_InitDef.GPIO_Pin=GPIO_Pin_5;
  GPIO_InitDef.GPIO_Mode=GPIO_Mode_OUT;
  GPIO_InitDef.GPIO_OType=GPIO_OType_OD;
  GPIO_InitDef.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitDef);
  
      GPIO_InitStruct.GPIO_Pin = (GPIO_Pin_7 | GPIO_Pin_15);
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Mode =GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
  
     GPIO_InitStruct.GPIO_Pin = (GPIO_Pin_13 | GPIO_Pin_14);
   // GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    //GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
     
    GPIO_InitStruct.GPIO_Mode =GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    
  GPIO_InitDef.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1);
  GPIO_InitDef.GPIO_Mode=GPIO_Mode_AN;
  GPIO_Init(GPIOB, &GPIO_InitDef);//pin 0, 1, 2 analog
    

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource13);
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource14);
     
  /* PA  is connected to EXTI_Line */
    EXTI_InitStruct.EXTI_Line = EXTI_Line14 + EXTI_Line13;
    /* Enable interrupt */
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    /* Interrupt mode */
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    /* Triggers on rising and falling edge */
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
    /* Add to EXTI */
    EXTI_Init(&EXTI_InitStruct);
  


      NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn ;//PB7 and PB8
    /* Set priority */
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
    /* Set sub priority */
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
    /* Enable interrupt */
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    /* Add to NVIC */
    NVIC_Init(&NVIC_InitStruct);
  
  
  
}
void init_GPIOA(void)
{
  //enable clock
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
  
  //configure GPIOB
  GPIO_InitTypeDef GPIO_InitDef;//structure
  
  GPIO_InitDef.GPIO_Pin=GPIO_Pin_8;
  GPIO_InitDef.GPIO_Mode=GPIO_Mode_IN;
  GPIO_InitDef.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitDef);//pin 8 input
  
  GPIO_InitDef.GPIO_Pin=(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6);
  GPIO_InitDef.GPIO_Mode=GPIO_Mode_AN;
  GPIO_Init(GPIOA, &GPIO_InitDef);//pin 0, 1, 2 analog
}

void init_GPIOC(void)
{
   GPIO_InitTypeDef GPIO_InitStruct;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);

    
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOC, &GPIO_InitStruct);
    
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_14  ;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Mode =GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOC, &GPIO_InitStruct);
    

    
    
    
    
}
void init_Interrupts(void)
{
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//systick is here??
  //sysTick config
     SysTick_Config(SystemCoreClock / 1000);
}




void init_ADC1(void)
{
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
   ADC_InitTypeDef ADC_InitStructure;
   
 //  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
   ADC_InitStructure.ADC_Resolution=ADC_Resolution_12b;
   ADC_InitStructure.ADC_ScanConvMode = ENABLE;
 //  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
   ADC_InitStructure.ADC_ContinuousConvMode=ENABLE;
   ADC_InitStructure.ADC_DataAlign=ADC_DataAlign_Right;
   ADC_InitStructure.ADC_NbrOfConversion=9;
   ADC_Init(ADC1, &ADC_InitStructure);
   
   ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_56Cycles);
   ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_56Cycles);
   ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_56Cycles);
     ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_56Cycles);
   ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_56Cycles);
   ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_56Cycles);
     ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 7, ADC_SampleTime_56Cycles);
   ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 8, ADC_SampleTime_56Cycles);
   ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 9, ADC_SampleTime_56Cycles);
   
   ADC_Cmd(ADC1, ENABLE);//enable ADC1
   ADC_DMACmd(ADC1, ENABLE); //enable DMA for ADC
  // ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void init_DMA2(void)
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
 (*((uint32_t *)(0x40026400u+0x18u))) = (uint32_t)0x4001204cu; //set ADC1_DR port address DMA_SxPAR
 (*((uint32_t *)(0x40026400u+0x1cu))) = (uint32_t)&adc1[0] ;// SRAM adrese..vajag aspkatiities, vai stack kkur tur neparaskta
 (*((uint32_t *)(0x40026400u+0x14u))) = 9;//number of data to be transferred 
 (*((uint32_t *)(0x40026400u+0x10u))) |=(1<<8|1<<10|1<<11|1<<13); //(1<<8|1<<10|1<<12|1<<14);//circular mode,memory pointer is incremented,32 bit peripheral and 32 bit memory 
(*((uint32_t *)(0x40026400u+0x10u))) |= 1<<0;//starts DMA
// DMA_Cmd(DMA2_Channel1, ENABLE);
}

void init_TIM3(void)
{
 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
/*
  (*((uint32_t *)(0x40000400u+0x14u))) |= (1<<1|1<<2|1<<3|1<<4);//can modify ch1..ch4
  (*((uint32_t *)(0x40000400u+0x18u))) |= (1<<11|1<<13|1<<14|1<<3|1<<5|1<<6);//OC2PE, OC2M, OC1PE, 
  (*((uint32_t *)(0x40000400u+0x1cu))) |= (1<<11|1<<13|1<<14|1<<3|1<<5|1<<6);//OC3PE, OC3M, OC4PE, OC4M.
  (*((uint32_t *)(0x40000400u+0x20u))) |= (1<<0|1<<4|1<<8|1<<12);//capture/compare output enable
  (*((uint32_t *)(0x40000400u+0x00u))) |= 1<<7; //ARPE enable
  (*((uint32_t *)(0x40000400u+0x2cu))) = 200;//pwm frekvence
  (*((uint32_t *)(0x40000400u+0x28u))) = 10;//timer prescaler 10
*/

  TIM_ARRPreloadConfig(TIM3, ENABLE);//auto reload preload
  TIM_UpdateDisableConfig(TIM3, DISABLE);
 
 TIM_TimeBaseInitTypeDef TIM3_BaseStruct;
  TIM3_BaseStruct.TIM_Prescaler=50;
  TIM3_BaseStruct.TIM_CounterMode=TIM_CounterMode_Up;
  TIM3_BaseStruct.TIM_Period=200;//should be calculated
  TIM3_BaseStruct.TIM_ClockDivision=TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM3, &TIM3_BaseStruct);
    

  
  TIM_OCInitTypeDef TIM_OCStruct;
  TIM_OCStruct.TIM_OCMode=TIM_OCMode_PWM2;
  TIM_OCStruct.TIM_OutputState=TIM_OutputState_Enable;
  TIM_OCStruct.TIM_OCPolarity=TIM_OCPolarity_Low;
  
   TIM_OCStruct.TIM_Pulse = 0; 
    TIM_OC1Init(TIM3, &TIM_OCStruct);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    
    TIM_OCStruct.TIM_Pulse = 0; 
    TIM_OC2Init(TIM3, &TIM_OCStruct);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    
    TIM_OCStruct.TIM_Pulse = 0;
    TIM_OC3Init(TIM3, &TIM_OCStruct);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    
    TIM_OCStruct.TIM_Pulse = 0; 
    TIM_OC4Init(TIM3, &TIM_OCStruct);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);


}



void toggle_led_sec(uint32_t time)
{
  if(sec%time==0)
{
  LED1_ON;
}
else
{
  LED1_OFF;
}
}


void init_USART1(uint32_t baudrate){
	
	/* This is a concept that has to do with the libraries provided by ST
	 * to make development easier the have made up something similar to 
	 * classes, called TypeDefs, which actually just define the common
	 * parameters that every peripheral needs to work correctly
	 * 
	 * They make our life easier because we don't have to mess around with 
	 * the low level stuff of setting bits in the correct registers
	 */
	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART1 initilization
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)
	
	/* enable APB2 peripheral clock for USART1 
	 * note that only USART1 and USART6 are connected to APB2
	 * the other USARTs are connected to APB1
	 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	/* enable the peripheral clock for the pins used by 
	 * USART1, PB6 for TX and PB7 for RX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	/* This sequence sets up the TX and RX pins 
	 * so they work correctly with the USART1 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOA, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers
	
	/* The RX and TX pins are now connected to their AF
	 * so that the USART1 can take over control of the 
	 * pins
	 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1); //
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	/* Now the USART_InitStruct is used to define the 
	 * properties of USART1 
	 */
	USART_InitStruct.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART1, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting
	
	
	/* Here the USART1 receive interrupt is enabled
	 * and the interrupt controller is configured 
	 * to jump to the USART1_IRQHandler() function
	 * if the USART1 receive interrupt occurs
	 */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt 
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		 // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff	

	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART1, ENABLE);
}

void set_direction(uint8_t direction, uint16_t angle)
{
  uint32_t sec_del=sec+1;
  if(angle>60) {angle=60;}
  
  if(direction==STRAIGHT || angle ==0)
  {
if(ANGLE>920)
{
  while(ANGLE>920)
  {
    trn_lft();
    BRAKE; 
  }
}
else
{
  while(ANGLE<840)
  {
    trn_rght();
    BRAKE;
  }
  }
  }
  else if(direction==LEFT)
  {
if(angle==60)
{
  while(ANGLE>320)
  {
    trn_lft();
    BRAKE;
  }
  
}
else if(angle==50)
{
if(ANGLE>440)
{
  while(ANGLE>440)
  {
    trn_lft();
    BRAKE;
  }
}
else
{
  while(ANGLE<360)
  {
    trn_rght();
    BRAKE;
  }
}
}
else if(angle==40)
{
if(ANGLE>520)
{
  while(ANGLE>520)
  {
    trn_lft();
    BRAKE;
  }
  }
else
{
  while(ANGLE<440)
  {
    trn_rght();
    BRAKE;
  }
}
}
else if(angle==30)
{
if(ANGLE>600)
{
  while(ANGLE>600)
  {
    trn_lft();
    BRAKE;
  }
}
else
{
  while(ANGLE<520)
  {
    trn_rght();
    BRAKE;
  }
}
}
else if(angle==20)
{
if(ANGLE>680)
{
  while(ANGLE>680)
  {
    trn_lft();
    BRAKE;
  }
}
else
{
  while(ANGLE<600)
  {
    trn_rght();
    BRAKE;
  }
}
}
else if(angle==10)
{
if(ANGLE>750)
{
  while(ANGLE>750)
  {
    trn_lft();
    BRAKE;
  }
}
else
{
  while(ANGLE<680)
  {
    trn_rght();
    BRAKE;
  }
}
}
  }
  
  else if(direction==RIGHT)
  {
if(angle==60)
{
  while(ANGLE<1270)
  {
    trn_rght();
    BRAKE;
  }
  
}
else if(angle==50)
{
if(ANGLE>1240)
{
  while(ANGLE>1240)
  {
    trn_lft();
    BRAKE;
  }
  }
else
{
  while(ANGLE<1160)
  {
    trn_rght();
    BRAKE;
  }
}
}
else if(angle==40)
{
if(ANGLE>1160)
{
  while(ANGLE>1160)
  {
    trn_lft();
    BRAKE;
  }
}
else
{
  while(ANGLE<1080)
  {
    trn_rght();
    BRAKE;
  }
}
}
else if(angle==30)
{
if(ANGLE>1080)
{
  while(ANGLE>1080)
  {
    trn_lft();
    BRAKE;
  }
}
else
{
  while(ANGLE<1000)
  {
    trn_rght();
    BRAKE;
  }
}
}
else if(angle==20)
{
if(ANGLE>1000)
{
  while(ANGLE>1000)
  {
    trn_lft();
    BRAKE;
  }
}
else
{
  while(ANGLE<920)
  {
    trn_rght();
    BRAKE;
  }
}
}
else if(angle==10)
{
if(ANGLE>920)
{
  while(ANGLE>920)
  {
    trn_lft();
    BRAKE;
  }
}
else
{
  while(ANGLE<840)
  {
    trn_rght();
    BRAKE;
  }
}
}
  }  
}

void trn_lft(void)
{
   GPIO_SetBits(GPIOC,GPIO_Pin_9);//turn left
 ms_delay(3);
 GPIO_ResetBits(GPIOC,GPIO_Pin_9);
 ms_delay(20);
  
}
void trn_rght(void)
{
 GPIO_SetBits(GPIOC,GPIO_Pin_8);//turn left
 ms_delay(3);
 GPIO_ResetBits(GPIOC,GPIO_Pin_8);
 ms_delay(20);
  
}

void  update_IMU(void)
{
       TM_MPU6050_ReadAll(&MPU6050_Data);
    	accel_x=MPU6050_Data.Accelerometer_X;
	accel_y=MPU6050_Data.Accelerometer_Y;
	accel_z=MPU6050_Data.Accelerometer_Z;
	gyro_x=MPU6050_Data.Gyroscope_X;
	gyro_y=MPU6050_Data.Gyroscope_Y;
	gyro_z=MPU6050_Data.Gyroscope_Z;
	temp=MPU6050_Data.Temperature;

        
        gyro_y_avg[2]=gyro_y_avg[1];
        gyro_y_avg[1]=gyro_y_avg[0];
        gyro_y_avg[0]=gyro_y;
gyro_y=(gyro_y_avg[0]+gyro_y_avg[1]+gyro_y_avg[2])/3;
        
        
        
        accel_z_avg[2]=accel_z_avg[1];
        accel_z_avg[1]=accel_z_avg[0];
        accel_z_avg[0]=accel_z;
accel_z=(accel_z_avg[0]+accel_z_avg[1]+accel_z_avg[2])/3;

        accel_x_avg[2]=accel_x_avg[1];
        accel_x_avg[1]=accel_x_avg[0];
        accel_x_avg[0]=accel_x;
accel_x=(accel_x_avg[0]+accel_x_avg[1]+accel_x_avg[2])/3;

        accel_y_avg[2]=accel_y_avg[1];
        accel_y_avg[1]=accel_y_avg[0];
        accel_y_avg[0]=accel_y;
accel_y=(accel_y_avg[0]+accel_y_avg[1]+accel_y_avg[2])/3;
}




void USART1_IRQHandler(void){
	
	// check if the USART1 receive interrupt flag was set
	if( USART_GetITStatus(USART1, USART_IT_RXNE) ){
		
		 // this counter is used to determine the string length
		uint16_t t = USART1->DR; // the character from the USART1 data register is saved in t
		
		/* check if the received character is not the LF character (used to determine end of string) 
		 * or the if the maximum string length has been been reached 
		 */
		if( (cnt < MAX_STRLEN) ){ 
                
			received_string[cnt] = t;
			cnt++;
		}
		else{ // otherwise reset the character counter and print the received string
			received_string[cnt] = t;
                  cnt = 0;
			//USART_puts(USART1, received_string);
		}
                update=1;
	}
}

void EXTI15_10_IRQHandler(void)
{
  if (EXTI_GetITStatus(EXTI_Line13) != RESET)
  {
        /* Do your stuff when PB13 is changed */
 if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)==0)
 {
 mode=STOP;

 }

    
    
        EXTI_ClearITPendingBit(EXTI_Line13);
    }
   
  if (EXTI_GetITStatus(EXTI_Line14) != RESET)
  {
        /* Do your stuff when PB14 is changed */

 if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)==0)
 {
 mode=PAUSE;

 }
        EXTI_ClearITPendingBit(EXTI_Line14);
    }
}



void set_speed(uint8_t speed,uint8_t direction)
{
  if(speed>140) 
  {
    speed=140;//max speed
  }
    if(direction==BACK)
  {
   MOVE_FORWARD=0;
MOVE_BACK=speed; 
  }
  else 
  {
     MOVE_FORWARD=speed;
MOVE_BACK=0; 
  }
  
}


void calibrate_IMU(void)
{
  uint32_t counter_IMU=0;
  offset=0;
  while(counter_IMU<200)
  {
    counter_IMU++;
 update_IMU();
   offset_gyro_y+=gyro_y;
   offset_gyro_x+=gyro_x;
   offset_gyro_z+=gyro_z;
   ms_delay(10);
  }
  offset_gyro_y/=200;
  offset_gyro_z/=200;
  offset_gyro_x/=200;
}