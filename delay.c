#include "delay.h"
#include "stm32f4xx_exti.h"
#include "losi2.h"

uint32_t m_sec=0,sec=0;
uint8_t update_sensors=0;
extern uint8_t update_IMU_sensors;
extern float dist;
float dist1=0;
int16_t pitch=0;
extern float distance[30];
extern float distance1[10];
extern uint8_t i;
extern float angle_1sec;
extern float angle_2sec;
extern int16_t angle1sec;
extern int16_t angle2sec;
extern int16_t gyro_z;
extern int16_t gyro_y;
extern uint32_t mode_cntr;
extern uint8_t mode;
extern  int16_t offset_gyro_y;
int16_t pitch_angle[3]={0};
int16_t pitch_delta=0,pitch_last=0;
uint8_t on_hill=0;//to check if veichle is on hill
uint32_t hill_sec=0;
void del(uint delay)
{
  while(delay >1)
  {
  delay--;
  }
}

void ms_delay(uint32_t miliseconds)
{
  uint32_t counter=m_sec;
  while(m_sec<counter+miliseconds)
  {
  }
  
}

void SysTick_Handler(void)//systc should be enabled
{
    m_sec++;
    if(m_sec%1000==0)//1000ms
    {
     sec++;
    
     pitch_angle[2]=pitch_angle[1];
     pitch_angle[0]=pitch;
     pitch_delta=pitch_last-(pitch+pitch_angle[0]+pitch_angle[1]+pitch_angle[2]);
     pitch_last=pitch+pitch_angle[0]+pitch_angle[1]+pitch_angle[2];
     if(pitch_delta>15)//needs more to check.. for example raw accelerometer output
     {
     on_hill=1; //when to reset?
    hill_sec=sec;
     }
     else if(pitch_delta<-20)
     {
       on_hill=0;
     }
     
     
    
     
    }
    if(m_sec%100==0)
    {
      update_IMU_sensors=1;

    dist=(gyro_z/32.8)*0.1;
    dist1+=((gyro_y- offset_gyro_y)/32.8)*0.1;
    //pitch+=dist1;
   pitch=dist1;
  if(mode!=TURN_180)
  {
    angle_1sec=0;
   
    for(i=0;i<9;i++)
    {
      distance1[i]=distance1[i+1];
      angle_1sec+=distance1[i];
    }
distance1[9]=dist;
angle1sec=angle_1sec;

if((angle1sec<-110 && angle1sec>-250) || (angle1sec>=140&& angle1sec<=250))
{
  mode=TURN_180;
}
else if(angle1sec>110&& angle1sec<140)
{
  mode=TURN_90_RIGHT;
}

else if(angle1sec<-110 && angle1sec>-140  ) 
{
  mode=TURN_90_LEFT;
}
  }


    
    
    
    
    
  if((mode==SPEED1 || mode==SPEED2) && mode_cntr<10)
  {
  angle_2sec=0;
    for(i=0;i<19;i++)
    {
      distance[i]=distance[i+1];
      angle_2sec+=distance[i];
    }
distance[19]=dist;
angle2sec=angle_2sec;
if((angle2sec<-160 && angle2sec>-270) || (angle2sec>160 && angle2sec<270) ) 
{
  mode=TURN_180;
}
  }

    }
    if(m_sec%10==0)
    {
      update_sensors=1;
    }
 
    
}