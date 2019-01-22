//rev2 to rev1 -> improve steering + buttons + current measurments+ hill mode
//rev4 to rev5-> improve turn around and hill mode
//rev5 to rev6 -> calibration + improved hill climbing
//adc1[1] min 800-> low batt !!!!!!!!!!
#include <stdint.h>
#include <float.h>
#include <math.h>
#include "delay.h"
#include "LOSI2.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "LOSI2_math.h"
//#include "tm_stm32f4_mpu6050.h"
//#include <stm32f4xx.h>

extern uint32_t sec;
extern uint32_t m_sec;
extern volatile uint16_t received_string[MAX_STRLEN+1];
extern uint8_t update_sensors; //to update steering and sensors
extern volatile uint16_t front_low,front_up,front_left,front_right,right,left;
uint16_t angle1=0;
uint32_t m_sec_led;
uint8_t mode=STOP;
uint32_t mode_cntr=0;//to know when swich from STOP to SPEED1
uint8_t last_mode=STOP;//to remember last mode
uint16_t hill_cntr=0;//to flter noise
uint16_t hill_cntr2=0;//to filter noise when robot is in angle
int16_t accel_x=0;
int16_t accel_y=0;
int16_t accel_z=0;
int16_t gyro_x=0;
int16_t gyro_y=0;
int16_t gyro_z=0;
int16_t temp=0;
int16_t gyro_y_avg[3]={0};
int16_t accel_z_average=0;
int16_t accel_z_avg[3]={0};
int16_t accel_x_avg[3]={0};
int16_t accel_y_avg[3]={0};
TM_MPU6050_t MPU6050_Data;
uint8_t update_IMU_sensors=0;
 float final_velocity=0,initial_velocity=0,dist=0;
 float distance[30]={0};
 float distance1[10]={0};
 int16_t offset=0;
 uint8_t i=0;
 float angle_1sec=0;
 float angle_2sec=0;
 int16_t angle1sec=0;
 int16_t angle2sec=0;
 uint8_t turn_around=0;
 uint8_t speed=0;//set speed 
 uint16_t stucked=0;//to check if car is stucked (speed>0 but its not moving)
 extern uint16_t current;
 extern uint16_t adc1[]; 
 extern uint16_t crnt[];
extern uint8_t on_hill;
extern uint32_t hill_sec;
int main()
{

  
 init_losi2();

(*((uint *)(0x40012000u+0x8u))) |=1<<30;//start ADC


TIM_Cmd(TIM3, ENABLE);//start counter



received_string[1]=50;
 //set_direction(STRAIGHT,0);



TM_MPU6050_Init(&MPU6050_Data, TM_MPU6050_Device_1, TM_MPU6050_Accelerometer_8G, TM_MPU6050_Gyroscope_1000s);


calibrate_IMU();//should take 1 to 2 sec !!! 



//while(mode==PAUSE);
mode=STOP;
while(1)
  {
   // if((received_string[3]==1) || (mode==PAUSE ))
    if(mode==PAUSE )
{
    mode=PAUSE;
   
     speed=0;
     set_speed(speed, FRONT);
}
else
{
    if(update_sensors==1)
    {
 update_sensors=0;
 //update_steering();
 update_distance_sensors();
 update_IMU();

 if((speed<=70 && current >135))
 //if(speed>500)//uncomment to enter here
 {
   stucked++;
   if(stucked> 70)
   {
     stucked=0;
     current=0;
     for(i=0;i<9;i++)
     {
       crnt[i]=0;
     }
     
     set_speed(140, FRONT);
     //set_direction(STRAIGHT,0);
    // speed=80;
    // set_speed(speed,BACK);
     ms_delay(150);
  // speed=0;
  // set_speed(speed,FRONT);
   }
   
 }
 else 
 {
   stucked=0;
 }
 
 if(check_hill()==1)
 {
   last_mode=mode;//to remember mode before
  mode=HILL;

  
 }
  //---------------------------------------HILL------------------------------------
 if(mode==HILL)
 {//how to move in HILL mode??
  
 if(front_left>front_right+100)
 {
   set_direction(LEFT,30);
 }
 else if(front_right>front_left+100)
 {
   set_direction(RIGHT,30);
     
 }
 else 
 {
   set_direction(STRAIGHT,0);
 }
   if(front_up<2000)
   {
     //MOVE_FORWARD=140;
       speed=140;
     set_speed(speed, FRONT);
   }
  
 if(check_hill()==0)
 {
   mode=SPEED1;

 }
 }
 //-----------------------------------------TURN_180-------------------------------
 else if (mode==TURN_180)
 {
   array_set(distance,0,20);
   array_set(distance1,0,10);
   angle1sec=0;
   angle2sec=0;
       speed=0;
     set_speed(speed, FRONT);
     
   set_direction(RIGHT,60);//set steering
   ms_delay(200);
   array_set(distance,0,20);
    
       speed=70;
     set_speed(speed, BACK);//move
     ms_delay(1000);//improve here to check angle and current
  
       speed=0;
     set_speed(speed, FRONT);//stop
   ms_delay(100);
      set_direction(LEFT,50);//set steering
  // ms_delay(100);
   speed=70;
   set_speed(speed,FRONT);
   ms_delay(800);
   speed=0;
   set_speed(speed,FRONT);
    array_set(distance,0,20);
   array_set(distance1,0,10);
   angle1sec=0;
   angle2sec=0;
   mode=STOP;
 }
 //----------------------------------------TURN_90 LEFT or RIGHT---------------------------
 else if(mode==TURN_90_LEFT || mode==TURN_90_RIGHT)
 {
   
     array_set(distance,0,20);
   array_set(distance1,0,10);
   angle1sec=0;
   angle2sec=0;
      speed=0;
     set_speed(speed, FRONT);
   
     if(mode==TURN_90_LEFT)
     {
   set_direction(RIGHT,60);//set steering
   ms_delay(200);
   array_set(distance1,0,10);
     }
     else
     {
        set_direction(LEFT,60);//set steering
   ms_delay(200);
   array_set(distance1,0,10); 
     }
     
        speed=70;
     set_speed(speed, BACK);//move
     ms_delay(600);//improve here to check angle and current
     speed=0;
     set_speed(speed,FRONT);
      ms_delay(100);
        
      if(mode==TURN_90_LEFT)
     {
   set_direction(LEFT,60);//set steering
   ms_delay(200);

     }
     else
     {
        set_direction(RIGHT,60);//set steering
   ms_delay(200);
  
     }
     
   speed=70;
   set_speed(speed,FRONT);
   ms_delay(600);
   speed=0;
   set_speed(speed,FRONT);
      
 
       
         array_set(distance,0,20);
   array_set(distance1,0,10);
   angle1sec=0;
   angle2sec=0;
     mode=STOP; 
 }
   
   
   //---------------------------------------STOP------------------------------------
else if(mode==STOP)
 {
if((front_low>1600) && (front_left>1600 || front_right>1600))
{
  mode_cntr=0;
  
       speed=0;
     set_speed(speed, FRONT);
  
     ms_delay(100);
     //MOVE_BACK=60;
       speed=60;
     set_speed(speed, BACK);
//check side sensors..if neccesary adjust steering
 if(left>right)
 {
  
   set_direction(LEFT,60);
 }
 else 
 {
  
   set_direction(RIGHT,60);
 }
  
 while(front_low>1500)
 {
     speed=80;
     set_speed(speed, BACK);
   update_distance_sensors();
 }
     speed=0;
     set_speed(speed, FRONT);
  set_direction(STRAIGHT,0);
 ms_delay(50);
}
 else
 {
 
  speed=70;
     set_speed(speed, FRONT);
 mode_cntr++;
 if(mode_cntr>=50)
 {
   mode=SPEED1;
   mode_cntr=0;
 }
 }
 
 
   
 }
   //---------------------------------------SPEED1------------------------------------
 else if(mode==SPEED1)
 {
 if((front_low>1400) && (front_left>1400 || front_right>1400))
 {
  
     speed=0;
     set_speed(speed, FRONT);
   mode=STOP;
   mode_cntr=0;
 }
 else
 {
   if((front_left>front_right+200) && (front_left>1000))
   {
     mode_cntr=0;
    if(front_left<1100)
    {set_direction(RIGHT,20);}
    else if(front_left<1300)
    {set_direction(RIGHT,40);}
    else 
    {set_direction(RIGHT,60);}
   }
   else if((front_right>front_left+200) && (front_right>1000))
   {//should go left..or straigt
     mode_cntr=0;
     if(front_right<1500)
    {set_direction(LEFT,20);}
    else if(front_right<1700)
    {set_direction(LEFT,40);}
    else 
    {set_direction(LEFT,60);}
   }
     else 
     {
       set_direction(STRAIGHT,0);
       mode_cntr++;
     }
 }
 if(mode_cntr>50)
 {
   mode=SPEED2;
   mode_cntr=0;
 }
 
 }
 //----------------------------------------------------------------------------------
 
 
  //---------------------------------------SPEED2------------------------------------
 else if(mode==SPEED2)
 {
 if((front_low>1100) && (front_left>1100 || front_right>1100))
 {

     speed=0;
     set_speed(speed, FRONT);
   mode=STOP;
   mode_cntr=0;
   ms_delay(2);
  
  //   speed=80;
  //   set_speed(speed, BACK);
  // ms_delay(50);
 
     speed=0;
     set_speed(speed, FRONT);
   
 }
 else//should be improved
 {
   if((front_left>front_right+200) && (front_left>1000))
   {
     mode=SPEED1;
    if(front_left<1100)
    {set_direction(RIGHT,20);}
    else if(front_left<1300)
    {set_direction(RIGHT,40);}
    else 
    {set_direction(RIGHT,60);}
   }
   else if((front_right>front_left+200) && (front_right>900))
   {//should go left..or straigt
     mode=SPEED1;
     if(front_right<1300)
    {set_direction(LEFT,20);}
    else if(front_right<1500)
    {set_direction(LEFT,40);}
    else 
    {set_direction(LEFT,60);}
   }
     else 
     {
       set_direction(STRAIGHT,0);
      // mode_cntr++;
     }
 }

 
 }
 
 //-----------------------------------------------------------------------------------
 
    
    }
    
    
    //check if is stucked
    if(update_IMU_sensors==1)
    {
      update_IMU_sensors=0;
  update_IMU();
/*
 if(accel_z>3650 && accel_z<3900 && accel_x>800 && accel_x<1400)
{
   hill_cntr2++;
  if(hill_cntr2>5)
  {
    hill_cntr2=0;
    //MOVE_FORWARD=150;
   FRONT_RED_ON;
   LEFT_RED_ON;
   RIGHT_RED_ON;
   FRONT_GREEN_OFF;
   LEFT_GREEN_OFF;
   RIGHT_GREEN_OFF;
   
   if(front_up<2000)
   {  
   //MOVE_FORWARD=140;
     speed=140;
     set_speed(speed, FRONT);
ms_delay(100);
  speed=0;
     set_speed(speed, FRONT);
   }
   
  } 
}
else
{
  hill_cntr2=0;

} 
      */
      
      
    }
    
    
if(on_hill==1)
{
  if(sec-hill_sec>2)
  {
    on_hill=0;
  }
  else
  {
  
     if(accel_z>3650 && accel_z<3900 && accel_x>800 && accel_x<1400)
{
   hill_cntr2++;
  if(hill_cntr2>2)
  {
    hill_cntr2=0;
    //MOVE_FORWARD=150;
   FRONT_RED_ON;
   LEFT_RED_ON;
   RIGHT_RED_ON;
   FRONT_GREEN_OFF;
   LEFT_GREEN_OFF;
   RIGHT_GREEN_OFF;
   
   if(front_up<2000)
   {  
   //MOVE_FORWARD=140;
     speed=140;
     set_speed(speed, FRONT);
ms_delay(200);
  speed=0;
     set_speed(speed, FRONT);
   }
   
  } 
}
else
{
  hill_cntr2=0;

} 
    
    
  }
}
else
{
  
   FRONT_RED_OFF;
   LEFT_RED_OFF;
   RIGHT_RED_OFF;
   FRONT_GREEN_OFF;
   LEFT_GREEN_OFF;
   RIGHT_GREEN_OFF;
}
    
}



 if(mode==PAUSE)
 {
     FRONT_RED_OFF;
   LEFT_RED_OFF;
   RIGHT_RED_OFF;
   FRONT_GREEN_OFF;
   LEFT_GREEN_OFF;
   RIGHT_GREEN_OFF;
 }
 
 else if(mode==STOP)
 {
   FRONT_RED_ON;
   LEFT_RED_OFF;
   RIGHT_RED_OFF;
   FRONT_GREEN_OFF;
   LEFT_GREEN_OFF;
   RIGHT_GREEN_OFF;
 }
 else if(mode==SPEED1)
 {
     FRONT_RED_OFF;
   LEFT_RED_ON;
   RIGHT_RED_ON;
   FRONT_GREEN_ON;
   LEFT_GREEN_OFF;
   RIGHT_GREEN_OFF;


     speed=60;
     set_speed(speed, FRONT);
/*
 if(accel_z>3650 && accel_z<3900 && accel_x>1200 && accel_x<1400)
{
  hill_cntr2++;
  if(hill_cntr2>100)
  {
    hill_cntr2=0;
  //  MOVE_FORWARD=150;

    ms_delay(200);
  }
}
else
{
  hill_cntr2=0;

}
*/
 }
 else if (mode==SPEED2)
 {
      FRONT_RED_OFF;
   LEFT_RED_OFF;
   RIGHT_RED_OFF;
   FRONT_GREEN_ON;
   LEFT_GREEN_ON;
   RIGHT_GREEN_ON;
     speed=70;
     set_speed(speed, FRONT);
 }
 else if(mode==HILL)
 {
   FRONT_RED_OFF;
   LEFT_RED_OFF;
   RIGHT_RED_OFF;
   FRONT_GREEN_ON;
   LEFT_GREEN_ON;
   RIGHT_GREEN_ON;
   
 }
 else if(mode==TURN_180)
{
  
  
}




  }
  return 0;
}


