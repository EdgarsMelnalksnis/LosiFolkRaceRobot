#include "LOSI2_math.h"

extern volatile uint16_t received_string[MAX_STRLEN+1];
extern uint16_t adc1[9];
extern uint16_t hill_cntr;
uint16_t frnt_low[9]={0};
uint16_t frnt_low_average[6]={0};
uint16_t frnt_low_middle[3]={0};

uint16_t frnt_up[9]={0};
uint16_t frnt_up_middle[3]={0};
uint16_t frnt_up_average[6]={0};


uint16_t frnt_left[9]={0};
uint16_t frnt_left_middle[3]={0};
uint16_t frnt_left_average[6]={0};

uint16_t frnt_right[9]={0};
uint16_t frnt_right_middle[3]={0};
uint16_t frnt_right_average[6]={0};

uint16_t rght[9]={0};
uint16_t rght_middle[3]={0};
uint16_t rght_average[6]={0};

uint16_t lft[9]={0};
uint16_t lft_middle[3]={0};
uint16_t lft_average[6]={0};
volatile uint16_t front_low_average=0,front_up_average=0,front_left_average=0,front_right_average=0,right_average=0,left_average=0;
volatile uint16_t front_low=0,front_up=0,front_left=0,front_right=0,right=0,left=0;


uint16_t crnt[9]={0}, current=0;

void update_steering(void)
{
  if(received_string[1]>45 && received_string[1]<55)
 {
   set_direction(STRAIGHT,0);
 }
 else if (received_string[1]>35 && received_string[1]<45)
 {
   set_direction(LEFT,10);
 }
  else if (received_string[1]>25 && received_string[1]<35)
 {
   set_direction(LEFT,20);
 }
   else if (received_string[1]>15 && received_string[1]<25)
 {
   set_direction(LEFT,30);
 }
  else if (received_string[1]>5 && received_string[1]<15)
 {
   set_direction(LEFT,40);
 }
  else if (received_string[1]<5)
 {
   set_direction(LEFT,60);
 }
  else if (received_string[1]>55 && received_string[1]<65)
 {
   set_direction(RIGHT,10);
 }
  else if (received_string[1]>65 && received_string[1]<75)
 {
   set_direction(RIGHT,20);
 }
   else if (received_string[1]>75 && received_string[1]<85)
 {
   set_direction(RIGHT,30);
 }
  else if (received_string[1]>85 && received_string[1]<95)
 {
   set_direction(RIGHT,40);
 }
  else if (received_string[1]>95)
 {
   set_direction(RIGHT,60);
 } 
  
}
void update_distance_sensors(void)
{//math
  frnt_low[8]=frnt_low[7];
  frnt_low[7]=frnt_low[6];
  frnt_low[6]=frnt_low[5];
  frnt_low[5]=frnt_low[4];
  frnt_low[4]=frnt_low[3];
  frnt_low[3]=frnt_low[2];
  frnt_low[2]=frnt_low[1];
  frnt_low[1]=frnt_low[0];
  frnt_low[0]=adc1[3];
  frnt_low_middle[0]=middle_of_3(frnt_low[0],frnt_low[1],frnt_low[2]);
  frnt_low_middle[1]=middle_of_3(frnt_low[3],frnt_low[4],frnt_low[5]);
  frnt_low_middle[2]=middle_of_3(frnt_low[6],frnt_low[7],frnt_low[8]);
  front_low_average=middle_of_3(frnt_low_middle[0],frnt_low_middle[1],frnt_low_middle[2]);
  
   frnt_low_average[5]= frnt_low_average[4];
   frnt_low_average[4]= frnt_low_average[3];
   frnt_low_average[3]= frnt_low_average[2];
  frnt_low_average[2]= frnt_low_average[1];
   frnt_low_average[1]= frnt_low_average[0];
  frnt_low_average[0]=front_low_average;
  front_low=((frnt_low_average[0]+frnt_low_average[1]+frnt_low_average[2]+frnt_low_average[3]+frnt_low_average[4]+frnt_low_average[5])/6);
  
  
  frnt_up[8]=frnt_up[7];
  frnt_up[7]=frnt_up[6];
  frnt_up[6]=frnt_up[5];
  frnt_up[5]=frnt_up[4];
  frnt_up[4]=frnt_up[3];
  frnt_up[3]=frnt_up[2];
  frnt_up[2]=frnt_up[1];
  frnt_up[1]=frnt_up[0];
  frnt_up[0]=adc1[4];
  frnt_up_middle[0]=middle_of_3(frnt_up[0],frnt_up[1],frnt_up[2]);
  frnt_up_middle[1]=middle_of_3(frnt_up[3],frnt_up[4],frnt_up[5]);
  frnt_up_middle[2]=middle_of_3(frnt_up[6],frnt_up[7],frnt_up[8]);
  front_up_average=middle_of_3(frnt_up_middle[0],frnt_up_middle[1],frnt_up_middle[2]);
  
  
   frnt_up_average[5]= frnt_up_average[4];
   frnt_up_average[4]= frnt_up_average[3];
   frnt_up_average[3]= frnt_up_average[2];
  frnt_up_average[2]= frnt_up_average[1];
   frnt_up_average[1]= frnt_up_average[0];
  frnt_up_average[0]=front_up_average;
  front_up=((frnt_up_average[0]+frnt_up_average[1]+frnt_up_average[2]+frnt_up_average[3]+frnt_up_average[4]+frnt_up_average[5])/6);
  
  
  
  frnt_right[8]=frnt_right[7];
  frnt_right[7]=frnt_right[6];
  frnt_right[6]=frnt_right[5];
  frnt_right[5]=frnt_right[4];
  frnt_right[4]=frnt_right[3];
  frnt_right[3]=frnt_right[2];
  frnt_right[2]=frnt_right[1];
  frnt_right[1]=frnt_right[0];
  frnt_right[0]=adc1[6];
  frnt_right_middle[0]=middle_of_3(frnt_right[0],frnt_right[1],frnt_right[2]);
  frnt_right_middle[1]=middle_of_3(frnt_right[3],frnt_right[4],frnt_right[5]);
  frnt_right_middle[2]=middle_of_3(frnt_right[6],frnt_right[7],frnt_right[8]);
  front_right_average=middle_of_3(frnt_right_middle[0],frnt_right_middle[1],frnt_right_middle[2]);
  
   frnt_right_average[5]= frnt_right_average[4];
   frnt_right_average[4]= frnt_right_average[3];
   frnt_right_average[3]= frnt_right_average[2];
  frnt_right_average[2]= frnt_right_average[1];
   frnt_right_average[1]= frnt_right_average[0];
  frnt_right_average[0]=front_right_average;
  front_right=((frnt_right_average[0]+frnt_right_average[1]+frnt_right_average[2]+frnt_right_average[3]+frnt_right_average[4]+frnt_right_average[5])/6);
  
  
  
  
  
  frnt_left[8]=frnt_left[7];
  frnt_left[7]=frnt_left[6];
  frnt_left[6]=frnt_left[5];
  frnt_left[5]=frnt_left[4];
  frnt_left[4]=frnt_left[3];
  frnt_left[3]=frnt_left[2];
  frnt_left[2]=frnt_left[1];
  frnt_left[1]=frnt_left[0];
  frnt_left[0]=adc1[5];
  frnt_left_middle[0]=middle_of_3(frnt_left[0],frnt_left[1],frnt_left[2]);
  frnt_left_middle[1]=middle_of_3(frnt_left[3],frnt_left[4],frnt_left[5]);
  frnt_left_middle[2]=middle_of_3(frnt_left[6],frnt_left[7],frnt_left[8]);
  front_left_average=middle_of_3(frnt_left_middle[0],frnt_left_middle[1],frnt_left_middle[2]);
  
   frnt_left_average[5]= frnt_left_average[4];
   frnt_left_average[4]= frnt_left_average[3];
   frnt_left_average[3]= frnt_left_average[2];
  frnt_left_average[2]= frnt_left_average[1];
   frnt_left_average[1]= frnt_left_average[0];
  frnt_left_average[0]=front_left_average;
  front_left=((frnt_left_average[0]+frnt_left_average[1]+frnt_left_average[2]+frnt_left_average[3]+frnt_left_average[4]+frnt_left_average[5])/6);
  
  
  
  
  
rght[8]=rght[7];
rght[7]=rght[6];
rght[6]=rght[5];
rght[5]=rght[4];
rght[4]=rght[3];
rght[3]=rght[2];
rght[2]=rght[1];
rght[1]=rght[0];
rght[0]=adc1[8];
rght_middle[0]=middle_of_3(rght[0],rght[1],rght[2]);
rght_middle[1]=middle_of_3(rght[3],rght[4],rght[5]);
rght_middle[2]=middle_of_3(rght[6],rght[7],rght[8]);
right_average=middle_of_3(rght_middle[0],rght_middle[1],rght_middle[2]);
 
   rght_average[5]= rght_average[4];
   rght_average[4]= rght_average[3];
   rght_average[3]= rght_average[2];
  rght_average[2]= rght_average[1];
   rght_average[1]= rght_average[0];
  rght_average[0]=right_average;
 right=((rght_average[0]+rght_average[1]+rght_average[2]+rght_average[3]+rght_average[4]+rght_average[5])/6);




lft[8]=lft[7];
lft[7]=lft[6];
lft[6]=lft[5];
lft[5]=lft[4];
lft[4]=lft[3];
lft[3]=lft[2];
lft[2]=lft[1];
lft[1]=lft[0];
lft[0]=adc1[7];
lft_middle[0]=middle_of_3(lft[0],lft[1],lft[2]);
lft_middle[1]=middle_of_3(lft[3],lft[4],lft[5]);
lft_middle[2]=middle_of_3(lft[6],lft[7],lft[8]);
left_average=middle_of_3(lft_middle[0],lft_middle[1],lft_middle[2]); 
  
   lft_average[5]= lft_average[4];
   lft_average[4]= lft_average[3];
   lft_average[3]= lft_average[2];
   lft_average[2]= lft_average[1];
   lft_average[1]= lft_average[0];
   lft_average[0]=left_average;
 left=((lft_average[0]+lft_average[1]+lft_average[2]+lft_average[3]+lft_average[4]+lft_average[5])/6);

 crnt[8]=crnt[7];
  crnt[7]=crnt[6];
 crnt[6]=crnt[5];
  crnt[5]=crnt[4];
crnt[4]=crnt[3];
  crnt[3]=crnt[2];
  crnt[2]=crnt[1];
  crnt[1]=crnt[0];
  crnt[0]=adc1[1];
  current=(crnt[0]+crnt[1]+crnt[2]+crnt[3]+crnt[4]+crnt[5]+crnt[6]+crnt[7]+crnt[8])/9;
}

uint8_t check_hill(void)
{
  uint8_t hill_state=0;
 
  if((front_up<1500) &&(((front_low>front_up+500)&& front_low>1200) ||((front_low>front_up+300)&& front_low>900)||((front_low>front_up+150)&& front_low>600)))
  {
  hill_cntr++;
if(hill_cntr>=20)
{
  hill_state=1;
  hill_cntr=0;
}
  }
  else 
  {
    if(hill_cntr>0)
    {
    hill_cntr--;
    }
    else 
    {
    hill_cntr=0;  
    }
    
  }
  
  
  return hill_state;
}



uint16_t middle_of_3(uint16_t a, uint16_t b, uint16_t c)
{
 uint16_t middle;

 if ((a <= b) && (a <= c))
 {
   middle = (b <= c) ? b : c;
 }
 else if ((b <= a) && (b <= c))
 {
   middle = (a <= c) ? a : c;
 }
 else
 {
   middle = (a <= b) ? a : b;
 }
 return middle;
}


void array_set(float *array,uint8_t val,uint8_t number)
{
  uint8_t cntr=0;
  for(cntr=0;cntr<number;cntr++)
  {
    array[cntr]=val;
    
  }
    
  
}
