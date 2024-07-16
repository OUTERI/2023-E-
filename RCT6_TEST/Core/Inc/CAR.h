#ifndef  __CAR_H
#define   __CAR_H
#include "main.h"
#include "tim.h"
typedef struct 
{
	 float angel;
	 float pwm_angz;			 //小车舵机pwm角度输出
   float pwm_v1;
   float pwm_v2;        //PID的输出
	 float x1;
	 float x2;
	 float distance;
	 float v1;            //right speed
	 float v2;            //left speed
	 char direction;
	 int action_flag;
}gesture;

typedef struct
{
	float bias;
	int stop_flag;
	int obj_count;
	int obj_flag;
	float obj_x[3];   //分别为0，内，外
	float obj_y[3];
}messg;


typedef struct
{
	int timeflag;
	int crossflag;
	int buzzerflag;
	int buzzertime;
}flag;

typedef struct
{
	int timeflag;
	int timecount;
}timer;

typedef struct
{
	TIM_HandleTypeDef *tim;
	uint16_t channel;
	GPIO_TypeDef *GPIO_BASE;
	uint16_t GPIO_PIN;
  TIM_TypeDef *ENCODER;
}car_tim_channel;

void car_task();
void  car_init();
void calculate();

#endif