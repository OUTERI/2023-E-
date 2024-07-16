#ifndef __Actuator_H_ 
#define __Actuator_H_
#include "tim.h"


typedef struct 
{
	float x;
	float y;
	float vx;
	float vy;	
	float angel_x;
	float angel_y;
}actuator_gesture;

typedef struct
{
	TIM_HandleTypeDef *tim;
	uint16_t channel;
}tim_channel;

typedef struct
{
	int task_flag;
	int detect_flag;
	int object_state[3];
	int shoot_state;
	char led_state;
	int buzzer_state;
	char shoot_targest[6];
}state;

typedef struct
{
	float position[6][2];  //a,b,c,d,o ,¼¤¹â   x,y
	float step[5][2];    //step_x step_y 
	int delay_time;
	
}move;

void init_actuator();
void cal_PID();
void detect();
void set_buzlasled();
#endif