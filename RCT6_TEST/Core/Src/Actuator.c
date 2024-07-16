#include "Actuator.h"
#include "main.h"
#include "tim.h"
#include "Communicate.h"
#include "PID.h"
#include "CAR.h"
#include "math.h"

//U3 为x方向舵机   U4为y方向舵机 
//x轴限幅60-120   y轴限幅
#define angel_div   85  //动作分频       40
#define step_time   65 //步进时间 ms    120
#define step_angel  0.5   //步进角度

//对应x,y角度的上下限 
#define max_angelx  130  //125      
#define min_angelx  60   //55   
#define max_angely  135  //140  
#define min_angely  60   //55

//初始化对应的角度
#define initangel_x 93
#define initangel_y 100

//具体任务左右转对应的角度     
#define left_angel  123
#define right_angel 65
#define up_angel    137
#define down_angel  63

//初始坐标中心值
#define init_x       320    
#define init_y       240  

//  x轴  PB8   y轴  PB9
pidstruct xpid={0,0,0};        //位置pid   kp,ki,kd
pidstruct ypid={0,0,0};
pidstruct vxpid={0.8,0.0035,0.1};       //速度pid
pidstruct vypid={0.8,0.0035,0.1};      

float set_angel[2]={0,0};
char tasklist[8]={'0'};

move ptz_move;   //ptz_move.position[5]     t最上，r最右，b最下，l最左
actuator_gesture  ptz_curgest;  //云台当前姿态
actuator_gesture  ptz_targest;  //云台目标姿态

state actuator_state;  //云台状态

extern Rx_Str *pr_RxS1 ;
extern Rx_Str *pr_RxS2 ;

tim_channel angelx_ch={&htim3,TIM_CHANNEL_1};
tim_channel angely_ch={&htim3,TIM_CHANNEL_2};




void set_actuator(float angel,tim_channel angel_ch,int max,int min)   //舵机角度设置函数
{
	if(angel>=max)	angel=max;            //舵机角度限幅
	else if(angel<=min) angel=min;          
	int pwm=angel*10/9+50;       //建立舵机角度和pwm的关系
	__HAL_TIM_SET_COMPARE(angel_ch.tim,angel_ch.channel,pwm);
}

void init_actuator_state()
{
	actuator_state.buzzer_state=0;
	actuator_state.led_state='0';
	actuator_state.shoot_state=0;
	actuator_state.task_flag=0;
	for(int i=0;i<3;i++)
	{
		actuator_state.object_state[i]=0;
	}
	for(int i=0;i<6;i++)
	{
    actuator_state.shoot_targest[i]='z';	
	}
}

void judge_move(move *move)     //步进值计算
{
	
	  for(int i=0;i<4;i++)
	  {
			if(i<=2)
			{
				 move->step[i][0]=(move->position[i+1][0]-move->position[i][0])/angel_div; 
				 move->step[i][1]=(move->position[i+1][1]-move->position[i][1])/angel_div; 
			}
			if(i==3)
			{
				 move->step[i][0]=(move->position[0][0]-move->position[i][0])/angel_div; 
				 move->step[i][1]=(move->position[0][1]-move->position[i][1])/angel_div; 
			}
	  }
	
}

actuator_gesture initgesture(actuator_gesture *e)      //云台状态初始化函数
{
	e->x=init_x;
	e->y=init_y;
	e->vx =0;
	e->vy=0;	
	e->angel_x=initangel_x;
	e->angel_y=initangel_y;
	return *e;
}




void cal_PID()    //更新检测物体的位置  PID计算调整舵机瞄准
{
	  
		if(ptz_targest.angel_x>max_angelx) ptz_targest.angel_x=max_angelx;
		if(ptz_targest.angel_y>max_angely) ptz_targest.angel_y=max_angely;
		if(ptz_targest.angel_y<min_angely) ptz_targest.angel_y=min_angely;
		if(ptz_targest.angel_x<min_angelx) ptz_targest.angel_x=min_angelx;
	  if(actuator_state.task_flag>2&&actuator_state.task_flag<5)
		{
			ptz_curgest.x=ptz_move.position[5][0];
			ptz_curgest.y=ptz_move.position[5][1];
			ptz_targest.angel_x-=Incremental_PID(&xpid,ptz_targest.x-ptz_curgest.x,4,-4);
			ptz_targest.angel_y-=Incremental_PID(&ypid,ptz_targest.y-ptz_curgest.y,4,-4);
			set_actuator(ptz_targest.angel_x,angelx_ch,max_angelx,min_angelx);	
			set_actuator(ptz_targest.angel_y,angely_ch,max_angely,min_angely);	
		}
}

void  aim()
{
	  float angelx=0;
	  float angely=0;
	  
	  if(set_angel[0]!=0)
		{
			angelx=left_angel-((set_angel[0]-70)/180)*(left_angel-right_angel);
			angely=up_angel-((set_angel[1]-5)/180)*(up_angel-down_angel);
			set_actuator(angelx,angelx_ch,max_angelx,min_angelx);	
			set_actuator(angely,angely_ch,max_angely,min_angely);	
		}
		else
		{
			set_actuator(initangel_x,angelx_ch,max_angelx,min_angelx);	
			set_actuator(initangel_y,angely_ch,max_angely,min_angely);	
		}
}



void set_buzlasled()     //设置蜂鸣器  激光笔  红绿灯
{
	static int buzzertime=0;
	static int shoottime=0;
	if(actuator_state.buzzer_state==1 && buzzertime <= 200)
	{
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);   
		  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);   
			buzzertime++;
	}
	else
	{
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);   
		  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);   
			actuator_state.buzzer_state=0;
		  buzzertime=0;
	}
	
//	if(actuator_state.shoot_state==1 && shoottime <= 300)
//	{
//			HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);   
//			shoottime++;
//	}
//	else
//	{
//			HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);   
//			actuator_state.shoot_state=0;
//		  shoottime=0;
//	}
	
	
//	if(actuator_state.shoot_state==0)
//	{
//		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);  
//	}
	
	
}


void init_actuator()                 //舵机,初始目标点以及pid参数的初始化
{
	init_actuator_state();
	
	ptz_curgest=initgesture(&ptz_curgest);
	ptz_targest=initgesture(&ptz_targest);
	
	set_actuator(ptz_targest.angel_x,angelx_ch,max_angelx,min_angely);	
	set_actuator(ptz_targest.angel_y,angely_ch,max_angely,min_angely);	

	xpid=initpid(0.1,0.01,0.0005,xpid);   //0.1，0.012，0.005
	ypid=initpid(0.1,0.012,0.0005,ypid);   //0.1，0.016，0.005
  
	actuator_state.task_flag=0;
	HAL_UART_Receive_IT(pr_RxS1->uart, pr_RxS1->rDataBuffer, 1);    //开启串口中断
	HAL_UART_Receive_IT(pr_RxS2->uart, pr_RxS2->rDataBuffer, 1);
	HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);   
	HAL_Delay(3000);
}





void task()          //主任务函数
{
	static int taskbase[6]={0};
	static int task_time[6]={0};
	static float scan_angelx=0,scan_angely=0;
	if(actuator_state.task_flag==1)  
	{
		set_actuator(set_angel[0],angelx_ch,max_angelx,min_angelx);	
		set_actuator(set_angel[1],angely_ch,max_angely,min_angely);	
	}
	if(actuator_state.task_flag==2)  //实现屏幕扫描
	{
		 
      switch(taskbase[0])
			{
				case 0:  task_time[0]++;
								set_actuator(left_angel ,angelx_ch,max_angelx,min_angelx);	
		             set_actuator(up_angel ,angely_ch,max_angely,min_angely);	
								 if(task_time[0]>20)
								 {
									 taskbase[0]++;
									 task_time[0]=0;
									 scan_angelx=left_angel;
									 scan_angely=up_angel;
								 }
								 break;
			 case 1:  task_time[0]++;				 
								set_actuator(scan_angelx ,angelx_ch,max_angelx,min_angelx);	
		            set_actuator(up_angel ,angely_ch,max_angely,min_angely);	
								if(task_time[0]>20)
								{
									scan_angelx-= step_angel;
									task_time[0]=0;
								}
								if(scan_angelx<right_angel)
								{
									 taskbase[0]++;
									 task_time[0]=0;
									 scan_angelx=right_angel;
									 scan_angely=up_angel;
								}
								 break;
			case 2:   task_time[0]++;				 
								set_actuator(right_angel ,angelx_ch,max_angelx,min_angelx);	
		             set_actuator(scan_angely ,angely_ch,max_angely,min_angely);	
								if(task_time[0]>20)
								{
									scan_angely-= step_angel;
									task_time[0]=0;
								}
								if(scan_angely<down_angel)
								{
									 taskbase[0]++;
									 task_time[0]=0;
									 scan_angelx=right_angel;
									 scan_angely=down_angel;
								}
								 break;
			case 3:   task_time[0]++;				 
								set_actuator(scan_angelx ,angelx_ch,max_angelx,min_angelx);	
		             set_actuator(down_angel ,angely_ch,max_angely,min_angely);	
								if(task_time[0]>20)
								{
									scan_angelx+= step_angel;
									task_time[0]=0;
								}
								if(scan_angelx>left_angel)
								{
									 taskbase[0]++;
									 task_time[0]=0;
									 scan_angelx=left_angel;
									 scan_angely=down_angel;
								}
								 break;
			 case 4:   task_time[0]++;				 
								set_actuator(left_angel ,angelx_ch,max_angelx,min_angelx);	
		             set_actuator(scan_angely ,angely_ch,max_angely,min_angely);	
								if(task_time[0]>20)
								{
									scan_angely+= step_angel;
									task_time[0]=0;
								}
								if(scan_angely>up_angel)
								{
									 taskbase[0]++;
									 task_time[0]=0;
									 scan_angelx=left_angel;
									 scan_angely=up_angel;
								}
								 break;
			 case 5: 	 set_actuator(left_angel ,angelx_ch,max_angelx,min_angelx);	
		             set_actuator(up_angel ,angely_ch,max_angely,min_angely);	
								 break;
			}


	}
	if(actuator_state.task_flag==3)     //实现扫描矩形
	{
		
		switch(taskbase[1])        //taskbase[1]记录任务  taskbase[2]记录分段
		{
			
			case 0: task_time[1]++;
				      if(task_time[1]>step_time)
					    {   
									ptz_targest.x=ptz_move.position[taskbase[1]][0]+ptz_move.step[taskbase[1]][0]*taskbase[2];
									ptz_targest.y=ptz_move.position[taskbase[1]][1]+ptz_move.step[taskbase[1]][1]*taskbase[2];
                  taskbase[2]++;
								  task_time[1]=0;
								  if(taskbase[2]>angel_div)
									{
										taskbase[1]++;
										taskbase[2]=0;
									}
							}
			        break;
			case 1: task_time[1]++;
				      if(task_time[1]>step_time)
					    {
									ptz_targest.x=ptz_move.position[taskbase[1]][0]+ptz_move.step[taskbase[1]][0]*taskbase[2];
									ptz_targest.y=ptz_move.position[taskbase[1]][1]+ptz_move.step[taskbase[1]][1]*taskbase[2];
                  taskbase[2]++;
									task_time[1]=0;
								  if(taskbase[2]>angel_div)
									{
										taskbase[1]++;
										taskbase[2]=0;
									}
							}
			        break;
			case 2: task_time[1]++;
				      if(task_time[1]>step_time)
					    {
									ptz_targest.x=ptz_move.position[taskbase[1]][0]+ptz_move.step[taskbase[1]][0]*taskbase[2];
									ptz_targest.y=ptz_move.position[taskbase[1]][1]+ptz_move.step[taskbase[1]][1]*taskbase[2];
                  taskbase[2]++;
									task_time[1]=0;
								  if(taskbase[2]>angel_div)
									{
										taskbase[1]++;
										taskbase[2]=0;
									}
							}
			        break;
							
			case 3: task_time[1]++;
				      if(task_time[1]>step_time)
					    {
									ptz_targest.x=ptz_move.position[taskbase[1]][0]+ptz_move.step[taskbase[1]][0]*taskbase[2];
									ptz_targest.y=ptz_move.position[taskbase[1]][1]+ptz_move.step[taskbase[1]][1]*taskbase[2];
                  taskbase[2]++;
									task_time[1]=0;
								  if(taskbase[2]>=angel_div)
									{
										taskbase[1]++;
										taskbase[2]=0;
									}
							}
			        break;
		}
	
	}
	if(actuator_state.task_flag==4)  
	{
		 aim();
	}
	if(actuator_state.task_flag==5)     //用于一键复位
	{
		set_actuator(initangel_x,angelx_ch,max_angelx,min_angelx);	
		set_actuator(initangel_y,angely_ch,max_angely,min_angely);	
	}		
	if(actuator_state.task_flag==6)    //用于一键暂停
	{
		
	}		
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)         //定时器中断  包括任务进行 PID计算
{
	
	if(htim->Instance == TIM5)    //定时器频率1000Hz  
	{
		    task();
        judge_move(&ptz_move);
		   	if(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_ORE)) 
				{
					__HAL_UART_CLEAR_OREFLAG (&huart3);
					huart3.RxState=HAL_UART_STATE_READY;
					huart3.Lock=HAL_UNLOCKED;
					HAL_UART_Receive_IT(pr_RxS1->uart, pr_RxS1->rDataBuffer, 1); 
				}
				
	}
	if(htim->Instance == TIM7)    //定时器频率50Hz  
	{

		cal_PID();
//		set_buzlasled();
	}
	
}

//测试函数
float test_angel=90;
void test1()
{
	 for(int i=0;;i++)
	{
		set_actuator(test_angel ,angelx_ch,120,60);	
		HAL_Delay(20);
		test_angel+=0.1;
		if(test_angel>120)
		{
				break;
		}
	}
	for(int i=0;;i++)
	{
		set_actuator(test_angel ,angelx_ch,120,60);	
		HAL_Delay(20);
		test_angel-=0.1;
		if(test_angel<60)
		{
				break;
		}
	}
	  
	  
}