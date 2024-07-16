#include "CAR.h"
#include "JY901.h"
#include "usart.h"
#include "math.h"
#include "PID.h"
#include "Actuator.h"
#include "Communicate.h"
#define initspeed 4
#define starbias 40
#define turnspeed  3
//移植代码时需要注意判断左右轮对应 
//编码器对应的是定时器2和8
//TW1对应 left  TW2 对应right
car_tim_channel wheel_left={&htim4,TIM_CHANNEL_1,GPIOB,GPIO_PIN_4,TIM8};
car_tim_channel wheel_right={&htim4,TIM_CHANNEL_2,GPIOB,GPIO_PIN_5,TIM2};

flag  carflag={0};
flag  *pr_carflag = &carflag;

extern Rx_Str *pr_RxS1 ;
messg car_messg={0};
messg *pr_msg = &car_messg;

gesture car_curgest;
gesture car_targest;

pidstruct v1pid;
pidstruct v2pid;
pidstruct x1pid;
pidstruct x2pid;
pidstruct angelzpid;


void  get_angel()
{
	static int angeltime=0;
	static float angel[3]={0};
	angeltime++;
	if(angeltime>4)
	{
		angel[1]=angel[0];
		angel[0]=JY901_data.angle.angle[2];
		angel[2]=angel[0]-angel[1];
		car_curgest.angel+=angel[2];
		angeltime=0;
	}
}


short get_pos(car_tim_channel wheel)    //获取编码器的值  逆时针增加
{
   short Encoder_TIM;    
	 Encoder_TIM = (short)wheel.ENCODER -> CNT;   	
	 return Encoder_TIM;
}



void Wheel(car_tim_channel wheel,int pwm)      //通过PWM信号直接控制电机转速
{
	int direct;
	if(pwm>1000) pwm=1000;
	if(pwm<-1000) pwm=-1000;
	
	if(pwm>=0)  direct=1;
	if(pwm<0)   direct=0;
	if(car_targest.v1==0&&car_targest.v2==0)
	{
		__HAL_TIM_SET_COMPARE(wheel.tim,wheel.channel,999);
		HAL_GPIO_WritePin(wheel.GPIO_BASE,wheel.GPIO_PIN,1);
	}
	if(wheel.channel==TIM_CHANNEL_1)
	{
		switch(direct){
			case 0:
				__HAL_TIM_SET_COMPARE(wheel.tim,wheel.channel,1000+pwm);//w1_pwm_out
				HAL_GPIO_WritePin(wheel.GPIO_BASE,wheel.GPIO_PIN,1);//w3_gpio_out
				break;
			case 1:
				__HAL_TIM_SET_COMPARE(wheel.tim,wheel.channel,pwm);//w1_pwm_out
				HAL_GPIO_WritePin(wheel.GPIO_BASE,wheel.GPIO_PIN,0);//w2_gpio_out
				break;
		}
	}
	if(wheel.channel==TIM_CHANNEL_2)
	{
			switch(direct){
				case 0:
					__HAL_TIM_SET_COMPARE(wheel.tim,wheel.channel,-pwm);//w2_pwm_out
					HAL_GPIO_WritePin(wheel.GPIO_BASE,wheel.GPIO_PIN,0);//w2_gpio_out
					break;
				case 1:
					__HAL_TIM_SET_COMPARE(wheel.tim,wheel.channel,1000-pwm);//w2_pwm_out
					HAL_GPIO_WritePin(wheel.GPIO_BASE,wheel.GPIO_PIN,1);//w2_gpio_out
					break;
			}
  }
	
}



void calculate()        //计算速度和距离  同时给pwm赋值
{
	static int speedtime=0;
	static float distance1[3]={0};
	static float distance2[3]={0};
	speedtime++;
	if(speedtime>4)
	{
		 distance1[1]=distance1[0];
		 distance1[0]=get_pos(wheel_left);
		 distance1[2]=distance1[0]-distance1[1];
		 if(distance1[2]<-32768) distance1[2]+=65535;
		 if(distance1[2]>32767) distance1[2]-=65535;
		 car_curgest.v1=distance1[2]/20;
		 car_curgest.x1+=distance1[2]/2;
		
		 distance2[1]=distance2[0];
		 distance2[0]=get_pos(wheel_right);
		 distance2[2]=distance2[0]-distance2[1];
		 if(distance2[2]<-32768) distance2[2]+=65535;
		 if(distance2[2]>32767) distance2[2]-=65535;
		 car_curgest.v2=-distance2[2]/20;
		 car_curgest.x2-=distance2[2]/2;
		
		 speedtime=0;
	}
	
	 car_targest.pwm_v1+=Incremental_PID(&v1pid,car_targest.v1-car_curgest.v1,500,-500);
	 car_targest.pwm_v2+=Incremental_PID(&v2pid,car_targest.v2-car_curgest.v2,500,-500);
	 Wheel(wheel_left,car_targest.pwm_v1);
	 Wheel(wheel_right,car_targest.pwm_v2);
}


void follow()                //巡线
{
	 static float flag=0;
	 static float lastangel=0;
	 if(car_targest.action_flag=='g')
	 {
		 if(car_messg.bias==160) car_messg.bias=40;
		 car_targest.v1=initspeed-PID(&angelzpid,starbias-car_messg.bias,-3,3);
		 car_targest.v2=initspeed+PID(&angelzpid,starbias-car_messg.bias,-3,3);
	 }
	 if(car_targest.action_flag=='s')
	 {
		  car_targest.v1=0;
		  car_targest.v2=0;
	 }
	 if(car_targest.action_flag=='t')
	 {
	
		 car_targest.v1=turnspeed;
		 car_targest.v2=-turnspeed;
		 flag++;
		 if(flag<=4) lastangel=car_curgest.angel;
		 if(fabs(car_curgest.angel-lastangel)>150&&fabs(car_curgest.angel-lastangel)<185)
		 {
			  car_targest.action_flag='s';
		 }
		
		 
	 }
}






//void car_init()      //小车的初始化函数
//{
//   car_targest.action_flag='s';
//	 car_targest.v1=initspeed;
//	 car_targest.v2=initspeed;
//	 car_curgest.x1=0;
//	 car_curgest.x2=0;
//	 car_curgest.angel=0;
//	 v1pid=initpid(50,4.0,0.005,v1pid);
//	 v2pid=initpid(45,6.5,0.02,v2pid);
//	 angelzpid=initpid(0.09,0,0.015,angelzpid);
//	 
//	 car_messg.bias=starbias;
//	 Wheel(wheel_left,200);
//	 Wheel(wheel_right,200);
//	
//	 HAL_Delay(3000);
//}






