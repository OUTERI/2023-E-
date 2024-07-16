#include "PID.h"

pidstruct initpid(float kp,float ki,float kd,pidstruct e)  //pid参数设置函数
{
	e.kp=kp;
	e.ki=ki;
	e.kd=kd;
	e.p=0;
	e.i=0;
	e.d=0;
	e.lastde=0;
	e.thisde=0;
	e.beforede=0;
	e.integralde=0;
	return e;
}


float PID(pidstruct *e,float err,float outlow,float outhigh)     //位置式PID函数
{
	float out=0;
	e->thisde=err;
  e->integralde+=e->thisde;
	
	if (e->integralde >=8) e->integralde = 8;   //积分限幅
	
	e->p = e->kp * e->thisde;
	e->i = e->ki * e->integralde;
	e->d = e->kd * (e->thisde - e->lastde);
	
	if(err>=0&&err<=3) {e->d=0; }              //微分分离
	else if(err>=-3&&err<=0) {e->d=0;}
	
	e->lastde = e->thisde;
	out = e->p + e->i + e->d;
	
	if(out>outhigh) {out=outhigh;}    //输出限幅
	if(out<outlow) {out=outlow ;}	
	
	return out;
}


float Incremental_PID(pidstruct *e,float err, float MaxOutput,float MinOutput)  //增量式PID
{
	float out;
	e->thisde=err;
  e->p = e->kp*(e->thisde - e->lastde);
	e->i = e->ki * e->thisde;
	e->d = e->kd *(e->thisde - 2.0f*e->lastde + e->beforede);	
	out= (e->p + e->i + e->d );	
	e->beforede = e->lastde;
	e->lastde = e->thisde;
	if(out>MaxOutput)              //积分限幅
	{
		out=MaxOutput;
	}
	if(out<MinOutput)
	{
		out=MinOutput;
	}
	return out;
}