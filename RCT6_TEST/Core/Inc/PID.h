#ifndef __PID_H_ 
#define __PID_H_


typedef struct 
{
	float kp,ki,kd;
	float p,i,d;
	float thisde,lastde,beforede;
	float integralde;
}pidstruct;

pidstruct initpid(float kp,float ki,float kd,pidstruct e);
float PID(pidstruct *e,float err,float outlow,float outhigh);
float Incremental_PID(pidstruct *e,float err, float MaxOutput,float MinOutput) ; //ÔöÁ¿Ê½PID
#endif