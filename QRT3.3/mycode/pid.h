#ifndef __PID_H
#define __PID_H
#include "stm32f4xx_hal.h"
#include "stdbool.h"

typedef struct
{
	float kp;
	float ki;
	float kd;
} pidInit_t;

typedef struct
{
	pidInit_t roll;
	pidInit_t pitch;
	pidInit_t yaw;
} pidParam_t;

typedef struct
{
	pidInit_t vx;
	pidInit_t vy;
	pidInit_t vz;
} pidParamPos_t;

typedef struct
{
	pidParam_t pidAngle;  /*角度PID*/
	pidParam_t pidRate;	  /*角速度PID*/
	pidParamPos_t pidPos; /*位置PID*/
	float thrustBase;		  /*油门基础值*/
	uint8_t cksum;
} configParam_t;

extern configParam_t configParam;
extern configParam_t configParamDefault;

typedef struct
{
	float desired;	 //< set point
	float error;	 //< error
	float prevError; //< previous error
	float integ;	 //< integral
	float deriv;	 //< derivative
	float kp;		 //< proportional gain
	float ki;		 //< integral gain
	float kd;		 //< derivative gain
	float outP;		 //< proportional output (debugging)
	float outI;		 //< integral output (debugging)
	float outD;		 //< derivative output (debugging)
	float iLimit;	 //< integral limit
	float iLimitLow; //< integral limit
	float maxOutput;
	float dt; //< delta-time dt
} PidObject;

void pidInit(PidObject *pid, const float desired, const pidInit_t pidParam, const float dt);
void pidSetIntegralLimit(PidObject *pid, const float limit); /*pid积分限幅设置*/
void pidSetOutLimit(PidObject *pid, const float maxoutput);	 /*pid输出限幅设置*/
void pidSetDesired(PidObject *pid, const float desired);	 /*pid设置期望值*/
float pidUpdate(PidObject *pid, const float error);			 /*pid更新*/
float pidGetDesired(PidObject *pid);						 /*pid获取期望值*/
bool pidIsActive(PidObject *pid);							 /*pid状态*/
void pidReset(PidObject *pid);								 /*pid结构体复位*/
void pidSetError(PidObject *pid, const float error);		 /*pid偏差设置*/
void pidSetKp(PidObject *pid, const float kp);				 /*pid Kp设置*/
void pidSetKi(PidObject *pid, const float ki);				 /*pid Ki设置*/
void pidSetKd(PidObject *pid, const float kd);				 /*pid Kd设置*/
void pidSetPID(PidObject *pid, const float kp, const float ki, const float kd);
void pidSetDt(PidObject *pid, const float dt); /*pid dt设置*/
uint8_t configParamCksum(configParam_t* data);


#endif /* __PID_H */
