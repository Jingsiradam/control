#include "position_pid.h"
#include "stabilizer.h"
#include <math.h>
#include "pid.h"
#include "pwm_control.h"

//#define THRUST_SCALE	(5.0f)
#define THRUST_SCALE	(1.0f)
#define START_DEPTH     (0.0f)
#define THRUSTBASE_HIGH	(10000.f)
#define THRUSTBASE_LOW	(-10000.f)

posPid_t posPid;


/*基础油门值限制*/
float limitThrustBase(float input)
{
	if(input > THRUSTBASE_HIGH)
		return THRUSTBASE_HIGH;
	else if(input < THRUSTBASE_LOW)
		return THRUSTBASE_LOW;
	else 
		return input;
}

void positionControlInit(void)
{
	pidInit(&posPid.pidVZ, 0, configParam.pidPos.vz, POS_UPDATE_DT);           /*vz PID初始化*/
	pidSetIntegralLimit(&posPid.pidVZ, PID_DEPTH_INTEGRATION_LIMIT);		   /*roll  角度积分限幅设置*/
	pidSetOutLimit(&posPid.pidVZ, PID_DEPTH_INTEGRATION_LIMIT);
	positionResetAllPID();
	posPid.thrustBase = limitThrustBase(configParam.thrustBase);		// 每次初始化重新给定油门基值
}

//深度环PID
void depthPID(float *actualDepth, float *desiredDepth, control_t *output)
{
	float PIDoutThrust;
	float depthError = *desiredDepth - *actualDepth;
	PIDoutThrust = THRUST_SCALE * pidUpdate(&posPid.pidVZ,depthError);

	if (posPid.isAltHoldMode == true)		// 在定高模式下检测机体的重量，自动调整定高油门基准值
	{
		//detectWeight(PIDoutThrust);			// 检测重量，更新油门值
	}

	// thrustBase是负值
	output->thrust = limitThrust(posPid.thrustBase - PIDoutThrust);		// z轴向下为正，油门值向上为正
}

void positionResetAllPID(void)
{
	//pidReset(&posPid.pidVX);
	//pidReset(&posPid.pidVY);
	pidReset(&posPid.pidVZ);
}

void getPositionPIDZ(float* kp, float* ki, float* kd)
{
	*kp = posPid.pidVZ.kp;
	*ki = posPid.pidVZ.ki;
	*kd = posPid.pidVZ.kd ;
}

void setPositionPIDZ(float kp, float ki, float kd)
{
	posPid.pidVZ.kp = kp;
	posPid.pidVZ.ki = ki;
	posPid.pidVZ.kd = kd;
}

