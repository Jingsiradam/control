#include "pid.h"
uint8_t DEFAULT_PID_INTEGRATION_LIMIT =200;
configParam_t configParam;

configParam_t configParamDefault =
{
	.pidAngle=	/*角度PID*/
	{
		.roll=
		{
			.kp=1.0,
			.ki=0.1,
			.kd=0.0,
		},
		.pitch=
		{
			.kp=1.0,
			.ki=0.1,
			.kd=0.0,
		},
		.yaw=
		{
			.kp=1.0,
			.ki=0.1,
			.kd=0.0,
		},
	},
	.pidRate=	/*角速度PID*/
	{
		.roll=
		{
			.kp=1.0,
			.ki=0.1,
			.kd=0.0,
		},
		.pitch=
		{
			.kp=1.0,
			.ki=0.1,
			.kd=0.0,
		},
		.yaw=
		{
			.kp=1.0,
			.ki=0.1,
			.kd=0.0,
		},
	},
	.pidPos=	/*位置PID*/
	{
		.vx=
		{
			.kp=0.0,
			.ki=0.0,
			.kd=0.0,
		},
		.vy=
		{
			.kp=0.0,
			.ki=0.0,
			.kd=0.0,
		},
		.vz=
		{
			.kp=35.0,
			.ki=0.1,
			.kd=0.0,
		},
	},
	.thrustBase = -2000.0,
};

void abs_outlimit(float *a, float ABS_MAX){
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}

void pidInit(PidObject* pid, const float desired, const pidInit_t pidParam, const float dt)
{
	pid->error     = 0;
	pid->prevError = 0;
	pid->integ     = 0;
	pid->deriv     = 0;
	pid->desired = desired;
	pid->kp = pidParam.kp;
	pid->ki = pidParam.ki;
	pid->kd = pidParam.kd;
	pid->iLimit    = DEFAULT_PID_INTEGRATION_LIMIT;
	pid->iLimitLow = -DEFAULT_PID_INTEGRATION_LIMIT;
	pid->dt        = dt;
}

float pidUpdate(PidObject* pid, const float error)
{
	float output;

	pid->error = error;
	pid->integ += pid->error * pid->dt;
	pid->deriv = (pid->error - pid->prevError) / pid->dt;

	pid->outP = pid->kp * pid->error;
	pid->outI = pid->ki * pid->integ;
	pid->outD = pid->kd * pid->deriv;

	abs_outlimit(&(pid->integ), pid->iLimit);
	output = pid->outP + pid->outI + pid->outD;
	abs_outlimit(&(output), pid->maxOutput);
	pid->prevError = pid->error;

	return output;
}

void pidSetIntegralLimit(PidObject* pid, const float limit)
{
    pid->iLimit = limit;
}

void pidSetIntegralLimitLow(PidObject* pid, const float limitLow)
{
    pid->iLimitLow = limitLow;
}

void pidSetOutLimit(PidObject* pid, const float maxoutput)
{
    pid->maxOutput = maxoutput;
}

void pidReset(PidObject* pid)
{
	pid->error     = 0;
	pid->prevError = 0;
	pid->integ     = 0;
	pid->deriv     = 0;
}

void pidSetError(PidObject* pid, const float error)
{
	pid->error = error;
}

void pidSetDesired(PidObject* pid, const float desired)
{
	pid->desired = desired;
}

float pidGetDesired(PidObject* pid)
{
	return pid->desired;
}

bool pidIsActive(PidObject* pid)
{
	bool isActive = true;

	if (pid->kp < 0.0001f && pid->ki < 0.0001f && pid->kd < 0.0001f)
	{
		isActive = false;
	}

	return isActive;
}

void pidSetKp(PidObject* pid, const float kp)
{
	pid->kp = kp;
}

void pidSetKi(PidObject* pid, const float ki)
{
	pid->ki = ki;
}

void pidSetKd(PidObject* pid, const float kd)
{
	pid->kd = kd;
}

void pidSetPID(PidObject* pid, const float kp,const float ki,const float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}
void pidSetDt(PidObject* pid, const float dt)
{
    pid->dt = dt;
}


uint8_t configParamCksum(configParam_t* data)
{
	int i;
	uint8_t cksum=0;	
	uint8_t* c = (uint8_t*)data;  	
	size_t len=sizeof(configParam_t);

	for (i=0; i<len; i++)
		cksum += *(c++);
	cksum-=data->cksum;
	return cksum;
}


