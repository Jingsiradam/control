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


/*��������ֵ����*/
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
	pidInit(&posPid.pidVZ, 0, configParam.pidPos.vz, POS_UPDATE_DT);           /*vz PID��ʼ��*/
	pidSetIntegralLimit(&posPid.pidVZ, PID_DEPTH_INTEGRATION_LIMIT);		   /*roll  �ǶȻ����޷�����*/
	pidSetOutLimit(&posPid.pidVZ, PID_DEPTH_INTEGRATION_LIMIT);
	positionResetAllPID();
	posPid.thrustBase = limitThrustBase(configParam.thrustBase);		// ÿ�γ�ʼ�����¸������Ż�ֵ
}

//��Ȼ�PID
void depthPID(float *actualDepth, float *desiredDepth, control_t *output)
{
	float PIDoutThrust;
	float depthError = *desiredDepth - *actualDepth;
	PIDoutThrust = THRUST_SCALE * pidUpdate(&posPid.pidVZ,depthError);

	if (posPid.isAltHoldMode == true)		// �ڶ���ģʽ�¼�������������Զ������������Ż�׼ֵ
	{
		//detectWeight(PIDoutThrust);			// �����������������ֵ
	}

	// thrustBase�Ǹ�ֵ
	output->thrust = limitThrust(posPid.thrustBase - PIDoutThrust);		// z������Ϊ��������ֵ����Ϊ��
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

