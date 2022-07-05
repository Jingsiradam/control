#ifndef __POSITION_PID_H
#define __POSITION_PID_H
#include "stm32f4xx_hal.h"
#include "stabilizer.h"
#include "pid.h"

#define POS_UPDATE_RATE 		200
#define POS_UPDATE_DT 			(1.0f / POS_UPDATE_RATE)
#define PID_DEPTH_INTEGRATION_LIMIT 10000.0

typedef struct  
{
	PidObject pidVX;
	PidObject pidVY;
	PidObject pidVZ;
	float thrustBase; 			// ����ʱ�����Ż�׼ֵ�����ֵ������������ͣ
	bool preMode;				// ǰһ�ε�ģʽ��Ϊtrueʱ��Ӧ����ģʽ
	bool isAltHoldMode;         // Ϊtrueʱ��Ӧ����ģʽ
}posPid_t;

void positionControlInit(void);
void positionResetAllPID(void);
void depthPID(float *actualDepth, float *desiredDepth, control_t *output);
void getPositionPIDZ(float* kp, float* ki, float* kd);
void setPositionPIDZ(float kp, float ki, float kd);

extern posPid_t posPid;

#endif
