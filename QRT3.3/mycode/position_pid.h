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
	float thrustBase; 			// 定深时的油门基准值，这个值可以让四轴悬停
	bool preMode;				// 前一次的模式，为true时对应定深模式
	bool isAltHoldMode;         // 为true时对应定深模式
}posPid_t;

void positionControlInit(void);
void positionResetAllPID(void);
void depthPID(float *actualDepth, float *desiredDepth, control_t *output);
void getPositionPIDZ(float* kp, float* ki, float* kd);
void setPositionPIDZ(float kp, float ki, float kd);

extern posPid_t posPid;

#endif
