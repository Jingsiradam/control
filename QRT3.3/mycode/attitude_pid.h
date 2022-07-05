#ifndef __ATTITUDE_PID_H
#define __ATTITUDE_PID_H
#include <stdbool.h>
#include "pid.h"
#include "stabilizer.h"

#define ATTITUDE_UPDATE_RATE 	200  //更新频率100hz
#define ATTITUDE_UPDATE_DT 		(1.0f / ATTITUDE_UPDATE_RATE)

/*角度环积分限幅*/
#define PID_ANGLE_ROLL_INTEGRATION_LIMIT 500.0
#define PID_ANGLE_PITCH_INTEGRATION_LIMIT 500.0
#define PID_ANGLE_YAW_INTEGRATION_LIMIT 500.0

/*角速度环积分限幅*/
#define PID_RATE_ROLL_INTEGRATION_LIMIT 800.0
#define PID_RATE_PITCH_INTEGRATION_LIMIT 800.0
#define PID_RATE_YAW_INTEGRATION_LIMIT 800.0

extern PidObject pidAngleRoll;
extern PidObject pidAnglePitch;
extern PidObject pidAngleYaw;
extern PidObject pidRateRoll;
extern PidObject pidRatePitch;
extern PidObject pidRateYaw;

void attitudeControlInit(void);
void attitudeRatePID(attitude_t *actualRate, attitude_t *desiredRate,control_t *output);	/* 角速度环PID */
void attitudeAnglePID(attitude_t *actualAngle,attitude_t *desiredAngle,attitude_t *outDesiredRate);	/* 角度环PID */
void attitudeResetAllPID(void);		/*复位PID*/
#endif /* __ATTITUDE_PID_H */
