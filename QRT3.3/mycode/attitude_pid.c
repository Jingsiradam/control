#include "attitude_pid.h"

PidObject pidAngleRoll;
PidObject pidAnglePitch;
PidObject pidAngleYaw;
PidObject pidRateRoll;
PidObject pidRatePitch;
PidObject pidRateYaw;


static inline int16_t pidOutLimit(float in)
{
	if (in > INT16_MAX)
		return INT16_MAX;
	else if (in < -INT16_MAX)
		return -INT16_MAX;
	else
		return (int16_t)in;
}

void attitudeControlInit()
{

	pidInit(&pidAngleRoll, 0, configParamDefault.pidAngle.roll, ATTITUDE_UPDATE_DT);   /*roll  角度PID初始化*/
	pidInit(&pidAnglePitch, 0, configParamDefault.pidAngle.pitch, ATTITUDE_UPDATE_DT); /*pitch 角度PID初始化*/
	pidInit(&pidAngleYaw, 0, configParamDefault.pidAngle.yaw, ATTITUDE_UPDATE_DT);	   /*yaw   角度PID初始化*/
	pidSetIntegralLimit(&pidAngleRoll, PID_ANGLE_ROLL_INTEGRATION_LIMIT);		   /*roll  角度积分限幅设置*/
	pidSetIntegralLimit(&pidAnglePitch, PID_ANGLE_PITCH_INTEGRATION_LIMIT);		   /*pitch 角度积分限幅设置*/
	pidSetIntegralLimit(&pidAngleYaw, PID_ANGLE_YAW_INTEGRATION_LIMIT);			   /*yaw   角度积分限幅设置*/
	pidSetOutLimit(&pidAngleRoll, PID_ANGLE_ROLL_INTEGRATION_LIMIT);
	pidSetOutLimit(&pidAnglePitch, PID_ANGLE_PITCH_INTEGRATION_LIMIT);
	pidSetOutLimit(&pidAngleYaw, PID_ANGLE_YAW_INTEGRATION_LIMIT);

	pidInit(&pidRateRoll, 0, configParamDefault.pidRate.roll, ATTITUDE_UPDATE_DT);	 /*roll  角速度PID初始化*/
	pidInit(&pidRatePitch, 0, configParamDefault.pidRate.pitch, ATTITUDE_UPDATE_DT); /*pitch 角速度PID初始化*/
	pidInit(&pidRateYaw, 0, configParamDefault.pidRate.yaw, ATTITUDE_UPDATE_DT);	 /*yaw   角速度PID初始化*/
	pidSetIntegralLimit(&pidRateRoll, PID_RATE_ROLL_INTEGRATION_LIMIT);			 /*roll  角速度积分限幅设置*/
	pidSetIntegralLimit(&pidRatePitch, PID_RATE_PITCH_INTEGRATION_LIMIT);		 /*pitch 角速度积分限幅设置*/
	pidSetIntegralLimit(&pidRateYaw, PID_RATE_YAW_INTEGRATION_LIMIT);			 /*yaw   角速度积分限幅设置*/
	pidSetOutLimit(&pidRateRoll, PID_RATE_ROLL_INTEGRATION_LIMIT);
	pidSetOutLimit(&pidRatePitch, PID_RATE_PITCH_INTEGRATION_LIMIT);
	pidSetOutLimit(&pidRateYaw, PID_RATE_YAW_INTEGRATION_LIMIT);

}

void attitudeRatePID(attitude_t *actualRate, attitude_t *desiredRate, control_t *output) /* 角速度环PID */
{
	output->roll = pidOutLimit(pidUpdate(&pidRateRoll, desiredRate->roll - actualRate->roll));
	output->pitch = pidOutLimit(pidUpdate(&pidRatePitch, desiredRate->pitch - actualRate->pitch));
	output->yaw = pidOutLimit(pidUpdate(&pidRateYaw, desiredRate->yaw - actualRate->yaw));
}

void attitudeAnglePID(attitude_t *actualAngle, attitude_t *desiredAngle, attitude_t *outDesiredRate) /* 角度环PID */
{
	outDesiredRate->roll = pidUpdate(&pidAngleRoll, desiredAngle->roll - actualAngle->roll);
	outDesiredRate->pitch = pidUpdate(&pidAnglePitch, desiredAngle->pitch - actualAngle->pitch);
	
	float yawError = desiredAngle->yaw - actualAngle->yaw;
	if (yawError > 180.0f)
		yawError -= 360.0f;
	else if (yawError < -180.0)
		yawError += 360.0f;
	outDesiredRate->yaw = pidUpdate(&pidAngleYaw, yawError);
}

void attitudeResetAllPID(void) /*复位PID*/
{
	pidReset(&pidAngleRoll);
	pidReset(&pidAnglePitch);
	pidReset(&pidAngleYaw);
	pidReset(&pidRateRoll);
	pidReset(&pidRatePitch);
	pidReset(&pidRateYaw);
}
