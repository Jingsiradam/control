#ifndef __STABILIZER_TYPES_H
#define __STABILIZER_TYPES_H
#include "stm32f4xx_hal.h"

#if defined(__CC_ARM)
	#pragma anon_unions
#endif

#include "stdbool.h"


struct vec3_s
{
	float x;
	float y;
	float z;
};

typedef struct vec3_s positon_t;
typedef struct vec3_s velocity_t;
typedef struct vec3_s acc_t;

//姿态集
typedef struct
{
	float roll;
	float pitch;
	float yaw;
} attitude_t;

typedef struct
{
	struct vec3_s acc;
	struct vec3_s gyro;
	struct vec3_s mag;
	positon_t position;
	float height;
	float depth;
	float voltage;
	float current;
	float water_temp;
	float water_press;
	float air_press_robot;		// 这个值是 机器人 舱内的压力
	float imu_temp;
} sensorData_t;

typedef struct
{
	attitude_t realAngle;
	positon_t position;
	velocity_t velocity;
	acc_t acc;
	attitude_t realRate;
	float realDepth;
	float preDepth;
} state_t;

typedef struct
{
	attitude_t expectedAngle;
	positon_t position;
	velocity_t velocity;
	attitude_t expectedRate;
	float expectedDepth;
	float thrust;
	bool isAltHold;
} setstate_t;

// 控制量，速度环输出
typedef struct
{
//	s16 roll;
//	s16 pitch;
//	s16 yaw;
	float roll;
	float pitch;
	float yaw;
	float thrust;			// 油门值 针对四轴
	float forward_thrust;
	float turn_thrust;
	float hightOut;
	float depthOut;
	bool isLandMode;
	bool preMode;
	
} control_t;

// 六个电机的油门值
typedef struct
{
	uint16_t m1;
	uint16_t m2;
	uint16_t m3;
	uint16_t m4;
	uint16_t m5;
	uint16_t m6;
}motorPWM_t;

extern control_t control;
extern setstate_t setstate;
extern state_t state;

#endif
