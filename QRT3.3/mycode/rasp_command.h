#ifndef __RASP_COMMAND_H
#define __RASP_COMMAND_H

#include "stm32f4xx_hal.h"
#include <string.h>

#define LOW_SPEED 0
#define HIGH_SPEED 1

//定深与手动模式
#define HAND_MODE 0
#define DEPTH_MODE 1

// 一键起飞、一键降落
#define ONEKEY_LANDING 0
#define ONEKEY_TAKEOFF 1

// 定深模式使能、失能
#define HOLD_DISABLE 0
#define HOLD_ENABLE 1

//右手X：方向 CH1 Y：油门 CH2
//左手X：滚转 CH4 Y：深度 CH3

#define ROLL 0
#define PITCH 1
#define YAW 2

#define FORWARD_SPEED 0
#define THROTTLE 2
#define turn_SPEED 1



#define SPEED_MODE 0
#define CARRY_MODE 1
#define ONEKEY_UP_AND_DOWN 2// 一键起飞和着陆按钮
#define ALTHOLD_ENABLE 3

struct command_angle
{
	short Angle[3];
};

struct command_motion
{
	short xyz[3];
};

struct rasp_command
{
	  uint8_t   speed_mode;
	  uint8_t   carry_mode;
	  uint8_t   onekey_up_and_down;
	  uint8_t   althold_enable;
		short T;
};

extern struct rasp_command rasp;
extern uint8_t command[14];
extern uint8_t aRxBuffer1[1];
void CopeSerialData(unsigned char ucData);
void UART_ReadCOMMAND(float *angle_v, float *velocity, float *model);
#endif