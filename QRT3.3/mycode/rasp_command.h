#ifndef __RASP_COMMAND_H
#define __RASP_COMMAND_H

#include "stm32f4xx_hal.h"
#include <string.h>

#define LOW_SPEED 0
#define HIGH_SPEED 1

//�������ֶ�ģʽ
#define HAND_MODE 0
#define DEPTH_MODE 1

// һ����ɡ�һ������
#define ONEKEY_LANDING 0
#define ONEKEY_TAKEOFF 1

// ����ģʽʹ�ܡ�ʧ��
#define HOLD_DISABLE 0
#define HOLD_ENABLE 1

//����X������ CH1 Y������ CH2
//����X����ת CH4 Y����� CH3

#define ROLL 0
#define PITCH 1
#define YAW 2

#define FORWARD_SPEED 0
#define THROTTLE 2
#define turn_SPEED 1



#define SPEED_MODE 0
#define CARRY_MODE 1
#define ONEKEY_UP_AND_DOWN 2// һ����ɺ���½��ť
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