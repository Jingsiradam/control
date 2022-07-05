#include "rasp_command.h"
										 																 
struct command_angle stc_command_angle;
struct command_motion stc_command_motion;
struct rasp_command stc_rasp_command;
uint8_t aRxBuffer1[1];

void CopeSerialData(unsigned char ucData)
{
	static unsigned char ucRxBuffer1[250];
	static unsigned char ucRxCnt1 = 0;	
	
	ucRxBuffer1[ucRxCnt1++]=ucData;	//将收到的数据存入缓冲区中
	if (ucRxBuffer1[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt1=0;
		return;
	}
	if (ucRxCnt1<9) {return;}//数据不满9个，则返回
	else
	{
		switch(ucRxBuffer1[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
		{
			case 0x50:	memcpy(&stc_command_angle,&ucRxBuffer1[2],6);break;//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
			case 0x51:	memcpy(&stc_command_motion,&ucRxBuffer1[2],6);break;
			case 0x52:	memcpy(&stc_rasp_command,&ucRxBuffer1[2],6);break;
		}

		ucRxCnt1=0;//清空缓存区
	}
}

void UART_ReadCOMMAND(float *angle, float *velocity, float *model)
{
	 //控制角度
    angle[0] = (float)stc_command_angle.Angle[0];
    angle[1] = (float)stc_command_angle.Angle[1];
    angle[2] = (float)stc_command_angle.Angle[2];
    //控制 运动
    velocity[0] = (float)stc_command_motion.xyz[0];
    velocity[1] = (float)stc_command_motion.xyz[1];
    velocity[2] = (float)stc_command_motion.xyz[2];
    //command_model
    model[0] = stc_rasp_command.althold_enable;
    model[1] = stc_rasp_command.carry_mode;
    model[2] = stc_rasp_command.onekey_up_and_down;
	model[3] = stc_rasp_command.speed_mode;
}