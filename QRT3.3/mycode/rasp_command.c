#include "rasp_command.h"
										 																 
struct command_angle stc_command_angle;
struct command_motion stc_command_motion;
struct rasp_command stc_rasp_command;
uint8_t aRxBuffer1[1];

void CopeSerialData(unsigned char ucData)
{
	static unsigned char ucRxBuffer1[250];
	static unsigned char ucRxCnt1 = 0;	
	
	ucRxBuffer1[ucRxCnt1++]=ucData;	//���յ������ݴ��뻺������
	if (ucRxBuffer1[0]!=0x55) //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
	{
		ucRxCnt1=0;
		return;
	}
	if (ucRxCnt1<9) {return;}//���ݲ���9�����򷵻�
	else
	{
		switch(ucRxBuffer1[1])//�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������
		{
			case 0x50:	memcpy(&stc_command_angle,&ucRxBuffer1[2],6);break;//memcpyΪ�������Դ����ڴ濽��������������"string.h"�������ջ��������ַ����������ݽṹ�����棬�Ӷ�ʵ�����ݵĽ�����
			case 0x51:	memcpy(&stc_command_motion,&ucRxBuffer1[2],6);break;
			case 0x52:	memcpy(&stc_rasp_command,&ucRxBuffer1[2],6);break;
		}

		ucRxCnt1=0;//��ջ�����
	}
}

void UART_ReadCOMMAND(float *angle, float *velocity, float *model)
{
	 //���ƽǶ�
    angle[0] = (float)stc_command_angle.Angle[0];
    angle[1] = (float)stc_command_angle.Angle[1];
    angle[2] = (float)stc_command_angle.Angle[2];
    //���� �˶�
    velocity[0] = (float)stc_command_motion.xyz[0];
    velocity[1] = (float)stc_command_motion.xyz[1];
    velocity[2] = (float)stc_command_motion.xyz[2];
    //command_model
    model[0] = stc_rasp_command.althold_enable;
    model[1] = stc_rasp_command.carry_mode;
    model[2] = stc_rasp_command.onekey_up_and_down;
	model[3] = stc_rasp_command.speed_mode;
}