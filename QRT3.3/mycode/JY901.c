/*
��д�ߣ�Kevin
��ַ��http://RobotControl.taobao.com
����E-mail��1609370741@qq.com
���뻷����MDK-Lite  Version: 5.17
����ʱ��: 2016-1-31
���ԣ� ���������ڡ������ǿء���STM32Coreƽ̨����ɲ���
���ܣ�
��STM32Coreƽ̨����2��ȡJY901�����ݣ�Ȼ��ͨ������1��ӡ����������,�������ֲ�����ҪѡΪ9600��
JY-901�Ĳ�����Ҫ�޸�Ϊ9600.
ע�⣺ʾ�������������ASCLL�룬��16���ƣ�HEX����ʾ�ǲ��ܿ���׼ȷ���ݵġ�
Ӳ�����ߣ�
USB-TTL����                 STM32Core               JY901
VCC          -----           VCC        ----         VCC
TX           -----           RX1���ܽ�10��     
RX           -----           TX1���ܽ�9��
GND          -----           GND        ----          GND
                             RX2 ���ܽ�3��       ----  TX
														 TX2 ���ܽ�2��       ----  RX
------------------------------------
 */
#include <string.h>
#include <stdio.h>
#include "main.h"
#include "usart.h"
#include "JY901.h"
#include "stabilizer.h"


struct STime		stcTime;
struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 	stcAngle;
struct SMag 		stcMag;
struct SDStatus stcDStatus;
struct SPress 	stcPress;
struct SLonLat 	stcLonLat;
struct SGPSV 		stcGPSV;
struct SQ       stcQ;


// �ߵ�ֵ�˲�����

float filterAngleYaw[10];  //�˲�
float filterAngleRoll[10];
float filterAnglePitch[10];
float sumYaw,sumRoll,sumPitch;
float filterAngleYawRate[10];  //�˲�
float filterAngleRollRate[10];
float filterAnglePitchRate[10];
float sumYawRate,sumRollRate,sumPitchRate;
uint8_t aRxBuffer2[1];

char ACCCALSW[5] = {0XFF,0XAA,0X01,0X01,0X00};//������ٶ�У׼ģʽ
char SAVACALSW[5]= {0XFF,0XAA,0X00,0X00,0X00};//���浱ǰ����

										 
void UART5_Put_Char(unsigned char ucData)	
{
	HAL_UART_Transmit(&huart5,(uint8_t *)&ucData,(uint16_t)sizeof(ucData),0xffff);
}	
										 
//�ô���5��JYģ�鷢��ָ��
void sendcmd(char cmd[])
{
	char i;
	for(i=0;i<5;i++)
		{
		 UART5_Put_Char(cmd[i]);
		}
}


//CopeSerialDataΪ����5�жϵ��ú���������ÿ�յ�һ�����ݣ�����һ�����������
void CopeSerial5Data(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	
	ucRxBuffer[ucRxCnt++]=ucData;	//���յ������ݴ��뻺������
	if (ucRxBuffer[0]!=0x55) //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//���ݲ���11�����򷵻�
	else
	{
		switch(ucRxBuffer[1])//�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������
		{
			case 0x50:	memcpy(&stcTime,&ucRxBuffer[2],8);break;//memcpyΪ�������Դ����ڴ濽��������������"string.h"�������ջ��������ַ����������ݽṹ�����棬�Ӷ�ʵ�����ݵĽ�����
			case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;
			case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);break;
			case 0x54:	memcpy(&stcMag,&ucRxBuffer[2],8);break;
			case 0x55:	memcpy(&stcDStatus,&ucRxBuffer[2],8);break;
			case 0x56:	memcpy(&stcPress,&ucRxBuffer[2],8);break;
			case 0x57:	memcpy(&stcLonLat,&ucRxBuffer[2],8);break;
			case 0x58:	memcpy(&stcGPSV,&ucRxBuffer[2],8);break;
			case 0x59:	memcpy(&stcQ,&ucRxBuffer[2],8);break;
		}
		ucRxCnt=0;//��ջ�����
	}
}

void UART_ReadIMU(float *Gyro, float *Acc, float *Mag, float *Angle)
{
	 //���ٶ�ֵ
    Acc[0] = (float)stcAcc.a[0]/32768*16;
    Acc[1] = (float)stcAcc.a[1]/32768*16;
    Acc[2] = (float)stcAcc.a[2]/32768*16;
    //���ٶ�ֵ
    Gyro[0] = (float)stcGyro.w[0]/32768*2000;
    Gyro[1] = (float)stcGyro.w[1]/32768*2000;
    Gyro[2] = (float)stcGyro.w[2]/32768*2000;
    //������ֵ
    Mag[0] = stcMag.h[0];
    Mag[1] = stcMag.h[1];
    Mag[2] = stcMag.h[2];
    //��̬��
    Angle[0] = (float)stcAngle.Angle[0]/32768*180;
    Angle[1] = (float)stcAngle.Angle[1]/32768*180;
    Angle[2] = (float)stcAngle.Angle[2]/32768*180;
}


void CopeSerial1Data(unsigned char ucData)
{	
	UART5_Put_Char(ucData);//ת������1�յ������ݸ�����5��JYģ�飩
}

// FIR�˲�
void sensorReadAngle(float *Gyro, float *Angle)
{
	static uint8_t count=0;
	float gyro[3], acc[3],mag[3],angle[3];
	float tempYaw,tempRoll,tempPitch;
	float tempYawRate,tempRollRate,tempPitchRate;

	UART_ReadIMU(gyro, acc, mag, angle);

	tempRoll = filterAngleRoll[count];
	tempPitch = filterAnglePitch[count];
	tempYaw = filterAngleYaw[count];
	filterAngleRoll[count] = angle[0];
	filterAnglePitch[count] = angle[1];
	filterAngleYaw[count] = angle[2];
	sumRoll += filterAngleRoll[count] - tempRoll;
	sumPitch += filterAnglePitch[count] - tempPitch;
	sumYaw += filterAngleYaw[count] - tempYaw;
	Angle[0] = sumRoll/10.0f;
	Angle[1] = sumPitch/10.0f;
	Angle[2] = sumYaw/10.0f;
	
	tempRollRate = filterAngleRollRate[count]; 
	tempPitchRate = filterAnglePitchRate[count];
	tempYawRate = filterAngleYawRate[count];
	filterAngleRollRate[count] = gyro[0];
	filterAnglePitchRate[count] = gyro[1];
	filterAngleYawRate[count] = gyro[2];
	sumRollRate += filterAngleRollRate[count] - tempRollRate;
	sumPitchRate += filterAnglePitchRate[count] - tempPitchRate;
	sumYawRate += filterAngleYawRate[count] - tempYawRate;
	Gyro[0] = sumRollRate/10.0f;
	Gyro[1] = sumPitchRate/10.0f;
	Gyro[2] = sumYawRate/10.0f;
	count++;
	if (count == 10) count = 0;
}





