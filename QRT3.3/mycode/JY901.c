/*
编写者：Kevin
网址：http://RobotControl.taobao.com
作者E-mail：1609370741@qq.com
编译环境：MDK-Lite  Version: 5.17
初版时间: 2016-1-31
测试： 本程序已在【君悦智控】的STM32Core平台上完成测试
功能：
用STM32Core平台串口2读取JY901的数据，然后通过串口1打印到串口助手,串口助手波特率要选为9600。
JY-901的波特率要修改为9600.
注意：示例程序输出的是ASCLL码，用16进制（HEX）显示是不能看到准确数据的。
硬件接线：
USB-TTL工具                 STM32Core               JY901
VCC          -----           VCC        ----         VCC
TX           -----           RX1（管脚10）     
RX           -----           TX1（管脚9）
GND          -----           GND        ----          GND
                             RX2 （管脚3）       ----  TX
														 TX2 （管脚2）       ----  RX
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


// 惯导值滤波参数

float filterAngleYaw[10];  //滤波
float filterAngleRoll[10];
float filterAnglePitch[10];
float sumYaw,sumRoll,sumPitch;
float filterAngleYawRate[10];  //滤波
float filterAngleRollRate[10];
float filterAnglePitchRate[10];
float sumYawRate,sumRollRate,sumPitchRate;
uint8_t aRxBuffer2[1];

char ACCCALSW[5] = {0XFF,0XAA,0X01,0X01,0X00};//进入加速度校准模式
char SAVACALSW[5]= {0XFF,0XAA,0X00,0X00,0X00};//保存当前配置

										 
void UART5_Put_Char(unsigned char ucData)	
{
	HAL_UART_Transmit(&huart5,(uint8_t *)&ucData,(uint16_t)sizeof(ucData),0xffff);
}	
										 
//用串口5给JY模块发送指令
void sendcmd(char cmd[])
{
	char i;
	for(i=0;i<5;i++)
		{
		 UART5_Put_Char(cmd[i]);
		}
}


//CopeSerialData为串口5中断调用函数，串口每收到一个数据，调用一次这个函数。
void CopeSerial5Data(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	
	ucRxBuffer[ucRxCnt++]=ucData;	//将收到的数据存入缓冲区中
	if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//数据不满11个，则返回
	else
	{
		switch(ucRxBuffer[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
		{
			case 0x50:	memcpy(&stcTime,&ucRxBuffer[2],8);break;//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
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
		ucRxCnt=0;//清空缓存区
	}
}

void UART_ReadIMU(float *Gyro, float *Acc, float *Mag, float *Angle)
{
	 //加速度值
    Acc[0] = (float)stcAcc.a[0]/32768*16;
    Acc[1] = (float)stcAcc.a[1]/32768*16;
    Acc[2] = (float)stcAcc.a[2]/32768*16;
    //角速度值
    Gyro[0] = (float)stcGyro.w[0]/32768*2000;
    Gyro[1] = (float)stcGyro.w[1]/32768*2000;
    Gyro[2] = (float)stcGyro.w[2]/32768*2000;
    //磁力计值
    Mag[0] = stcMag.h[0];
    Mag[1] = stcMag.h[1];
    Mag[2] = stcMag.h[2];
    //姿态角
    Angle[0] = (float)stcAngle.Angle[0]/32768*180;
    Angle[1] = (float)stcAngle.Angle[1]/32768*180;
    Angle[2] = (float)stcAngle.Angle[2]/32768*180;
}


void CopeSerial1Data(unsigned char ucData)
{	
	UART5_Put_Char(ucData);//转发串口1收到的数据给串口5（JY模块）
}

// FIR滤波
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





