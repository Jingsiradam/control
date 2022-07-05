#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include <sstream>
#include <Eigen/Eigen>
#include <geometry_msgs/Accel.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <Eigen/Core>
#include <stdio.h>
#include <serial/serial.h>
#include <sensor_msgs/Joy.h>
#include <thread>
#include "time.h"

using namespace Eigen;
using namespace sensor_msgs;
using namespace message_filters;
using namespace std;

serial::Serial sp;

#define ANGLE_HEADER 0x50
#define MOTION_HEADER 0x51
#define COMMAND_HEADER 0x52
#define HEADER 0x55
#define ENDER 0x60
#define SPEED_MODE 0
#define CARRY_MODE 1
#define ONEKEY_UP_AND_DOWN 0
#define ALTHOLD_ENABLE 1

union vect3d
{
	short xyz[3];
	uint8_t data[6];
} Angle3, motion3;
uint8_t command[6] = {0};

//######################functions############################

void Serial_Init()
{
	serial::Timeout to = serial::Timeout::simpleTimeout(100);			//创建timeout
	serial::parity_t pt = serial::parity_t::parity_none;				//创建校验位为0位
	serial::bytesize_t bt = serial::bytesize_t::eightbits;				//创建发送字节数为8位
	serial::flowcontrol_t ft = serial::flowcontrol_t::flowcontrol_none; //创建数据流控制，不使用
	serial::stopbits_t st = serial::stopbits_t::stopbits_one;			//创建终止位为1位

	sp.setPort("/dev/ttyACM0"); //设置要打开的串口名称
	sp.setBaudrate(115200);		//设置串口通信的波特率
	sp.setParity(pt);			//设置校验位
	sp.setBytesize(bt);			//设置发送字节数
	sp.setFlowcontrol(ft);		//设置数据流控制
	sp.setStopbits(st);			//设置终止位

	sp.setTimeout(to); //串口设置timeout

	try
	{
		//打开串口
		sp.open();
	}
	catch (serial::IOException &e)
	{
		ROS_ERROR_STREAM("Unable to open port.");
		return;
	}
}

// adjoint the joystick key
void joy_control(const sensor_msgs::Joy::ConstPtr &joy)
{
	motion3.xyz[0] = joy->axes[1] * 1000;
	motion3.xyz[1] = joy->axes[0] * 1000;
	motion3.xyz[2] = joy->axes[4] * 1800;
	Angle3.xyz[2] = joy->axes[3] * 180;
	Angle3.xyz[1] = -(joy->axes[2] - 1) + (joy->axes[5] - 1) * 15; // pitch (-2,2)
}

void send_command()
{
	uint8_t buffer[27];
	uint8_t read_buf[256];
	buffer[0] = buffer[9] = buffer[18] = HEADER;
	buffer[8] = buffer[17] = buffer[26] = ENDER;
	buffer[1] = ANGLE_HEADER;
	buffer[10] = MOTION_HEADER;
	buffer[19] = COMMAND_HEADER;
	while (ros::ok)
	{
		for (uint8_t i = 0; i < 6; i++)
		{
			buffer[2 + i] = Angle3.data[i];
			buffer[11 + i] = motion3.data[i];
			buffer[20 + i] = command[i];
		}
		sp.write(buffer, 27);
		size_t n = sp.available();

		// for (int i = 0; i < 27; i++)
		// {
		// 	// 16进制的方式打印到屏幕
		// 	cout << hex << (buffer[i]&0xff);
		// }
		// cout << endl;

		n = sp.read(read_buf, n);
		if (n > 0)
		{
			for (int i = 0; i < n; i++)
			{
				// 16进制的方式打印到屏幕
				cout << read_buf[i];
			}
			cout << endl;
		}
		sleep(0.1);
	}
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "attitude_control");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("joy", 1000, joy_control);
	Serial_Init();
	thread t1{send_command};
	ros::spin();
	sp.close();
	return 0;
}
