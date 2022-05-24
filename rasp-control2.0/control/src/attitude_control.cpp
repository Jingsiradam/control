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

using namespace Eigen;
using namespace sensor_msgs;
using namespace message_filters;
using namespace std;

struct pid_param
{
	float roll_p = 5;
	float pitch_p = 5;
	float yaw_p = 3;
	float roll_pd = 5;
	float pitch_pd = 5;
	float yaw_pd = 3;
} pid_paramd;

serial::Serial sp;


const unsigned char ender[2] = {0x0d, 0x0a};
const unsigned char header[2] = {0x55, 0xaa};

uint8_t buf_jy901[55];
union sendData
{
	float speed;
	unsigned char data[4];
} motorspeed[6];

union jy901_16unino
{
	uint8_t temp[2];
	int16_t value;
};
union jy901_32unino
{
	uint8_t temp[4];
	uint32_t value;
};
struct jy901_struct
{
	uint8_t r51[2]; //加速度
	union jy901_16unino A[4];
	uint8_t sum1;

	uint8_t r52[2]; //角速度
	union jy901_16unino w[4];
	uint8_t sum2;

	uint8_t r54[2]; //磁场
	union jy901_16unino H[4];
	uint8_t sum3;

	uint8_t r56[2]; //气压高度
	union jy901_32unino PH[2];
	uint8_t sum4;

	uint8_t r59[2]; //四元数
	union jy901_16unino Q[4];
	uint8_t sum5;
};

sensor_msgs::Imu IMU;

struct control_msg
{
	float x, y, z, roll, pitch, yaw;
} control_msgs;

struct Quater
{
	float w, x, y, z;
};

Quater ToQuaternion(float yaw, float pitch, float roll) // yaw (Z), pitch (Y), roll (X)
{
	// Abbreviations for the various angular functions
	double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);

	Quater q;
	q.w = cy * cp * cr + sy * sp * sr;
	q.x = cy * cp * sr - sy * sp * cr;
	q.y = sy * cp * sr + cy * sp * cr;
	q.z = sy * cp * cr - cy * sp * sr;

	return q;
}

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

void attitude_control()
{
	Quater set_attitude = ToQuaternion(control_msgs.yaw, control_msgs.pitch, control_msgs.roll);
	Quaternionf set_att(set_attitude.w,set_attitude.x, set_attitude.y, set_attitude.z);
	Quaternionf att(IMU.orientation.w,IMU.orientation.x, IMU.orientation.y, IMU.orientation.z);
	float roll_u;
	float pitch_u;
	float yaw_u;
	float force_G = control_msgs.z;
	float thrust_x=control_msgs.x;
	float thrust_y=control_msgs.y;
	float thrust_z=control_msgs.z;

	MatrixX3f rot_des = set_att.toRotationMatrix();
	MatrixX3f rot_att = att.toRotationMatrix();

	Vector3f e_R_vec;
	Vector3f torques;

	Matrix3f e_R = (rot_des.transpose() * rot_att - rot_att.transpose() * rot_des) * 0.5;

	e_R_vec(0) = e_R(2, 1); /**< Roll  */
	e_R_vec(1) = e_R(0, 2); /**< Pitch */
	e_R_vec(2) = e_R(1, 0); /**< Yaw   */

	Vector3f omega(IMU.angular_velocity.x, IMU.angular_velocity.y, IMU.angular_velocity.z);
	omega(0) -= e_R_vec(0);
	omega(1) -= e_R_vec(1);
	omega(2) -= e_R_vec(2);

	torques(0) = -e_R_vec(0) * pid_paramd.roll_p;  /**< Roll  */
	torques(1) = -e_R_vec(1) * pid_paramd.pitch_p; /**< Pitch */
	torques(2) = -e_R_vec(2) * pid_paramd.yaw_p;   /**< Yaw   */

	torques(0) = torques(0) - omega(0) * pid_paramd.roll_pd;  /**< Roll  */
	torques(1) = torques(1) - omega(1) * pid_paramd.pitch_pd; /**< Pitch */
	torques(2) = torques(2) - omega(2) * pid_paramd.yaw_pd;	  /**< Yaw   */
	roll_u = torques(0);
	pitch_u = torques(1);
	yaw_u = torques(2);
	Vector4f output;
	output << force_G, roll_u, pitch_u, yaw_u;
	//#################################### mixer ################### need rewrite
	Matrix4f trans_mx;//m1-m4
// mixer matrix (G,tx,ty,tz)=MIXER_M * (n1^2,n2^2,n3^2,n4^2)
	trans_mx << 1,1,1,1,1,-1,-1,1,2,2,-2,-2,sqrt(5),-sqrt(5),sqrt(5),-sqrt(5);
	trans_mx=trans_mx.inverse().eval();
	Vector4f motor_speed;
	motor_speed <<trans_mx * output;
	for (size_t i = 0; i < 4; i++)
	{
		motorspeed[i].speed = motor_speed(i);
	}
	//m5,m6
	Matrix2f trans_mx_;
	Vector2f motor_speed_;
	Vector2f output_;
	output_ << thrust_x,thrust_y;
	trans_mx_ << 1,1,-1,1;
	trans_mx_=trans_mx_.inverse().eval();
	motor_speed_ << trans_mx_*output_;
	motorspeed[4].speed=motor_speed_(0);
	motorspeed[5].speed=motor_speed_(1);
	//#################################### mixer ################### need rewrite
	printf("output:%f,%f,%f,%f,%f,%f\n",motorspeed[0].speed,motorspeed[1].speed,motorspeed[2].speed,motorspeed[3].speed,motorspeed[4].speed,motorspeed[5].speed);
}

void Send_motorspeed()
{
	unsigned char buf[28] = {0};
	unsigned char data[50] = {0};
	int i = 6;

	for (i = 0; i < 2; i++)
	{
		buf[i] = header[i];
	}

	for (i = 0; i < 6; i++)
	{
		buf[i + 2] = motorspeed[0].data[i];
		buf[i + 6] = motorspeed[1].data[i];
		buf[i + 10] = motorspeed[2].data[i];
		buf[i + 14] = motorspeed[3].data[i];
		buf[i + 18] = motorspeed[4].data[i];
		buf[i + 22] = motorspeed[5].data[i];
	}
	// 设置校验值、消息尾
	buf[18] = ender[0]; // buf[9]
	buf[19] = ender[1]; // buf[10]
	// 通过串口下发数据
	sp.write(buf, 28);
}

void get_IMU()
{
	sp.read(buf_jy901, sizeof(buf_jy901));
	uint8_t idx_sum = 0;
	if (((struct jy901_struct *)buf_jy901)->r51[0] == 0x55 &&
		((struct jy901_struct *)buf_jy901)->r51[1] == 0x51)
	{
		// uint8_t sum = 0;
		// for (uint8_t i = 0; i < 10; i++)
		// {
		// 	sum += buf_jy901[i + 11 * idx_sum];
		// }
		// idx_sum++;
		// if (sum == ((struct jy901_struct *)buf_jy901)->sum1)
		// {
		// 	IMU.orientation.w = ((struct jy901_struct *)buf_jy901)->A[0].value / 32768.0 * 16.0;
		// 	IMU.orientation.x = ((struct jy901_struct *)buf_jy901)->A[1].value / 32768.0 * 16.0;
		// 	IMU.orientation.y = ((struct jy901_struct *)buf_jy901)->A[2].value / 32768.0 * 16.0;
		// 	IMU.orientation.z = ((struct jy901_struct *)buf_jy901)->A[3].value / 32768.0 * 16.0;
		// 	// INFO("%d,",((struct jy901_struct*)buf_jy901)->A[i].value);/// a/32768*16
		// 	ROS_INFO("decoding success!");
		// }
		// printf("%f, %f, %f,%f\n",IMU.orientation.w,IMU.orientation.x,IMU.orientation.y,IMU.orientation.z);
		IMU.linear_acceleration.x=((struct jy901_struct *)buf_jy901)->A[0].value / 32768.0 * 16.0;
		IMU.linear_acceleration.y=((struct jy901_struct *)buf_jy901)->A[1].value / 32768.0 * 16.0;
		IMU.linear_acceleration.z=((struct jy901_struct *)buf_jy901)->A[2].value / 32768.0 * 16.0;
		ROS_INFO("decoding success!");
	}
	if (((struct jy901_struct *)buf_jy901)->r52[0] == 0x55 &&
		((struct jy901_struct *)buf_jy901)->r52[1] == 0x52)
	{
		uint8_t sum = 0;
		for (uint8_t i = 0; i < 10; i++)
		{
			sum += buf_jy901[i + 11 * idx_sum];
		}
		idx_sum++;
		if (sum == ((struct jy901_struct *)buf_jy901)->sum2)
		{
			IMU.angular_velocity.x = ((struct jy901_struct *)buf_jy901)->w[0].value / 32768.0 * 2000.0;
			IMU.angular_velocity.y = ((struct jy901_struct *)buf_jy901)->w[1].value / 32768.0 * 2000.0;
			IMU.angular_velocity.z = ((struct jy901_struct *)buf_jy901)->w[2].value / 32768.0 * 2000.0;
		}
	}
	idx_sum = idx_sum + 2;
	if (((struct jy901_struct *)buf_jy901)->r59[0] == 0x55 &&
		((struct jy901_struct *)buf_jy901)->r59[1] == 0x59)
	{
		uint8_t sum = 0;
		for (uint8_t i = 0; i < 10; i++)
		{
			sum += buf_jy901[i + 11 * idx_sum];
		}
		idx_sum++;
		if (sum == ((struct jy901_struct *)buf_jy901)->sum5)
		{
			IMU.orientation.w = ((struct jy901_struct *)buf_jy901)->Q[0].value;
			IMU.orientation.x = ((struct jy901_struct *)buf_jy901)->Q[1].value;
			IMU.orientation.y = ((struct jy901_struct *)buf_jy901)->Q[2].value;
			IMU.orientation.z = ((struct jy901_struct *)buf_jy901)->Q[3].value;
		}
	}
	// ROS_INFO(IMU);
}
// adjoint the joystick key
void joy_control(const sensor_msgs::Joy::ConstPtr &joy)
{
	control_msgs.x = joy->axes[1];//up + down -
	control_msgs.y = joy->axes[0];//left + right -
	control_msgs.z = joy->axes[4];//up + down -
	control_msgs.yaw = joy->axes[3];//left + right -
	control_msgs.pitch = 0;
	control_msgs.roll=0;
}

void stm32_uart()
{
	while (1)
	{
		get_IMU();
		attitude_control();
		// Send_motorspeed();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "attitude_control");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("joy", 1000, joy_control);
	Serial_Init();
	std::thread t1{stm32_uart};
	ros::spin();
	sp.close();
	return 0;
}
