/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "includes.h"
#include "attitude_pid.h"
#include "rasp_command.h"
#include "motortest.h"
#include "JY901.h"
#include "pwm_control.h"
#include "position_pid.h"
#include "STMFlash.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define START_TASK_PRIO 3
//任务堆栈大小
#define START_STK_SIZE 128
//任务控制块
OS_TCB StartTaskTCB;
//任务堆栈
CPU_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *p_arg);

void Water_Attitude_Control(control_t *output);
uint8_t ubuf[128];
#define INFO(...) HAL_UART_Transmit(&huart1,\
										 (uint8_t *)ubuf,\
										 sprintf((char *)ubuf,__VA_ARGS__),\
										 0xffff);
										 
int fputc(int ch,FILE *f){
	HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,0xffff);
	return ch;
};

struct ORDER{
float angle[3];
float velocity[3];
float model[4];}order;


//communicate任务
//设置任务优先级
#define COMMUNICATE_TASK_PRIO 6					// SBUS 信号的更新是在串口中断中进行的
//任务堆栈大小
#define COMMUNICATE_STK_SIZE 512
//任务控制块
OS_TCB CommunicateTaskTCB;
//任务堆栈
CPU_STK COMMUNICATE_TASK_STK[COMMUNICATE_STK_SIZE];
//led0任务
void communicate_task(void *p_arg);

//sensorTask 参数配置任务 在线调试参数并写入flash
//设置任务优先级
#define SENSOR_TASK_PRIO 5
//任务堆栈大小
#define SENSOR_STK_SIZE 512
//任务控制块
OS_TCB SensorTaskTCB;
//任务堆栈
CPU_STK SENSOR_TASK_STK[SENSOR_STK_SIZE];
//motor任务
uint8_t sensor_task(void *p_arg);


//stabalizer任务
//设置任务优先级
#define STABILIZATION_TASK_PRIO 4
//任务堆栈大小
#define STABILIZATION_STK_SIZE 2048
//任务控制块
OS_TCB StabilizationTaskTCB;
//任务堆栈
CPU_STK STABILIZATION_TASK_STK[STABILIZATION_STK_SIZE];
//led0任务
void stabilization_task(void *p_arg);



//config 参数配置任务 在线调试参数并写入flash
//设置任务优先级
#define TRANSMISSION_TASK_PRIO 6			// 这个任务的优先级对串口收发的速度影响很大，不可太低，更改之后要重新测试串口3通信
//任务堆栈大小
#define TRANSMISSION_STK_SIZE 1024
//任务控制块
OS_TCB TransmissionTaskTCB;
//任务堆栈
CPU_STK TRANSMISSION_TASK_STK[TRANSMISSION_STK_SIZE];
//motor任务
uint8_t transmission_task(void *p_arg);





//开始任务函数
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;
	BSP_Init();

#if OS_CFG_STAT_TASK_EN > 0u
	OSStatTaskCPUUsageInit(&err); //统计任务
#endif

#ifdef CPU_CFG_INT_DIS_MEAS_EN //如果使能了测量中断关闭时间
	CPU_IntDisMeasMaxCurReset();
#endif

#if OS_CFG_SCHED_ROUND_ROBIN_EN //当使用时间片轮转的时候
	//使能时间片轮转调度功能,设置默认的时间片长度
	OSSchedRoundRobinCfg(DEF_ENABLED, 1, &err);
#endif
	__HAL_RCC_CRC_CLK_ENABLE(); //使能CRC时钟

	OS_CRITICAL_ENTER();		//进入临界区

	//communicate任务 通信任务
	OSTaskCreate((OS_TCB *)&CommunicateTaskTCB,
				 (CPU_CHAR *)"Communicate task",
				 (OS_TASK_PTR)communicate_task,
				 (void *)0,
				 (OS_PRIO)COMMUNICATE_TASK_PRIO,
				 (CPU_STK *)&COMMUNICATE_TASK_STK[0],
				 (CPU_STK_SIZE)COMMUNICATE_STK_SIZE / 10,
				 (CPU_STK_SIZE)COMMUNICATE_STK_SIZE,
				 (OS_MSG_QTY)0,
				 (OS_TICK)10,
				 (void *)0,
				 (OS_OPT)OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP,
				 (OS_ERR *)&err);
	
				 
				 //Sensor任务
	OSTaskCreate((OS_TCB *)&SensorTaskTCB,
				(CPU_CHAR *)"Sensor task",
				(OS_TASK_PTR)sensor_task,
				(void *)0,
				(OS_PRIO)SENSOR_TASK_PRIO,
				(CPU_STK *)&SENSOR_TASK_STK[0],
				(CPU_STK_SIZE)SENSOR_STK_SIZE / 10,
				(CPU_STK_SIZE)SENSOR_STK_SIZE,
				(OS_MSG_QTY)0,
				(OS_TICK)10,
				(void *)0,
				(OS_OPT)OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR |OS_OPT_TASK_SAVE_FP,
				(OS_ERR *)&err);

				 
	OSTaskCreate((OS_TCB *)&StabilizationTaskTCB,
				 (CPU_CHAR *)"Stabilization task",
				 (OS_TASK_PTR)stabilization_task,
				 (void *)0,
				 (OS_PRIO)STABILIZATION_TASK_PRIO,
				 (CPU_STK *)&STABILIZATION_TASK_STK[0],
				 (CPU_STK_SIZE)STABILIZATION_STK_SIZE / 10,
				 (CPU_STK_SIZE)STABILIZATION_STK_SIZE,
				 (OS_MSG_QTY)0,
				 (OS_TICK)10,
				 (void *)0,
				 (OS_OPT)OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR |OS_OPT_TASK_SAVE_FP,
				 (OS_ERR *)&err);
	
//	OSTaskCreate((OS_TCB *)&TransmissionTaskTCB,
//				 (CPU_CHAR *)"Transmission task",
//				 (OS_TASK_PTR)transmission_task,
//				 (void *)0,
//				 (OS_PRIO)TRANSMISSION_TASK_PRIO,
//				 (CPU_STK *)&TRANSMISSION_TASK_STK[0],
//				 (CPU_STK_SIZE)TRANSMISSION_STK_SIZE / 10,
//				 (CPU_STK_SIZE)TRANSMISSION_STK_SIZE,
//				 (OS_MSG_QTY)0,
//				 (OS_TICK)10,
//				 (void *)0,
//				 (OS_OPT)OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP,
//				 (OS_ERR *)&err);

				 
				 
	OS_TaskSuspend((OS_TCB *)&StartTaskTCB, &err); //挂起开始任务

	OS_CRITICAL_EXIT(); //退出临界区
}

// 在线调试参数用
// 上传参数到地面站
uint8_t transmission_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	uint8_t lenth;

	lenth = sizeof(configParam);
	lenth = lenth / 4 + (lenth % 4 ? 1 : 0);
	STMFLASH_Read(CONFIG_PARAM_ADDR, (uint32_t *)&configParam, lenth);
	
	while (1)
	{
//		uploadPID();
//		uploadStateInfo();
//		uploadMotoInfo();
//		uploadPowerAndWaterInfo();
//		uploadIMUInfo();
//		uploadGPSInfo();

		// // 校验值不一致，说明参数发生了改变,更新到flash中
		if (configParamCksum(&configParam) != configParam.cksum) // 参数的校验值
		{
			INFO("enter this progress\r\n");
			configParam.cksum = configParamCksum(&configParam);  // 更新校验值
			STMFLASH_Write(CONFIG_PARAM_ADDR, (uint32_t *)&configParam, lenth); /*写入stm32 flash*/
			attitudeResetAllPID();												 //PID复位
			positionResetAllPID();
			attitudeControlInit(); // 用新的参数重新初始化pid
			positionControlInit();
		}

		OSTimeDly(20, OS_OPT_TIME_DLY, &err);
	}
}



void stabilization_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
//    motor_test();
	uint8_t lenth = sizeof(configParam);
	lenth = lenth / 4 + (lenth % 4 ? 1 : 0);
	STMFLASH_Read(CONFIG_PARAM_ADDR, (uint32_t *)&configParam, lenth);   //载入PID参数
	attitudeControlInit();
	while(1)
	{
	    Water_Attitude_Control(&control);
		pwmControl(&control);
		OSTimeDly(60, OS_OPT_TIME_DLY, &err);
	}
}


uint8_t sensor_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	
	float Gyro[3], Angle[3];
	uint8_t count = 20;
	
	HAL_UART_Receive_DMA(&huart1,aRxBuffer1,sizeof(aRxBuffer1));
	HAL_UART_Receive_DMA(&huart5,aRxBuffer2,sizeof(aRxBuffer2));

	//滤波初始化

	while (count--)
	{
		sensorReadAngle(Gyro, Angle);
	}

	// 初始化之后，所有期望值复制为实际值
	state.realAngle.roll = Angle[0];
	state.realAngle.pitch = Angle[1];
	state.realAngle.yaw = Angle[2];
	setstate.expectedAngle.roll = state.realAngle.roll;
	setstate.expectedAngle.pitch = state.realAngle.pitch;
	setstate.expectedAngle.yaw = state.realAngle.yaw; //初始化之后将当前的姿态角作为期望姿态角初值

	while (1)
	{

		/********************************************** 获取期望值与测量值*******************************************/
		sensorReadAngle(Gyro, Angle);
		//反馈值
		state.realAngle.roll = Angle[0];
		state.realAngle.pitch = Angle[1];
		state.realAngle.yaw = Angle[2];
//		INFO("Agnle:%f,%f,%f\n ",Angle[0],Angle[1],Angle[2]);
		state.realRate.roll = Gyro[0];
		state.realRate.pitch = Gyro[1];
		state.realRate.yaw = Gyro[2];
		OSTimeDly(30, OS_OPT_TIME_DLY, &err);
		
	}
}

//void Water_Attitude_Control2(control_t *output)
//{
//    float turn_speed;
//    float z_speed;
//    float forward_speed;

///*************************** 针对 手动模式和定高模式 以及 模式切换做预处理 ***********************************/
//    // 手动模式下油门通道转化为油门基准值
//    if (command[CARRY_MODE] == HAND_MODE)
//    {
//        posPid.isAltHoldMode = false;
//    }
//    // 定高（定深）时油门保持在设定的基准值，这个基准值刚好让机器人悬浮，油门通道此时转化为 z 轴速度
//    else if (command[CARRY_MODE] == DEPTH_MODE)
//    {
//        posPid.isAltHoldMode = true;
//    }
//    // 由手动模式切换到定深模式 // 由定深模式切换到手动模式
//    if ((posPid.isAltHoldMode == true && posPid.preMode == false) ||
//        (posPid.isAltHoldMode == false && posPid.preMode == true))
//    {
//        positionResetAllPID();
//        control.depthOut = 0; // 深度环PID置0
//    }
//    posPid.preMode = posPid.isAltHoldMode; // 当前模式变为pre模式

//    turn_speed = pwm2Range(command[YAW], -1000.0f, 1000.0f);
//    if (turn_speed < 30.0f && turn_speed > -30.0f)
//        turn_speed = 0.f; // 死区
//    z_speed = pwm2Range(command[THROTTLE], -1000.0f, 1000.0f);
//    if (z_speed < 30.0f && z_speed > -30.0f)
//        z_speed = 0.f; // 死区
///*************************** 针对 手动模式和定高模式 以及 模式切换做预处理 ***********************************/

///*************************************** 定高模式下控制 ************************************************/
//    // 高度环 PID 计算并作用到油门值，调整直到达到悬浮状态
//    if (posPid.isAltHoldMode == true)
//    {
//        // 不使能时，电机锁定
//        if (command[ALTHOLD_ENABLE] == HOLD_DISABLE) // 定高禁止，不运动，复位所有PID
//        {
//            attitudeResetAllPID(); //PID复位
//            positionResetAllPID();
//            setstate.expectedAngle.yaw = state.realAngle.yaw;
//            setstate.expectedAngle.roll = state.realAngle.roll;
//            setstate.expectedAngle.pitch = state.realAngle.pitch;
//            setstate.expectedDepth = state.realDepth;
//            control.thrust = 0;
//            control.yaw = 0;
//            control.roll = 0;
//            control.pitch = 0;
//            control.depthOut = 0; // 深度环PID置0
//        }
//        // 使能时 开始定高控制
//        else if (command[ALTHOLD_ENABLE] == HOLD_ENABLE)
//        {
//            setstate.expectedAngle.roll = pwm2Range(command[ROLL], -30.0f, 30.0f);
//            if (setstate.expectedAngle.roll < 0.9f && setstate.expectedAngle.roll > -0.9f)
//                setstate.expectedAngle.roll = 0.f; // 摇杆死区
//            setstate.expectedAngle.pitch = pwm2Range(command[PITCH], -30.0f, 30.0f);
//            if (setstate.expectedAngle.pitch < 0.9f && setstate.expectedAngle.pitch > -0.9f)
//                setstate.expectedAngle.pitch = 0.f; //遥感死区

////            setstate.expectedAngle.yaw -= turn_speed * zoom_factor_yaw * ft;
//            if (setstate.expectedAngle.yaw > 180.0f)
//                setstate.expectedAngle.yaw -= 360.0f;
//            if (setstate.expectedAngle.yaw < -180.0f)
//                setstate.expectedAngle.yaw += 360.0f;
////            setstate.expectedDepth -= z_speed * zoom_factor_vz * ft; // 向上调整时期望深度减小
//            depthPID(&state.realDepth, &setstate.expectedDepth, &control);
//            attitudeAnglePID(&state.realAngle, &setstate.expectedAngle, &setstate.expectedRate); /* 角度环PID */
//            attitudeRatePID(&state.realRate, &setstate.expectedRate, &control);                  /* 角速度环PID */
//        }
//    }
///*************************************** 定高模式下控制 ************************************************/

///******************************************* 手动模式下控制 ********************************************/
//    if (posPid.isAltHoldMode == false)
//    {
//        // 手动模式下 油门通道为0时不运动，复位所有 姿态pid与pid输出
//        control.thrust = pwm2thrust(command[THROTTLE]);
//        setstate.expectedDepth = state.realDepth;          // 手动模式下期望高度始终等于当前高度/
//                                                           // 切换到定高时从当前高度开始定高
//        if (control.thrust < 200 && control.thrust > -200) 
//            control.thrust = 0;                            // 油门死区

//        if ((int)control.thrust == 0 && (int)turn_speed == 0) // 油门和方向摇杆都居中，机器人不使能
//        {
//            attitudeResetAllPID(); //PID复位
//            setstate.expectedAngle.yaw = state.realAngle.yaw;
//            setstate.expectedAngle.roll = state.realAngle.roll;
//            setstate.expectedAngle.pitch = state.realAngle.pitch;
//            control.yaw = 0;
//            control.roll = 0;
//            control.pitch = 0;
//        }
//        else // 油门有输出，从遥控器获得期望值，姿态PID		// 或者油门没输出，原地转圈
//        {
//            setstate.expectedAngle.roll = pwm2Range(command[ROLL], -30.0f, 30.0f);
//            if (setstate.expectedAngle.roll < 0.9f && setstate.expectedAngle.roll > -0.9f)
//                setstate.expectedAngle.roll = 0.f; // 摇杆死区
//            setstate.expectedAngle.pitch = pwm2Range(command[PITCH], -30.0f, 30.0f);
//            if (setstate.expectedAngle.pitch < 0.9f && setstate.expectedAngle.pitch > -0.9f)
//                setstate.expectedAngle.pitch = 0.f; //遥感死区	
////            setstate.expectedAngle.yaw -= turn_speed * zoom_factor_yaw * ft;
//            if (setstate.expectedAngle.yaw > 180.0f)
//                setstate.expectedAngle.yaw -= 360.0f;
//            if (setstate.expectedAngle.yaw < -180.0f)
//                setstate.expectedAngle.yaw += 360.0f;
//            attitudeAnglePID(&state.realAngle, &setstate.expectedAngle, &setstate.expectedRate); /* 角度环PID */
//            attitudeRatePID(&state.realRate, &setstate.expectedRate, &control);                  /* 角速度环PID */
//        }
//    }
///******************************************* 手动模式下控制 ********************************************/
//}




void Water_Attitude_Control(control_t *output)
{

	control.forward_thrust=order.velocity[FORWARD_SPEED];
	control.turn_thrust=order.velocity[turn_SPEED];
	
/******************************************* 手动模式下控制 ********************************************/
    if (order.model[CARRY_MODE] == HAND_MODE)
    {

        // 手动模式下 油门通道为0时不运动，复位所有 姿态pid与pid输出
        setstate.expectedDepth = state.realDepth;          // 手动模式下期望高度始终等于当前高度/
                                                           // 切换到定高时从当前高度开始定高		
        if (order.velocity[THROTTLE] < 200 && order.velocity[THROTTLE] > -200) 
            order.velocity[THROTTLE] = 0;                            // 油门死区
				
		if (order.angle[YAW] < 30.0f && order.angle[YAW] > -30.0f)
			order.angle[YAW]= 0.f; // 死区
		
        if ((int)order.velocity[THROTTLE] == 1 && (int)order.angle[YAW] == 1) // 油门和方向摇杆都居中，机器人不使能
        {
            attitudeResetAllPID(); //PID复位
            setstate.expectedAngle.yaw = state.realAngle.yaw;
            setstate.expectedAngle.roll = state.realAngle.roll;
            setstate.expectedAngle.pitch = state.realAngle.pitch;
            control.yaw = 0;
            control.roll = 0;
            control.pitch = 0;
        }
        else // 油门有输出，从遥控器获得期望值，姿态PID		// 或者油门没输出，原地转圈
        {
			control.thrust=order.velocity[THROTTLE];
			setstate.expectedAngle.roll = order.angle[ROLL];
            if (setstate.expectedAngle.roll < 0.9f && setstate.expectedAngle.roll > -0.9f)
                setstate.expectedAngle.roll = 0.f; // 摇杆死区
			setstate.expectedAngle.pitch=order.angle[PITCH];
            if (setstate.expectedAngle.pitch < 0.9f && setstate.expectedAngle.pitch > -0.9f)
                setstate.expectedAngle.pitch = 0.f; //遥感死区	
            setstate.expectedAngle.yaw = order.angle[YAW];//need  adjustment;
            if (setstate.expectedAngle.yaw > 180.0f)
                setstate.expectedAngle.yaw -= 360.0f; 
            if (setstate.expectedAngle.yaw < -180.0f)
                setstate.expectedAngle.yaw += 360.0f;
//			INFO("order:%f,%f,%f,%f\n",order.angle[ROLL],order.angle[PITCH],order.angle[YAW],order.velocity[THROTTLE]);
//			INFO("realangle:%f,%f,%f\n",state.realAngle.roll,state.realAngle.pitch,state.realAngle.yaw);
//			INFO("setstate:%f,%f,%f\n",setstate.expectedAngle.roll,setstate.expectedAngle.pitch,setstate.expectedAngle.yaw);
            attitudeAnglePID(&state.realAngle, &setstate.expectedAngle, &setstate.expectedRate); /* 角度环PID */
//			INFO("setstate.expectedRate:%f,%f,%f\n",setstate.expectedRate.roll,setstate.expectedRate.pitch,setstate.expectedRate.yaw);
            attitudeRatePID(&state.realRate, &setstate.expectedRate, &control);                  /* 角速度环PID */
			INFO("control:%f,%f,%f,%f\n",control.roll,control.pitch,control.yaw,control.thrust);

			
        }
    }
/******************************************* 手动模式下控制 ********************************************/
}


void communicate_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();

	//等待遥控器通讯正常，否则一直等待。遥控器校正，初值初始化
	//读遥控器数据
	while (1)
	{
		UART_ReadCOMMAND(order.angle,order.velocity,order.model);
		OSTimeDly(40, OS_OPT_TIME_DLY, &err);
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if (huart==&huart5) //如果是串口5
	{
		CopeSerial5Data(aRxBuffer2[0]);
	}

	if(huart->Instance==USART1)//如果是串口1
	{
		CopeSerialData(aRxBuffer1[0]);
//		INFO("%d",aRxBuffer1[0]);
	}

	if (huart->Instance==USART3)//如果是串口3
	{
			
	}

	if (huart->Instance == UART4) //如果是串口4
	{
		
	}
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

										 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */





/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  OS_ERR err;
  CPU_SR_ALLOC();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  PWM_init();
	OSInit(&err);		//初始化UCOSIII
    OS_CRITICAL_ENTER();//进入临界区
    //创建开始任务
    OSTaskCreate((OS_TCB 	* )&StartTaskTCB,			//任务控制块
                 (CPU_CHAR* )"start task", 				//任务名字
                 (OS_TASK_PTR)start_task, 				//任务函数
                 (void		* )0,						//传递给任务函数的参数
                 (OS_PRIO	  )START_TASK_PRIO,     //任务优先级
                 (CPU_STK * )&START_TASK_STK[0],	//任务堆栈基地址
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//任务堆栈深度限位
                 (CPU_STK_SIZE)START_STK_SIZE,		//任务堆栈大小
                 (OS_MSG_QTY)0,						//任务内部消息队列能够接收的最大消息数目,为0时禁止接收消息
                 (OS_TICK	  )0,						//当使能时间片轮转时的时间片长度，为0时为默认长度，
                 (void   	* )0,						//用户补充的存储区
                 (OS_OPT    )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //任务选项
                 (OS_ERR 	* )&err);				//存放该函数错误时的返回值
    OS_CRITICAL_EXIT();	//退出临界区	

    OSStart(&err); //启动多任务系统，控制权交给uC/OS-III
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE BEGIN 2 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

#pragma pack()

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
