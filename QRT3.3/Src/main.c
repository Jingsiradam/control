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
//�����ջ��С
#define START_STK_SIZE 128
//������ƿ�
OS_TCB StartTaskTCB;
//�����ջ
CPU_STK START_TASK_STK[START_STK_SIZE];
//������
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


//communicate����
//�����������ȼ�
#define COMMUNICATE_TASK_PRIO 6					// SBUS �źŵĸ������ڴ����ж��н��е�
//�����ջ��С
#define COMMUNICATE_STK_SIZE 512
//������ƿ�
OS_TCB CommunicateTaskTCB;
//�����ջ
CPU_STK COMMUNICATE_TASK_STK[COMMUNICATE_STK_SIZE];
//led0����
void communicate_task(void *p_arg);

//sensorTask ������������ ���ߵ��Բ�����д��flash
//�����������ȼ�
#define SENSOR_TASK_PRIO 5
//�����ջ��С
#define SENSOR_STK_SIZE 512
//������ƿ�
OS_TCB SensorTaskTCB;
//�����ջ
CPU_STK SENSOR_TASK_STK[SENSOR_STK_SIZE];
//motor����
uint8_t sensor_task(void *p_arg);


//stabalizer����
//�����������ȼ�
#define STABILIZATION_TASK_PRIO 4
//�����ջ��С
#define STABILIZATION_STK_SIZE 2048
//������ƿ�
OS_TCB StabilizationTaskTCB;
//�����ջ
CPU_STK STABILIZATION_TASK_STK[STABILIZATION_STK_SIZE];
//led0����
void stabilization_task(void *p_arg);



//config ������������ ���ߵ��Բ�����д��flash
//�����������ȼ�
#define TRANSMISSION_TASK_PRIO 6			// �����������ȼ��Դ����շ����ٶ�Ӱ��ܴ󣬲���̫�ͣ�����֮��Ҫ���²��Դ���3ͨ��
//�����ջ��С
#define TRANSMISSION_STK_SIZE 1024
//������ƿ�
OS_TCB TransmissionTaskTCB;
//�����ջ
CPU_STK TRANSMISSION_TASK_STK[TRANSMISSION_STK_SIZE];
//motor����
uint8_t transmission_task(void *p_arg);





//��ʼ������
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;
	BSP_Init();

#if OS_CFG_STAT_TASK_EN > 0u
	OSStatTaskCPUUsageInit(&err); //ͳ������
#endif

#ifdef CPU_CFG_INT_DIS_MEAS_EN //���ʹ���˲����жϹر�ʱ��
	CPU_IntDisMeasMaxCurReset();
#endif

#if OS_CFG_SCHED_ROUND_ROBIN_EN //��ʹ��ʱ��Ƭ��ת��ʱ��
	//ʹ��ʱ��Ƭ��ת���ȹ���,����Ĭ�ϵ�ʱ��Ƭ����
	OSSchedRoundRobinCfg(DEF_ENABLED, 1, &err);
#endif
	__HAL_RCC_CRC_CLK_ENABLE(); //ʹ��CRCʱ��

	OS_CRITICAL_ENTER();		//�����ٽ���

	//communicate���� ͨ������
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
	
				 
				 //Sensor����
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

				 
				 
	OS_TaskSuspend((OS_TCB *)&StartTaskTCB, &err); //����ʼ����

	OS_CRITICAL_EXIT(); //�˳��ٽ���
}

// ���ߵ��Բ�����
// �ϴ�����������վ
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

		// // У��ֵ��һ�£�˵�����������˸ı�,���µ�flash��
		if (configParamCksum(&configParam) != configParam.cksum) // ������У��ֵ
		{
			INFO("enter this progress\r\n");
			configParam.cksum = configParamCksum(&configParam);  // ����У��ֵ
			STMFLASH_Write(CONFIG_PARAM_ADDR, (uint32_t *)&configParam, lenth); /*д��stm32 flash*/
			attitudeResetAllPID();												 //PID��λ
			positionResetAllPID();
			attitudeControlInit(); // ���µĲ������³�ʼ��pid
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
	STMFLASH_Read(CONFIG_PARAM_ADDR, (uint32_t *)&configParam, lenth);   //����PID����
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

	//�˲���ʼ��

	while (count--)
	{
		sensorReadAngle(Gyro, Angle);
	}

	// ��ʼ��֮����������ֵ����Ϊʵ��ֵ
	state.realAngle.roll = Angle[0];
	state.realAngle.pitch = Angle[1];
	state.realAngle.yaw = Angle[2];
	setstate.expectedAngle.roll = state.realAngle.roll;
	setstate.expectedAngle.pitch = state.realAngle.pitch;
	setstate.expectedAngle.yaw = state.realAngle.yaw; //��ʼ��֮�󽫵�ǰ����̬����Ϊ������̬�ǳ�ֵ

	while (1)
	{

		/********************************************** ��ȡ����ֵ�����ֵ*******************************************/
		sensorReadAngle(Gyro, Angle);
		//����ֵ
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

///*************************** ��� �ֶ�ģʽ�Ͷ���ģʽ �Լ� ģʽ�л���Ԥ���� ***********************************/
//    // �ֶ�ģʽ������ͨ��ת��Ϊ���Ż�׼ֵ
//    if (command[CARRY_MODE] == HAND_MODE)
//    {
//        posPid.isAltHoldMode = false;
//    }
//    // ���ߣ����ʱ���ű������趨�Ļ�׼ֵ�������׼ֵ�պ��û���������������ͨ����ʱת��Ϊ z ���ٶ�
//    else if (command[CARRY_MODE] == DEPTH_MODE)
//    {
//        posPid.isAltHoldMode = true;
//    }
//    // ���ֶ�ģʽ�л�������ģʽ // �ɶ���ģʽ�л����ֶ�ģʽ
//    if ((posPid.isAltHoldMode == true && posPid.preMode == false) ||
//        (posPid.isAltHoldMode == false && posPid.preMode == true))
//    {
//        positionResetAllPID();
//        control.depthOut = 0; // ��Ȼ�PID��0
//    }
//    posPid.preMode = posPid.isAltHoldMode; // ��ǰģʽ��Ϊpreģʽ

//    turn_speed = pwm2Range(command[YAW], -1000.0f, 1000.0f);
//    if (turn_speed < 30.0f && turn_speed > -30.0f)
//        turn_speed = 0.f; // ����
//    z_speed = pwm2Range(command[THROTTLE], -1000.0f, 1000.0f);
//    if (z_speed < 30.0f && z_speed > -30.0f)
//        z_speed = 0.f; // ����
///*************************** ��� �ֶ�ģʽ�Ͷ���ģʽ �Լ� ģʽ�л���Ԥ���� ***********************************/

///*************************************** ����ģʽ�¿��� ************************************************/
//    // �߶Ȼ� PID ���㲢���õ�����ֵ������ֱ���ﵽ����״̬
//    if (posPid.isAltHoldMode == true)
//    {
//        // ��ʹ��ʱ���������
//        if (command[ALTHOLD_ENABLE] == HOLD_DISABLE) // ���߽�ֹ�����˶�����λ����PID
//        {
//            attitudeResetAllPID(); //PID��λ
//            positionResetAllPID();
//            setstate.expectedAngle.yaw = state.realAngle.yaw;
//            setstate.expectedAngle.roll = state.realAngle.roll;
//            setstate.expectedAngle.pitch = state.realAngle.pitch;
//            setstate.expectedDepth = state.realDepth;
//            control.thrust = 0;
//            control.yaw = 0;
//            control.roll = 0;
//            control.pitch = 0;
//            control.depthOut = 0; // ��Ȼ�PID��0
//        }
//        // ʹ��ʱ ��ʼ���߿���
//        else if (command[ALTHOLD_ENABLE] == HOLD_ENABLE)
//        {
//            setstate.expectedAngle.roll = pwm2Range(command[ROLL], -30.0f, 30.0f);
//            if (setstate.expectedAngle.roll < 0.9f && setstate.expectedAngle.roll > -0.9f)
//                setstate.expectedAngle.roll = 0.f; // ҡ������
//            setstate.expectedAngle.pitch = pwm2Range(command[PITCH], -30.0f, 30.0f);
//            if (setstate.expectedAngle.pitch < 0.9f && setstate.expectedAngle.pitch > -0.9f)
//                setstate.expectedAngle.pitch = 0.f; //ң������

////            setstate.expectedAngle.yaw -= turn_speed * zoom_factor_yaw * ft;
//            if (setstate.expectedAngle.yaw > 180.0f)
//                setstate.expectedAngle.yaw -= 360.0f;
//            if (setstate.expectedAngle.yaw < -180.0f)
//                setstate.expectedAngle.yaw += 360.0f;
////            setstate.expectedDepth -= z_speed * zoom_factor_vz * ft; // ���ϵ���ʱ������ȼ�С
//            depthPID(&state.realDepth, &setstate.expectedDepth, &control);
//            attitudeAnglePID(&state.realAngle, &setstate.expectedAngle, &setstate.expectedRate); /* �ǶȻ�PID */
//            attitudeRatePID(&state.realRate, &setstate.expectedRate, &control);                  /* ���ٶȻ�PID */
//        }
//    }
///*************************************** ����ģʽ�¿��� ************************************************/

///******************************************* �ֶ�ģʽ�¿��� ********************************************/
//    if (posPid.isAltHoldMode == false)
//    {
//        // �ֶ�ģʽ�� ����ͨ��Ϊ0ʱ���˶�����λ���� ��̬pid��pid���
//        control.thrust = pwm2thrust(command[THROTTLE]);
//        setstate.expectedDepth = state.realDepth;          // �ֶ�ģʽ�������߶�ʼ�յ��ڵ�ǰ�߶�/
//                                                           // �л�������ʱ�ӵ�ǰ�߶ȿ�ʼ����
//        if (control.thrust < 200 && control.thrust > -200) 
//            control.thrust = 0;                            // ��������

//        if ((int)control.thrust == 0 && (int)turn_speed == 0) // ���źͷ���ҡ�˶����У������˲�ʹ��
//        {
//            attitudeResetAllPID(); //PID��λ
//            setstate.expectedAngle.yaw = state.realAngle.yaw;
//            setstate.expectedAngle.roll = state.realAngle.roll;
//            setstate.expectedAngle.pitch = state.realAngle.pitch;
//            control.yaw = 0;
//            control.roll = 0;
//            control.pitch = 0;
//        }
//        else // �������������ң�����������ֵ����̬PID		// ��������û�����ԭ��תȦ
//        {
//            setstate.expectedAngle.roll = pwm2Range(command[ROLL], -30.0f, 30.0f);
//            if (setstate.expectedAngle.roll < 0.9f && setstate.expectedAngle.roll > -0.9f)
//                setstate.expectedAngle.roll = 0.f; // ҡ������
//            setstate.expectedAngle.pitch = pwm2Range(command[PITCH], -30.0f, 30.0f);
//            if (setstate.expectedAngle.pitch < 0.9f && setstate.expectedAngle.pitch > -0.9f)
//                setstate.expectedAngle.pitch = 0.f; //ң������	
////            setstate.expectedAngle.yaw -= turn_speed * zoom_factor_yaw * ft;
//            if (setstate.expectedAngle.yaw > 180.0f)
//                setstate.expectedAngle.yaw -= 360.0f;
//            if (setstate.expectedAngle.yaw < -180.0f)
//                setstate.expectedAngle.yaw += 360.0f;
//            attitudeAnglePID(&state.realAngle, &setstate.expectedAngle, &setstate.expectedRate); /* �ǶȻ�PID */
//            attitudeRatePID(&state.realRate, &setstate.expectedRate, &control);                  /* ���ٶȻ�PID */
//        }
//    }
///******************************************* �ֶ�ģʽ�¿��� ********************************************/
//}




void Water_Attitude_Control(control_t *output)
{

	control.forward_thrust=order.velocity[FORWARD_SPEED];
	control.turn_thrust=order.velocity[turn_SPEED];
	
/******************************************* �ֶ�ģʽ�¿��� ********************************************/
    if (order.model[CARRY_MODE] == HAND_MODE)
    {

        // �ֶ�ģʽ�� ����ͨ��Ϊ0ʱ���˶�����λ���� ��̬pid��pid���
        setstate.expectedDepth = state.realDepth;          // �ֶ�ģʽ�������߶�ʼ�յ��ڵ�ǰ�߶�/
                                                           // �л�������ʱ�ӵ�ǰ�߶ȿ�ʼ����		
        if (order.velocity[THROTTLE] < 200 && order.velocity[THROTTLE] > -200) 
            order.velocity[THROTTLE] = 0;                            // ��������
				
		if (order.angle[YAW] < 30.0f && order.angle[YAW] > -30.0f)
			order.angle[YAW]= 0.f; // ����
		
        if ((int)order.velocity[THROTTLE] == 1 && (int)order.angle[YAW] == 1) // ���źͷ���ҡ�˶����У������˲�ʹ��
        {
            attitudeResetAllPID(); //PID��λ
            setstate.expectedAngle.yaw = state.realAngle.yaw;
            setstate.expectedAngle.roll = state.realAngle.roll;
            setstate.expectedAngle.pitch = state.realAngle.pitch;
            control.yaw = 0;
            control.roll = 0;
            control.pitch = 0;
        }
        else // �������������ң�����������ֵ����̬PID		// ��������û�����ԭ��תȦ
        {
			control.thrust=order.velocity[THROTTLE];
			setstate.expectedAngle.roll = order.angle[ROLL];
            if (setstate.expectedAngle.roll < 0.9f && setstate.expectedAngle.roll > -0.9f)
                setstate.expectedAngle.roll = 0.f; // ҡ������
			setstate.expectedAngle.pitch=order.angle[PITCH];
            if (setstate.expectedAngle.pitch < 0.9f && setstate.expectedAngle.pitch > -0.9f)
                setstate.expectedAngle.pitch = 0.f; //ң������	
            setstate.expectedAngle.yaw = order.angle[YAW];//need  adjustment;
            if (setstate.expectedAngle.yaw > 180.0f)
                setstate.expectedAngle.yaw -= 360.0f; 
            if (setstate.expectedAngle.yaw < -180.0f)
                setstate.expectedAngle.yaw += 360.0f;
//			INFO("order:%f,%f,%f,%f\n",order.angle[ROLL],order.angle[PITCH],order.angle[YAW],order.velocity[THROTTLE]);
//			INFO("realangle:%f,%f,%f\n",state.realAngle.roll,state.realAngle.pitch,state.realAngle.yaw);
//			INFO("setstate:%f,%f,%f\n",setstate.expectedAngle.roll,setstate.expectedAngle.pitch,setstate.expectedAngle.yaw);
            attitudeAnglePID(&state.realAngle, &setstate.expectedAngle, &setstate.expectedRate); /* �ǶȻ�PID */
//			INFO("setstate.expectedRate:%f,%f,%f\n",setstate.expectedRate.roll,setstate.expectedRate.pitch,setstate.expectedRate.yaw);
            attitudeRatePID(&state.realRate, &setstate.expectedRate, &control);                  /* ���ٶȻ�PID */
			INFO("control:%f,%f,%f,%f\n",control.roll,control.pitch,control.yaw,control.thrust);

			
        }
    }
/******************************************* �ֶ�ģʽ�¿��� ********************************************/
}


void communicate_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();

	//�ȴ�ң����ͨѶ����������һֱ�ȴ���ң����У������ֵ��ʼ��
	//��ң��������
	while (1)
	{
		UART_ReadCOMMAND(order.angle,order.velocity,order.model);
		OSTimeDly(40, OS_OPT_TIME_DLY, &err);
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if (huart==&huart5) //����Ǵ���5
	{
		CopeSerial5Data(aRxBuffer2[0]);
	}

	if(huart->Instance==USART1)//����Ǵ���1
	{
		CopeSerialData(aRxBuffer1[0]);
//		INFO("%d",aRxBuffer1[0]);
	}

	if (huart->Instance==USART3)//����Ǵ���3
	{
			
	}

	if (huart->Instance == UART4) //����Ǵ���4
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
	OSInit(&err);		//��ʼ��UCOSIII
    OS_CRITICAL_ENTER();//�����ٽ���
    //������ʼ����
    OSTaskCreate((OS_TCB 	* )&StartTaskTCB,			//������ƿ�
                 (CPU_CHAR* )"start task", 				//��������
                 (OS_TASK_PTR)start_task, 				//������
                 (void		* )0,						//���ݸ��������Ĳ���
                 (OS_PRIO	  )START_TASK_PRIO,     //�������ȼ�
                 (CPU_STK * )&START_TASK_STK[0],	//�����ջ����ַ
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//�����ջ�����λ
                 (CPU_STK_SIZE)START_STK_SIZE,		//�����ջ��С
                 (OS_MSG_QTY)0,						//�����ڲ���Ϣ�����ܹ����յ������Ϣ��Ŀ,Ϊ0ʱ��ֹ������Ϣ
                 (OS_TICK	  )0,						//��ʹ��ʱ��Ƭ��תʱ��ʱ��Ƭ���ȣ�Ϊ0ʱΪĬ�ϳ��ȣ�
                 (void   	* )0,						//�û�����Ĵ洢��
                 (OS_OPT    )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //����ѡ��
                 (OS_ERR 	* )&err);				//��Ÿú�������ʱ�ķ���ֵ
    OS_CRITICAL_EXIT();	//�˳��ٽ���	

    OSStart(&err); //����������ϵͳ������Ȩ����uC/OS-III
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
