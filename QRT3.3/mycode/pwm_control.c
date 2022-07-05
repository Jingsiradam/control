#include "pwm_control.h"


float limitThrust(float value)
{
    if (value > THRUST_MAX)
    {
        value = THRUST_MAX;
    }
    else if (value < THRUST_MIN)
    {
        value = THRUST_MIN;
    }
    return value;
}

uint16_t limitPWM(int input)
{
    if (input > 2000)
        return 2000;
    else if (input < 1000)
        return 1000;
    else
        return input;
}



void setMotorPWM(float m1_set, float m2_set, float m3_set, float m4_set, float m5_set, float m6_set)
{
    // 直接设定pwm值 
	set_motorspeed(m1_set,1);
	set_motorspeed(m2_set,2);
	set_motorspeed(m3_set,3);
	set_motorspeed(m4_set,4);
	set_motorspeed(m5_set,5);
	set_motorspeed(m6_set,6);
}

// 油门值转化为 PWM 值
uint16_t thrust2pwm(float thrust_value)
{
    float pwm_value;
    // 油门值转化为 PWM 1000-2000, [THRUST_MIN, THRUST_MAX] 到 [1000,2000]的映射
    pwm_value = 1000.0f + (thrust_value - THRUST_MIN) * 1000.0f / (THRUST_MAX - THRUST_MIN);
    return limitPWM((int)pwm_value);
}

float pwm2thrust(uint16_t pwm_value)
{
    float thrust_value;
    thrust_value = THRUST_MIN + ((float)pwm_value - 1000.0f) * (THRUST_MAX - THRUST_MIN) / 1000.0f;
    return limitThrust(thrust_value);
}

uint16_t thrust2pwmRange(float thrust_value, float p_min, float p_max)
{
    float pwm_value;
    // 油门值转化为 PWM pmin-pmax, [THRUST_MIN, THRUST_MAX] 到 [p_min,p_max]的映射
    pwm_value = p_min + (thrust_value - THRUST_MIN) * (p_max - p_min) / (THRUST_MAX - THRUST_MIN);
    return limitPWM((int)pwm_value);
}

float pwm2Range(int pwm_value, float p_min, float p_max)
{
    float p;
    p = p_min + ((float)(pwm_value)-1000.0f) * (p_max - p_min) / 1000.0f;
    if (p > p_max)
        p = p_max;
    if (p < p_min)
        p = p_min;
    return p;
}

void pwmControl(control_t *control) /*功率输出控制*/
{
    float r = (float)control->roll * 0.4472136f;
    float p = (float)control->pitch * 0.8944272f;
    float y = (float)control->yaw;
	
    float forward=(float)control->forward_thrust;
    float turnspeed=(float)control->turn_thrust;
		
    float thrust1, thrust2, thrust3, thrust4,thrust5,thrust6;
    float m1,m2,m3,m4,m5,m6;
	
	thrust1 = limitThrust(control->thrust + r + p + y); //      |--------> Y
	thrust2 = limitThrust(control->thrust + r - p - y); //      |
	thrust3 = limitThrust(control->thrust - r - p + y); //      V
	thrust4 = limitThrust(control->thrust - r + p - y); //	     X

	
	m1=-thrust2pwmRange(thrust1,-2000,2000);
	m2=thrust2pwmRange(thrust2,-2000,2000);
	m3=-thrust2pwmRange(thrust3,-2000,2000);
	m4=thrust2pwmRange(thrust4,-2000,2000);
	
	m5 = -(forward+0.3f*turnspeed);
	m6 = forward-0.3f*turnspeed;

	setMotorPWM(m1,m2,m3,m4,m5,m6);


}

