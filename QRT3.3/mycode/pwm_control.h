#ifndef __POWER_CONTROL_H
#define __POWER_CONTROL_H


#include "stabilizer.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "motortest.h"

#define KV 500.0f           // 500 KV µç»ú
#define VOTAGE 4.0f
#define THRUST_MAX (KV*VOTAGE)
#define THRUST_MIN (-KV*VOTAGE)



void pwmControl(control_t *control);
void setMotorPWM(float m1_set, float m2_set, float m3_set, float m4_set, float m5_set, float m6_set);
uint16_t thrust2pwm(float thrust_value);
uint16_t thrust2pwmRange(float thrust_value, float p_min, float p_max);
float pwm2thrust(uint16_t pwm_value);
float pwm2Range(int pwm_value, float p_min, float p_max);
float limitThrust(float value);
uint16_t limitPWM(int input);





#endif 
