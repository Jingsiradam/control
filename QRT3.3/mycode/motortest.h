#ifndef __MOTORTEST_H
#define __MOTORTEST_H
#include "includes.h"
#include "tim.h"
void motor_test();
void set_motorspeed(float speed,uint8_t num);
void PWM_init();
void motor_init();
#endif