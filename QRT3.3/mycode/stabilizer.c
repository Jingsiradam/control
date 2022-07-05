#include "stabilizer.h"

control_t control = {0};
setstate_t setstate = {0};
state_t state = {0};
sensorData_t sensorData = {0};

motorPWM_t motorPWM = {0};
motorPWM_t motorPWMSet = {0, 0, 0, 0};
