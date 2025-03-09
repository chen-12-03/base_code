#ifndef PID_H
#define PID_H

#include "stm32f1xx.h"
#include "arm_math.h"
#include "motor_control.h"
#include "can.h"

extern Motor_Status motor;
extern const int16_t MAX_CURRENT;

static arm_pid_instance_f32 pid={0};
static float32_t pid_output=0;
static float32_t pid_target=0;

void PID_Init();
void PID_Control(int32_t target);

#endif
