#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include "stm32f1xx_hal.h"
#include "stdbool.h"
#include "can.h"
#include "arm_math.h"
#include "motor_control.h"
#include "stdlib.h"
#include "pid.h"

/*工作状态枚举*/
typedef enum
{
  MODE_IDLE,
  MODE_PENDULUM,
  MODE_POSITION,
  MODE_INIT
} WorkMode;

extern volatile int32_t target_pos;
extern Motor_Status motor;
extern const int16_t MAX_CURRENT;
extern WorkMode work_mode;
void Limit_Init(); // 左右端角度初始化
void Motor_Control_Task();

#endif
