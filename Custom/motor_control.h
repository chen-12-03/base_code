#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "stm32f1xx_hal.h"

#define FILTER_ALPHA 0.4f // 低通滤波系数

/*电机数据结构体*/
typedef struct
{
  int32_t raw_angle;
  int32_t total_angle;
  int16_t speed_rpm;
  int16_t torque_current;
  uint8_t temp;
  int32_t left_limit;
  int32_t right_limit;
} Motor_Status;

extern Motor_Status motor;
void CAN_Filter_Init(CAN_HandleTypeDef *hcan);
void Update_Angle_Data(uint8_t *rx_data);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void cmd_motor(CAN_HandleTypeDef *hcan, uint32_t stdid, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

#endif
