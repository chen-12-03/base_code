#include "motor_control.h"

// CAN通信初始化
void CAN_Filter_Init(CAN_HandleTypeDef *hcan)
{
  CAN_FilterTypeDef can_filter;
  can_filter.FilterIdHigh = 0x0000;
  can_filter.FilterIdLow = 0x0000;
  can_filter.FilterMaskIdHigh = 0x0000;
  can_filter.FilterMaskIdLow = 0x0000;
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  can_filter.FilterBank = 0;
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterActivation = ENABLE;

  HAL_CAN_ConfigFilter(hcan, &can_filter);
  HAL_CAN_Start(hcan);
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

// CAN接收回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

  // 数据更新
  Update_Angle_Data(rx_data);
  motor.speed_rpm = (rx_data[2] << 8) | rx_data[3];
  motor.torque_current = (int16_t)(rx_data[4] << 8) | rx_data[5];
  motor.temp = rx_data[6];
}

// 电机控制
void cmd_motor(CAN_HandleTypeDef *hcan, uint32_t stdid, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
  CAN_TxHeaderTypeDef motor_tx_message;
  uint32_t send_mail_box;
  uint8_t can_tx_data[8];

  motor_tx_message.StdId = stdid;
  motor_tx_message.IDE = CAN_ID_STD;
  motor_tx_message.RTR = CAN_RTR_DATA;
  motor_tx_message.DLC = 0x08;

  can_tx_data[0] = motor1 >> 8;
  can_tx_data[1] = motor1;
  can_tx_data[2] = motor2 >> 8;
  can_tx_data[3] = motor2;
  can_tx_data[4] = motor3 >> 8;
  can_tx_data[5] = motor3;
  can_tx_data[6] = motor4 >> 8;
  can_tx_data[7] = motor4;

  HAL_CAN_AddTxMessage(hcan, &motor_tx_message, can_tx_data, &send_mail_box);
}

// 角度处理函数（CAN回调中调用）+滤波
void Update_Angle_Data(uint8_t *rx_data)
{
  static int32_t last_angle = 0;
  static float filtered_angle = 0.0f;

  // 获取原始角度并滤波
  int32_t new_angle = (rx_data[0] << 8) | rx_data[1];
  motor.raw_angle = new_angle;
//  filtered_angle = FILTER_ALPHA * new_angle + (1 - FILTER_ALPHA) * filtered_angle;
//  int32_t filtered_angle_int = (int32_t)filtered_angle;

  // 处理突变
//  int32_t delta = filtered_angle_int - last_angle;
  int32_t delta = new_angle - last_angle;
  const int32_t overflow_threshold = 8192 / 2; // 4096 (半圈阈值)

  if (delta > overflow_threshold) // 逆转过圈
  {
    delta -= 8192;
  }
  else if (delta < -overflow_threshold) // 正转过圈
  {
    delta += 8192;
  }

  motor.total_angle += delta;
  last_angle = new_angle;
}
