# 通信协议
## 电机CAN通信
1 000 000 HZ
```C
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

// 角度处理函数（CAN回调中调用）
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
```
## PID控制
```C
// PID初始化 参数待调
const float PID_KP = 10.0f;
const float PID_KI = 1.0f;
const float PID_KD = 3.0f;
void PID_Init()
{
  pid.Kp = PID_KP;
  pid.Ki = PID_KI;
  pid.Kd = PID_KD;
  arm_pid_init_f32(&pid, 1);
}

void PID_Control(int32_t target)
{

  const float integral_max = MAX_CURRENT * 1.5f; // 积分最大值
  const float integral_sep_threshold = 500.0f; // 积分分离阈值
  const float deadzone_threshold = 5000.0f; // 死区阈值
  const float min_effective_current = MAX_CURRENT * 0.25f; // 最小有效电流
  const float dt = 0.01f;//10ms周期
  /* 目标值转换 */
  pid_target = (float32_t)target;
  float error = pid_target - (float32_t)motor.total_angle;

  /* 积分抗饱和处理 */
  // 积分分离判断
  if (fabsf(error) > integral_sep_threshold)
  {
    pid.state[2] = pid.state[2]; // 冻结积分项
  }
  else
  {
    // 正常积分并限幅
    pid.state[2] += error * pid.Ki * dt; // 采样时间补偿
    arm_clip_f32(&pid.state[2], &pid.state[2], -integral_max, integral_max, 1);
  }

  /* PID计算 */
  pid_output = arm_pid_f32(&pid, error);

  /* 输出动态限幅 */
  if (fabsf(error) > deadzone_threshold)
  {
    // 大偏差时强制最小有效电流
    if (error > 0)
    {
      arm_clip_f32(&pid_output, &pid_output, min_effective_current, MAX_CURRENT, 1);
    }
    else
    {
      arm_clip_f32(&pid_output, &pid_output, -MAX_CURRENT, -min_effective_current, 1);
    }
  }
  else
  {
    // 正常工况限幅
    arm_clip_f32(&pid_output, &pid_output, -MAX_CURRENT, MAX_CURRENT, 1);
  }

  /* 执行控制 */
  cmd_motor(&hcan, 0x200, (int16_t)pid_output, (int16_t)pid_output, (int16_t)pid_output, (int16_t)pid_output);
}
```
## UART指令处理
9600 波特率
```C
// UART指令处理（UART中断回调中调用）
void Process_UART_Command(uint8_t *buf)
{
  if (strstr((char *)buf, "MODE1"))
  {
    work_mode = MODE_PENDULUM;
  }
  else if (strstr((char *)buf, "STOP"))
  {
    work_mode = MODE_IDLE;
  }
  else if (strstr((char *)buf, "POS:"))
  {
    char *endptr;
    float percent = strtof((char *)(buf + 4), &endptr) / 100.0f;

    // 限制百分比范围
    percent = percent < 0.0f ? 0.0f : (percent > 1.0f ? 1.0f : percent);

    // 精确计算目标位置
    int32_t range = motor.right_limit - motor.left_limit;
    target_pos = motor.left_limit + (int32_t)((float)range * percent);

    work_mode = MODE_POSITION;
  }
  else if (strstr((char *)buf, "INIT"))
  {
    work_mode = MODE_INIT;
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart3)
  {
    uart_tx_busy = 0;
  }
}

// UART接收回调函数
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  if (huart == &huart3)
  {
    char *start = (char *)uart_rx_buf;
    char *end = strchr(start, '\n');

    while (end != NULL)
    {
      // 拷贝单条指令到处理缓冲区
      uint8_t len = end - start;
      len = (len > UART_CMD_MAX_LEN - 1) ? UART_CMD_MAX_LEN - 1 : len;
      memcpy(uart_cmd_buf, start, len);
      uart_cmd_buf[len] = '\0';

      // 处理下一条指令
      start = end + 1;
      end = strchr(start, '\n');
    }
    // HAL_UART_Transmit_DMA(&huart3,uart_cmd_buf,sizeof(uart_cmd_buf));
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
    Process_UART_Command(uart_cmd_buf);
    // 重启DMA接收
    memset(uart_rx_buf, 0, UART_BUF_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart_rx_buf, UART_BUF_SIZE);
  }
}
```
## TIM定时器中断（安全控制+数据回传）
TIM 3 500 HZ
TIM 4 1000 HZ
```C
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM4)
    {
        control_task_flag = 1; // 安全控制任务标志
    }
    
    if(htim->Instance == TIM3 && !uart_tx_busy) 
    {
        uart_tx_busy = 1;
        HAL_CAN_DeactivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
		
		//显示转换 保证串口绘图中电流正常
		int32_t current_32bit = (int32_t)motor.torque_current;

		//搓！serialplot通信正常
		uart_buf[0]=0xAA;
		memcpy(&uart_buf[1],&motor.total_angle,4);
		memcpy(&uart_buf[5],&current_32bit,4);
		memcpy(&uart_buf[9],(const void*)&target_pos,4);
		memcpy(&uart_buf[13],&motor.left_limit,4);
		memcpy(&uart_buf[17],&motor.right_limit,4);

		/*发送*/
		HAL_UART_Transmit_IT(&huart3,uart_buf,21);
		
		HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
	}
}
```
