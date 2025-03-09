#include "control_task.h"

#define INIT_DURATION_MS 3000 // 单方向最大初始化时间
#define INIT_STABLE_THRESH 5  // 角度稳定阈值(连续5次变化小于10)

// 脱困
void EscapeFromLimit()
{
  uint32_t start_tick = HAL_GetTick();
  int32_t initial_angle = motor.total_angle;

  while (HAL_GetTick() - start_tick < 3000)
  {
    cmd_motor(&hcan, 0x200, -MAX_CURRENT / 2, -MAX_CURRENT / 2, -MAX_CURRENT / 2, -MAX_CURRENT / 2);

//    // 检测有效位移
//    if (abs(motor.total_angle - initial_angle) > 50000)
//    {
//      break;
//    }

    HAL_Delay(10);
  }
}

// 输入电流单侧初始化
bool Init_MoveToLimit(int16_t current, int32_t *limit)
{
  uint32_t start_tick = HAL_GetTick();
  int32_t last_angle = motor.total_angle;
  uint8_t stable_count = 0;
	
  //初始位置留存
  int32_t init_base_angle = motor.total_angle;

  while (HAL_GetTick() - start_tick < INIT_DURATION_MS)
  {
    // 每10ms发送一次指令
    static uint32_t last_cmd = 0;
    if (HAL_GetTick() - last_cmd >= 10)
    {
      cmd_motor(&hcan, 0x200, current, current, current, current);
      last_cmd = HAL_GetTick();
    }

    // 稳定性检测 连续五次小位移
    int32_t delta = motor.total_angle - last_angle;
    if (abs(delta) < 10)
    {
      if (++stable_count >= INIT_STABLE_THRESH)
      {
        *limit = motor.total_angle - init_base_angle;
        return true;
      }
    }
    else
    {
      stable_count = 0;
      last_angle = motor.total_angle;
    }

    HAL_Delay(5);
  }
  return false;
}

// 左右端角度初始化
void Limit_Init()
{
  // 保存原始工作模式
  WorkMode prev_mode = work_mode;
  work_mode = MODE_INIT;

  // 左限初始化
  if (!Init_MoveToLimit(-MAX_CURRENT, &motor.left_limit))
  {
    Error_Handler();
  }
  // 右限初始化
  if (!Init_MoveToLimit(MAX_CURRENT, &motor.right_limit))
  {
    Error_Handler();
  }
  motor.right_limit+=motor.left_limit;

  // 脱困
  EscapeFromLimit();

  // 恢复原模式
  work_mode = prev_mode;
  cmd_motor(&hcan, 0x200, 0, 0, 0, 0); // 停止电机
}

int32_t constrain(int32_t value, int32_t minVal, int32_t maxVal) {
    if (value < minVal) {
        return minVal;
    } else if (value > maxVal) {
        return maxVal;
    } else {
        return value;
    }
}

/**
  *@brief 电机控制主任务（10ms周期）
  *@note 执行模式切换、PID控制
  */
void Motor_Control_Task()
{
	switch(work_mode){
		case MODE_PENDULUM: {
            // 参数合法性检查
            if ((motor.left_limit == 0) || (motor.right_limit == 0)) {
                Error_Handler();
                work_mode = MODE_IDLE;
                break;
            }

            // 钟摆方向控制
            static int8_t direction = -1; 
            const int32_t margin = 500; // 摆动边界余量
            
            // 动态计算目标位置（含速度前馈）
            target_pos = (direction > 0) ? 
                (motor.right_limit - margin) : 
                (motor.left_limit + margin);
            
            // 执行PID控制
            PID_Control(target_pos);

            // 方向切换逻辑
            int32_t current_pos = motor.total_angle;
            if (direction == -1 && current_pos <= (motor.left_limit + margin)) {
                direction = 1;
            } else if (direction == 1 && current_pos >= (motor.right_limit - margin)) {
                direction = -1;
            }
            
            break;
        }
		
		case MODE_POSITION: {
            // 目标位置限幅
            target_pos = constrain(target_pos, 
                motor.left_limit + (motor.right_limit-motor.left_limit)*0.05, // 最小安全位置
                motor.right_limit - (motor.right_limit-motor.left_limit)*0.05  // 最大安全位置
            );
            PID_Control(target_pos);
            break;
        }
		
		case MODE_IDLE: {
			cmd_motor(&hcan,0x200,0,0,0,0);
			break;
		}
		
		case MODE_INIT : {
			Limit_Init();
			work_mode = MODE_IDLE;
			break;
		}
	}
}