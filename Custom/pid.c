#include "pid.h"

// PID初始化
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