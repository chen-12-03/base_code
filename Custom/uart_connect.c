#include "uart_connect.h"

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