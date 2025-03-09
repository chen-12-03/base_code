#ifndef UART_CONNECT_H
#define UART_CONNECT_H

#include "stm32f1xx.h"
#include "usart.h"
#include "motor_control.h"
#include "control_task.h"
#include "string.h"

#define UART_BUF_SIZE 32
#define UART_CMD_MAX_LEN 16

extern volatile uint8_t uart_tx_busy;
extern volatile int32_t target_pos;
extern Motor_Status motor;
extern uint8_t uart_rx_buf[UART_BUF_SIZE];
extern uint8_t uart_cmd_buf[UART_CMD_MAX_LEN];
extern uint8_t uart_buf[64];
void Process_UART_Command(uint8_t *buf);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);

#endif
