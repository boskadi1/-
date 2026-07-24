#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_

#include "main.h"

void Bluetooth_Init(void);
void Bluetooth_Update(void);
void Bluetooth_TxCompleteCallback(UART_HandleTypeDef *huart);
void Bluetooth_UartErrorCallback(UART_HandleTypeDef *huart);

#endif /* BLUETOOTH_H_ */
