#ifndef VISION_H_
#define VISION_H_

#include "main.h"

void Vision_Init(void);
void Vision_Update(void);
void Vision_UartRxCompleteCallback(UART_HandleTypeDef *huart);
void Vision_UartErrorCallback(UART_HandleTypeDef *huart);

#endif /* VISION_H_ */
