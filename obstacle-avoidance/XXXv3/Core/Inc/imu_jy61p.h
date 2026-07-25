#ifndef IMU_JY61P_H_
#define IMU_JY61P_H_

#include "main.h"

void ImuJY61P_Init(void);
void ImuJY61P_Update(void);
void ImuJY61P_AnchorYaw(void);
void ImuJY61P_UartRxCompleteCallback(UART_HandleTypeDef *huart);
void ImuJY61P_UartErrorCallback(UART_HandleTypeDef *huart);

#endif /* IMU_JY61P_H_ */
