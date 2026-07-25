#ifndef VISION_H_
#define VISION_H_

#include "main.h"

typedef enum
{
  VISION_PROCESS_RED = 0,
  VISION_PROCESS_START,
  VISION_PROCESS_LINE,
  VISION_PROCESS_UNLOAD
} VisionProcessingMode_t;

void Vision_Init(void);
void Vision_Update(void);
void Vision_SetProcessingMode(VisionProcessingMode_t mode);
VisionProcessingMode_t Vision_GetProcessingMode(void);
uint8_t Vision_IsModeConfirmed(VisionProcessingMode_t mode);
void Vision_UartRxCompleteCallback(UART_HandleTypeDef *huart);
void Vision_UartErrorCallback(UART_HandleTypeDef *huart);

#endif /* VISION_H_ */
