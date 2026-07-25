#ifndef APP_STATE_H_
#define APP_STATE_H_

#include <stdint.h>

void AppState_Init(void);
void AppState_Update(void);
uint8_t AppState_EscapeMode(void);
uint8_t AppState_HardUltrasonicMask(void);
uint8_t AppState_ZeroHeadingBiasActive(void);
uint8_t AppState_NavStatus(void);
uint8_t AppState_NavPathLength(void);
uint8_t AppState_NavObstacleCount(void);
uint8_t AppState_BlueObstacleActive(void);

#endif /* APP_STATE_H_ */
