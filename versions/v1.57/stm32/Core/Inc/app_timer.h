#ifndef APP_TIMER_H_
#define APP_TIMER_H_

#include <stdint.h>

void AppTimer_Init(void);
void AppTimer_OnSysTick(void);
uint8_t AppTimer_ConsumeStateFlag(void);
uint8_t AppTimer_ConsumeDebugFlag(void);

#endif /* APP_TIMER_H_ */
