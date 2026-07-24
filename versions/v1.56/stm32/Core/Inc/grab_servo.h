#ifndef GRAB_SERVO_H_
#define GRAB_SERVO_H_

#include <stdint.h>

void GrabServo_Init(void);
void GrabServo_Update(void);
void GrabServo_StartPickup(void);
void GrabServo_StartDropoff(void);
void GrabServo_AdjustClaw(int16_t delta_deg);
void GrabServo_AdjustLift(int16_t delta_deg);
uint8_t GrabServo_IsBusy(void);
uint8_t GrabServo_HasObject(void);

#endif /* GRAB_SERVO_H_ */
