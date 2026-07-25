#ifndef CARGO_SERVO_H_
#define CARGO_SERVO_H_

#include <stdint.h>

void CargoServo_Init(void);
void CargoServo_Load(void);
void CargoServo_Unload(void);
void CargoServo_Update(void);
void CargoServo_AdjustLeft(int16_t delta_deg);
void CargoServo_AdjustRight(int16_t delta_deg);
void CargoServo_Open(void);
void CargoServo_Close(void);

#endif /* CARGO_SERVO_H_ */
