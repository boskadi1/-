#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>

void Motor_Init(void);
void Motor_SetTarget(float left_speed, float right_speed);
void Motor_SetFourTarget(int16_t front_left_speed, int16_t rear_left_speed,
                         int16_t front_right_speed, int16_t rear_right_speed);
void Motor_Brake(void);
void Motor_Stop(void);
void Motor_Update(void);

#endif /* MOTOR_H_ */
