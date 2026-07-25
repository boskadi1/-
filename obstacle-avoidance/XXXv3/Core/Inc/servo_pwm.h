#ifndef SERVO_PWM_H_
#define SERVO_PWM_H_

#include <stdint.h>

typedef enum
{
  SERVO_CHANNEL_CARGO_LEFT = 0,
  SERVO_CHANNEL_CARGO_RIGHT,
  SERVO_CHANNEL_CAMERA_PAN,
  SERVO_CHANNEL_CAMERA_TILT
} ServoChannel_t;

void ServoPwm_Init(void);
void ServoPwm_SetPulseUs(ServoChannel_t channel, uint16_t pulse_us);
void ServoPwm_SetAbsoluteAngle(ServoChannel_t channel, int16_t angle_deg);
void ServoPwm_SetScaledAngle(ServoChannel_t channel, int16_t angle_deg,
                             int16_t full_scale_deg);
void ServoPwm_SetCenteredAngle(ServoChannel_t channel, int16_t angle_deg);

#endif /* SERVO_PWM_H_ */
