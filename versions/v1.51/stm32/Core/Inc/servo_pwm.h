#ifndef SERVO_PWM_H_
#define SERVO_PWM_H_

#include <stdint.h>

typedef enum
{
  SERVO_CHANNEL_GRAB_CLAW = 0,
  SERVO_CHANNEL_GRAB_LIFT,
  SERVO_CHANNEL_CAMERA_PAN,
  SERVO_CHANNEL_CAMERA_TILT,
  /* Compatibility aliases; CargoServo is not run in the gripper version. */
  SERVO_CHANNEL_CARGO_LEFT = SERVO_CHANNEL_GRAB_CLAW,
  SERVO_CHANNEL_CARGO_RIGHT = SERVO_CHANNEL_GRAB_LIFT
} ServoChannel_t;

void ServoPwm_Init(void);
void ServoPwm_SetPulseUs(ServoChannel_t channel, uint16_t pulse_us);
void ServoPwm_SetAbsoluteAngle(ServoChannel_t channel, int16_t angle_deg);
void ServoPwm_SetScaledAngle(ServoChannel_t channel, int16_t angle_deg,
                             int16_t full_scale_deg);
void ServoPwm_SetCenteredAngle(ServoChannel_t channel, int16_t angle_deg);

#endif /* SERVO_PWM_H_ */
