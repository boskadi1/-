#ifndef CAMERA_SERVO_H_
#define CAMERA_SERVO_H_

#include <stdint.h>

void CameraServo_Init(void);
void CameraServo_SetAngle(int16_t angle_deg);
void CameraServo_SetPanTilt(int16_t pan_deg, int16_t tilt_deg);
void CameraServo_Update(void);
uint8_t CameraServo_BlueLockActive(void);
uint8_t CameraServo_BlueLockReady(void);
uint8_t CameraServo_StartupScanComplete(void);

#endif /* CAMERA_SERVO_H_ */
