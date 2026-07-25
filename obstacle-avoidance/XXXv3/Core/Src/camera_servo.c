#include "camera_servo.h"
#include "main.h"
#include "car_config.h"
#include "car_data.h"
#include "servo_pwm.h"

/*
 * XXXv2 camera ownership:
 *   - OpenMV Servo(1) owns horizontal pan and receives IMU yaw over USART3;
 *   - STM32 owns only the PE14/TIM1_CH4 vertical servo at +10 degrees.
 *
 * PE13/TIM1_CH3 is deliberately held at its neutral value. It is not the
 * physical pan output used by the supplied OpenMV gimbal reference.
 */

void CameraServo_Init(void)
{
  gCar.camera.current_deg = CAMERA_PAN_INITIAL_DEG;
  gCar.camera.target_deg = CAMERA_PAN_INITIAL_DEG;
  gCar.camera.current_tilt_deg = CAMERA_FIXED_TILT_DEG;
  gCar.camera.target_tilt_deg = CAMERA_FIXED_TILT_DEG;
  gCar.camera.scanning = 0U;
  gCar.camera.blue_lock_active = 0U;
  gCar.camera.blue_lock_ready = 1U;

  ServoPwm_SetCenteredAngle(SERVO_CHANNEL_CAMERA_PAN,
                            CAMERA_PAN_INITIAL_DEG);
  ServoPwm_SetCenteredAngle(SERVO_CHANNEL_CAMERA_TILT,
                            CAMERA_FIXED_TILT_DEG);
}

void CameraServo_SetAngle(int16_t angle_deg)
{
  (void)angle_deg;
  /* OpenMV Servo(1) owns the horizontal target in XXXv2. */
}

void CameraServo_SetPanTilt(int16_t pan_deg, int16_t tilt_deg)
{
  (void)pan_deg;
  (void)tilt_deg;
  /* OpenMV owns pan; the fixed +10-degree tilt cannot be overridden. */
}

void CameraServo_Update(void)
{
  gCar.camera.current_deg = CAMERA_PAN_INITIAL_DEG;
  gCar.camera.target_deg = CAMERA_PAN_INITIAL_DEG;
  gCar.camera.target_tilt_deg = CAMERA_FIXED_TILT_DEG;
  gCar.camera.current_tilt_deg = CAMERA_FIXED_TILT_DEG;
  gCar.camera.scanning = 0U;
  gCar.camera.blue_lock_active = 0U;
  gCar.camera.blue_lock_ready = 1U;

  ServoPwm_SetCenteredAngle(SERVO_CHANNEL_CAMERA_TILT,
                            CAMERA_FIXED_TILT_DEG);
}

uint8_t CameraServo_BlueLockActive(void)
{
  return 0U;
}

uint8_t CameraServo_BlueLockReady(void)
{
  return 1U;
}

uint8_t CameraServo_StartupScanComplete(void)
{
  return 1U;
}
