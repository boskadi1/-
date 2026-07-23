#include "app.h"
#include "app_timer.h"
#include "app_state.h"
#include "bluetooth.h"
#include "camera_servo.h"
#include "car_config.h"
#include "car_data.h"
#include "debug_uart.h"
#include "encoder.h"
#include "grab_servo.h"
#include "imu_jy61p.h"
#include "motor.h"
#include "servo_pwm.h"
#include "ultrasonic.h"
#include "vision.h"

void App_Init(void)
{
  CarData_Init();
  AppTimer_Init();
  Bluetooth_Init();
  Ultrasonic_Init();
  ImuJY61P_Init();
  Vision_Init();
  Motor_Init();
  ServoPwm_Init();
  Encoder_Init();
  GrabServo_Init();
  CameraServo_Init();

#if GRAB_STARTUP_AUTO_PICKUP_ENABLE
  /* The red block starts directly between/in front of the open claw. */
  GrabServo_StartPickup();
#endif

  AppState_Init();
  DebugUart_Print("STM32 modular car framework v" CAR_SOFTWARE_VERSION " start\r\n");
}

void App_Update(void)
{
  Bluetooth_Update();
  Vision_Update();
  ImuJY61P_Update();
  Ultrasonic_Update();
  Encoder_Update();

  if ((GrabServo_IsBusy() == 0U) && AppTimer_ConsumeStateFlag())
  {
    AppState_Update();
  }

  GrabServo_Update();
  if (GrabServo_IsBusy() != 0U)
  {
    /* Never move the chassis while the startup pickup or a load/unload
     * sequence is physically moving the claw. */
    Motor_Stop();
  }
  else
  {
    Motor_Update();
  }
  CameraServo_Update();

  if (AppTimer_ConsumeDebugFlag())
  {
    DebugUart_PrintStatus();
  }
}
