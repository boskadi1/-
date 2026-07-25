#include "app.h"
#include "app_timer.h"
#include "app_state.h"
#include "bluetooth.h"
#include "car_config.h"
#include "car_data.h"
#include "camera_servo.h"
#include "debug_uart.h"
#include "encoder.h"
#include "imu_jy61p.h"
#include "motor.h"
#include "servo_pwm.h"
#include "ultrasonic.h"
#include "vision.h"

void App_Init(void)
{
  CarData_Init();
  AppTimer_Init();
  ServoPwm_Init();
  CameraServo_Init();
  Ultrasonic_Init();
  Vision_Init();
  Motor_Init();
  Encoder_Init();
  ImuJY61P_Init();
  Bluetooth_Init();
  DebugUart_Init();
  AppState_Init();
  if (CAR_PERIODIC_DEBUG_OUTPUT != 0U)
  {
    DebugUart_Print("STM32 visual-state navigator v" CAR_SOFTWARE_VERSION " start\r\n");
    if (VISION_V2_TEST_ONLY != 0U)
    {
      DebugUart_Print("MODE,VISION_V2_TEST_ONLY\r\n");
      DebugUart_Print("DRIVE,VISION_STATE_MACHINE,IMU_HEADING_CLOSED_LOOP\r\n");
      DebugUart_Print("SONAR_MONITOR,TX=PA9,BAUD=9600,PERIOD=250MS\r\n");
    }
  }
}

void App_Update(void)
{
  Vision_Update();
  Bluetooth_Update();
  DebugUart_Update();
  Ultrasonic_Update();
  Encoder_Update();
  ImuJY61P_Update();
  CameraServo_Update();

  if (AppTimer_ConsumeStateFlag())
  {
    AppState_Update();
  }

  Motor_Update();

  if (AppTimer_ConsumeDebugFlag())
  {
    if (CAR_PERIODIC_DEBUG_OUTPUT != 0U)
    {
      DebugUart_PrintStatus();
    }
  }
}
