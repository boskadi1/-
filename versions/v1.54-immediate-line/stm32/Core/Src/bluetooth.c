#include "bluetooth.h"
#include "main.h"
#include "car_config.h"
#include "car_data.h"
#include "debug_uart.h"
#include "imu_jy61p.h"
#include "motor.h"
#include "camera_servo.h"
#include "grab_servo.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart2;

static uint8_t s_rx_byte;
static char s_line[CAR_BLUETOOTH_RX_BUFFER_SIZE];
static uint8_t s_index;
static uint32_t s_telemetry_tick;

static void Bluetooth_Send(const char *text)
{
  uint16_t len = 0U;
  while ((text[len] != '\0') && (len < 100U)) len++;
  HAL_UART_Transmit(&huart2, (uint8_t *)text, len, 100U);
}

static void MoveCameraManual(char command)
{
  int16_t pan_deg = gCar.camera.current_deg;
  int16_t tilt_deg = gCar.camera.current_tilt_deg;

  gCar.sys.start_enable = 0U;
  gCar.sys.action = CAR_ACTION_STOP;
  Motor_Stop();

  if (command == 'j') pan_deg -= CAMERA_MANUAL_STEP_DEG;
  if (command == 'l') pan_deg += CAMERA_MANUAL_STEP_DEG;
  if (command == 'i') tilt_deg += CAMERA_MANUAL_STEP_DEG;
  if (command == 'k') tilt_deg -= CAMERA_MANUAL_STEP_DEG;

  CameraServo_SetPanTilt(pan_deg, tilt_deg);
}

static void MoveGrabManual(char command)
{
  gCar.sys.start_enable = 0U;
  gCar.sys.action = CAR_ACTION_STOP;
  Motor_Stop();

  if (command == 'z') GrabServo_AdjustClaw(GRAB_CLAW_MANUAL_STEP_DEG);
  if (command == 'x') GrabServo_AdjustClaw(-GRAB_CLAW_MANUAL_STEP_DEG);
  if (command == 'u') GrabServo_AdjustLift(GRAB_LIFT_MANUAL_STEP_DEG);
  if (command == 'm') GrabServo_AdjustLift(-GRAB_LIFT_MANUAL_STEP_DEG);
}

static void ParseCommand(const char *cmd)
{
  if (strcmp(cmd, "z") == 0)
  {
    MoveGrabManual('z');
    Bluetooth_Send("OK,GRAB,CLAW,+\r\n");
  }
  else if (strcmp(cmd, "x") == 0)
  {
    MoveGrabManual('x');
    Bluetooth_Send("OK,GRAB,CLAW,-\r\n");
  }
  else if (strcmp(cmd, "u") == 0)
  {
    MoveGrabManual('u');
    Bluetooth_Send("OK,GRAB,LIFT,+\r\n");
  }
  else if (strcmp(cmd, "m") == 0)
  {
    MoveGrabManual('m');
    Bluetooth_Send("OK,GRAB,LIFT,-\r\n");
  }
  else if ((strcmp(cmd, "GRAB_PICKUP") == 0) ||
           (strcmp(cmd, "p") == 0))
  {
    gCar.sys.start_enable = 0U;
    Motor_Stop();
    GrabServo_StartPickup();
    Bluetooth_Send("OK,GRAB,PICKUP\r\n");
  }
  else if ((strcmp(cmd, "GRAB_DROPOFF") == 0) ||
           (strcmp(cmd, "d") == 0))
  {
    gCar.sys.start_enable = 0U;
    Motor_Stop();
    GrabServo_StartDropoff();
    Bluetooth_Send("OK,GRAB,DROPOFF\r\n");
  }
  else if (strcmp(cmd, "j") == 0)
  {
    MoveCameraManual('j');
    Bluetooth_Send("OK,CAMERA,LEFT\r\n");
  }
  else if (strcmp(cmd, "l") == 0)
  {
    MoveCameraManual('l');
    Bluetooth_Send("OK,CAMERA,RIGHT\r\n");
  }
  else if (strcmp(cmd, "i") == 0)
  {
    MoveCameraManual('i');
    Bluetooth_Send("OK,CAMERA,UP\r\n");
  }
  else if (strcmp(cmd, "k") == 0)
  {
    MoveCameraManual('k');
    Bluetooth_Send("OK,CAMERA,DOWN\r\n");
  }
  else if (strcmp(cmd, "s") == 0)
  {
    gCar.sys.start_enable = 0U;
    gCar.sys.action = CAR_ACTION_STOP;
    Motor_Stop();
    Bluetooth_Send("OK,PAUSE\r\n");
  }
  else if ((strcmp(cmd, "STOP") == 0) || (strcmp(cmd, "S") == 0))
  {
    CarMode_t selected_mode = gCar.sys.mode;
    gCar.sys.start_enable = 0U;
    CarData_ResetMission(selected_mode);
    Motor_Stop();
    Bluetooth_Send("OK,STOP\r\n");
  }
  else if ((strcmp(cmd, "FORWARD") == 0) || (strcmp(cmd, "F") == 0))
  {
    CarData_ResetMission(CAR_MODE_REMOTE);
    gCar.sys.remote_action = CAR_ACTION_FORWARD;
    gCar.sys.start_enable = 1U;
    Bluetooth_Send("OK,FORWARD\r\n");
  }
  else if ((strcmp(cmd, "BACKWARD") == 0) || (strcmp(cmd, "B") == 0))
  {
    CarData_ResetMission(CAR_MODE_REMOTE);
    gCar.sys.remote_action = CAR_ACTION_BACKWARD;
    gCar.sys.start_enable = 1U;
    Bluetooth_Send("OK,BACKWARD\r\n");
  }
  else if ((strcmp(cmd, "LEFT") == 0) || (strcmp(cmd, "L") == 0))
  {
    CarData_ResetMission(CAR_MODE_REMOTE);
    gCar.sys.remote_action = CAR_ACTION_TURN_LEFT;
    gCar.sys.start_enable = 1U;
    Bluetooth_Send("OK,LEFT\r\n");
  }
  else if ((strcmp(cmd, "RIGHT") == 0) || (strcmp(cmd, "R") == 0))
  {
    CarData_ResetMission(CAR_MODE_REMOTE);
    gCar.sys.remote_action = CAR_ACTION_TURN_RIGHT;
    gCar.sys.start_enable = 1U;
    Bluetooth_Send("OK,RIGHT\r\n");
  }
  else if ((strcmp(cmd, "START") == 0) || (strcmp(cmd, "g") == 0))
  {
    if (gCar.sys.mode == CAR_MODE_STOP)
    {
      Bluetooth_Send("ERR,SELECT_MODE\r\n");
      return;
    }
    gCar.sys.start_enable = 1U;
    Bluetooth_Send("OK,START\r\n");
  }
  else if ((strcmp(cmd, "MODE,AVOID") == 0) ||
           (strcmp(cmd, "AVOID") == 0) ||
           (strcmp(cmd, "a") == 0))
  {
    Motor_Stop();
    CarData_ResetMission(CAR_MODE_AVOID);
    gCar.sys.start_enable = 0U;
    Bluetooth_Send("OK,MODE,AVOID\r\n");
  }
  else if ((strcmp(cmd, "MODE,VISION") == 0) ||
           (strcmp(cmd, "VISION") == 0) ||
           (strcmp(cmd, "LINE") == 0) ||
           (strcmp(cmd, "v") == 0))
  {
    Motor_Stop();
    CarData_ResetMission(CAR_MODE_VISION_LINE);
    gCar.sys.start_enable = 0U;
    Bluetooth_Send("OK,MODE,VISION\r\n");
  }
  else if ((strcmp(cmd, "MODE,LINE_TEST") == 0) ||
           (strcmp(cmd, "LINE_TEST") == 0) ||
           (strcmp(cmd, "TRACK") == 0) ||
           (strcmp(cmd, "t") == 0))
  {
    Motor_Stop();
    CarData_ResetMission(CAR_MODE_LINE_TEST);
    gCar.sys.start_enable = 0U;
    Bluetooth_Send("OK,MODE,LINE_TEST\r\n");
  }
  else if (strcmp(cmd, "ZERO_YAW") == 0)
  {
    ImuJY61P_AnchorYaw();
    Bluetooth_Send("OK,ZERO_YAW\r\n");
  }
  else if (strcmp(cmd, "STATUS") == 0)
  {
    DebugUart_PrintStatus();
    Bluetooth_Send("OK,STATUS\r\n");
  }
  else
  {
    Bluetooth_Send("ERR,CMD\r\n");
  }
}

void Bluetooth_Init(void)
{
  s_index = 0U;
  s_telemetry_tick = HAL_GetTick();
}

void Bluetooth_Update(void)
{
  char telemetry[96];

  while (HAL_UART_Receive(&huart2, &s_rx_byte, 1U, 0U) == HAL_OK)
  {
    /* Stop commands always take priority and clear any unfinished line. */
    if ((s_rx_byte == 'S') || (s_rx_byte == 's'))
    {
      char command[2] = {(char)s_rx_byte, '\0'};
      s_index = 0U;
      ParseCommand(command);
      continue;
    }

    if ((s_index == 0U) &&
        ((s_rx_byte == 'F') || (s_rx_byte == 'B') ||
         (s_rx_byte == 'L') || (s_rx_byte == 'R') ||
         (s_rx_byte == 'a') || (s_rx_byte == 'g') ||
         (s_rx_byte == 'd') ||
         (s_rx_byte == 'i') || (s_rx_byte == 'j') ||
         (s_rx_byte == 'k') || (s_rx_byte == 'l') ||
         (s_rx_byte == 'm') || (s_rx_byte == 'p') ||
         (s_rx_byte == 's') ||
         (s_rx_byte == 't') || (s_rx_byte == 'v') ||
         (s_rx_byte == 'u') || (s_rx_byte == 'x') ||
         (s_rx_byte == 'z')))
    {
      char command[2] = {(char)s_rx_byte, '\0'};
      ParseCommand(command);
      continue;
    }

    if ((s_rx_byte == '\n') || (s_rx_byte == '\r'))
    {
      if (s_index > 0U)
      {
        s_line[s_index] = '\0';
        ParseCommand(s_line);
        s_index = 0U;
      }
    }
    else if (s_index < (CAR_BLUETOOTH_RX_BUFFER_SIZE - 1U))
    {
      s_line[s_index++] = (char)s_rx_byte;
    }
    else
    {
      s_index = 0U;
    }
  }

  if ((HAL_GetTick() - s_telemetry_tick) >= CAR_BLUETOOTH_TELEMETRY_MS)
  {
    s_telemetry_tick = HAL_GetTick();
    (void)snprintf(telemetry, sizeof(telemetry),
                   "LINE,%d,%d,%u,%u,%d,%s\r\n",
                   (int)gCar.vision.line_offset,
                   (int)gCar.vision.line_heading,
                   (unsigned int)gCar.vision.line_magnitude,
                   (unsigned int)gCar.vision.line_valid,
                   (int)gCar.motion.line_correction,
                   CarData_ActionName(gCar.sys.action));
    Bluetooth_Send(telemetry);
  }
}
