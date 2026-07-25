#include "bluetooth.h"
#include "app_state.h"
#include "main.h"
#include "car_config.h"
#include "car_data.h"
#include "imu_jy61p.h"
#include "motor.h"
#include "camera_servo.h"
#include "cargo_servo.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart2;

static uint8_t s_rx_byte;
static char s_line[CAR_BLUETOOTH_RX_BUFFER_SIZE];
static char s_status_line[512];
static char s_telemetry_line[512];
static uint8_t s_index;
static uint32_t s_telemetry_tick;
static uint32_t s_handshake_prompt_tick;
static uint32_t s_last_rx_tick;
static uint8_t s_start_ack_pending;

static void Bluetooth_Send(const char *text)
{
  uint16_t len = 0U;
  while ((text[len] != '\0') && (len < (sizeof(s_status_line) - 1U))) len++;
  HAL_UART_Transmit(&huart2, (uint8_t *)text, len, 100U);
}

static int32_t BluetoothHeadingDeg(void)
{
  /* Navigation internally uses clockwise/right as positive.  Telemetry uses
     the requested compass convention: power-on is 0 deg, clockwise is
     negative and counter-clockwise is positive. */
  const int32_t heading_cd = -gCar.imu.yaw_rel_cd;

  return (heading_cd >= 0L) ? ((heading_cd + 50L) / 100L) :
                              ((heading_cd - 50L) / 100L);
}

static void Bluetooth_SendSonarStatus(void)
{
  (void)snprintf(s_status_line, sizeof(s_status_line),
                 "SONAR | FL=%ucm OK=%u | FR=%ucm OK=%u | LEFT=%ucm OK=%u | RIGHT=%ucm OK=%u | BLUE=%u A=%lu LOCK=%u/%u TILT=%d PAN=%d/%uus YRX=%u/%d | IMU_OK=%u HDG=%lddeg ACC=%u/%u/%d/%d DISP=%ld/%ld COL=%u/%d/%lu IRX=%lu IF=%lu/%lu IC=%lu IX=%lu IB=%lu IE=%lu/%lu/%lu IT=%02X | ENC_YAW_CD=%ld | V=%ld/%ld TV=%d/%d PWM=%d/%d\r\n",
                 gCar.us.front_left_cm,
                 gCar.us.front_left_valid,
                 gCar.us.front_right_cm,
                 gCar.us.front_right_valid,
                 gCar.us.left_cm,
                 gCar.us.left_valid,
                 gCar.us.right_cm,
                 gCar.us.right_valid,
                 AppState_BlueObstacleActive(),
                 (unsigned long)gCar.vision.obstacle_area_pixels,
                 CameraServo_BlueLockActive(),
                 CameraServo_BlueLockReady(),
                 gCar.camera.current_tilt_deg,
                 gCar.vision.camera_pan_deg,
                 gCar.vision.camera_pan_pulse_us,
                 gCar.vision.heading_link_valid,
                 gCar.vision.heading_rx_yaw_cd,
                 gCar.imu.angle_valid,
                 (long)BluetoothHeadingDeg(),
                 gCar.imu.accel_valid,
                 gCar.imu.accel_calibrated,
                 gCar.imu.accel_forward_mg,
                 gCar.imu.accel_right_mg,
                 (long)gCar.imu.displacement_forward_mm,
                 (long)gCar.imu.displacement_right_mm,
                 gCar.imu.collision_valid,
                 gCar.imu.collision_bearing_deg,
                 (unsigned long)gCar.imu.collision_count,
                 (unsigned long)gCar.imu.rx_byte_count,
                 (unsigned long)gCar.imu.rx_angle_frame_count,
                 (unsigned long)gCar.imu.rx_frame_count,
                 (unsigned long)gCar.imu.rx_checksum_error_count,
                 (unsigned long)gCar.imu.rx_format_error_count,
                 (unsigned long)gCar.imu.rx_ring_overflow_count,
                 (unsigned long)gCar.imu.uart_error_count,
                 (unsigned long)gCar.imu.uart_overrun_count,
                 (unsigned long)gCar.imu.rx_rearm_error_count,
                 gCar.imu.last_frame_type,
                 (long)gCar.encoder.yaw_cd,
                 (long)gCar.encoder.left_speed_mm_s,
                 (long)gCar.encoder.right_speed_mm_s,
                 gCar.motion.left_target_mm_s,
                 gCar.motion.right_target_mm_s,
                 gCar.motion.left_pwm,
                 gCar.motion.right_pwm);
  Bluetooth_Send(s_status_line);
}

static const char *Bluetooth_MotionName(void)
{
  /* Report commanded physical wheel speeds, not unequal calibrated PWM.
     Comparing 83/70 PWM directly incorrectly labelled straight travel as a
     right turn even when both wheel targets were the same 220 mm/s. */
  const int16_t left = gCar.motion.left_target_mm_s;
  const int16_t right = gCar.motion.right_target_mm_s;

  if ((left == 0) && (right == 0)) return "STOP";
  if ((left < 0) && (right > 0)) return "TURN_LEFT";
  if ((left > 0) && (right < 0)) return "TURN_RIGHT";
  if ((left > 0) && (right > 0))
  {
    if (left > (right + 5)) return "FORWARD_RIGHT";
    if (right > (left + 5)) return "FORWARD_LEFT";
    return "FORWARD";
  }
  if ((left < 0) && (right < 0)) return "BACKWARD";
  if (left != 0) return "LEFT_ONLY";
  if (right != 0) return "RIGHT_ONLY";
  return "MIXED";
}

static void Bluetooth_SendRealtimeStatus(void)
{
  int len;

  /* The transmit buffer cannot be changed while an interrupt transfer is in
     progress. Drop this sample instead of blocking the 20 ms control loop. */
  if (huart2.gState != HAL_UART_STATE_READY) return;

  len = snprintf(s_telemetry_line, sizeof(s_telemetry_line),
                 "M=%s,A=%s,FL=%u/%u,FR=%u/%u,L=%u/%u,R=%u/%u,BLUE=%u,BA=%lu,COL=%u/%d/%lu,IMU=%u,HDG=%ld,IA=%u/%u/%d/%d,ID=%ld/%ld,IV=%ld/%ld,YTX=%lu/%lu/%u/%02X,YRX=%u/%d,PAN=%d/%u,NAV=%u,NP=%u,NO=%u,HS=%s,EM=%u,HM=%u,OMV=%u,VD=%u,VB=%d,LOCK=%u/%u,TILT=%d,TV=%u,TYC=%d,YEC=%d,IRX=%lu,IF=%lu/%lu,IC=%lu,IX=%lu,IB=%lu,IE=%lu/%lu/%lu,IT=%02X,IAGE=%lu,P=%d/%d\r\n",
                 Bluetooth_MotionName(),
                 CarData_ActionName(gCar.sys.action),
                 gCar.us.front_left_cm, gCar.us.front_left_valid,
                 gCar.us.front_right_cm, gCar.us.front_right_valid,
                 gCar.us.left_cm, gCar.us.left_valid,
                 gCar.us.right_cm, gCar.us.right_valid,
                 AppState_BlueObstacleActive(),
                 (unsigned long)gCar.vision.obstacle_area_pixels,
                 gCar.imu.collision_valid,
                 gCar.imu.collision_bearing_deg,
                 (unsigned long)gCar.imu.collision_count,
                 gCar.imu.angle_valid,
                 (long)BluetoothHeadingDeg(),
                 gCar.imu.accel_valid,
                 gCar.imu.accel_calibrated,
                 gCar.imu.accel_forward_mg,
                 gCar.imu.accel_right_mg,
                 (long)gCar.imu.displacement_forward_mm,
                 (long)gCar.imu.displacement_right_mm,
                 (long)gCar.imu.velocity_forward_mm_s,
                 (long)gCar.imu.velocity_right_mm_s,
                 (unsigned long)gCar.vision.heading_tx_ok_count,
                 (unsigned long)gCar.vision.heading_tx_fail_count,
                 gCar.vision.heading_tx_last_status,
                 gCar.vision.heading_tx_uart_state,
                 gCar.vision.heading_link_valid,
                 gCar.vision.heading_rx_yaw_cd,
                 gCar.vision.camera_pan_deg,
                 gCar.vision.camera_pan_pulse_us,
                 (unsigned int)AppState_NavStatus(),
                 AppState_NavPathLength(),
                 AppState_NavObstacleCount(),
                 (gCar.sys.start_enable != 0U) ? "RUN" : "WAIT_S",
                 AppState_EscapeMode(),
                 AppState_HardUltrasonicMask(),
                 gCar.vision.nav_frame_valid,
                 gCar.vision.obstacle_distance_cm,
                 gCar.vision.obstacle_bearing_deg,
                 CameraServo_BlueLockActive(),
                 CameraServo_BlueLockReady(),
                 gCar.camera.current_tilt_deg,
                 gCar.imu.target_yaw_valid,
                 gCar.imu.target_yaw_cd,
                 gCar.imu.yaw_error_cd,
                 (unsigned long)gCar.imu.rx_byte_count,
                 (unsigned long)gCar.imu.rx_angle_frame_count,
                 (unsigned long)gCar.imu.rx_frame_count,
                 (unsigned long)gCar.imu.rx_checksum_error_count,
                 (unsigned long)gCar.imu.rx_format_error_count,
                 (unsigned long)gCar.imu.rx_ring_overflow_count,
                 (unsigned long)gCar.imu.uart_error_count,
                 (unsigned long)gCar.imu.uart_overrun_count,
                 (unsigned long)gCar.imu.rx_rearm_error_count,
                 gCar.imu.last_frame_type,
                 (unsigned long)(HAL_GetTick() - gCar.imu.last_frame_tick),
                 gCar.motion.left_pwm,
                 gCar.motion.right_pwm);
  if (len <= 0) return;
  if (len >= (int)sizeof(s_telemetry_line))
    len = (int)sizeof(s_telemetry_line) - 1;
  (void)HAL_UART_Transmit_IT(&huart2, (uint8_t *)s_telemetry_line,
                             (uint16_t)len);
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

static void MoveCargoManual(char command)
{
  gCar.sys.start_enable = 0U;
  gCar.sys.action = CAR_ACTION_STOP;
  Motor_Stop();

  if (command == 'q') CargoServo_AdjustLeft(-CARGO_LEFT_MANUAL_STEP_DEG);
  if (command == 'w') CargoServo_AdjustLeft(CARGO_LEFT_MANUAL_STEP_DEG);
  if (command == 'e') CargoServo_AdjustRight(-CARGO_RIGHT_MANUAL_STEP_DEG);
  if (command == 'r') CargoServo_AdjustRight(CARGO_RIGHT_MANUAL_STEP_DEG);
  if (command == 'o') CargoServo_Open();
  if (command == 'c') CargoServo_Close();
}

static void ParseCommand(const char *cmd)
{
  /* Uppercase S is the power-on Bluetooth handshake. It is intentionally
     handled before obstacle-test command filtering. STOP remains available
     as the explicit stop command; lowercase s remains pause. */
  if (strcmp(cmd, "S") == 0)
  {
    gCar.sys.start_enable = 1U;
    s_start_ack_pending = 1U;
    return;
  }

  if (VISION_V2_TEST_ONLY != 0U)
  {
    /* Commissioning mode accepts only a read-only sonar query. */
    if ((strcmp(cmd, "STATUS") == 0) || (strcmp(cmd, "SONAR") == 0))
    {
      Bluetooth_SendSonarStatus();
    }
    else
    {
      Bluetooth_Send("ERR,TEST_ONLY_USE_STATUS\r\n");
    }
    return;
  }

  if (strcmp(cmd, "q") == 0)
  {
    MoveCargoManual('q');
    Bluetooth_Send("OK,CLAW,LEFT,-\r\n");
  }
  else if (strcmp(cmd, "w") == 0)
  {
    MoveCargoManual('w');
    Bluetooth_Send("OK,CLAW,LEFT,+\r\n");
  }
  else if (strcmp(cmd, "e") == 0)
  {
    MoveCargoManual('e');
    Bluetooth_Send("OK,CLAW,RIGHT,-\r\n");
  }
  else if (strcmp(cmd, "r") == 0)
  {
    MoveCargoManual('r');
    Bluetooth_Send("OK,CLAW,RIGHT,+\r\n");
  }
  else if (strcmp(cmd, "o") == 0)
  {
    MoveCargoManual('o');
    Bluetooth_Send("OK,CLAW,OPEN\r\n");
  }
  else if (strcmp(cmd, "c") == 0)
  {
    MoveCargoManual('c');
    Bluetooth_Send("OK,CLAW,CLOSE\r\n");
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
  else if (strcmp(cmd, "STOP") == 0)
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
  else if (strcmp(cmd, "ZERO_YAW") == 0)
  {
    ImuJY61P_AnchorYaw();
    Bluetooth_Send("OK,ZERO_YAW\r\n");
  }
  else if (strcmp(cmd, "STATUS") == 0)
  {
    Bluetooth_SendSonarStatus();
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
  s_handshake_prompt_tick = HAL_GetTick() - CAR_BLUETOOTH_PROMPT_MS;
  s_last_rx_tick = HAL_GetTick();
  s_start_ack_pending = 0U;
  /* USART3/OpenMV remains the higher-priority vision link. USART2 telemetry
     uses a lower-priority interrupt so it cannot delay a camera frame. */
  HAL_NVIC_SetPriority(USART2_IRQn, 6U, 0U);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

void Bluetooth_Update(void)
{
  uint32_t now;
  uint8_t rx_budget = CAR_BLUETOOTH_RX_BUDGET_BYTES;

  /* Bound work per pass so a noisy or continuously transmitting peer cannot
     monopolise the main loop and starve sonar, motor and state updates. */
  while ((rx_budget > 0U) &&
         (HAL_UART_Receive(&huart2, &s_rx_byte, 1U, 0U) == HAL_OK))
  {
    rx_budget--;
    if ((s_index == 0U) &&
        ((s_rx_byte == 'a') || (s_rx_byte == 'g') ||
         (s_rx_byte == 'c') || (s_rx_byte == 'e') ||
         (s_rx_byte == 'i') || (s_rx_byte == 'j') ||
         (s_rx_byte == 'k') || (s_rx_byte == 'l') ||
         (s_rx_byte == 'o') || (s_rx_byte == 'q') ||
         (s_rx_byte == 'r') || (s_rx_byte == 's') ||
         (s_rx_byte == 'v') || (s_rx_byte == 'w')))
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
      s_last_rx_tick = HAL_GetTick();
    }
    else
    {
      s_index = 0U;
    }
  }

  now = HAL_GetTick();

  /* Accept a single S without requiring CR/LF. The short idle interval also
     preserves complete words such as STATUS and STOP that begin with S. */
  if ((s_index > 0U) &&
      ((now - s_last_rx_tick) >= CAR_BLUETOOTH_CMD_IDLE_MS))
  {
    s_line[s_index] = '\0';
    ParseCommand(s_line);
    s_index = 0U;
  }

  if ((s_start_ack_pending != 0U) &&
      (huart2.gState == HAL_UART_STATE_READY))
  {
    Bluetooth_Send("ACK,S,START\r\n");
    s_start_ack_pending = 0U;
  }

  if ((CAR_BLUETOOTH_START_HANDSHAKE != 0U) &&
      (gCar.sys.start_enable == 0U) &&
      ((now - s_handshake_prompt_tick) >= CAR_BLUETOOTH_PROMPT_MS) &&
      (huart2.gState == HAL_UART_STATE_READY))
  {
    s_handshake_prompt_tick = now;
    Bluetooth_Send("READY,S_TO_START\r\n");
  }

  /* Do not flood the link with long telemetry frames before the handshake.
     This keeps the short READY/ACK exchange reliable at 9600 baud. */
  if ((gCar.sys.start_enable != 0U) &&
      (CAR_BLUETOOTH_TELEMETRY_OUTPUT != 0U) &&
      ((now - s_telemetry_tick) >= CAR_BLUETOOTH_TELEMETRY_MS))
  {
    s_telemetry_tick = now;
    Bluetooth_SendRealtimeStatus();
  }
}
