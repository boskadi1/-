#include "debug_uart.h"
#include "main.h"
#include "car_config.h"
#include "car_data.h"
#include <stdio.h>

extern UART_HandleTypeDef huart1;

static char s_debug_buf[384];
static uint8_t s_rx_byte;
static char s_rx_line[16];
static uint8_t s_rx_index;

void DebugUart_Init(void)
{
  s_rx_byte = 0U;
  s_rx_index = 0U;
  s_rx_line[0] = '\0';
}

void DebugUart_Print(const char *text)
{
  uint16_t len = 0U;
  while ((text[len] != '\0') && (len < 280U)) len++;
  HAL_UART_Transmit(&huart1, (uint8_t *)text, len, 100U);
}

void DebugUart_PrintStatus(void)
{
  uint16_t front_cm = US_INVALID_CM;
  uint8_t front_valid = 0U;
  int len;

  if ((gCar.us.front_left_valid != 0U) &&
      (gCar.us.front_right_valid != 0U))
  {
    front_cm = (gCar.us.front_left_cm < gCar.us.front_right_cm) ?
               gCar.us.front_left_cm : gCar.us.front_right_cm;
    front_valid = 1U;
  }
  else if (gCar.us.front_left_valid != 0U)
  {
    front_cm = gCar.us.front_left_cm;
    front_valid = 1U;
  }
  else if (gCar.us.front_right_valid != 0U)
  {
    front_cm = gCar.us.front_right_cm;
    front_valid = 1U;
  }

  if (VISION_V2_TEST_ONLY != 0U)
  {
    /* Dedicated commissioning output: one compact line for all four sensors. */
    len = snprintf(s_debug_buf, sizeof(s_debug_buf),
                   "SONAR | FL=%ucm OK=%u | FR=%ucm OK=%u | LEFT=%ucm OK=%u | RIGHT=%ucm OK=%u | IMU_OK=%u YAW=%ld IRX=%lu IF=%lu/%lu IC=%lu IX=%lu IB=%lu IE=%lu/%lu/%lu IT=%02X AGE=%lu | ENC_YAW=%ld | V=%ld/%ld TV=%d/%d PWM=%d/%d\r\n",
                   gCar.us.front_left_cm,
                   gCar.us.front_left_valid,
                   gCar.us.front_right_cm,
                   gCar.us.front_right_valid,
                   gCar.us.left_cm,
                   gCar.us.left_valid,
                   gCar.us.right_cm,
                   gCar.us.right_valid,
                   gCar.imu.angle_valid,
                   (long)gCar.imu.yaw_rel_cd,
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
                   (long)gCar.encoder.yaw_cd,
                   (long)gCar.encoder.left_speed_mm_s,
                   (long)gCar.encoder.right_speed_mm_s,
                   gCar.motion.left_target_mm_s,
                   gCar.motion.right_target_mm_s,
                   gCar.motion.left_pwm,
                   gCar.motion.right_pwm);
  }
  else
  {
    len = snprintf(s_debug_buf, sizeof(s_debug_buf),
                   "US,FL_CM=%u,FL_OK=%u,FR_CM=%u,FR_OK=%u,L_CM=%u,L_OK=%u,R_CM=%u,R_OK=%u\r\n"
                   "NAV,D=%u/%u,OBS=%d/%u,CARGO=%d,A=%lu,Y=%u/%u,REACHED=%u,GOAL=%d,R=%u,Y=%u/%u,YAW=%ld,DH=%d,HERR=%d,ESC=%u,F=%d/%d,M=%d/%d,ENC=%ld/%ld/%ld/%ld,RX=%lu/%lu/%lu\r\n",
                   gCar.us.front_left_cm,
                   gCar.us.front_left_valid,
                   gCar.us.front_right_cm,
                   gCar.us.front_right_valid,
                   gCar.us.left_cm,
                   gCar.us.left_valid,
                   gCar.us.right_cm,
                   gCar.us.right_valid,
                   front_cm,
                   front_valid,
                   gCar.vision.obstacle_bearing_deg,
                   gCar.vision.obstacle_valid,
                   gCar.vision.load_relative_deg,
                   (unsigned long)gCar.vision.load_area,
                   gCar.vision.load_y_px,
                   gCar.vision.load_seen,
                   gCar.vision.cargo_reached,
                   gCar.vision.goal_bearing_deg,
                   gCar.vision.goal_radius_px,
                   gCar.vision.goal_y_px,
                   gCar.vision.goal_valid,
                   (long)gCar.encoder.yaw_cd,
                   gCar.motion.desired_world_heading_cd,
                   gCar.motion.nav_heading_deg,
                   gCar.motion.escape_mode,
                   0,
                   0,
                   gCar.motion.left_set_speed,
                   gCar.motion.right_set_speed,
                   (long)gCar.encoder.front_left_count,
                   (long)gCar.encoder.rear_left_count,
                   (long)gCar.encoder.front_right_count,
                   (long)gCar.encoder.rear_right_count,
                   (unsigned long)gCar.vision.rx_byte_count,
                   (unsigned long)gCar.vision.rx_line_count,
                   (unsigned long)gCar.vision.rx_parse_error_count);
  }

  if (len > 0)
  {
    if (len >= (int)sizeof(s_debug_buf))
    {
      len = (int)sizeof(s_debug_buf) - 1;
    }
    HAL_UART_Transmit(&huart1, (uint8_t *)s_debug_buf, (uint16_t)len, 100U);
  }
}

void DebugUart_Update(void)
{
  while (HAL_UART_Receive(&huart1, &s_rx_byte, 1U, 0U) == HAL_OK)
  {
    if ((s_rx_byte == '\r') || (s_rx_byte == '\n'))
    {
      if (s_rx_index > 0U)
      {
        s_rx_line[s_rx_index] = '\0';
        /* USART1 is a read-only commissioning port.  Any complete text line
           requests one snapshot, which also tolerates terminal-added bytes. */
        DebugUart_PrintStatus();
        s_rx_index = 0U;
      }
    }
    else if ((s_rx_byte >= 0x20U) && (s_rx_byte <= 0x7EU) &&
             (s_rx_index < (sizeof(s_rx_line) - 1U)))
    {
      s_rx_line[s_rx_index++] = (char)s_rx_byte;
    }
    else
    {
      s_rx_index = 0U;
    }
  }
}
