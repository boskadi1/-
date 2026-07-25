#include "debug_uart.h"
#include "main.h"
#include "car_config.h"
#include "car_data.h"
#include <stdio.h>

extern UART_HandleTypeDef huart1;

static char s_debug_buf[360];

void DebugUart_Print(const char *text)
{
  uint16_t len = 0U;

  while ((text[len] != '\0') && (len < 260U))
  {
    len++;
  }

  HAL_UART_Transmit(&huart1, (uint8_t *)text, len, 100U);
}

void DebugUart_PrintStatus(void)
{
  int len = snprintf(s_debug_buf, sizeof(s_debug_buf),
                     "STAT,V=%s,MODE=%s,STAGE=%s,ACT=%s,US=%u/%u/%u/%u,V=%u%u%u%u,LINE=%d/%d/%u/%u/%d,D=%d,LOAD=%d/%d/%lu/%u,TAG=%u/%d/%d/%lu/%u,VRX=%lu/%lu/%lu/%lu,CAM=%d/%d>%d/%d,YAW=%ld.%02ld,TYAW=%d,ERR=%d\r\n",
                     CAR_SOFTWARE_VERSION,
                     CarData_ModeName(gCar.sys.mode),
                     CarData_StageName(gCar.sys.stage),
                     CarData_ActionName(gCar.sys.action),
                     gCar.us.front_left_cm,
                     gCar.us.front_right_cm,
                     gCar.us.left_cm,
                     gCar.us.right_cm,
                     gCar.us.front_left_valid,
                     gCar.us.front_right_valid,
                     gCar.us.left_valid,
                     gCar.us.right_valid,
                     gCar.vision.line_offset,
                     gCar.vision.line_heading,
                     gCar.vision.line_magnitude,
                     gCar.vision.line_valid,
                     gCar.motion.line_correction,
                     gCar.motion.line_d_correction,
                     gCar.vision.load_relative_deg,
                     gCar.vision.load_vertical_deg,
                     gCar.vision.load_area,
                     gCar.vision.load_seen,
                     gCar.vision.tag_id,
                     gCar.vision.tag_relative_deg,
                     gCar.vision.tag_vertical_deg,
                     gCar.vision.tag_area,
                     gCar.vision.tag_valid,
                     (unsigned long)gCar.vision.rx_byte_count,
                     (unsigned long)gCar.vision.rx_line_count,
                     (unsigned long)gCar.vision.rx_parse_error_count,
                     (unsigned long)gCar.vision.rx_overflow_count,
                     gCar.camera.current_deg,
                     gCar.camera.current_tilt_deg,
                     gCar.camera.target_deg,
                     gCar.camera.target_tilt_deg,
                     gCar.imu.yaw_rel_cd / 100L,
                     (gCar.imu.yaw_rel_cd < 0 ? -gCar.imu.yaw_rel_cd : gCar.imu.yaw_rel_cd) % 100L,
                     gCar.imu.target_yaw_cd,
                     gCar.imu.yaw_error_cd);

  if (len > 0)
  {
    if (len >= (int)sizeof(s_debug_buf))
    {
      len = (int)sizeof(s_debug_buf) - 1;
    }
    HAL_UART_Transmit(&huart1, (uint8_t *)s_debug_buf, (uint16_t)len, 100U);
  }
}
