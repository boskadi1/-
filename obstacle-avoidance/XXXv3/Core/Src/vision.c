#include "vision.h"
#include "main.h"
#include "car_config.h"
#include "car_data.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart3;

static uint8_t s_irq_rx_byte;
static uint8_t s_rx_ring[CAR_VISION_RX_RING_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static char s_rx_line[CAR_VISION_RX_BUFFER_SIZE];
static uint8_t s_rx_index;
static uint8_t s_heading_tx_buffer[24];
static uint32_t s_last_heading_tx_tick;

static void StartUartReceive(void)
{
  (void)HAL_UART_Receive_IT(&huart3, &s_irq_rx_byte, 1U);
}

static int16_t ClampBearing(int value)
{
  if (value > 89) value = 89;
  if (value < -89) value = -89;
  return (int16_t)value;
}

static int16_t NormalizeCd(int32_t angle_cd)
{
  while (angle_cd > 18000L) angle_cd -= 36000L;
  while (angle_cd < -18000L) angle_cd += 36000L;
  return (int16_t)angle_cd;
}

static int16_t CurrentNavigationYawCd(void)
{
  if ((gCar.imu.angle_valid != 0U) &&
      (gCar.imu.yaw_anchored != 0U))
  {
    return NormalizeCd(gCar.imu.yaw_rel_cd);
  }

  return NormalizeCd(gCar.encoder.yaw_cd);
}

/*
 * Full-duplex USART3 link:
 *   OpenMV -> STM32: NAV,...\n
 *   STM32 -> OpenMV: YAW,<centidegrees>\n
 *
 * This frame is only 8-12 bytes. At 115200 baud the blocking transmit takes
 * about 1 ms and leaves the independent interrupt RX path armed. Using the
 * blocking API also avoids depending on a TX-complete interrupt while the
 * same UART is continuously servicing one-byte RX interrupts.
 */
static void SendHeadingToOpenMv(void)
{
  const uint32_t now = HAL_GetTick();
  HAL_StatusTypeDef status;
  int length;

  if ((now - s_last_heading_tx_tick) < CAR_OPENMV_HEADING_TX_MS) return;
  s_last_heading_tx_tick = now;

  length = snprintf((char *)s_heading_tx_buffer,
                    sizeof(s_heading_tx_buffer),
                    "YAW,%d\n", (int)CurrentNavigationYawCd());
  if ((length <= 0) || (length >= (int)sizeof(s_heading_tx_buffer))) return;

  status = HAL_UART_Transmit(&huart3, s_heading_tx_buffer,
                             (uint16_t)length, 5U);
  gCar.vision.heading_tx_last_status = (uint8_t)status;
  gCar.vision.heading_tx_uart_state = (uint8_t)huart3.gState;
  if (status == HAL_OK)
  {
    gCar.vision.heading_tx_ok_count++;
  }
  else
  {
    gCar.vision.heading_tx_fail_count++;
  }
}

/*
 * OpenMV frame:
 * NAV,obstacle_valid,obstacle_bearing_deg,
 *     cargo_valid,cargo_bearing_deg,cargo_vertical_deg,cargo_area,cargo_y,
 *     goal_valid,goal_bearing_deg,goal_vertical_deg,goal_radius_px,goal_y_px,
 *     obstacle_distance_cm,obstacle_vertical_deg,obstacle_area_pixels,
 *     pan_deg,pan_pulse_us,heading_link_valid,heading_rx_yaw_cd,
 *     obstacle_top_full,obstacle_free_bias\n
 * Fields after the original first twelve are optional for compatibility.
 */
static uint8_t ParseNavLine(const char *line)
{
  int obstacle_valid;
  int obstacle_bearing;
  int cargo_valid;
  int cargo_bearing;
  int cargo_vertical;
  int cargo_area;
  int cargo_y;
  int goal_valid;
  int goal_bearing;
  int goal_vertical;
  int goal_radius;
  int goal_y;
  int obstacle_distance = 0;
  int obstacle_vertical = 0;
  int obstacle_area = 0;
  int camera_pan = 0;
  int camera_pan_pulse = 1500;
  int heading_link_valid = 0;
  int heading_rx_yaw = 0;
  int obstacle_top_full = 0;
  int obstacle_free_bias = 0;
  int parsed;

  parsed = sscanf(line,
             "NAV,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
             &obstacle_valid, &obstacle_bearing,
             &cargo_valid, &cargo_bearing, &cargo_vertical,
             &cargo_area, &cargo_y,
             &goal_valid, &goal_bearing, &goal_vertical,
             &goal_radius, &goal_y,
             &obstacle_distance, &obstacle_vertical, &obstacle_area,
             &camera_pan, &camera_pan_pulse,
             &heading_link_valid, &heading_rx_yaw,
             &obstacle_top_full, &obstacle_free_bias);
  if (parsed < 12)
  {
    return 0U;
  }
  if (parsed < 13) obstacle_distance = 0;
  if (parsed < 14) obstacle_vertical = 0;
  if (parsed < 15) obstacle_area = 0;
  if (parsed < 16) camera_pan = 0;
  if (parsed < 17) camera_pan_pulse = 1500;
  if (parsed < 18) heading_link_valid = 0;
  if (parsed < 19) heading_rx_yaw = 0;
  if (parsed < 20) obstacle_top_full = 0;
  if (parsed < 21) obstacle_free_bias = 0;

  if (cargo_area < 0) cargo_area = 0;
  if (cargo_area > 1000000) cargo_area = 1000000;
  if (cargo_y < 0) cargo_y = 0;
  if (cargo_y > 1000) cargo_y = 1000;
  if (goal_radius < 0) goal_radius = 0;
  if (goal_radius > 1000) goal_radius = 1000;
  if (goal_y < 0) goal_y = 0;
  if (goal_y > 1000) goal_y = 1000;
  if (obstacle_distance < 0) obstacle_distance = 0;
  if (obstacle_distance > 1000) obstacle_distance = 1000;
  if (obstacle_area < 0) obstacle_area = 0;
  if (obstacle_area > 1000000) obstacle_area = 1000000;
  if (camera_pan < -90) camera_pan = -90;
  if (camera_pan > 90) camera_pan = 90;
  if (camera_pan_pulse < 400) camera_pan_pulse = 400;
  if (camera_pan_pulse > 2600) camera_pan_pulse = 2600;
  if (heading_rx_yaw < -18000) heading_rx_yaw = -18000;
  if (heading_rx_yaw > 18000) heading_rx_yaw = 18000;
  if (obstacle_free_bias < -1) obstacle_free_bias = -1;
  if (obstacle_free_bias > 1) obstacle_free_bias = 1;

  gCar.vision.obstacle_valid = (obstacle_valid != 0) ? 1U : 0U;
  gCar.vision.obstacle_bearing_deg = ClampBearing(obstacle_bearing);
  gCar.vision.obstacle_vertical_deg = ClampBearing(obstacle_vertical);
  gCar.vision.obstacle_distance_cm =
      (gCar.vision.obstacle_valid != 0U) ?
      (uint16_t)obstacle_distance : 0U;
  gCar.vision.obstacle_area_pixels =
      (gCar.vision.obstacle_valid != 0U) ?
      (uint32_t)obstacle_area : 0U;
  gCar.vision.load_seen = (cargo_valid != 0) ? 1U : 0U;
  gCar.vision.load_relative_deg = ClampBearing(cargo_bearing);
  gCar.vision.load_vertical_deg = ClampBearing(cargo_vertical);
  gCar.vision.load_area = (uint32_t)cargo_area;
  gCar.vision.load_y_px = (uint16_t)cargo_y;
  gCar.vision.last_load_tick = HAL_GetTick();
  gCar.vision.goal_valid = (goal_valid != 0) ? 1U : 0U;
  gCar.vision.goal_bearing_deg = ClampBearing(goal_bearing);
  gCar.vision.goal_vertical_deg = ClampBearing(goal_vertical);
  gCar.vision.goal_radius_px = (uint16_t)goal_radius;
  gCar.vision.goal_y_px = (uint16_t)goal_y;
  gCar.vision.nav_frame_valid = 1U;
  gCar.vision.last_nav_tick = HAL_GetTick();
  gCar.vision.camera_pan_deg = (int16_t)camera_pan;
  gCar.vision.camera_pan_pulse_us = (uint16_t)camera_pan_pulse;
  gCar.vision.heading_rx_yaw_cd = (int16_t)heading_rx_yaw;
  gCar.vision.heading_link_valid =
      (heading_link_valid != 0) ? 1U : 0U;
  gCar.vision.obstacle_top_full =
      (obstacle_top_full != 0) ? 1U : 0U;
  gCar.vision.obstacle_free_bias = (int8_t)obstacle_free_bias;
  return 1U;
}

void Vision_Init(void)
{
  memset(s_rx_line, 0, sizeof(s_rx_line));
  s_rx_head = 0U;
  s_rx_tail = 0U;
  s_rx_index = 0U;
  s_last_heading_tx_tick = HAL_GetTick() - CAR_OPENMV_HEADING_TX_MS;
  gCar.vision.obstacle_valid = 0U;
  gCar.vision.obstacle_distance_cm = 0U;
  gCar.vision.obstacle_vertical_deg = 0;
  gCar.vision.obstacle_area_pixels = 0U;
  gCar.vision.load_seen = 0U;
  gCar.vision.goal_valid = 0U;
  gCar.vision.nav_frame_valid = 0U;
  gCar.vision.camera_pan_deg = 0;
  gCar.vision.camera_pan_pulse_us = 1500U;
  gCar.vision.heading_rx_yaw_cd = 0;
  gCar.vision.heading_link_valid = 0U;
  gCar.vision.obstacle_top_full = 0U;
  gCar.vision.obstacle_free_bias = 0;
  gCar.vision.heading_tx_ok_count = 0U;
  gCar.vision.heading_tx_fail_count = 0U;
  gCar.vision.heading_tx_last_status = (uint8_t)HAL_OK;
  gCar.vision.heading_tx_uart_state = (uint8_t)huart3.gState;
  gCar.vision.last_nav_tick = HAL_GetTick();
  gCar.vision.last_rx_tick = HAL_GetTick();
  StartUartReceive();
}

void Vision_Update(void)
{
  SendHeadingToOpenMv();

  while (s_rx_tail != s_rx_head)
  {
    const uint8_t rx_byte = s_rx_ring[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) % CAR_VISION_RX_RING_SIZE);
    gCar.vision.rx_byte_count++;
    gCar.vision.last_rx_tick = HAL_GetTick();

    if ((rx_byte == '\n') || (rx_byte == '\r'))
    {
      if (s_rx_index > 0U)
      {
        s_rx_line[s_rx_index] = '\0';
        gCar.vision.rx_line_count++;
        if (ParseNavLine(s_rx_line) == 0U)
        {
          gCar.vision.rx_parse_error_count++;
        }
        s_rx_index = 0U;
      }
    }
    else if (s_rx_index < (CAR_VISION_RX_BUFFER_SIZE - 1U))
    {
      s_rx_line[s_rx_index++] = (char)rx_byte;
    }
    else
    {
      s_rx_index = 0U;
      gCar.vision.rx_parse_error_count++;
    }
  }

  if ((HAL_GetTick() - gCar.vision.last_nav_tick) > CAR_VISION_TIMEOUT_MS)
  {
    gCar.vision.obstacle_valid = 0U;
    gCar.vision.obstacle_distance_cm = 0U;
    gCar.vision.obstacle_vertical_deg = 0;
    gCar.vision.obstacle_area_pixels = 0U;
    gCar.vision.load_seen = 0U;
    gCar.vision.goal_valid = 0U;
    gCar.vision.nav_frame_valid = 0U;
    gCar.vision.heading_link_valid = 0U;
    gCar.vision.obstacle_top_full = 0U;
    gCar.vision.obstacle_free_bias = 0;
  }
}

void Vision_UartRxCompleteCallback(UART_HandleTypeDef *huart)
{
  uint16_t next_head;

  if (huart->Instance != USART3) return;

  next_head = (uint16_t)((s_rx_head + 1U) % CAR_VISION_RX_RING_SIZE);
  if (next_head != s_rx_tail)
  {
    s_rx_ring[s_rx_head] = s_irq_rx_byte;
    s_rx_head = next_head;
  }
  else
  {
    gCar.vision.rx_overflow_count++;
  }

  StartUartReceive();
}

void Vision_UartErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART3) return;

  gCar.vision.rx_overflow_count++;
  __HAL_UART_CLEAR_OREFLAG(huart);
  huart->ErrorCode = HAL_UART_ERROR_NONE;
  StartUartReceive();
}
