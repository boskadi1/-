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
static VisionProcessingMode_t s_processing_mode;
static uint32_t s_last_mode_tx_tick;

static void SendProcessingMode(void)
{
  const char *command = (s_processing_mode == VISION_PROCESS_LINE) ?
                        "MODE,LINE\r\n" : "MODE,RED\r\n";
  (void)HAL_UART_Transmit(&huart3, (uint8_t *)command,
                         (uint16_t)strlen(command), 20U);
  s_last_mode_tx_tick = HAL_GetTick();
}

static void StartUartReceive(void)
{
  (void)HAL_UART_Receive_IT(&huart3, &s_irq_rx_byte, 1U);
}

static int16_t ClampInt16(int value)
{
  if (value > 32767) return 32767;
  if (value < -32768) return -32768;
  return (int16_t)value;
}

static uint16_t ClampUint16(int value)
{
  if (value < 0) return 0U;
  if (value > 65535) return 65535U;
  return (uint16_t)value;
}

static void StoreTag(int tag_id, int horizontal_deg, int vertical_deg,
                     long area, int valid, uint32_t now)
{
  uint8_t is_valid = (valid != 0) ? 1U : 0U;

  if (area < 0) area = 0;
  gCar.vision.tag_id = (uint8_t)tag_id;
  gCar.vision.tag_relative_deg = ClampInt16(horizontal_deg);
  gCar.vision.tag_vertical_deg = ClampInt16(vertical_deg);
  gCar.vision.tag_area = (uint32_t)area;
  gCar.vision.tag_valid = is_valid;
  gCar.vision.unload_seen = ((tag_id == TAG_UNLOAD_ID) && is_valid) ? 1U : 0U;
  if (tag_id == TAG_UNLOAD_ID)
  {
    /* Legacy TAG packets do not contain the calibrated circle position. */
    gCar.vision.unload_position_state = CAR_UNLOAD_STATE_NOT_READY;
    gCar.vision.unload_position_valid = 0U;
  }
  gCar.vision.last_tag_tick = now;

  if (tag_id == TAG_LOAD_ID)
  {
    gCar.vision.load_relative_deg = ClampInt16(horizontal_deg);
    gCar.vision.load_vertical_deg = ClampInt16(vertical_deg);
    gCar.vision.load_area = (uint32_t)area;
    gCar.vision.load_seen = is_valid;
    gCar.vision.last_load_tick = now;
  }
}

static uint8_t Vision_ParseLine(const char *line)
{
  int offset = 0;
  int heading = 0;
  int magnitude = 0;
  int valid = 0;
  int tag_id = 0;
  int horizontal_deg = 0;
  int vertical_deg = 0;
  long area = 0;
  int target_valid = 0;
  int grip_state = 0;
  int unload_state = 0;
  int unload_entry_valid = 0;
  uint32_t now = HAL_GetTick();

  if (sscanf(line, "MV,LINE,%d,%d,%d,%d", &offset, &heading,
             &magnitude, &valid) == 4)
  {
    gCar.vision.line_offset = ClampInt16(offset);
    gCar.vision.line_heading = ClampInt16(heading);
    gCar.vision.line_magnitude = ClampUint16(magnitude);
    gCar.vision.line_valid = ((valid != 0) &&
                              (gCar.vision.line_magnitude >=
                               CAR_LINE_MIN_MAGNITUDE)) ? 1U : 0U;
    gCar.vision.last_line_tick = now;
    return 1U;
  }

  /* Backward compatibility with MV,LINE,<offset>,<valid>. */
  if (sscanf(line, "MV,LINE,%d,%d", &offset, &valid) == 2)
  {
    gCar.vision.line_offset = ClampInt16(offset);
    gCar.vision.line_heading = 0;
    gCar.vision.line_magnitude = 0U;
    gCar.vision.line_valid = (valid != 0) ? 1U : 0U;
    gCar.vision.last_line_tick = now;
    return 1U;
  }

  if (sscanf(line, "MV,LOAD,%d,%d,%ld,%d,%d", &horizontal_deg, &vertical_deg,
             &area, &grip_state, &target_valid) == 5)
  {
    if (area < 0) area = 0;
    if (grip_state < (int)CAR_GRIP_STATE_NOT_READY) grip_state = CAR_GRIP_STATE_NOT_READY;
    if (grip_state > (int)CAR_GRIP_STATE_TOO_CLOSE) grip_state = CAR_GRIP_STATE_TOO_CLOSE;
    gCar.vision.load_relative_deg = ClampInt16(horizontal_deg);
    gCar.vision.load_vertical_deg = ClampInt16(vertical_deg);
    gCar.vision.load_area = (uint32_t)area;
    gCar.vision.load_grip_state = (uint8_t)grip_state;
    gCar.vision.load_seen = (target_valid != 0) ? 1U : 0U;
    gCar.vision.last_load_tick = now;
    return 1U;
  }

  /* Black circular unloading zone from OpenMV sensor v2.4:
   * MV,UNLOAD,horizontal,vertical,area,position_state,valid */
  if (sscanf(line, "MV,UNLOAD,%d,%d,%ld,%d,%d", &horizontal_deg,
             &vertical_deg, &area, &unload_state, &target_valid) == 5)
  {
    if (area < 0) area = 0;
    if (unload_state < (int)CAR_UNLOAD_STATE_NOT_READY)
      unload_state = CAR_UNLOAD_STATE_NOT_READY;
    if (unload_state > (int)CAR_UNLOAD_STATE_TOO_CLOSE)
      unload_state = CAR_UNLOAD_STATE_TOO_CLOSE;
    gCar.vision.tag_id = TAG_UNLOAD_ID;
    gCar.vision.tag_relative_deg = ClampInt16(horizontal_deg);
    gCar.vision.tag_vertical_deg = ClampInt16(vertical_deg);
    gCar.vision.tag_area = (uint32_t)area;
    gCar.vision.tag_valid = (target_valid != 0) ? 1U : 0U;
    gCar.vision.unload_seen = gCar.vision.tag_valid;
    gCar.vision.unload_position_state = (uint8_t)unload_state;
    gCar.vision.unload_position_valid = gCar.vision.tag_valid;
    gCar.vision.last_tag_tick = now;
    return 1U;
  }

  if (sscanf(line, "MV,UNLOAD_ENTRY,%d", &unload_entry_valid) == 1)
  {
    gCar.vision.unload_zone_seen =
        (unload_entry_valid != 0) ? 1U : 0U;
    gCar.vision.last_unload_zone_tick = now;
    return 1U;
  }

  /* Backward compatibility with the v2.0 red packet. */
  if (sscanf(line, "MV,LOAD,%d,%d,%ld,%d", &horizontal_deg, &vertical_deg,
             &area, &target_valid) == 4)
  {
    if (area < 0) area = 0;
    gCar.vision.load_relative_deg = ClampInt16(horizontal_deg);
    gCar.vision.load_vertical_deg = ClampInt16(vertical_deg);
    gCar.vision.load_area = (uint32_t)area;
    gCar.vision.load_grip_state = CAR_GRIP_STATE_NOT_READY;
    gCar.vision.load_seen = (target_valid != 0) ? 1U : 0U;
    gCar.vision.last_load_tick = now;
    return 1U;
  }

  if (sscanf(line, "MV,TAG,%d,%d,%d,%ld,%d", &tag_id, &horizontal_deg,
             &vertical_deg, &area, &target_valid) == 5)
  {
    StoreTag(tag_id, horizontal_deg, vertical_deg, area, target_valid, now);
    return 1U;
  }

  if (sscanf(line, "MV,TAG,%d,%d,%ld,%d", &tag_id, &horizontal_deg,
             &area, &target_valid) == 4)
  {
    StoreTag(tag_id, horizontal_deg, 0, area, target_valid, now);
    return 1U;
  }

  return 0U;
}

void Vision_Init(void)
{
  memset(s_rx_line, 0, sizeof(s_rx_line));
  s_rx_head = 0U;
  s_rx_tail = 0U;
  s_rx_index = 0U;
  gCar.vision.last_line_tick = HAL_GetTick();
  gCar.vision.last_load_tick = HAL_GetTick();
  gCar.vision.last_tag_tick = HAL_GetTick();
  gCar.vision.last_rx_tick = HAL_GetTick();
  s_processing_mode = VISION_PROCESS_RED;
  s_last_mode_tx_tick = 0U;
  StartUartReceive();
  SendProcessingMode();
}

void Vision_SetProcessingMode(VisionProcessingMode_t mode)
{
  if (mode != s_processing_mode)
  {
    s_processing_mode = mode;
    if (mode == VISION_PROCESS_RED)
    {
      gCar.vision.line_valid = 0U;
    }
    else
    {
      gCar.vision.load_seen = 0U;
    }
    SendProcessingMode();
  }
}

void Vision_Update(void)
{
  uint32_t now = HAL_GetTick();

  while (s_rx_tail != s_rx_head)
  {
    uint8_t rx_byte = s_rx_ring[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) % CAR_VISION_RX_RING_SIZE);
    gCar.vision.rx_byte_count++;
    gCar.vision.last_rx_tick = HAL_GetTick();

    if ((rx_byte == '\n') || (rx_byte == '\r'))
    {
      if (s_rx_index > 0U)
      {
        s_rx_line[s_rx_index] = '\0';
        gCar.vision.rx_line_count++;
        if (Vision_ParseLine(s_rx_line) == 0U)
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

  if ((HAL_GetTick() - gCar.vision.last_line_tick) > CAR_VISION_TIMEOUT_MS)
  {
    gCar.vision.line_valid = 0U;
  }

  if ((HAL_GetTick() - gCar.vision.last_load_tick) > CAR_VISION_TIMEOUT_MS)
  {
    gCar.vision.load_seen = 0U;
    gCar.vision.load_grip_state = CAR_GRIP_STATE_NOT_READY;
  }

  if ((HAL_GetTick() - gCar.vision.last_tag_tick) > CAR_VISION_TIMEOUT_MS)
  {
    gCar.vision.tag_valid = 0U;
    gCar.vision.unload_seen = 0U;
    gCar.vision.unload_position_state = CAR_UNLOAD_STATE_NOT_READY;
    gCar.vision.unload_position_valid = 0U;
  }

  /* Repeat the desired mode so OpenMV can boot/reboot after STM32 without
   * permanently missing the one-time switch command. */
  if ((now - s_last_mode_tx_tick) >= CAR_VISION_MODE_REPEAT_MS)
  {
    SendProcessingMode();
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
