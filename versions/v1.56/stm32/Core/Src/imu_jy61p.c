#include "imu_jy61p.h"
#include "main.h"
#include "car_config.h"
#include "car_data.h"

extern UART_HandleTypeDef huart4;

static uint8_t s_rx_byte;
static uint8_t s_frame[11];
static uint8_t s_index;

static int16_t ToInt16(uint8_t low, uint8_t high)
{
  return (int16_t)((uint16_t)low | ((uint16_t)high << 8));
}

static int32_t AngleRawToCentiDeg(int16_t raw)
{
  return ((int32_t)raw * 18000L) / 32768L;
}

static int32_t NormalizeAngleCd(int32_t angle_cd)
{
  while (angle_cd > 18000L)
  {
    angle_cd -= 36000L;
  }

  while (angle_cd < -18000L)
  {
    angle_cd += 36000L;
  }

  return angle_cd;
}

static void ProcessByte(uint8_t byte)
{
  uint8_t checksum = 0U;
  uint8_t i;

  if (s_index == 0U)
  {
    if (byte != 0x55U)
    {
      return;
    }
  }
  else if (s_index == 1U)
  {
    if ((byte != 0x51U) && (byte != 0x52U) && (byte != 0x53U))
    {
      s_index = 0U;
      return;
    }
  }

  s_frame[s_index++] = byte;
  if (s_index < sizeof(s_frame))
  {
    return;
  }

  s_index = 0U;
  for (i = 0U; i < 10U; i++)
  {
    checksum += s_frame[i];
  }

  if (checksum != s_frame[10])
  {
    return;
  }

  if (s_frame[1] == 0x53U)
  {
    gCar.imu.last_frame_tick = HAL_GetTick();
    gCar.imu.roll_cd = AngleRawToCentiDeg(ToInt16(s_frame[2], s_frame[3]));
    gCar.imu.pitch_cd = AngleRawToCentiDeg(ToInt16(s_frame[4], s_frame[5]));
    gCar.imu.yaw_cd = AngleRawToCentiDeg(ToInt16(s_frame[6], s_frame[7]));

    if (gCar.imu.yaw_anchored == 0U)
    {
      ImuJY61P_AnchorYaw();
    }

    gCar.imu.yaw_rel_cd = NormalizeAngleCd(gCar.imu.yaw_cd - gCar.imu.yaw_zero_cd);
    gCar.imu.angle_valid = 1U;
  }
}

void ImuJY61P_Init(void)
{
  s_index = 0U;
  gCar.imu.last_frame_tick = HAL_GetTick();
}

void ImuJY61P_Update(void)
{
  while (HAL_UART_Receive(&huart4, &s_rx_byte, 1U, 1U) == HAL_OK)
  {
    ProcessByte(s_rx_byte);
  }

  if ((HAL_GetTick() - gCar.imu.last_frame_tick) > CAR_IMU_TIMEOUT_MS)
  {
    gCar.imu.angle_valid = 0U;
  }
}

void ImuJY61P_AnchorYaw(void)
{
  gCar.imu.yaw_zero_cd = gCar.imu.yaw_cd;
  gCar.imu.yaw_rel_cd = 0L;
  gCar.imu.yaw_anchored = 1U;
}
