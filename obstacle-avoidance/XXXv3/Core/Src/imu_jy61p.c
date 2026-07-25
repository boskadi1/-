#include "imu_jy61p.h"
#include "main.h"
#include "car_config.h"
#include "car_data.h"

extern UART_HandleTypeDef huart4;

static uint8_t s_irq_rx_byte;
static uint8_t s_rx_ring[CAR_IMU_RX_RING_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static uint8_t s_frame[11];
static uint8_t s_index;
static int32_t s_accel_bias_sum_forward_mg;
static int32_t s_accel_bias_sum_right_mg;
static int16_t s_accel_bias_forward_mg;
static int16_t s_accel_bias_right_mg;
static uint16_t s_accel_cal_samples;
static uint32_t s_accel_integrator_tick;
static float s_velocity_forward_mm_s;
static float s_velocity_right_mm_s;
static float s_displacement_forward_mm;
static float s_displacement_right_mm;

static void StartUartReceive(void)
{
  if (HAL_UART_Receive_IT(&huart4, &s_irq_rx_byte, 1U) != HAL_OK)
  {
    gCar.imu.rx_rearm_error_count++;
  }
}

static int16_t ToInt16(uint8_t low, uint8_t high)
{
  return (int16_t)((uint16_t)low | ((uint16_t)high << 8));
}

static int16_t AccelRawToMg(int16_t raw)
{
  return (int16_t)(((int32_t)raw * 16000L) / 32768L);
}

static int32_t FloatToInt32(float value)
{
  if (value > 2147483000.0f) value = 2147483000.0f;
  if (value < -2147483000.0f) value = -2147483000.0f;
  return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static float ClampFloat(float value, float low, float high)
{
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

/*
 * JY61P 0x51 frames use the configured +/-16 g scale. The car is stationary
 * while waiting for the Bluetooth S handshake, so the first samples provide
 * an X/Y zero-bias calibration. Integration is deliberately limited to
 * translating wheel commands and is zero-velocity-updated whenever the car
 * stops or pivots. It is a short-window collision cue, not absolute odometry.
 */
static void ProcessAccelerationFrame(void)
{
  const uint32_t now = HAL_GetTick();
  const int16_t raw_forward_mg =
      (int16_t)(CAR_IMU_FORWARD_X_SIGN *
                AccelRawToMg(ToInt16(s_frame[2], s_frame[3])));
  const int16_t raw_right_mg =
      (int16_t)(CAR_IMU_RIGHT_Y_SIGN *
                AccelRawToMg(ToInt16(s_frame[4], s_frame[5])));
  const int16_t raw_up_mg =
      AccelRawToMg(ToInt16(s_frame[6], s_frame[7]));
  int16_t forward_mg;
  int16_t right_mg;
  uint32_t elapsed_ms;
  float dt_s;
  float forward_accel_mm_s2;
  float right_accel_mm_s2;
  uint8_t translating;
  uint8_t stopped;

  gCar.imu.rx_accel_frame_count++;
  gCar.imu.last_accel_tick = now;
  gCar.imu.accel_valid = 1U;
  gCar.imu.accel_up_mg = raw_up_mg;

  if (s_accel_cal_samples < CAR_IMU_ACCEL_CAL_SAMPLES)
  {
    s_accel_bias_sum_forward_mg += raw_forward_mg;
    s_accel_bias_sum_right_mg += raw_right_mg;
    s_accel_cal_samples++;
    s_accel_integrator_tick = now;
    gCar.imu.accel_forward_mg = 0;
    gCar.imu.accel_right_mg = 0;
    if (s_accel_cal_samples >= CAR_IMU_ACCEL_CAL_SAMPLES)
    {
      s_accel_bias_forward_mg =
          (int16_t)(s_accel_bias_sum_forward_mg /
                    (int32_t)CAR_IMU_ACCEL_CAL_SAMPLES);
      s_accel_bias_right_mg =
          (int16_t)(s_accel_bias_sum_right_mg /
                    (int32_t)CAR_IMU_ACCEL_CAL_SAMPLES);
      gCar.imu.accel_calibrated = 1U;
    }
    return;
  }

  forward_mg = (int16_t)(raw_forward_mg - s_accel_bias_forward_mg);
  right_mg = (int16_t)(raw_right_mg - s_accel_bias_right_mg);
  if ((forward_mg < CAR_IMU_ACCEL_DEADBAND_MG) &&
      (forward_mg > -CAR_IMU_ACCEL_DEADBAND_MG)) forward_mg = 0;
  if ((right_mg < CAR_IMU_ACCEL_DEADBAND_MG) &&
      (right_mg > -CAR_IMU_ACCEL_DEADBAND_MG)) right_mg = 0;
  gCar.imu.accel_forward_mg = forward_mg;
  gCar.imu.accel_right_mg = right_mg;

  elapsed_ms = now - s_accel_integrator_tick;
  s_accel_integrator_tick = now;
  if ((elapsed_ms == 0U) || (elapsed_ms > 100U))
  {
    return;
  }

  stopped = ((gCar.motion.left_target_mm_s == 0) &&
             (gCar.motion.right_target_mm_s == 0)) ? 1U : 0U;
  translating =
      (((gCar.motion.left_target_mm_s > 0) &&
        (gCar.motion.right_target_mm_s > 0)) ||
       ((gCar.motion.left_target_mm_s < 0) &&
        (gCar.motion.right_target_mm_s < 0))) ? 1U : 0U;

  if (translating == 0U)
  {
    /* A physical stop or in-place pivot is a zero-velocity observation. */
    s_velocity_forward_mm_s = 0.0f;
    s_velocity_right_mm_s = 0.0f;
    if (stopped != 0U)
    {
      /* Slowly track temperature-dependent bias only while stationary. */
      s_accel_bias_forward_mg +=
          (int16_t)((raw_forward_mg - s_accel_bias_forward_mg) / 64);
      s_accel_bias_right_mg +=
          (int16_t)((raw_right_mg - s_accel_bias_right_mg) / 64);
    }
  }
  else
  {
    dt_s = (float)elapsed_ms / 1000.0f;
    forward_accel_mm_s2 = (float)forward_mg * 9.80665f;
    right_accel_mm_s2 = (float)right_mg * 9.80665f;
    s_displacement_forward_mm +=
        s_velocity_forward_mm_s * dt_s +
        0.5f * forward_accel_mm_s2 * dt_s * dt_s;
    s_displacement_right_mm +=
        s_velocity_right_mm_s * dt_s +
        0.5f * right_accel_mm_s2 * dt_s * dt_s;
    s_velocity_forward_mm_s = ClampFloat(
        s_velocity_forward_mm_s + forward_accel_mm_s2 * dt_s,
        -CAR_IMU_ACCEL_VELOCITY_MAX_MM_S,
        CAR_IMU_ACCEL_VELOCITY_MAX_MM_S);
    s_velocity_right_mm_s = ClampFloat(
        s_velocity_right_mm_s + right_accel_mm_s2 * dt_s,
        -CAR_IMU_ACCEL_VELOCITY_MAX_MM_S,
        CAR_IMU_ACCEL_VELOCITY_MAX_MM_S);
  }

  gCar.imu.velocity_forward_mm_s =
      FloatToInt32(s_velocity_forward_mm_s);
  gCar.imu.velocity_right_mm_s = FloatToInt32(s_velocity_right_mm_s);
  gCar.imu.displacement_forward_mm =
      FloatToInt32(s_displacement_forward_mm);
  gCar.imu.displacement_right_mm =
      FloatToInt32(s_displacement_right_mm);
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
    /* WIT/JY61 streams may contain time, acceleration, gyro, angle,
       magnetic-field or quaternion frames. Keep their 11-byte framing even
       though only the 0x53 angle frame drives navigation. */
    if ((byte < 0x50U) || (byte > 0x59U))
    {
      gCar.imu.rx_format_error_count++;
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
    gCar.imu.rx_checksum_error_count++;
    return;
  }

  gCar.imu.rx_frame_count++;
  gCar.imu.last_frame_type = s_frame[1];

  if (s_frame[1] == 0x51U)
  {
    ProcessAccelerationFrame();
  }
  else if (s_frame[1] == 0x53U)
  {
    gCar.imu.rx_angle_frame_count++;
    gCar.imu.last_frame_tick = HAL_GetTick();
    gCar.imu.roll_cd = AngleRawToCentiDeg(ToInt16(s_frame[2], s_frame[3]));
    gCar.imu.pitch_cd = AngleRawToCentiDeg(ToInt16(s_frame[4], s_frame[5]));
    gCar.imu.yaw_cd = NormalizeAngleCd(
        (int32_t)CAR_IMU_YAW_SIGN *
        AngleRawToCentiDeg(ToInt16(s_frame[6], s_frame[7])));

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
  s_irq_rx_byte = 0U;
  s_rx_head = 0U;
  s_rx_tail = 0U;
  s_accel_bias_sum_forward_mg = 0L;
  s_accel_bias_sum_right_mg = 0L;
  s_accel_bias_forward_mg = 0;
  s_accel_bias_right_mg = 0;
  s_accel_cal_samples = 0U;
  s_accel_integrator_tick = HAL_GetTick();
  s_velocity_forward_mm_s = 0.0f;
  s_velocity_right_mm_s = 0.0f;
  s_displacement_forward_mm = 0.0f;
  s_displacement_right_mm = 0.0f;
  gCar.imu.roll_cd = 0L;
  gCar.imu.pitch_cd = 0L;
  gCar.imu.yaw_cd = 0L;
  gCar.imu.yaw_zero_cd = 0L;
  gCar.imu.yaw_rel_cd = 0L;
  gCar.imu.angle_valid = 0U;
  gCar.imu.yaw_anchored = 0U;
  gCar.imu.last_frame_tick = HAL_GetTick();
  gCar.imu.last_rx_tick = HAL_GetTick();
  gCar.imu.rx_byte_count = 0U;
  gCar.imu.rx_frame_count = 0U;
  gCar.imu.rx_accel_frame_count = 0U;
  gCar.imu.rx_angle_frame_count = 0U;
  gCar.imu.rx_checksum_error_count = 0U;
  gCar.imu.rx_format_error_count = 0U;
  gCar.imu.rx_ring_overflow_count = 0U;
  gCar.imu.uart_error_count = 0U;
  gCar.imu.uart_overrun_count = 0U;
  gCar.imu.rx_rearm_error_count = 0U;
  gCar.imu.last_rx_byte = 0U;
  gCar.imu.last_frame_type = 0U;
  gCar.imu.accel_forward_mg = 0;
  gCar.imu.accel_right_mg = 0;
  gCar.imu.accel_up_mg = 0;
  gCar.imu.velocity_forward_mm_s = 0L;
  gCar.imu.velocity_right_mm_s = 0L;
  gCar.imu.displacement_forward_mm = 0L;
  gCar.imu.displacement_right_mm = 0L;
  gCar.imu.last_accel_tick = HAL_GetTick();
  gCar.imu.accel_valid = 0U;
  gCar.imu.accel_calibrated = 0U;
  gCar.imu.collision_valid = 0U;
  gCar.imu.collision_bearing_deg = 0;
  gCar.imu.collision_count = 0U;
  gCar.imu.collision_tick = 0U;

  __HAL_UART_CLEAR_OREFLAG(&huart4);
  StartUartReceive();
}

void ImuJY61P_Update(void)
{
  uint8_t remaining = CAR_IMU_RX_BUDGET_BYTES;

  /* UART4 IRQ captures every byte while ultrasonic ranging blocks the main
     loop. Drain a bounded number here so parsing cannot starve motor control. */
  while ((remaining > 0U) && (s_rx_tail != s_rx_head))
  {
    const uint8_t rx_byte = s_rx_ring[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) % CAR_IMU_RX_RING_SIZE);
    ProcessByte(rx_byte);
    remaining--;
  }

  if ((HAL_GetTick() - gCar.imu.last_frame_tick) > CAR_IMU_TIMEOUT_MS)
  {
    gCar.imu.angle_valid = 0U;
  }
  if ((HAL_GetTick() - gCar.imu.last_accel_tick) > CAR_IMU_TIMEOUT_MS)
  {
    gCar.imu.accel_valid = 0U;
  }
}

void ImuJY61P_AnchorYaw(void)
{
  gCar.imu.yaw_zero_cd = gCar.imu.yaw_cd;
  gCar.imu.yaw_rel_cd = 0L;
  gCar.imu.yaw_anchored = 1U;
}

void ImuJY61P_UartRxCompleteCallback(UART_HandleTypeDef *huart)
{
  uint16_t next_head;

  if (huart->Instance != UART4) return;

  gCar.imu.rx_byte_count++;
  gCar.imu.last_rx_byte = s_irq_rx_byte;
  gCar.imu.last_rx_tick = HAL_GetTick();
  next_head = (uint16_t)((s_rx_head + 1U) % CAR_IMU_RX_RING_SIZE);
  if (next_head != s_rx_tail)
  {
    s_rx_ring[s_rx_head] = s_irq_rx_byte;
    s_rx_head = next_head;
  }
  else
  {
    gCar.imu.rx_ring_overflow_count++;
  }

  StartUartReceive();
}

void ImuJY61P_UartErrorCallback(UART_HandleTypeDef *huart)
{
  uint32_t error;

  if (huart->Instance != UART4) return;

  error = huart->ErrorCode;
  gCar.imu.uart_error_count++;
  if ((error & HAL_UART_ERROR_ORE) != 0U)
  {
    gCar.imu.uart_overrun_count++;
  }
  __HAL_UART_CLEAR_OREFLAG(huart);
  huart->ErrorCode = HAL_UART_ERROR_NONE;
  StartUartReceive();
}
