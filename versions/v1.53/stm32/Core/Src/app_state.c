#include "app_state.h"
#include "main.h"
#include "car_config.h"
#include "car_data.h"
#include "camera_servo.h"
#include "grab_servo.h"
#include "vision.h"

static CarMode_t s_last_mode = CAR_MODE_STOP;
static uint8_t s_target_ready_active;
static uint32_t s_target_ready_tick;
static uint8_t s_ultrasonic_seen_once;
static uint32_t s_last_ultrasonic_valid_tick;
static uint8_t s_line_seen_once;
static uint8_t s_line_was_valid;
static uint32_t s_line_lost_tick;
static int16_t s_last_line_offset;
static int16_t s_last_line_raw_correction;
static int16_t s_last_line_correction;
static int16_t s_last_line_error;
static int16_t s_line_derivative_filtered;
static uint32_t s_last_line_error_tick;
static uint8_t s_line_derivative_ready;
static int16_t s_line_delay_buffer[CAR_LINE_DELAY_BUFFER_SIZE];
static uint8_t s_line_delay_write_index;
static uint8_t s_line_delay_count;

typedef enum
{
  UNLOAD_SWEEP_FORWARD = 0,
  UNLOAD_SWEEP_LEFT,
  UNLOAD_SWEEP_RIGHT
} UnloadSweepPhase_t;

static UnloadSweepPhase_t s_unload_sweep_phase;
static uint8_t s_unload_sweep_started;
static uint8_t s_unload_sweep_first_left;
static uint32_t s_unload_sweep_tick;

static void ResetLineCorrectionDelay(void)
{
  uint8_t i;

  for (i = 0U; i < CAR_LINE_DELAY_BUFFER_SIZE; ++i)
  {
    s_line_delay_buffer[i] = 0;
  }
  s_line_delay_write_index = 0U;
  s_line_delay_count = 0U;
}

static int16_t ApplyLineCorrectionDelay(int16_t raw_correction)
{
  uint32_t delay_samples;
  int16_t delayed_correction;

  if (CAR_LINE_STEERING_DELAY_MS == 0U)
  {
    return raw_correction;
  }

  delay_samples = (CAR_LINE_STEERING_DELAY_MS + CAR_TASK_STATE_MS - 1U) /
                  CAR_TASK_STATE_MS;
  if (delay_samples > CAR_LINE_DELAY_BUFFER_SIZE)
  {
    delay_samples = CAR_LINE_DELAY_BUFFER_SIZE;
  }

  if (s_line_delay_count < delay_samples)
  {
    s_line_delay_buffer[s_line_delay_write_index] = raw_correction;
    s_line_delay_write_index = (uint8_t)((s_line_delay_write_index + 1U) %
                                         delay_samples);
    ++s_line_delay_count;
    return 0;
  }

  delayed_correction = s_line_delay_buffer[s_line_delay_write_index];
  s_line_delay_buffer[s_line_delay_write_index] = raw_correction;
  s_line_delay_write_index = (uint8_t)((s_line_delay_write_index + 1U) %
                                       delay_samples);
  return delayed_correction;
}

static int16_t NormalizeCd(int32_t angle_cd)
{
  while (angle_cd > 18000L) angle_cd -= 36000L;
  while (angle_cd < -18000L) angle_cd += 36000L;
  return (int16_t)angle_cd;
}

static void SetStage(CarStage_t stage)
{
  if (gCar.sys.stage != stage)
  {
    gCar.sys.stage = stage;
    gCar.sys.stage_start_tick = HAL_GetTick();
    s_target_ready_active = 0U;
    if (stage == CAR_STAGE_UNLOAD_ALIGN)
    {
      s_unload_sweep_phase = UNLOAD_SWEEP_FORWARD;
      s_unload_sweep_started = 0U;
      s_unload_sweep_first_left = 1U;
      s_unload_sweep_tick = HAL_GetTick();
    }
  }
}

static uint8_t StageElapsed(uint32_t ms)
{
  return ((HAL_GetTick() - gCar.sys.stage_start_tick) >= ms) ? 1U : 0U;
}

static uint8_t AnyUltrasonicValid(void)
{
  return (gCar.us.front_left_valid || gCar.us.front_right_valid ||
          gCar.us.left_valid || gCar.us.right_valid) ? 1U : 0U;
}

/* 0: waiting for first/recovered data, 1: usable, 2: timed out. */
static uint8_t UltrasonicTaskStatus(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t timeout_ms;

  if (AnyUltrasonicValid())
  {
    s_ultrasonic_seen_once = 1U;
    s_last_ultrasonic_valid_tick = now;
    return 1U;
  }

  timeout_ms = s_ultrasonic_seen_once ? US_TIMEOUT_MS : CAR_ULTRASONIC_STARTUP_MS;
  return ((now - s_last_ultrasonic_valid_tick) > timeout_ms) ? 2U : 0U;
}

static uint8_t FrontBlocked(void)
{
  uint8_t fl_block = (gCar.us.front_left_valid && (gCar.us.front_left_cm <= US_FRONT_TURN_CM)) ? 1U : 0U;
  uint8_t fr_block = (gCar.us.front_right_valid && (gCar.us.front_right_cm <= US_FRONT_TURN_CM)) ? 1U : 0U;
  return (fl_block || fr_block) ? 1U : 0U;
}

static uint8_t FrontEmergency(void)
{
  uint8_t fl_block = (gCar.us.front_left_valid && (gCar.us.front_left_cm <= US_EMERGENCY_CM)) ? 1U : 0U;
  uint8_t fr_block = (gCar.us.front_right_valid && (gCar.us.front_right_cm <= US_EMERGENCY_CM)) ? 1U : 0U;
  return (fl_block || fr_block) ? 1U : 0U;
}

/* During low-speed unloading alignment, one front sensor can see the carried
 * block or the unloading-area border. Stop only for a very close obstacle
 * confirmed by both front sensors, instead of latching on one reflection. */
static uint8_t FrontUnloadEmergency(void)
{
  uint8_t fl_block =
      (gCar.us.front_left_valid &&
       (gCar.us.front_left_cm <= US_UNLOAD_EMERGENCY_CM)) ? 1U : 0U;
  uint8_t fr_block =
      (gCar.us.front_right_valid &&
       (gCar.us.front_right_cm <= US_UNLOAD_EMERGENCY_CM)) ? 1U : 0U;
  return (fl_block && fr_block) ? 1U : 0U;
}

static CarAction_t MakeUnloadSweepAction(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t elapsed;

  if (s_unload_sweep_started == 0U)
  {
    s_unload_sweep_started = 1U;
    s_unload_sweep_phase = UNLOAD_SWEEP_FORWARD;
    s_unload_sweep_first_left = 1U;
    s_unload_sweep_tick = now;
  }

  elapsed = now - s_unload_sweep_tick;
  switch (s_unload_sweep_phase)
  {
    case UNLOAD_SWEEP_FORWARD:
      if (elapsed >= CAR_UNLOAD_INITIAL_FORWARD_MS)
      {
        s_unload_sweep_phase = UNLOAD_SWEEP_LEFT;
        s_unload_sweep_tick = now;
        return CAR_ACTION_TURN_LEFT;
      }
      return CAR_ACTION_SLOW_FORWARD;

    case UNLOAD_SWEEP_LEFT:
      if (elapsed >= (s_unload_sweep_first_left ?
                      CAR_UNLOAD_SWEEP_SIDE_MS :
                      CAR_UNLOAD_SWEEP_CROSS_MS))
      {
        s_unload_sweep_first_left = 0U;
        s_unload_sweep_phase = UNLOAD_SWEEP_RIGHT;
        s_unload_sweep_tick = now;
        return CAR_ACTION_TURN_RIGHT;
      }
      return CAR_ACTION_TURN_LEFT;

    case UNLOAD_SWEEP_RIGHT:
      if (elapsed >= CAR_UNLOAD_SWEEP_CROSS_MS)
      {
        s_unload_sweep_phase = UNLOAD_SWEEP_LEFT;
        s_unload_sweep_tick = now;
        return CAR_ACTION_TURN_LEFT;
      }
      return CAR_ACTION_TURN_RIGHT;

    default:
      s_unload_sweep_phase = UNLOAD_SWEEP_LEFT;
      s_unload_sweep_tick = now;
      return CAR_ACTION_TURN_LEFT;
  }
}

static uint8_t FrontClear(void)
{
  return (gCar.us.front_left_valid && gCar.us.front_right_valid &&
          (gCar.us.front_left_cm >= US_CLEAR_CM) &&
          (gCar.us.front_right_cm >= US_CLEAR_CM)) ? 1U : 0U;
}

static void ResetLineDerivative(void)
{
  s_last_line_error = 0;
  s_line_derivative_filtered = 0;
  s_last_line_error_tick = HAL_GetTick();
  s_line_derivative_ready = 0U;
  gCar.motion.line_d_correction = 0;
}

static void PrimeLineDerivative(int16_t error)
{
  s_last_line_error = error;
  s_line_derivative_filtered = 0;
  s_last_line_error_tick = HAL_GetTick();
  s_line_derivative_ready = 1U;
  gCar.motion.line_d_correction = 0;
}

static int16_t CalculateLineDerivative(int16_t error)
{
  uint32_t now = HAL_GetTick();
  uint32_t dt_ms;
  int32_t normalized_delta;
  int32_t filtered_delta;
  int32_t d_correction;

  if (s_line_derivative_ready == 0U)
  {
    PrimeLineDerivative(error);
    return 0;
  }

  dt_ms = now - s_last_line_error_tick;
  s_last_line_error_tick = now;

  if ((dt_ms == 0U) || (dt_ms > CAR_LINE_D_RESET_MS))
  {
    PrimeLineDerivative(error);
    return 0;
  }

  /* Normalize the error difference to the nominal state-update period.  This
   * keeps D gain stable when a blocking debug transmission delays one cycle. */
  normalized_delta = ((int32_t)error - s_last_line_error) *
                     CAR_TASK_STATE_MS / (int32_t)dt_ms;
  s_last_line_error = error;

  if (normalized_delta > CAR_LINE_D_DELTA_MAX)
    normalized_delta = CAR_LINE_D_DELTA_MAX;
  if (normalized_delta < -CAR_LINE_D_DELTA_MAX)
    normalized_delta = -CAR_LINE_D_DELTA_MAX;

  filtered_delta =
      ((int32_t)s_line_derivative_filtered *
       CAR_LINE_D_FILTER_OLD_WEIGHT +
       normalized_delta * CAR_LINE_D_FILTER_NEW_WEIGHT) /
      (CAR_LINE_D_FILTER_OLD_WEIGHT + CAR_LINE_D_FILTER_NEW_WEIGHT);
  s_line_derivative_filtered = (int16_t)filtered_delta;

  d_correction = filtered_delta * CAR_LINE_KD_NUM / CAR_LINE_KD_DEN;
  if (d_correction > CAR_LINE_D_TERM_MAX)
    d_correction = CAR_LINE_D_TERM_MAX;
  if (d_correction < -CAR_LINE_D_TERM_MAX)
    d_correction = -CAR_LINE_D_TERM_MAX;

  gCar.motion.line_d_correction = (int16_t)d_correction;
  return (int16_t)d_correction;
}

static void ResetLineTracking(void)
{
  s_line_seen_once = 0U;
  s_line_was_valid = 0U;
  s_line_lost_tick = HAL_GetTick();
  s_last_line_offset = 0;
  s_last_line_raw_correction = 0;
  s_last_line_correction = 0;
  gCar.motion.line_correction = 0;
  ResetLineDerivative();
  ResetLineCorrectionDelay();
}

static CarAction_t MakeLineAction(void)
{
  int32_t correction;
  uint32_t lost_ms;

  if (gCar.vision.line_valid == 0U)
  {
    if (s_line_seen_once == 0U)
    {
      gCar.motion.line_correction = 0;
      gCar.motion.line_d_correction = 0;
      return CAR_ACTION_STOP;
    }

    if (s_line_was_valid != 0U)
    {
      s_line_was_valid = 0U;
      s_line_lost_tick = HAL_GetTick();
      ResetLineDerivative();
    }

    lost_ms = HAL_GetTick() - s_line_lost_tick;
    if (lost_ms <= CAR_LINE_LOST_GRACE_MS)
    {
      gCar.motion.line_correction =
          ApplyLineCorrectionDelay(s_last_line_raw_correction);
      s_last_line_correction = gCar.motion.line_correction;
      return CAR_ACTION_LINE_FOLLOW;
    }

    if (lost_ms <= CAR_LINE_SEARCH_TIMEOUT_MS)
    {
      ResetLineCorrectionDelay();
      if (s_last_line_correction == 0)
      {
        s_last_line_correction = (s_last_line_offset >= 0) ? 1 : -1;
      }
      gCar.motion.line_correction = s_last_line_correction;
      return CAR_ACTION_SEARCH_LINE;
    }

    gCar.motion.line_correction = 0;
    gCar.motion.line_d_correction = 0;
    return CAR_ACTION_STOP;
  }

  if ((gCar.vision.line_offset <= CAR_LINE_OFFSET_DEADZONE) &&
      (gCar.vision.line_offset >= -CAR_LINE_OFFSET_DEADZONE) &&
      (gCar.vision.line_heading <= CAR_LINE_HEADING_DEADZONE) &&
      (gCar.vision.line_heading >= -CAR_LINE_HEADING_DEADZONE))
  {
    correction = 0;
    PrimeLineDerivative(gCar.vision.line_offset);
  }
  else
  {
    correction = ((int32_t)gCar.vision.line_offset * CAR_LINE_KP_NUM) /
                 CAR_LINE_KP_DEN;
    correction += ((int32_t)gCar.vision.line_heading *
                   CAR_LINE_HEADING_KP_NUM) /
                  CAR_LINE_HEADING_KP_DEN;
    correction += CalculateLineDerivative(gCar.vision.line_offset);
  }

  if (correction > CAR_LINE_CORRECTION_MAX) correction = CAR_LINE_CORRECTION_MAX;
  if (correction < -CAR_LINE_CORRECTION_MAX) correction = -CAR_LINE_CORRECTION_MAX;
  s_last_line_raw_correction = (int16_t)correction;
  correction = ApplyLineCorrectionDelay((int16_t)correction);
  gCar.motion.line_correction = (int16_t)correction;
  s_last_line_offset = gCar.vision.line_offset;
  s_last_line_correction = (int16_t)correction;
  s_line_seen_once = 1U;
  s_line_was_valid = 1U;

  return CAR_ACTION_LINE_FOLLOW;
}

static CarAction_t MakeHeadingAction(void)
{
  if ((gCar.imu.angle_valid == 0U) || (gCar.imu.target_yaw_valid == 0U))
  {
    return CAR_ACTION_SENSOR_ERROR;
  }

  gCar.imu.yaw_error_cd = NormalizeCd((int32_t)gCar.imu.target_yaw_cd - gCar.imu.yaw_rel_cd);

  if (gCar.imu.yaw_error_cd > CAR_YAW_DEADZONE_CD)
  {
    return CAR_ACTION_TURN_RIGHT;
  }

  if (gCar.imu.yaw_error_cd < -CAR_YAW_DEADZONE_CD)
  {
    return CAR_ACTION_TURN_LEFT;
  }

  return CAR_ACTION_FORWARD;
}

static void SaveTargetYawFromVision(void)
{
  int32_t target_cd = gCar.imu.yaw_rel_cd +
                      ((int32_t)gCar.camera.current_deg - CAMERA_PAN_INITIAL_DEG +
                       (int32_t)gCar.vision.tag_relative_deg) * 100L;
  gCar.imu.target_yaw_cd = NormalizeCd(target_cd);
  gCar.imu.yaw_error_cd = NormalizeCd((int32_t)gCar.imu.target_yaw_cd - gCar.imu.yaw_rel_cd);
  gCar.imu.target_yaw_valid = 1U;
}

static CarAction_t MakeTargetApproachAction(uint8_t load_target, uint8_t *ready)
{
  int16_t image_error_deg;
  int16_t bearing_deg;
  int16_t align_deadzone_deg;
  uint32_t area;
  uint32_t ready_area;

  *ready = 0U;
  if (load_target)
  {
    align_deadzone_deg = CAR_TARGET_ALIGN_DEADZONE_DEG;
    if (CameraServo_IsPickupViewReady() == 0U)
    {
      s_target_ready_active = 0U;
      return CAR_ACTION_STOP;
    }
    image_error_deg = gCar.vision.load_relative_deg;
    area = gCar.vision.load_area;
    ready_area = 0U;
  }
  else
  {
    align_deadzone_deg = CAR_UNLOAD_ALIGN_DEADZONE_DEG;
    /* After the black circle is first found from the line view, wait until
     * the camera has returned to the calibrated pickup/unload pose. */
    if (CameraServo_IsPickupViewReady() == 0U)
    {
      s_target_ready_active = 0U;
      return CAR_ACTION_STOP;
    }
    image_error_deg = gCar.vision.tag_relative_deg;
    area = gCar.vision.tag_area;
    ready_area = CAR_UNLOAD_READY_AREA;
  }

  bearing_deg = (int16_t)(gCar.camera.current_deg -
                          CAMERA_PAN_INITIAL_DEG + image_error_deg);
  if (bearing_deg > align_deadzone_deg)
  {
    s_target_ready_active = 0U;
    return CAR_ACTION_TURN_RIGHT;
  }
  if (bearing_deg < -align_deadzone_deg)
  {
    s_target_ready_active = 0U;
    return CAR_ACTION_TURN_LEFT;
  }
  if (load_target)
  {
    if (gCar.vision.load_grip_state == CAR_GRIP_STATE_APPROACH)
    {
      s_target_ready_active = 0U;
      return CAR_ACTION_SLOW_FORWARD;
    }
    if (gCar.vision.load_grip_state == CAR_GRIP_STATE_TOO_CLOSE)
    {
      /* Do not drive farther into the object after it has passed the visual
       * capture window. Reposition manually instead of risking a collision. */
      s_target_ready_active = 0U;
      return CAR_ACTION_ALIGN_TARGET;
    }
    if (gCar.vision.load_grip_state != CAR_GRIP_STATE_READY)
    {
      s_target_ready_active = 0U;
      return CAR_ACTION_ALIGN_TARGET;
    }
  }
  else if (gCar.vision.unload_position_valid != 0U)
  {
    if (gCar.vision.unload_position_state == CAR_UNLOAD_STATE_APPROACH)
    {
      s_target_ready_active = 0U;
      return CAR_ACTION_SLOW_FORWARD;
    }
    if (gCar.vision.unload_position_state == CAR_UNLOAD_STATE_TOO_CLOSE)
    {
      s_target_ready_active = 0U;
      return CAR_ACTION_BACKWARD;
    }
    if (gCar.vision.unload_position_state != CAR_UNLOAD_STATE_READY)
    {
      s_target_ready_active = 0U;
      return CAR_ACTION_ALIGN_TARGET;
    }
  }
  else if (area < ready_area)
  {
    s_target_ready_active = 0U;
    return CAR_ACTION_SLOW_FORWARD;
  }

  if (s_target_ready_active == 0U)
  {
    s_target_ready_active = 1U;
    s_target_ready_tick = HAL_GetTick();
  }
  else if ((HAL_GetTick() - s_target_ready_tick) >= CAR_TARGET_STABLE_MS)
  {
    *ready = 1U;
  }

  return CAR_ACTION_ALIGN_TARGET;
}

static CarAction_t MakeAvoidAction(void)
{
  uint8_t fl_valid = gCar.us.front_left_valid;
  uint8_t fr_valid = gCar.us.front_right_valid;
  uint8_t left_valid = gCar.us.left_valid;
  uint8_t right_valid = gCar.us.right_valid;
  uint8_t fl_block = ((!fl_valid) || (gCar.us.front_left_cm <= US_FRONT_TURN_CM)) ? 1U : 0U;
  uint8_t fr_block = ((!fr_valid) || (gCar.us.front_right_cm <= US_FRONT_TURN_CM)) ? 1U : 0U;

  if (AnyUltrasonicValid() == 0U)
  {
    return CAR_ACTION_SENSOR_ERROR;
  }

  if ((!fl_valid) && (!fr_valid))
  {
    return CAR_ACTION_SENSOR_ERROR;
  }

  if (FrontEmergency())
  {
    return CAR_ACTION_STOP;
  }

  if (fl_block && (!fr_block))
  {
    return CAR_ACTION_TURN_RIGHT;
  }

  if (fr_block && (!fl_block))
  {
    return CAR_ACTION_TURN_LEFT;
  }

  if (fl_block && fr_block)
  {
    if (left_valid && right_valid)
    {
      return (gCar.us.left_cm > gCar.us.right_cm) ? CAR_ACTION_TURN_LEFT : CAR_ACTION_TURN_RIGHT;
    }
    if (left_valid) return CAR_ACTION_TURN_LEFT;
    if (right_valid) return CAR_ACTION_TURN_RIGHT;
    return CAR_ACTION_STOP;
  }

  if (left_valid && (gCar.us.left_cm <= US_SIDE_LIMIT_CM))
  {
    return CAR_ACTION_TURN_RIGHT;
  }

  if (right_valid && (gCar.us.right_cm <= US_SIDE_LIMIT_CM))
  {
    return CAR_ACTION_TURN_LEFT;
  }

  return MakeHeadingAction();
}

static void UpdateAvoidTask(void)
{
  uint8_t ultrasonic_status = UltrasonicTaskStatus();

  if (ultrasonic_status != 1U)
  {
    gCar.sys.action = (ultrasonic_status == 0U) ? CAR_ACTION_STOP : CAR_ACTION_SENSOR_ERROR;
    if (ultrasonic_status == 2U) SetStage(CAR_STAGE_ERROR);
    return;
  }

  switch (gCar.sys.stage)
  {
    case CAR_STAGE_IDLE:
      SetStage(CAR_STAGE_LOAD_SEARCH);
      gCar.sys.action = CAR_ACTION_SCAN_TARGET;
      break;

    case CAR_STAGE_LOAD_SEARCH:
      if (gCar.vision.load_seen != 0U)
      {
        SetStage(CAR_STAGE_LOAD_ALIGN);
        gCar.sys.action = CAR_ACTION_ALIGN_TARGET;
      }
      else
      {
        gCar.sys.action = CAR_ACTION_SCAN_TARGET;
      }
      break;

    case CAR_STAGE_LOAD_ALIGN:
      if (gCar.vision.load_seen == 0U)
      {
        SetStage(CAR_STAGE_LOAD_SEARCH);
        gCar.sys.action = CAR_ACTION_SCAN_TARGET;
      }
      else
      {
        uint8_t ready;
        gCar.sys.action = MakeTargetApproachAction(1U, &ready);
        if (ready)
        {
          SetStage(CAR_STAGE_LOAD_CARGO);
          gCar.sys.action = CAR_ACTION_STOP;
        }
      }
      break;

    case CAR_STAGE_LOAD_CARGO:
      gCar.sys.action = CAR_ACTION_LOAD_CARGO;
      if (StageElapsed(CAR_CARGO_ACTION_MS))
      {
        SetStage(CAR_STAGE_TARGET_SEARCH);
      }
      break;

    case CAR_STAGE_TARGET_SEARCH:
      if (gCar.vision.unload_seen != 0U)
      {
        SaveTargetYawFromVision();
        SetStage(CAR_STAGE_TARGET_ANCHOR);
        gCar.sys.action = CAR_ACTION_ALIGN_TARGET;
      }
      else
      {
        gCar.sys.action = CAR_ACTION_SCAN_TARGET;
      }
      break;

    case CAR_STAGE_TARGET_ANCHOR:
      gCar.sys.action = CAR_ACTION_ALIGN_TARGET;
      if (gCar.imu.target_yaw_valid != 0U)
      {
        SetStage(CAR_STAGE_GOAL_NAVIGATE);
      }
      break;

    case CAR_STAGE_GOAL_NAVIGATE:
      if (gCar.vision.unload_seen != 0U)
      {
        SetStage(CAR_STAGE_UNLOAD_ALIGN);
        gCar.sys.action = CAR_ACTION_ALIGN_TARGET;
      }
      else if (FrontBlocked())
      {
        SetStage(CAR_STAGE_AVOID_OBSTACLE);
        gCar.sys.action = MakeAvoidAction();
      }
      else
      {
        gCar.sys.action = MakeAvoidAction();
      }
      break;

    case CAR_STAGE_AVOID_OBSTACLE:
      gCar.sys.action = MakeAvoidAction();
      if (FrontClear() && (gCar.sys.action != CAR_ACTION_SENSOR_ERROR))
      {
        SetStage(CAR_STAGE_RESTORE_HEADING);
      }
      break;

    case CAR_STAGE_RESTORE_HEADING:
      gCar.sys.action = MakeHeadingAction();
      if ((gCar.imu.target_yaw_valid == 0U) ||
          ((gCar.imu.yaw_error_cd <= CAR_YAW_DEADZONE_CD) && (gCar.imu.yaw_error_cd >= -CAR_YAW_DEADZONE_CD)))
      {
        SetStage(CAR_STAGE_GOAL_NAVIGATE);
      }
      break;

    case CAR_STAGE_UNLOAD_ALIGN:
      if (gCar.vision.unload_seen == 0U)
      {
        SetStage(CAR_STAGE_UNLOAD_SEARCH);
        gCar.sys.action = CAR_ACTION_SCAN_TARGET;
      }
      else
      {
        uint8_t ready;
        gCar.sys.action = MakeTargetApproachAction(0U, &ready);
        if (ready)
        {
          SetStage(CAR_STAGE_UNLOAD_CARGO);
          gCar.sys.action = CAR_ACTION_STOP;
        }
      }
      break;

    case CAR_STAGE_UNLOAD_SEARCH:
      if (gCar.vision.unload_seen != 0U)
      {
        SetStage(CAR_STAGE_UNLOAD_ALIGN);
        gCar.sys.action = CAR_ACTION_ALIGN_TARGET;
      }
      else
      {
        gCar.sys.action = CAR_ACTION_SCAN_TARGET;
      }
      break;

    case CAR_STAGE_UNLOAD_CARGO:
      gCar.sys.action = CAR_ACTION_UNLOAD_CARGO;
      if (StageElapsed(CAR_CARGO_ACTION_MS))
      {
        SetStage(CAR_STAGE_DONE);
      }
      break;

    case CAR_STAGE_DONE:
      gCar.sys.action = CAR_ACTION_MISSION_DONE;
      break;

    case CAR_STAGE_ERROR:
    default:
      gCar.sys.action = CAR_ACTION_SENSOR_ERROR;
      break;
  }
}

static void UpdateVisionTask(void)
{
  switch (gCar.sys.stage)
  {
    case CAR_STAGE_IDLE:
      Vision_SetProcessingMode(VISION_PROCESS_RED);
      SetStage(CAR_STAGE_LOAD_SEARCH);
      gCar.sys.action = CAR_ACTION_SCAN_TARGET;
      break;

    case CAR_STAGE_LOAD_SEARCH:
      if (gCar.vision.load_seen != 0U)
      {
        SetStage(CAR_STAGE_LOAD_ALIGN);
        gCar.sys.action = CAR_ACTION_ALIGN_TARGET;
      }
      else
      {
        gCar.sys.action = CAR_ACTION_SCAN_TARGET;
      }
      break;

    case CAR_STAGE_LOAD_ALIGN:
      if (gCar.vision.load_seen == 0U)
      {
        SetStage(CAR_STAGE_LOAD_SEARCH);
        gCar.sys.action = CAR_ACTION_SCAN_TARGET;
      }
      else
      {
        uint8_t ready;
        gCar.sys.action = MakeTargetApproachAction(1U, &ready);
        if (ready)
        {
          SetStage(CAR_STAGE_LOAD_CARGO);
          gCar.sys.action = CAR_ACTION_STOP;
        }
      }
      break;

    case CAR_STAGE_LOAD_CARGO:
      gCar.sys.action = CAR_ACTION_LOAD_CARGO;
      /* Enter line mode only after the real claw sequence has completed and
       * the gripper state confirms that an object is held. */
      if ((GrabServo_IsBusy() == 0U) && (GrabServo_HasObject() != 0U))
      {
        SetStage(CAR_STAGE_LEAVE_LOAD_ZONE);
        gCar.sys.action = CAR_ACTION_STOP;
      }
      break;

    case CAR_STAGE_LEAVE_LOAD_ZONE:
      /* After pickup, drive straight at base speed (83/70) for two seconds. */
      gCar.sys.action = CAR_ACTION_FORWARD;
      if (StageElapsed(CAR_LOAD_ZONE_EXIT_MS))
      {
        gCar.sys.action = CAR_ACTION_STOP;
        Vision_SetProcessingMode(VISION_PROCESS_LINE);
        SetStage(CAR_STAGE_LINE_ENTRY);
      }
      break;

    case CAR_STAGE_LINE_ENTRY:
      if (CameraServo_IsLineViewReady() == 0U)
      {
        gCar.sys.action = CAR_ACTION_STOP;
      }
      else if (gCar.vision.line_valid != 0U)
      {
        SetStage(CAR_STAGE_LINE_NAVIGATE);
        gCar.sys.action = CAR_ACTION_STOP;
      }
      else if (StageElapsed(CAR_LINE_ENTRY_TIMEOUT_MS))
      {
        gCar.sys.action = CAR_ACTION_STOP;
      }
      else
      {
        gCar.sys.action = CAR_ACTION_SLOW_FORWARD;
      }
      break;

    case CAR_STAGE_LINE_NAVIGATE:
      if (CameraServo_IsLineViewReady() == 0U)
      {
        gCar.sys.action = CAR_ACTION_STOP;
      }
      else if (FrontEmergency())
      {
        gCar.sys.action = CAR_ACTION_STOP;
      }
      else if ((gCar.vision.unload_zone_seen != 0U) ||
               (gCar.vision.unload_seen != 0U))
      {
        /* Leave the inverted line image. The unloading view uses normal RGB
         * so OpenMV can distinguish the carried red block from the black
         * circular drop point and align their centres. */
        Vision_SetProcessingMode(VISION_PROCESS_UNLOAD);
        SetStage(CAR_STAGE_UNLOAD_ALIGN);
        gCar.sys.action = CAR_ACTION_STOP;
      }
      else
      {
        gCar.sys.action = MakeLineAction();
      }
      break;

    case CAR_STAGE_UNLOAD_ALIGN:
      /* Hold the same 10/30 pose used for pickup. OpenMV is in normal RGB
       * unload mode and reports the red-block/black-circle centre error. */
      Vision_SetProcessingMode(VISION_PROCESS_UNLOAD);
      if (CameraServo_IsPickupViewReady() == 0U)
      {
        s_target_ready_active = 0U;
        gCar.sys.action = CAR_ACTION_STOP;
      }
      else if (FrontUnloadEmergency())
      {
        s_target_ready_active = 0U;
        gCar.sys.action = CAR_ACTION_STOP;
      }
      else if ((gCar.vision.unload_seen != 0U) &&
               (gCar.vision.unload_position_valid != 0U))
      {
        uint8_t ready;
        gCar.sys.action = MakeTargetApproachAction(0U, &ready);
        if (ready != 0U)
        {
          SetStage(CAR_STAGE_UNLOAD_CARGO);
          gCar.sys.action = CAR_ACTION_STOP;
        }
      }
      else
      {
        /* From the fixed 10/30 view, drive straight for one second, then
         * sweep the car left and right using the normal in-place-turn code. */
        s_target_ready_active = 0U;
        if (StageElapsed(CAR_UNLOAD_VIEW_SEARCH_TIMEOUT_MS))
        {
          gCar.sys.action = CAR_ACTION_STOP;
        }
        else
        {
          gCar.sys.action = MakeUnloadSweepAction();
        }
      }
      break;

    case CAR_STAGE_UNLOAD_SEARCH:
      Vision_SetProcessingMode(VISION_PROCESS_UNLOAD);
      if (CameraServo_IsPickupViewReady() == 0U)
      {
        s_target_ready_active = 0U;
        gCar.sys.action = CAR_ACTION_STOP;
      }
      else if (FrontEmergency())
      {
        s_target_ready_active = 0U;
        gCar.sys.action = CAR_ACTION_STOP;
      }
      else if ((gCar.vision.unload_seen != 0U) &&
               (gCar.vision.unload_position_valid != 0U) &&
               (gCar.vision.unload_position_state == CAR_UNLOAD_STATE_READY))
      {
        /* Stop only at the actual circular drop point, not at zone entry. */
        gCar.sys.action = CAR_ACTION_STOP;
        if (s_target_ready_active == 0U)
        {
          s_target_ready_active = 1U;
          s_target_ready_tick = HAL_GetTick();
        }
        else if ((HAL_GetTick() - s_target_ready_tick) >=
                 CAR_TARGET_STABLE_MS)
        {
          SetStage(CAR_STAGE_UNLOAD_ALIGN);
        }
      }
      else if ((gCar.vision.unload_seen != 0U) &&
               (gCar.vision.unload_position_valid != 0U) &&
               (gCar.vision.unload_position_state ==
                CAR_UNLOAD_STATE_TOO_CLOSE))
      {
        s_target_ready_active = 0U;
        gCar.sys.action = CAR_ACTION_STOP;
      }
      else
      {
        s_target_ready_active = 0U;
        gCar.sys.action = CAR_ACTION_SLOW_FORWARD;
      }
      break;

    case CAR_STAGE_UNLOAD_CARGO:
      gCar.sys.action = CAR_ACTION_UNLOAD_CARGO;
      if ((GrabServo_IsBusy() == 0U) && (GrabServo_HasObject() == 0U))
      {
        SetStage(CAR_STAGE_DONE);
      }
      break;

    case CAR_STAGE_DONE:
      gCar.sys.action = CAR_ACTION_MISSION_DONE;
      break;

    case CAR_STAGE_ERROR:
    default:
      gCar.sys.action = CAR_ACTION_SENSOR_ERROR;
      break;
  }
}

static void UpdateLineTest(void)
{
  if (gCar.sys.stage != CAR_STAGE_LINE_NAVIGATE)
  {
    SetStage(CAR_STAGE_LINE_NAVIGATE);
  }

  gCar.sys.action = (CameraServo_IsLineViewReady() != 0U) ?
                    MakeLineAction() : CAR_ACTION_STOP;
}

void AppState_Init(void)
{
  s_last_mode = gCar.sys.mode;
  gCar.sys.state = CAR_STATE_IDLE;
  gCar.sys.stage = CAR_STAGE_IDLE;
  gCar.sys.stage_start_tick = HAL_GetTick();
  gCar.sys.action = CAR_ACTION_STOP;
  s_target_ready_active = 0U;
  s_target_ready_tick = HAL_GetTick();
  s_ultrasonic_seen_once = 0U;
  s_last_ultrasonic_valid_tick = HAL_GetTick();
  ResetLineTracking();
}

void AppState_Update(void)
{
  if ((gCar.sys.start_enable == 0U) || (gCar.sys.mode == CAR_MODE_STOP))
  {
    gCar.sys.state = CAR_STATE_IDLE;
    SetStage(CAR_STAGE_IDLE);
    gCar.sys.action = CAR_ACTION_STOP;
    ResetLineTracking();
    return;
  }

  gCar.sys.state = CAR_STATE_RUN;

  if (s_last_mode != gCar.sys.mode)
  {
    s_last_mode = gCar.sys.mode;
    SetStage(CAR_STAGE_IDLE);
    gCar.sys.action = CAR_ACTION_STOP;
    s_ultrasonic_seen_once = 0U;
    s_last_ultrasonic_valid_tick = HAL_GetTick();
    ResetLineTracking();
    return;
  }

  switch (gCar.sys.mode)
  {
    case CAR_MODE_AVOID:
      UpdateAvoidTask();
      break;

    case CAR_MODE_VISION_LINE:
      UpdateVisionTask();
      break;

    case CAR_MODE_LINE_TEST:
      UpdateLineTest();
      break;

    case CAR_MODE_REMOTE:
      gCar.sys.action = gCar.sys.remote_action;
      break;

    default:
      gCar.sys.action = CAR_ACTION_STOP;
      break;
  }
}


