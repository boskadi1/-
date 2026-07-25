#include "app_state.h"
#include "main.h"
#include "car_config.h"
#include "car_data.h"
#include "motor.h"

typedef enum
{
  VISION_NAV_ZERO_FORWARD = 1,
  VISION_NAV_PRE_TURN_STOP,
  VISION_NAV_TURN_AWAY,
  VISION_NAV_BYPASS_FORWARD,
  VISION_NAV_CHECK_BRAKE,
  VISION_NAV_TURN_CHECK_ZERO,
  VISION_NAV_CHECK_ZERO,
  VISION_NAV_RETURN_BYPASS,
  VISION_NAV_ERROR
} VisionNavState_t;

static VisionNavState_t s_nav_state;
static int8_t s_turn_sign;
static int16_t s_bypass_heading_cd;
static uint32_t s_state_tick;
static uint32_t s_collision_window_tick;
static uint32_t s_collision_window_path_mm;
static int32_t s_collision_window_imu_mm;
static uint32_t s_next_check_tick;
static uint32_t s_check_interval_ms;
static int32_t s_check_start_center_mm;
static uint8_t s_route_blocked;
static uint8_t s_origin_blocked;
static uint8_t s_zero_clear_override;
static uint32_t s_pivot_settle_tick;
static int16_t s_pivot_target_cd;
static uint32_t s_pivot_motion_tick;
static int16_t s_pivot_motion_yaw_cd;
static int8_t s_locked_pivot_sign;
static int8_t s_last_pivot_command_sign;
static uint32_t s_pivot_collision_brake_tick;
static uint32_t s_turn_visual_clear_tick;

static uint8_t s_yaw_source_initialized;
static uint8_t s_yaw_using_imu;
static int16_t s_yaw_source_offset_cd;
static int16_t s_continuous_yaw_cd;

static float s_heading_pid_i;
static float s_heading_pid_prev_error;
static float s_heading_pid_d_filtered;
static uint32_t s_heading_pid_tick;
static uint8_t s_heading_pid_initialized;

static int16_t NormalizeCd(int32_t angle_cd)
{
  while (angle_cd > 18000L) angle_cd -= 36000L;
  while (angle_cd < -18000L) angle_cd += 36000L;
  return (int16_t)angle_cd;
}

static int16_t ClampScanHeadingCd(int32_t angle_cd)
{
  if (angle_cd > VISION_V2_SCAN_LIMIT_CD)
    return VISION_V2_SCAN_LIMIT_CD;
  if (angle_cd < -VISION_V2_SCAN_LIMIT_CD)
    return -VISION_V2_SCAN_LIMIT_CD;
  return (int16_t)angle_cd;
}

static int16_t AbsInt16(int16_t value)
{
  return (value < 0) ? (int16_t)(-value) : value;
}

static int32_t AbsInt32(int32_t value)
{
  return (value < 0L) ? -value : value;
}

static float ClampFloat(float value, float low, float high)
{
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

static int16_t FloatToInt16(float value)
{
  value = ClampFloat(value, -32768.0f, 32767.0f);
  return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static int16_t ClampMotorSpeed(float value)
{
  value = ClampFloat(value, -(float)MOTOR_SPEED_MAX,
                     (float)MOTOR_SPEED_MAX);
  return FloatToInt16(value);
}

/*
 * Use the JY61P power-on-relative yaw when it is available and retain the
 * current heading continuously if the source temporarily falls back to the
 * wheel encoders. Internal positive yaw is a clockwise/right turn.
 */
static int16_t CurrentYawCd(void)
{
  const uint8_t use_imu = ((gCar.imu.angle_valid != 0U) &&
                           (gCar.imu.yaw_anchored != 0U)) ? 1U : 0U;
  const int16_t raw_yaw_cd = use_imu ? NormalizeCd(gCar.imu.yaw_rel_cd) :
                                      NormalizeCd(gCar.encoder.yaw_cd);

  if (s_yaw_source_initialized == 0U)
  {
    s_yaw_source_initialized = 1U;
    s_yaw_using_imu = use_imu;
    s_yaw_source_offset_cd = 0;
    s_continuous_yaw_cd = raw_yaw_cd;
  }
  else
  {
    if (use_imu != s_yaw_using_imu)
    {
      s_yaw_source_offset_cd = NormalizeCd(
          (int32_t)s_continuous_yaw_cd - raw_yaw_cd);
      s_yaw_using_imu = use_imu;
    }
    s_continuous_yaw_cd = NormalizeCd(
        (int32_t)raw_yaw_cd + s_yaw_source_offset_cd);
  }
  return s_continuous_yaw_cd;
}

static void ResetHeadingPid(void)
{
  s_heading_pid_i = 0.0f;
  s_heading_pid_prev_error = 0.0f;
  s_heading_pid_d_filtered = 0.0f;
  s_heading_pid_tick = HAL_GetTick();
  s_heading_pid_initialized = 0U;
}

static float HeadingPidUpdate(float error_deg)
{
  const uint32_t now = HAL_GetTick();
  uint32_t elapsed_ms = now - s_heading_pid_tick;
  float dt_s;
  float derivative;
  float candidate_i;
  float raw_output;
  float output;

  if (elapsed_ms == 0U) elapsed_ms = CAR_TASK_STATE_MS;
  if (elapsed_ms > 200U) elapsed_ms = 200U;
  dt_s = (float)elapsed_ms / 1000.0f;
  s_heading_pid_tick = now;

  if (error_deg == 0.0f)
  {
    s_heading_pid_i *= 0.85f;
    s_heading_pid_prev_error = 0.0f;
    s_heading_pid_d_filtered = 0.0f;
    s_heading_pid_initialized = 1U;
    return 0.0f;
  }

  if (s_heading_pid_initialized == 0U)
  {
    derivative = 0.0f;
    s_heading_pid_initialized = 1U;
  }
  else
  {
    derivative = (error_deg - s_heading_pid_prev_error) / dt_s;
  }

  s_heading_pid_prev_error = error_deg;
  s_heading_pid_d_filtered += VISION_V2_HEADING_D_FILTER_ALPHA *
                              (derivative - s_heading_pid_d_filtered);
  candidate_i = ClampFloat(s_heading_pid_i + error_deg * dt_s,
                           -VISION_V2_HEADING_PID_I_LIMIT,
                           VISION_V2_HEADING_PID_I_LIMIT);
  raw_output = VISION_V2_HEADING_PID_KP * error_deg +
               VISION_V2_HEADING_PID_KI * candidate_i +
               VISION_V2_HEADING_PID_KD * s_heading_pid_d_filtered;
  output = ClampFloat(raw_output, -VISION_V2_MAX_TURN_SPEED,
                      VISION_V2_MAX_TURN_SPEED);
  if (output == raw_output) s_heading_pid_i = candidate_i;
  return output;
}

static uint8_t VisionLinkAlive(void)
{
  return (gCar.vision.nav_frame_valid != 0U) ? 1U : 0U;
}

static uint8_t BlueRawVisible(void)
{
  return ((VisionLinkAlive() != 0U) &&
          (gCar.vision.obstacle_valid != 0U)) ? 1U : 0U;
}

static uint8_t BlueObstacleAhead(void)
{
  return ((BlueRawVisible() != 0U) &&
          (gCar.vision.obstacle_area_pixels >=
           VISION_V2_BLUE_AREA_TRIGGER_PIXELS)) ? 1U : 0U;
}

static int16_t InferCollisionBearing(void)
{
  if ((BlueRawVisible() != 0U) &&
      (AbsInt16(gCar.vision.obstacle_bearing_deg) >= 3))
    return gCar.vision.obstacle_bearing_deg;

  if ((gCar.us.front_left_valid != 0U) &&
      (gCar.us.front_right_valid != 0U))
  {
    if (gCar.us.front_left_cm + 3U < gCar.us.front_right_cm)
      return US_FRONT_LEFT_BEARING_DEG;
    if (gCar.us.front_right_cm + 3U < gCar.us.front_left_cm)
      return US_FRONT_RIGHT_BEARING_DEG;
  }
  else if ((gCar.us.front_left_valid != 0U) &&
           (gCar.us.front_left_cm <= VISION_V2_COLLISION_SONAR_CM))
  {
    return US_FRONT_LEFT_BEARING_DEG;
  }
  else if ((gCar.us.front_right_valid != 0U) &&
           (gCar.us.front_right_cm <= VISION_V2_COLLISION_SONAR_CM))
  {
    return US_FRONT_RIGHT_BEARING_DEG;
  }

  if ((gCar.us.left_valid != 0U) &&
      (gCar.us.left_cm <= VISION_V2_COLLISION_SONAR_CM))
    return US_LEFT_BEARING_DEG;
  if ((gCar.us.right_valid != 0U) &&
      (gCar.us.right_cm <= VISION_V2_COLLISION_SONAR_CM))
    return US_RIGHT_BEARING_DEG;
  return 0;
}

static uint8_t FrontSonarCollisionEvidence(void)
{
  return (((gCar.us.front_left_valid != 0U) &&
           (gCar.us.front_left_cm <= VISION_V2_COLLISION_SONAR_CM)) ||
          ((gCar.us.front_right_valid != 0U) &&
           (gCar.us.front_right_cm <= VISION_V2_COLLISION_SONAR_CM))) ?
         1U : 0U;
}

/*
 * Collision detection uses a 500 ms translating window. A collision is
 * accepted only when the IMU short-window displacement/impact says the body
 * failed to move and camera/sonar evidence agrees. Encoder distance separates
 * a true stall from wheel spin but never replaces the IMU body-motion cue.
 */
static void UpdateCollisionAssessment(uint32_t now)
{
  uint8_t forward_command;

  if ((gCar.imu.collision_valid != 0U) &&
      ((now - gCar.imu.collision_tick) >=
       VISION_V2_COLLISION_HOLD_MS))
  {
    gCar.imu.collision_valid = 0U;
    gCar.imu.collision_bearing_deg = 0;
  }

  forward_command =
      ((gCar.motion.left_target_mm_s > 0) &&
       (gCar.motion.right_target_mm_s > 0)) ? 1U : 0U;
  if (forward_command == 0U)
  {
    s_collision_window_tick = 0U;
    return;
  }

  if (s_collision_window_tick == 0U)
  {
    s_collision_window_tick = now;
    s_collision_window_path_mm = gCar.encoder.path_distance_mm;
    s_collision_window_imu_mm = gCar.imu.displacement_forward_mm;
    return;
  }

  if ((now - s_collision_window_tick) >=
      VISION_V2_COLLISION_WINDOW_MS)
  {
    const uint32_t encoder_delta_mm =
        gCar.encoder.path_distance_mm - s_collision_window_path_mm;
    const int32_t imu_delta_mm = AbsInt32(
        gCar.imu.displacement_forward_mm - s_collision_window_imu_mm);
    const uint8_t imu_ready =
        ((gCar.imu.accel_valid != 0U) &&
         (gCar.imu.accel_calibrated != 0U)) ? 1U : 0U;
    const uint8_t impact =
        ((imu_ready != 0U) &&
         (gCar.imu.accel_forward_mg <=
          -VISION_V2_COLLISION_IMPACT_MG)) ? 1U : 0U;
    const uint8_t no_body_progress =
        ((imu_ready != 0U) &&
         (imu_delta_mm <= VISION_V2_COLLISION_IMU_MAX_MM)) ? 1U : 0U;
    const uint8_t encoder_stall =
        (encoder_delta_mm <= VISION_V2_COLLISION_ENCODER_MAX_MM) ? 1U : 0U;
    const uint8_t wheel_spin_mismatch =
        ((no_body_progress != 0U) &&
         (encoder_delta_mm >
          (2U * VISION_V2_COLLISION_ENCODER_MAX_MM))) ? 1U : 0U;
    const uint8_t sensor_support =
        ((BlueObstacleAhead() != 0U) ||
         ((BlueRawVisible() != 0U) &&
          (FrontSonarCollisionEvidence() != 0U))) ? 1U : 0U;

    if ((sensor_support != 0U) &&
        ((impact != 0U) ||
         ((no_body_progress != 0U) &&
          ((encoder_stall != 0U) ||
           (wheel_spin_mismatch != 0U)))))
    {
      gCar.imu.collision_valid = 1U;
      gCar.imu.collision_bearing_deg = InferCollisionBearing();
      gCar.imu.collision_count++;
      gCar.imu.collision_tick = now;
    }

    s_collision_window_tick = now;
    s_collision_window_path_mm = gCar.encoder.path_distance_mm;
    s_collision_window_imu_mm = gCar.imu.displacement_forward_mm;
  }
}

static int8_t ChooseTurnSign(void)
{
  /*
   * Follow the requested rule literally: turn toward the side ultrasonic
   * sensor reporting the smaller distance. Internal positive yaw is right.
   */
  if ((gCar.us.left_valid != 0U) && (gCar.us.right_valid != 0U))
  {
    if (gCar.us.left_cm < gCar.us.right_cm) return -1;
    if (gCar.us.right_cm < gCar.us.left_cm) return 1;
  }
  else if (gCar.us.left_valid != 0U)
  {
    return -1;
  }
  else if (gCar.us.right_valid != 0U)
  {
    return 1;
  }

  if (gCar.vision.obstacle_free_bias < 0) return -1;
  if (gCar.vision.obstacle_free_bias > 0) return 1;
  return VISION_V2_DEFAULT_TURN_SIGN;
}

static void SetMotionTelemetry(int16_t desired_cd, int16_t error_cd)
{
  gCar.motion.nav_heading_deg = error_cd / 100;
  gCar.motion.desired_world_heading_cd = desired_cd;
  gCar.imu.target_yaw_cd = desired_cd;
  gCar.imu.target_yaw_valid = 1U;
  gCar.imu.yaw_error_cd = error_cd;
  gCar.motion.escape_mode = 0U;
}

static void StopWithAction(CarAction_t action)
{
  gCar.sys.action = action;
  gCar.motion.nav_heading_deg = 0;
  gCar.motion.desired_world_heading_cd = CurrentYawCd();
  gCar.imu.target_yaw_valid = 0U;
  gCar.imu.yaw_error_cd = 0;
  gCar.motion.escape_mode = 0U;
  ResetHeadingPid();
  Motor_Stop();
}

static void BrakeWithAction(CarAction_t action)
{
  gCar.sys.action = action;
  gCar.motion.nav_heading_deg = 0;
  gCar.motion.desired_world_heading_cd = CurrentYawCd();
  gCar.imu.target_yaw_valid = 0U;
  gCar.imu.yaw_error_cd = 0;
  gCar.motion.escape_mode = 0U;
  ResetHeadingPid();
  Motor_Brake();
}

static uint8_t FrontSonarNear(void)
{
  return (((gCar.us.front_left_valid != 0U) &&
           (gCar.us.front_left_cm <= VISION_V2_FRONT_BRAKE_CM)) ||
          ((gCar.us.front_right_valid != 0U) &&
           (gCar.us.front_right_cm <= VISION_V2_FRONT_BRAKE_CM))) ? 1U : 0U;
}

static uint8_t FrontSonarsFar(void)
{
  return ((gCar.us.front_left_valid != 0U) &&
          (gCar.us.front_right_valid != 0U) &&
          (gCar.us.front_left_cm >= VISION_V2_FRONT_CLEAR_CM) &&
          (gCar.us.front_right_cm >= VISION_V2_FRONT_CLEAR_CM)) ? 1U : 0U;
}

static uint8_t BlueTopBlocked(void)
{
  return ((BlueRawVisible() != 0U) &&
          (gCar.vision.obstacle_top_full != 0U)) ? 1U : 0U;
}

static uint8_t InitialObstacleAhead(void)
{
  return ((BlueObstacleAhead() != 0U) ||
          (BlueTopBlocked() != 0U) ||
          (FrontSonarNear() != 0U)) ? 1U : 0U;
}

static uint8_t OriginDirectionClear(void)
{
  if (FrontSonarNear() != 0U) return 0U;
  if (BlueTopBlocked() != 0U) return 0U;

  /* "No blue OR sonar far" is sufficient after a periodic world-zero check. */
  return ((BlueObstacleAhead() == 0U) ||
          (FrontSonarsFar() != 0U)) ? 1U : 0U;
}

static void DriveAtWorldHeading(int16_t desired_cd)
{
  const int16_t yaw_cd = CurrentYawCd();
  const int16_t error_cd = NormalizeCd((int32_t)desired_cd - yaw_cd);
  float error_deg = (float)error_cd / 100.0f;
  float correction;
  int16_t left_speed = (int16_t)MOTOR_LEFT_BASE_SPEED;
  int16_t right_speed = (int16_t)MOTOR_RIGHT_BASE_SPEED;

  if ((error_deg <= VISION_V2_HEADING_DEADBAND_DEG) &&
      (error_deg >= -VISION_V2_HEADING_DEADBAND_DEG))
    error_deg = 0.0f;

  correction = HeadingPidUpdate(error_deg);
  if (correction > 0.0f)
    left_speed = ClampMotorSpeed((float)left_speed + correction);
  else if (correction < 0.0f)
    right_speed = ClampMotorSpeed((float)right_speed - correction);

  SetMotionTelemetry(desired_cd, error_cd);
  gCar.sys.action = CAR_ACTION_VISION_DRIVE;
  Motor_SetTarget(left_speed, right_speed);
}

static void DriveAtWorldHeadingWithVisionBias(int16_t base_heading_cd)
{
  int16_t desired_cd = base_heading_cd;

  if ((BlueRawVisible() != 0U) &&
      (gCar.vision.obstacle_top_full == 0U))
  {
    int8_t free_bias = gCar.vision.obstacle_free_bias;

    /*
     * Use the obstacle position directly when it is clearly off-centre:
     * blue on the left (negative bearing) steers right, while blue on the
     * right (positive bearing) steers left. The OpenMV free-side hint remains
     * the fallback while the blue block is near the image centre.
     */
    if (gCar.vision.obstacle_bearing_deg <=
        -VISION_V2_BLUE_BEARING_DEADBAND_DEG)
      free_bias = 1;
    else if (gCar.vision.obstacle_bearing_deg >=
             VISION_V2_BLUE_BEARING_DEADBAND_DEG)
      free_bias = -1;

    desired_cd = ClampScanHeadingCd(
        (int32_t)base_heading_cd +
        (int32_t)free_bias *
        VISION_V2_VISUAL_BIAS_DEG * 100);
  }

  DriveAtWorldHeading(desired_cd);
}

static void PivotWithSign(int8_t turn_sign, int16_t target_cd)
{
  const int16_t yaw_cd = CurrentYawCd();

  ResetHeadingPid();
  SetMotionTelemetry(target_cd,
                     NormalizeCd((int32_t)target_cd - yaw_cd));
  gCar.sys.action = (turn_sign > 0) ?
                    CAR_ACTION_TURN_RIGHT : CAR_ACTION_TURN_LEFT;
}

static uint8_t PivotToHeading(int16_t target_cd)
{
  const uint32_t now = HAL_GetTick();
  const int16_t yaw_cd = CurrentYawCd();
  int16_t error_cd;
  int8_t turn_sign;

  target_cd = ClampScanHeadingCd(target_cd);
  error_cd = NormalizeCd((int32_t)target_cd - yaw_cd);

  if (target_cd != s_pivot_target_cd)
  {
    s_pivot_target_cd = target_cd;
    s_pivot_settle_tick = 0U;
    s_pivot_motion_tick = now;
    s_pivot_motion_yaw_cd = yaw_cd;
    s_locked_pivot_sign = (error_cd >= 0) ? 1 : -1;
    s_last_pivot_command_sign = 0;
    s_pivot_collision_brake_tick = 0U;
  }

  if ((AbsInt16(error_cd) <= VISION_V2_TURN_TOLERANCE_CD) ||
      ((s_locked_pivot_sign > 0) && (error_cd < 0)) ||
      ((s_locked_pivot_sign < 0) && (error_cd > 0)))
  {
    SetMotionTelemetry(target_cd, error_cd);
    gCar.sys.action = CAR_ACTION_STOP;
    Motor_Brake();
    s_pivot_motion_tick = 0U;
    s_last_pivot_command_sign = 0;

    if (s_pivot_settle_tick == 0U)
    {
      s_pivot_settle_tick = now;
      return 0U;
    }

    return ((now - s_pivot_settle_tick) >=
            VISION_V2_TURN_SETTLE_MS) ? 1U : 0U;
  }

  s_pivot_settle_tick = 0U;
  turn_sign = s_locked_pivot_sign;

  if (s_pivot_collision_brake_tick != 0U)
  {
    SetMotionTelemetry(target_cd, error_cd);
    gCar.sys.action = CAR_ACTION_STOP;
    Motor_Brake();

    if ((now - s_pivot_collision_brake_tick) <
        VISION_V2_TURN_COLLISION_BRAKE_MS)
      return 0U;

    s_pivot_collision_brake_tick = 0U;
    s_pivot_motion_tick = now;
    s_pivot_motion_yaw_cd = yaw_cd;
    s_last_pivot_command_sign = turn_sign;
  }

  if ((s_pivot_motion_tick == 0U) ||
      (turn_sign != s_last_pivot_command_sign))
  {
    s_pivot_motion_tick = now;
    s_pivot_motion_yaw_cd = yaw_cd;
    s_last_pivot_command_sign = turn_sign;
  }
  else if ((now - s_pivot_motion_tick) >= VISION_V2_TURN_STALL_MS)
  {
    const int16_t yaw_change_cd = AbsInt16(
        NormalizeCd((int32_t)yaw_cd - s_pivot_motion_yaw_cd));

    if (yaw_change_cd < VISION_V2_TURN_STALL_MIN_CHANGE_CD)
    {
      /* Two seconds with less than 5 degrees of rotation means the chassis
         is physically blocked. Keep the calibrated turn side: brake briefly,
         then retry the same locked direction instead of reversing. */
      gCar.imu.collision_valid = 1U;
      gCar.imu.collision_bearing_deg =
          (turn_sign > 0) ? US_RIGHT_BEARING_DEG : US_LEFT_BEARING_DEG;
      gCar.imu.collision_count++;
      gCar.imu.collision_tick = now;
      s_last_pivot_command_sign = 0;
      s_pivot_motion_tick = 0U;
      s_pivot_collision_brake_tick = now;
      SetMotionTelemetry(target_cd, error_cd);
      gCar.sys.action = CAR_ACTION_STOP;
      Motor_Brake();
      return 0U;
    }

    s_pivot_motion_tick = now;
    s_pivot_motion_yaw_cd = yaw_cd;
  }

  PivotWithSign(turn_sign, target_cd);
  return 0U;
}

static void EnterPreTurnStop(void)
{
  s_turn_sign = ChooseTurnSign();
  s_bypass_heading_cd =
      (int16_t)(s_turn_sign * VISION_V2_BYPASS_HEADING_DEG * 100);
  s_nav_state = VISION_NAV_PRE_TURN_STOP;
  s_state_tick = HAL_GetTick();
  s_route_blocked = 0U;
  s_origin_blocked = 1U;
  s_zero_clear_override = 0U;
  s_pivot_settle_tick = 0U;
  s_pivot_target_cd = 0;
  s_pivot_motion_tick = 0U;
  s_pivot_motion_yaw_cd = 0;
  s_locked_pivot_sign = 0;
  s_last_pivot_command_sign = 0;
  s_pivot_collision_brake_tick = 0U;
  s_turn_visual_clear_tick = 0U;
  BrakeWithAction(CAR_ACTION_STOP);
}

static void EnterCheckBrake(uint8_t route_blocked)
{
  s_nav_state = VISION_NAV_CHECK_BRAKE;
  s_state_tick = HAL_GetTick();
  s_route_blocked = route_blocked;
  BrakeWithAction(CAR_ACTION_STOP);
}

void AppState_Init(void)
{
  s_nav_state = VISION_NAV_ZERO_FORWARD;
  s_turn_sign = VISION_V2_DEFAULT_TURN_SIGN;
  s_bypass_heading_cd = 0;
  s_state_tick = HAL_GetTick();
  s_collision_window_tick = 0U;
  s_collision_window_path_mm = gCar.encoder.path_distance_mm;
  s_collision_window_imu_mm = gCar.imu.displacement_forward_mm;
  s_next_check_tick = HAL_GetTick() + VISION_V2_PERIODIC_CHECK_MS;
  s_check_interval_ms = VISION_V2_PERIODIC_CHECK_MS;
  s_check_start_center_mm = gCar.encoder.center_distance_mm;
  s_turn_visual_clear_tick = 0U;
  s_route_blocked = 0U;
  s_origin_blocked = 0U;
  s_zero_clear_override = 0U;
  s_yaw_source_initialized = 0U;
  s_yaw_using_imu = 0U;
  s_yaw_source_offset_cd = 0;
  s_continuous_yaw_cd = 0;
  ResetHeadingPid();

  gCar.sys.mode = CAR_MODE_AVOID;
  gCar.sys.state = CAR_STATE_RUN;
  gCar.sys.stage = CAR_STAGE_AVOID_OBSTACLE;
  gCar.sys.stage_start_tick = HAL_GetTick();
  gCar.sys.start_enable =
      (CAR_BLUETOOTH_START_HANDSHAKE != 0U) ? 0U :
      VISION_V2_AUTO_START;
  gCar.sys.action = CAR_ACTION_STOP;
  Motor_Stop();
}

void AppState_Update(void)
{
  const uint32_t now = HAL_GetTick();

  if (gCar.sys.start_enable == 0U)
  {
    s_nav_state = VISION_NAV_ZERO_FORWARD;
    s_collision_window_tick = 0U;
    s_route_blocked = 0U;
    s_origin_blocked = 0U;
    s_zero_clear_override = 0U;
    gCar.imu.collision_valid = 0U;
    gCar.imu.collision_bearing_deg = 0;
    StopWithAction(CAR_ACTION_STOP);
    return;
  }

  gCar.sys.mode = CAR_MODE_AVOID;
  gCar.sys.state = CAR_STATE_RUN;
  gCar.sys.stage = CAR_STAGE_AVOID_OBSTACLE;

  if (VisionLinkAlive() == 0U)
  {
    /*
     * NAV is the safety-critical link. The periodic-check navigator keeps
     * OpenMV pan centred and uses the STM32-side IMU for chassis heading, so
     * OpenMV receiving the optional YAW diagnostic is not required to drive.
     */
    StopWithAction(CAR_ACTION_SENSOR_ERROR);
    return;
  }

  if (s_nav_state == VISION_NAV_ERROR)
  {
    gCar.sys.state = CAR_STATE_ERROR;
    StopWithAction(CAR_ACTION_SENSOR_ERROR);
    return;
  }

  UpdateCollisionAssessment(now);

  if (s_nav_state == VISION_NAV_ZERO_FORWARD)
  {
    if (BlueRawVisible() == 0U) s_zero_clear_override = 0U;

    if (AbsInt16(CurrentYawCd()) >
        VISION_V2_ZERO_HEADING_TOLERANCE_CD)
    {
      s_nav_state = VISION_NAV_TURN_CHECK_ZERO;
      s_state_tick = now;
      s_route_blocked = 0U;
      return;
    }

    if ((FrontSonarNear() != 0U) ||
        (BlueTopBlocked() != 0U) ||
        ((BlueObstacleAhead() != 0U) &&
         (s_zero_clear_override == 0U)))
    {
      EnterPreTurnStop();
      return;
    }

    DriveAtWorldHeadingWithVisionBias(0);
    return;
  }

  if (s_nav_state == VISION_NAV_PRE_TURN_STOP)
  {
    BrakeWithAction(CAR_ACTION_STOP);
    if ((now - s_state_tick) >= VISION_V2_PRE_TURN_STOP_MS)
    {
      s_nav_state = VISION_NAV_TURN_AWAY;
      s_state_tick = now;
      ResetHeadingPid();
      (void)PivotToHeading(s_bypass_heading_cd);
    }
    return;
  }

  if (s_nav_state == VISION_NAV_TURN_AWAY)
  {
    const int16_t turn_yaw_cd = CurrentYawCd();
    uint32_t visual_clear_hold_ms = VISION_V2_TURN_VISUAL_CLEAR_HOLD_MS;
    uint8_t visual_route_clear = 0U;

    /*
     * A visible blue obstacle with a small image share has priority over
     * continuing the pivot. A completely lost detection uses the longer
     * normal clear interval because its left/right position is unavailable.
     * Only accept the current direction while it is inside -90 to +90 degrees.
     */
    if (AbsInt16(turn_yaw_cd) <= VISION_V2_SCAN_LIMIT_CD)
    {
      if ((BlueRawVisible() != 0U) &&
          (gCar.vision.obstacle_area_pixels <=
           VISION_V2_BLUE_AREA_CLEAR_PIXELS))
      {
        visual_route_clear = 1U;
        visual_clear_hold_ms = VISION_V2_TURN_SMALL_BLUE_HOLD_MS;
      }
      else if (BlueRawVisible() == 0U)
      {
        visual_route_clear = 1U;
      }
    }

    if (visual_route_clear != 0U)
    {
      if (s_turn_visual_clear_tick == 0U)
      {
        s_turn_visual_clear_tick = now;
      }
      else if ((now - s_turn_visual_clear_tick) >=
               visual_clear_hold_ms)
      {
        s_bypass_heading_cd = turn_yaw_cd;
        s_turn_sign = (s_bypass_heading_cd >= 0) ? 1 : -1;
        s_nav_state = VISION_NAV_BYPASS_FORWARD;
        s_state_tick = now;
        s_next_check_tick = now + s_check_interval_ms;
        s_check_start_center_mm = gCar.encoder.center_distance_mm;
        s_route_blocked = 0U;
        s_turn_visual_clear_tick = 0U;
        ResetHeadingPid();
        gCar.sys.action = CAR_ACTION_STOP;
        Motor_Brake();
        return;
      }
    }
    else
    {
      s_turn_visual_clear_tick = 0U;
    }

    if (PivotToHeading(s_bypass_heading_cd) != 0U)
    {
      s_nav_state = VISION_NAV_BYPASS_FORWARD;
      s_state_tick = now;
      s_next_check_tick = now + s_check_interval_ms;
      s_check_start_center_mm = gCar.encoder.center_distance_mm;
      s_route_blocked = 0U;
      s_turn_visual_clear_tick = 0U;
      ResetHeadingPid();
      DriveAtWorldHeadingWithVisionBias(s_bypass_heading_cd);
      return;
    }

    return;
  }

  if (s_nav_state == VISION_NAV_BYPASS_FORWARD)
  {
    /* Every moving cycle checks both front sonars before issuing drive PWM. */
    if (InitialObstacleAhead() != 0U)
    {
      EnterCheckBrake(1U);
      return;
    }

    if (((int32_t)(now - s_next_check_tick) >= 0) ||
        (AbsInt32(gCar.encoder.center_distance_mm -
                  s_check_start_center_mm) >=
         VISION_V2_PERIODIC_CHECK_DISTANCE_MM))
    {
      EnterCheckBrake(0U);
      return;
    }

    DriveAtWorldHeadingWithVisionBias(s_bypass_heading_cd);
    return;
  }

  if (s_nav_state == VISION_NAV_CHECK_BRAKE)
  {
    BrakeWithAction(CAR_ACTION_STOP);
    if ((now - s_state_tick) < VISION_V2_PRE_TURN_STOP_MS) return;

    /*
     * The current route is blocked and the most recent world-zero inspection
     * was also blocked: reverse the route by 180 degrees immediately.
     */
    if ((s_route_blocked != 0U) && (s_origin_blocked != 0U))
    {
      s_bypass_heading_cd =
          (s_bypass_heading_cd >= 0) ?
          -VISION_V2_SCAN_LIMIT_CD : VISION_V2_SCAN_LIMIT_CD;
      s_turn_sign = (s_bypass_heading_cd >= 0) ? 1 : -1;
      s_check_interval_ms = VISION_V2_FAST_CHECK_MS;
      s_route_blocked = 0U;
      s_nav_state = VISION_NAV_RETURN_BYPASS;
      s_state_tick = now;
      (void)PivotToHeading(s_bypass_heading_cd);
      return;
    }

    s_nav_state = VISION_NAV_TURN_CHECK_ZERO;
    s_state_tick = now;
    (void)PivotToHeading(0);
    return;
  }

  if (s_nav_state == VISION_NAV_TURN_CHECK_ZERO)
  {
    if (PivotToHeading(0) != 0U)
    {
      s_nav_state = VISION_NAV_CHECK_ZERO;
      s_state_tick = now;
      BrakeWithAction(CAR_ACTION_STOP);
      return;
    }

    return;
  }

  if (s_nav_state == VISION_NAV_CHECK_ZERO)
  {
    BrakeWithAction(CAR_ACTION_STOP);
    if ((now - s_state_tick) < VISION_V2_CHECK_SETTLE_MS) return;

    if (OriginDirectionClear() != 0U)
    {
      s_origin_blocked = 0U;
      s_route_blocked = 0U;
      s_zero_clear_override = (BlueRawVisible() != 0U) ? 1U : 0U;
      s_check_interval_ms = VISION_V2_PERIODIC_CHECK_MS;
      s_nav_state = VISION_NAV_ZERO_FORWARD;
      s_state_tick = now;
      ResetHeadingPid();
      DriveAtWorldHeadingWithVisionBias(0);
      return;
    }

    /* Top-filled blue remains: return to the previously selected route. */
    s_origin_blocked = 1U;
    if (s_route_blocked != 0U)
    {
      s_bypass_heading_cd =
          (s_bypass_heading_cd >= 0) ?
          -VISION_V2_SCAN_LIMIT_CD : VISION_V2_SCAN_LIMIT_CD;
      s_turn_sign = (s_bypass_heading_cd >= 0) ? 1 : -1;
      s_check_interval_ms = VISION_V2_FAST_CHECK_MS;
      s_route_blocked = 0U;
    }
    s_nav_state = VISION_NAV_RETURN_BYPASS;
    s_state_tick = now;
    (void)PivotToHeading(s_bypass_heading_cd);
    return;
  }

  /* VISION_NAV_RETURN_BYPASS */
  if (PivotToHeading(s_bypass_heading_cd) != 0U)
  {
    s_nav_state = VISION_NAV_BYPASS_FORWARD;
    s_state_tick = now;
    s_next_check_tick = now + s_check_interval_ms;
    s_check_start_center_mm = gCar.encoder.center_distance_mm;
    ResetHeadingPid();
    DriveAtWorldHeadingWithVisionBias(s_bypass_heading_cd);
    return;
  }

}

uint8_t AppState_EscapeMode(void)
{
  return 0U;
}

uint8_t AppState_HardUltrasonicMask(void)
{
  /* XXXv2 no longer has a <10 cm emergency escape state. */
  return 0U;
}

uint8_t AppState_ZeroHeadingBiasActive(void)
{
  return ((s_nav_state == VISION_NAV_TURN_CHECK_ZERO) ||
          (s_nav_state == VISION_NAV_CHECK_ZERO)) ? 1U : 0U;
}

uint8_t AppState_NavStatus(void)
{
  return (uint8_t)s_nav_state;
}

uint8_t AppState_NavPathLength(void)
{
  return 0U;
}

uint8_t AppState_NavObstacleCount(void)
{
  return BlueObstacleAhead();
}

uint8_t AppState_BlueObstacleActive(void)
{
  return BlueObstacleAhead();
}
