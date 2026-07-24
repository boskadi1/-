#include "camera_servo.h"
#include "main.h"
#include "car_config.h"
#include "car_data.h"
#include "servo_pwm.h"

static uint8_t s_scan_column;
static uint8_t s_scan_row;
static uint32_t s_last_scan_tick;
static uint32_t s_last_target_tick;
static uint32_t s_last_track_tick;
static uint32_t s_last_servo_tick;
static uint8_t s_target_locked;
static uint32_t s_boot_tick;
static uint8_t s_line_pose_latched;

static const int16_t s_scan_pan_angles[5] = {-50, -20, 10, 40, 70};
static const int16_t s_scan_tilt_angles[3] =
{
  CAMERA_SCAN_TOP_DEG,
  CAMERA_SCAN_MIDDLE_DEG,
  CAMERA_SCAN_BOTTOM_DEG
};

static int16_t ClampAngle(int16_t value, int16_t min_value, int16_t max_value)
{
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

static int16_t PanLogicalToOutput(int16_t logical_deg)
{
  return (int16_t)(CAMERA_PAN_CENTER_OFFSET_DEG +
                   CAMERA_PAN_OUTPUT_SIGN *
                   (logical_deg - CAMERA_PAN_INITIAL_DEG));
}

static int16_t TiltLogicalToOutput(int16_t logical_deg)
{
  return (int16_t)(CAMERA_TILT_CENTER_OFFSET_DEG +
                   CAMERA_TILT_OUTPUT_SIGN * logical_deg);
}

static int16_t MakeTrackingStep(int16_t error_deg)
{
  int16_t step = (int16_t)(error_deg / CAMERA_TRACK_GAIN_DIVISOR);

  if (step > CAMERA_TRACK_MAX_STEP_DEG) return CAMERA_TRACK_MAX_STEP_DEG;
  if (step < -CAMERA_TRACK_MAX_STEP_DEG) return -CAMERA_TRACK_MAX_STEP_DEG;
  if ((step == 0) && (error_deg > 0)) return 1;
  if ((step == 0) && (error_deg < 0)) return -1;
  return step;
}

static int16_t MoveToward(int16_t current, int16_t target)
{
  if (current < target)
  {
    int16_t next = (int16_t)(current + CAMERA_SERVO_SLEW_STEP_DEG);
    return (next > target) ? target : next;
  }
  if (current > target)
  {
    int16_t next = (int16_t)(current - CAMERA_SERVO_SLEW_STEP_DEG);
    return (next < target) ? target : next;
  }
  return current;
}

static void ApplyServoMotion(void)
{
  uint32_t now = HAL_GetTick();

  if ((now - s_last_servo_tick) < CAMERA_SERVO_UPDATE_MS) return;
  s_last_servo_tick = now;

  gCar.camera.current_deg = MoveToward(gCar.camera.current_deg,
                                       gCar.camera.target_deg);
  gCar.camera.current_tilt_deg = MoveToward(gCar.camera.current_tilt_deg,
                                            gCar.camera.target_tilt_deg);

  ServoPwm_SetCenteredAngle(SERVO_CHANNEL_CAMERA_PAN,
                            PanLogicalToOutput(gCar.camera.current_deg));
  ServoPwm_SetCenteredAngle(SERVO_CHANNEL_CAMERA_TILT,
                            TiltLogicalToOutput(gCar.camera.current_tilt_deg));
}

static uint8_t IsLoadStage(void)
{
  return ((gCar.sys.stage == CAR_STAGE_LOAD_SEARCH) ||
          (gCar.sys.stage == CAR_STAGE_LOAD_ALIGN) ||
          (gCar.sys.stage == CAR_STAGE_LOAD_CARGO)) ? 1U : 0U;
}

static uint8_t IsPickupOrUnloadViewStage(void)
{
  return (IsLoadStage() ||
          (gCar.sys.stage == CAR_STAGE_UNLOAD_ALIGN) ||
          (gCar.sys.stage == CAR_STAGE_UNLOAD_CARGO)) ? 1U : 0U;
}

static uint8_t IsLineViewStage(void)
{
  return ((gCar.sys.stage == CAR_STAGE_LINE_ENTRY) ||
          (gCar.sys.stage == CAR_STAGE_LEAVE_LOAD_ZONE) ||
          (gCar.sys.stage == CAR_STAGE_LINE_NAVIGATE) ||
          (gCar.sys.stage == CAR_STAGE_UNLOAD_SEARCH)) ? 1U : 0U;
}

static uint8_t IsTargetTrackingStage(void)
{
  return ((gCar.sys.stage == CAR_STAGE_LOAD_ALIGN) ||
          (gCar.sys.stage == CAR_STAGE_TARGET_ANCHOR)) ? 1U : 0U;
}

static uint8_t IsExpectedTargetSeen(void)
{
  if (IsLoadStage()) return gCar.vision.load_seen;
  return gCar.vision.unload_seen;
}

static void LockCurrentPositionForTracking(void)
{
  if (s_target_locked != 0U) return;

  s_target_locked = 1U;
  gCar.camera.target_deg = gCar.camera.current_deg;
  gCar.camera.target_tilt_deg = gCar.camera.current_tilt_deg;
  s_last_target_tick = 0U;
  s_last_track_tick = 0U;
}

static uint8_t GetTargetError(int16_t *pan_error_deg, int16_t *tilt_error_deg,
                              uint32_t *target_tick)
{
  if (IsLoadStage())
  {
    if (gCar.vision.load_seen == 0U) return 0U;
    *pan_error_deg = gCar.vision.load_relative_deg;
    *tilt_error_deg = gCar.vision.load_vertical_deg;
    *target_tick = gCar.vision.last_load_tick;
    return 1U;
  }

  if ((gCar.vision.unload_seen == 0U) || (gCar.vision.tag_id != TAG_UNLOAD_ID))
  {
    return 0U;
  }

  *pan_error_deg = gCar.vision.tag_relative_deg;
  *tilt_error_deg = gCar.vision.tag_vertical_deg;
  *target_tick = gCar.vision.last_tag_tick;
  return 1U;
}

static void TrackTarget(void)
{
  int16_t pan_error;
  int16_t tilt_error;
  int16_t pan_target = gCar.camera.current_deg;
  int16_t tilt_target = gCar.camera.current_tilt_deg;
  uint32_t target_tick;
  uint32_t now = HAL_GetTick();

  if (GetTargetError(&pan_error, &tilt_error, &target_tick) == 0U) return;
  if (target_tick == s_last_target_tick) return;
  if ((now - s_last_track_tick) < CAMERA_TRACK_UPDATE_MS) return;
  s_last_target_tick = target_tick;
  s_last_track_tick = now;

  if ((pan_error > CAMERA_TRACK_DEADZONE_DEG) ||
      (pan_error < -CAMERA_TRACK_DEADZONE_DEG))
  {
    pan_target = (int16_t)(pan_target +
                 CAMERA_PAN_TRACK_SIGN * MakeTrackingStep(pan_error));
  }

  if ((tilt_error > CAMERA_TRACK_DEADZONE_DEG) ||
      (tilt_error < -CAMERA_TRACK_DEADZONE_DEG))
  {
    tilt_target = (int16_t)(tilt_target +
                  CAMERA_TILT_TRACK_SIGN * MakeTrackingStep(tilt_error));
  }

  CameraServo_SetPanTilt(pan_target, tilt_target);
}

static void ScanNextPosition(void)
{
  uint8_t pan_index;

  if ((HAL_GetTick() - s_last_scan_tick) < CAMERA_SCAN_STEP_MS) return;
  s_last_scan_tick = HAL_GetTick();

  pan_index = (s_scan_row & 1U) ? (uint8_t)(4U - s_scan_column) : s_scan_column;
  CameraServo_SetPanTilt(s_scan_pan_angles[pan_index], s_scan_tilt_angles[s_scan_row]);

  s_scan_column++;
  if (s_scan_column >= 5U)
  {
    s_scan_column = 0U;
    s_scan_row = (uint8_t)((s_scan_row + 1U) % 3U);
  }
}

void CameraServo_Init(void)
{
  s_scan_column = 0U;
  s_scan_row = 1U;
  s_last_scan_tick = HAL_GetTick();
  s_last_target_tick = 0U;
  s_last_track_tick = HAL_GetTick();
  s_last_servo_tick = HAL_GetTick();
  s_target_locked = 0U;
  s_line_pose_latched = 0U;
  s_boot_tick = HAL_GetTick();
  gCar.camera.current_deg = CAMERA_PICKUP_PAN_DEG;
  gCar.camera.target_deg = CAMERA_PICKUP_PAN_DEG;
  /* Start at the neutral pose, hold briefly, then CameraServo_Update moves
   * downward one degree at a time. This makes the startup motion explicit. */
  gCar.camera.current_tilt_deg = CAMERA_SCAN_MIDDLE_DEG;
  gCar.camera.target_tilt_deg = CAMERA_SCAN_MIDDLE_DEG;
  ServoPwm_SetCenteredAngle(SERVO_CHANNEL_CAMERA_PAN,
                            PanLogicalToOutput(CAMERA_PICKUP_PAN_DEG));
  ServoPwm_SetCenteredAngle(SERVO_CHANNEL_CAMERA_TILT,
                            TiltLogicalToOutput(CAMERA_SCAN_MIDDLE_DEG));
}

uint8_t CameraServo_IsPickupViewReady(void)
{
  return (((HAL_GetTick() - s_boot_tick) >= CAMERA_BOOT_CENTER_HOLD_MS) &&
           (gCar.camera.current_deg == CAMERA_PICKUP_PAN_DEG) &&
           (gCar.camera.current_tilt_deg == CAMERA_PICKUP_TILT_DEG)) ? 1U : 0U;
}

uint8_t CameraServo_IsLineViewReady(void)
{
  return ((gCar.camera.current_deg == CAMERA_LINE_PAN_DEG) &&
          (gCar.camera.current_tilt_deg == CAMERA_LINE_TILT_DEG)) ? 1U : 0U;
}

void CameraServo_SetAngle(int16_t angle_deg)
{
  CameraServo_SetPanTilt(angle_deg, gCar.camera.current_tilt_deg);
}

void CameraServo_SetPanTilt(int16_t pan_deg, int16_t tilt_deg)
{
  pan_deg = ClampAngle(pan_deg, CAMERA_PAN_MIN_DEG, CAMERA_PAN_MAX_DEG);
  tilt_deg = ClampAngle(tilt_deg, CAMERA_TILT_MIN_DEG, CAMERA_TILT_MAX_DEG);

  gCar.camera.target_deg = pan_deg;
  gCar.camera.target_tilt_deg = tilt_deg;
}

void CameraServo_Update(void)
{
  if (IsLineViewStage() == 0U)
  {
    s_line_pose_latched = 0U;
  }

  if (IsPickupOrUnloadViewStage())
  {
    /* Keep a fixed downward view so OpenMV's calibrated claw window remains
     * at the same image coordinates throughout approach and pickup. */
    gCar.camera.scanning = 0U;
    s_target_locked = 0U;
    if ((HAL_GetTick() - s_boot_tick) < CAMERA_BOOT_CENTER_HOLD_MS)
    {
      CameraServo_SetPanTilt(CAMERA_PICKUP_PAN_DEG, CAMERA_SCAN_MIDDLE_DEG);
    }
    else
    {
      CameraServo_SetPanTilt(CAMERA_PICKUP_PAN_DEG, CAMERA_PICKUP_TILT_DEG);
    }
    ApplyServoMotion();
    return;
  }

  if (IsLineViewStage() != 0U)
  {
    /* Restore the line pose only once. TIM1 autonomously keeps outputting
     * the final PWM, so line following must not repeatedly update or track
     * either camera servo after the pose has been reached. */
    gCar.camera.scanning = 0U;
    s_target_locked = 0U;
    if (s_line_pose_latched != 0U)
    {
      return;
    }
    CameraServo_SetPanTilt(CAMERA_LINE_PAN_DEG, CAMERA_LINE_TILT_DEG);
    ApplyServoMotion();
    if (CameraServo_IsLineViewReady() != 0U)
    {
      s_line_pose_latched = 1U;
    }
    return;
  }

  if (gCar.sys.action == CAR_ACTION_SCAN_TARGET)
  {
    gCar.camera.scanning = 1U;
    if (IsExpectedTargetSeen() != 0U)
    {
      LockCurrentPositionForTracking();
      TrackTarget();
    }
    else
    {
      s_target_locked = 0U;
      ScanNextPosition();
    }
  }
  else if (gCar.sys.action == CAR_ACTION_ALIGN_TARGET)
  {
    gCar.camera.scanning = 0U;
    if (IsExpectedTargetSeen() != 0U)
    {
      LockCurrentPositionForTracking();
      TrackTarget();
    }
    else
    {
      s_target_locked = 0U;
      gCar.camera.target_deg = gCar.camera.current_deg;
      gCar.camera.target_tilt_deg = gCar.camera.current_tilt_deg;
    }
  }
  else if (IsTargetTrackingStage())
  {
    gCar.camera.scanning = 0U;
    if (IsExpectedTargetSeen() != 0U)
    {
      LockCurrentPositionForTracking();
      TrackTarget();
    }
    else
    {
      s_target_locked = 0U;
      gCar.camera.target_deg = gCar.camera.current_deg;
      gCar.camera.target_tilt_deg = gCar.camera.current_tilt_deg;
    }
  }
  else
  {
    gCar.camera.scanning = 0U;
    s_target_locked = 0U;
  }

  ApplyServoMotion();
}
