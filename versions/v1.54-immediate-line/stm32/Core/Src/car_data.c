#include "car_data.h"
#include "car_config.h"
#include <string.h>

CarData_t gCar;

void CarData_ResetMission(CarMode_t mode)
{
  gCar.sys.mode = mode;
  gCar.sys.stage = CAR_STAGE_IDLE;
  gCar.sys.action = CAR_ACTION_STOP;
  gCar.sys.remote_action = CAR_ACTION_STOP;
  gCar.sys.stage_start_tick = 0U;
  gCar.imu.target_yaw_valid = 0U;
  gCar.imu.target_yaw_cd = 0;
  gCar.imu.yaw_error_cd = 0;
  gCar.vision.load_seen = 0U;
  gCar.vision.load_grip_state = CAR_GRIP_STATE_NOT_READY;
  gCar.vision.unload_seen = 0U;
  gCar.vision.unload_position_state = CAR_UNLOAD_STATE_NOT_READY;
  gCar.vision.unload_position_valid = 0U;
  gCar.vision.unload_zone_seen = 0U;
  gCar.vision.line_offset = 0;
  gCar.vision.line_heading = 0;
  gCar.vision.line_magnitude = 0U;
  gCar.vision.line_valid = 0U;
  gCar.motion.line_correction = 0;
  gCar.motion.line_d_correction = 0;
  gCar.cargo.busy = 0U;
}

void CarData_Init(void)
{
  memset(&gCar, 0, sizeof(gCar));
#if CAR_BOOT_DIRECT_FORWARD_ENABLE
  gCar.sys.mode = CAR_MODE_REMOTE;
  gCar.sys.remote_action = CAR_ACTION_FORWARD;
  gCar.sys.start_enable = 1U;
#elif CAR_BOOT_VISION_PICKUP_ENABLE
  gCar.sys.mode = CAR_MODE_VISION_LINE;
  gCar.sys.start_enable = 1U;
#elif CAR_BOOT_LINE_TEST_ENABLE
  gCar.sys.mode = CAR_MODE_LINE_TEST;
  gCar.sys.start_enable = 1U;
#else
  gCar.sys.mode = CAR_MODE_STOP;
  gCar.sys.start_enable = 0U;
#endif
  gCar.sys.state = CAR_STATE_IDLE;
  gCar.sys.stage = CAR_STAGE_IDLE;
  gCar.sys.action = CAR_ACTION_STOP;
#if !CAR_BOOT_DIRECT_FORWARD_ENABLE
  gCar.sys.remote_action = CAR_ACTION_STOP;
#endif
  gCar.us.front_left_cm = US_INVALID_CM;
  gCar.us.front_right_cm = US_INVALID_CM;
  gCar.us.left_cm = US_INVALID_CM;
  gCar.us.right_cm = US_INVALID_CM;
}

const char *CarData_ActionName(CarAction_t action)
{
  switch (action)
  {
    case CAR_ACTION_FORWARD: return "FORWARD";
    case CAR_ACTION_BACKWARD: return "BACKWARD";
    case CAR_ACTION_SLOW_FORWARD: return "SLOW_FORWARD";
    case CAR_ACTION_TURN_LEFT: return "TURN_LEFT";
    case CAR_ACTION_TURN_RIGHT: return "TURN_RIGHT";
    case CAR_ACTION_LINE_FOLLOW: return "LINE_FOLLOW";
    case CAR_ACTION_SEARCH_LINE: return "SEARCH_LINE";
    case CAR_ACTION_SCAN_TARGET: return "SCAN_TARGET";
    case CAR_ACTION_ALIGN_TARGET: return "ALIGN_TARGET";
    case CAR_ACTION_LOAD_CARGO: return "LOAD_CARGO";
    case CAR_ACTION_UNLOAD_CARGO: return "UNLOAD_CARGO";
    case CAR_ACTION_RESTORE_HEADING: return "RESTORE_HEADING";
    case CAR_ACTION_MISSION_DONE: return "MISSION_DONE";
    case CAR_ACTION_SENSOR_ERROR: return "SENSOR_ERROR";
    case CAR_ACTION_STOP:
    default: return "STOP";
  }
}

const char *CarData_ModeName(CarMode_t mode)
{
  switch (mode)
  {
    case CAR_MODE_REMOTE: return "REMOTE";
    case CAR_MODE_AVOID: return "AVOID_TASK";
    case CAR_MODE_VISION_LINE: return "VISION_TASK";
    case CAR_MODE_LINE_TEST: return "LINE_TEST";
    case CAR_MODE_STOP:
    default: return "STOP";
  }
}

const char *CarData_StageName(CarStage_t stage)
{
  switch (stage)
  {
    case CAR_STAGE_LOAD_SEARCH: return "LOAD_SEARCH";
    case CAR_STAGE_LOAD_ALIGN: return "LOAD_ALIGN";
    case CAR_STAGE_LOAD_CARGO: return "LOAD_CARGO";
    case CAR_STAGE_LEAVE_LOAD_ZONE: return "LEAVE_LOAD_ZONE";
    case CAR_STAGE_LINE_ENTRY: return "LINE_ENTRY";
    case CAR_STAGE_TARGET_SEARCH: return "TARGET_SEARCH";
    case CAR_STAGE_TARGET_ANCHOR: return "TARGET_ANCHOR";
    case CAR_STAGE_GOAL_NAVIGATE: return "GOAL_NAVIGATE";
    case CAR_STAGE_AVOID_OBSTACLE: return "AVOID_OBSTACLE";
    case CAR_STAGE_RESTORE_HEADING: return "RESTORE_HEADING";
    case CAR_STAGE_UNLOAD_SEARCH: return "UNLOAD_SEARCH";
    case CAR_STAGE_UNLOAD_ALIGN: return "UNLOAD_ALIGN";
    case CAR_STAGE_UNLOAD_CARGO: return "UNLOAD_CARGO";
    case CAR_STAGE_LINE_NAVIGATE: return "LINE_NAVIGATE";
    case CAR_STAGE_DONE: return "DONE";
    case CAR_STAGE_ERROR: return "ERROR";
    case CAR_STAGE_IDLE:
    default: return "IDLE";
  }
}
