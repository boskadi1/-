#ifndef CAR_TYPES_H_
#define CAR_TYPES_H_

#include <stdint.h>

typedef enum
{
  CAR_MODE_STOP = 0,
  CAR_MODE_REMOTE,
  CAR_MODE_AVOID,
  CAR_MODE_VISION_LINE,
  CAR_MODE_LINE_TEST
} CarMode_t;

typedef enum
{
  CAR_STATE_INIT = 0,
  CAR_STATE_IDLE,
  CAR_STATE_RUN,
  CAR_STATE_ERROR
} CarState_t;

typedef enum
{
  CAR_STAGE_IDLE = 0,
  CAR_STAGE_LOAD_SEARCH,
  CAR_STAGE_LOAD_ALIGN,
  CAR_STAGE_LOAD_CARGO,
  CAR_STAGE_LEAVE_LOAD_ZONE,
  CAR_STAGE_LINE_ENTRY,
  CAR_STAGE_TARGET_SEARCH,
  CAR_STAGE_TARGET_ANCHOR,
  CAR_STAGE_GOAL_NAVIGATE,
  CAR_STAGE_AVOID_OBSTACLE,
  CAR_STAGE_RESTORE_HEADING,
  CAR_STAGE_UNLOAD_SEARCH,
  CAR_STAGE_UNLOAD_ALIGN,
  CAR_STAGE_UNLOAD_CARGO,
  CAR_STAGE_LINE_NAVIGATE,
  CAR_STAGE_DONE,
  CAR_STAGE_ERROR
} CarStage_t;

typedef enum
{
  CAR_ACTION_STOP = 0,
  CAR_ACTION_FORWARD,
  CAR_ACTION_BACKWARD,
  CAR_ACTION_SLOW_FORWARD,
  CAR_ACTION_TURN_LEFT,
  CAR_ACTION_TURN_RIGHT,
  CAR_ACTION_LINE_FOLLOW,
  CAR_ACTION_SEARCH_LINE,
  CAR_ACTION_SCAN_TARGET,
  CAR_ACTION_ALIGN_TARGET,
  CAR_ACTION_LOAD_CARGO,
  CAR_ACTION_UNLOAD_CARGO,
  CAR_ACTION_RESTORE_HEADING,
  CAR_ACTION_MISSION_DONE,
  CAR_ACTION_SENSOR_ERROR
} CarAction_t;

typedef struct
{
  uint8_t start_enable;
  CarMode_t mode;
  CarState_t state;
  CarStage_t stage;
  CarAction_t action;
  CarAction_t remote_action;
  uint32_t stage_start_tick;
  uint32_t tick_ms;
} CarSystem_t;

typedef struct
{
  uint16_t front_left_cm;
  uint16_t front_right_cm;
  uint16_t left_cm;
  uint16_t right_cm;
  uint8_t front_left_valid;
  uint8_t front_right_valid;
  uint8_t left_valid;
  uint8_t right_valid;
  uint32_t front_left_tick;
  uint32_t front_right_tick;
  uint32_t left_tick;
  uint32_t right_tick;
} CarUltrasonic_t;

typedef struct
{
  int32_t roll_cd;
  int32_t pitch_cd;
  int32_t yaw_cd;
  int32_t yaw_zero_cd;
  int32_t yaw_rel_cd;
  int16_t target_yaw_cd;
  int16_t yaw_error_cd;
  uint8_t target_yaw_valid;
  uint8_t angle_valid;
  uint8_t yaw_anchored;
  uint32_t last_frame_tick;
} CarImu_t;

typedef struct
{
  int16_t line_offset;
  int16_t line_heading;
  uint16_t line_magnitude;
  uint8_t line_valid;
  uint8_t tag_id;
  int16_t tag_relative_deg;
  int16_t tag_vertical_deg;
  uint32_t tag_area;
  uint8_t tag_valid;
  int16_t load_relative_deg;
  int16_t load_vertical_deg;
  uint32_t load_area;
  uint8_t load_grip_state;
  uint8_t load_seen;
  uint8_t unload_seen;
  uint8_t unload_position_state;
  uint8_t unload_position_valid;
  uint8_t unload_zone_seen;
  uint8_t transverse_count;
  uint8_t transverse_active;
  uint8_t transverse_passed_count;
  uint32_t last_line_tick;
  uint32_t last_load_tick;
  uint32_t last_tag_tick;
  uint32_t last_unload_zone_tick;
  uint32_t last_transverse_tick;
  uint32_t rx_byte_count;
  uint32_t rx_line_count;
  uint32_t rx_parse_error_count;
  uint32_t rx_overflow_count;
  uint32_t last_rx_tick;
} CarVision_t;

typedef struct
{
  int16_t go_speed;
  int16_t turn_speed;
  int16_t left_set_speed;
  int16_t right_set_speed;
  int16_t front_left_set_speed;
  int16_t rear_left_set_speed;
  int16_t front_right_set_speed;
  int16_t rear_right_set_speed;
  int16_t left_pwm;
  int16_t right_pwm;
  int16_t front_left_pwm;
  int16_t rear_left_pwm;
  int16_t front_right_pwm;
  int16_t rear_right_pwm;
  int16_t line_correction;
  int16_t line_i_correction;
  int16_t line_d_correction;
  int16_t left_feedback_speed;
  int16_t right_feedback_speed;
  uint16_t left_encoder_counts;
  uint16_t right_encoder_counts;
  uint8_t left_encoder_valid;
  uint8_t right_encoder_valid;
  uint32_t encoder_sample_tick;
} CarMotion_t;

typedef struct
{
  int16_t current_deg;
  int16_t target_deg;
  int16_t current_tilt_deg;
  int16_t target_tilt_deg;
  uint8_t scanning;
} CameraServoData_t;

typedef struct
{
  uint8_t busy;
  uint8_t loaded;
} CargoServoData_t;

typedef struct
{
  uint8_t busy;
  uint8_t has_object;
  int16_t claw_deg;
  int16_t lift_deg;
} GrabServoData_t;

typedef struct
{
  CarSystem_t sys;
  CarUltrasonic_t us;
  CarImu_t imu;
  CarVision_t vision;
  CarMotion_t motion;
  CameraServoData_t camera;
  CargoServoData_t cargo;
  GrabServoData_t grab;
} CarData_t;

#endif /* CAR_TYPES_H_ */
