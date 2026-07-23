#ifndef CAR_CONFIG_H_
#define CAR_CONFIG_H_

#include "main.h"

#define CAR_SOFTWARE_VERSION           "1.52"

/* Boot test options. DIRECT_FORWARD takes priority when both are enabled. */
#define CAR_BOOT_DIRECT_FORWARD_ENABLE  0U
#define CAR_BOOT_VISION_PICKUP_ENABLE   1U
#define CAR_BOOT_LINE_TEST_ENABLE       0U

#define CAR_DEBUG_UART_BAUDRATE        9600U
#define CAR_BLUETOOTH_UART_BAUDRATE    9600U
#define CAR_IMU_UART_BAUDRATE          9600U
#define CAR_OPENMV_UART_BAUDRATE       115200U
#define CAR_BLUETOOTH_TELEMETRY_MS      500U

#define TAG_LOAD_ID                    1
#define TAG_UNLOAD_ID                  2

#define US_INVALID_CM                  0xFFFFU
#define US_EMERGENCY_CM                12U
#define US_UNLOAD_EMERGENCY_CM          6U
#define US_FRONT_TURN_CM               25U
#define US_SIDE_LIMIT_CM               18U
#define US_CLEAR_CM                    40U
#define US_TIMEOUT_MS                  300U
#define US_ECHO_TIMEOUT_US             25000U
#define US_TRIGGER_INTERVAL_MS         15U

#define US_FL_TRIG_PORT                GPIOD
#define US_FL_TRIG_PIN                 GPIO_PIN_10
#define US_FL_ECHO_PORT                GPIOD
#define US_FL_ECHO_PIN                 GPIO_PIN_11

#define US_FR_TRIG_PORT                GPIOD
#define US_FR_TRIG_PIN                 GPIO_PIN_12
#define US_FR_ECHO_PORT                GPIOD
#define US_FR_ECHO_PIN                 GPIO_PIN_13

#define US_LEFT_TRIG_PORT              GPIOG
#define US_LEFT_TRIG_PIN               GPIO_PIN_6
#define US_LEFT_ECHO_PORT              GPIOG
#define US_LEFT_ECHO_PIN               GPIO_PIN_7

#define US_RIGHT_TRIG_PORT             GPIOG
#define US_RIGHT_TRIG_PIN              GPIO_PIN_8
#define US_RIGHT_ECHO_PORT             GPIOC
#define US_RIGHT_ECHO_PIN              GPIO_PIN_6

#define CAR_TASK_STATE_MS              50U
#define CAR_TASK_DEBUG_MS              500U
#define CAR_CARGO_ACTION_MS            1500U
#define CAR_LOAD_ZONE_EXIT_MS          2000U
#define CAR_LINE_ENTRY_TIMEOUT_MS      5000U
/* CALIBRATE: mission timing and visual arrival criteria. */
#define CAR_TARGET_STABLE_MS            300U
#define CAR_UNLOAD_VIEW_SEARCH_TIMEOUT_MS 8000U
#define CAR_UNLOAD_INITIAL_FORWARD_MS   1000U
#define CAR_UNLOAD_SWEEP_SIDE_MS         350U
#define CAR_UNLOAD_SWEEP_CROSS_MS        700U
#define CAR_ULTRASONIC_STARTUP_MS      1000U
#define CAR_LINE_OFFSET_DEADZONE          6
#define CAR_LINE_HEADING_DEADZONE          5
#define CAR_LINE_KP_NUM                    3
#define CAR_LINE_KP_DEN                    4
#define CAR_LINE_HEADING_KP_NUM            1
#define CAR_LINE_HEADING_KP_DEN            2
#define CAR_LINE_KD_NUM                    1
#define CAR_LINE_KD_DEN                    2
#define CAR_LINE_D_FILTER_OLD_WEIGHT       3
#define CAR_LINE_D_FILTER_NEW_WEIGHT       1
#define CAR_LINE_D_DELTA_MAX              40
#define CAR_LINE_D_TERM_MAX                6
#define CAR_LINE_D_RESET_MS              300U
#define CAR_LINE_CORRECTION_MAX           25
#define CAR_LINE_PIVOT_CORRECTION          18
/* Sharp-curve manoeuvre: brake, pivot, brake, then creep forward. */
#define CAR_LINE_SHARP_BRAKE_MS            80U
#define CAR_LINE_SHARP_TURN_MS            220U
#define CAR_LINE_SHARP_FORWARD_MS         250U
#define CAR_LINE_SHARP_FORWARD_LEFT_SPEED   85
#define CAR_LINE_SHARP_FORWARD_RIGHT_SPEED  73
#define CAR_LINE_SHARP_TURN_RETRY_STEP       5
#define CAR_LINE_MIN_MAGNITUDE             5U
#define CAR_LINE_STEERING_DELAY_MS          0U
#define CAR_LINE_DELAY_BUFFER_SIZE         20U
#define CAR_LINE_LOST_GRACE_MS           200U
#define CAR_LINE_SEARCH_TIMEOUT_MS      1200U
#define CAR_TARGET_ALIGN_DEADZONE_DEG     8
/* Fixed downward camera pose used while placing the red block between claws. */
#define CAMERA_PICKUP_PAN_DEG             10
#define CAMERA_PICKUP_TILT_DEG            30
#define CAMERA_BOOT_CENTER_HOLD_MS        300U
#define CAMERA_LINE_PAN_DEG        10
#define CAMERA_LINE_TILT_DEG       10
/* Logical-to-physical servo mapping. This car's tilt servo is installed in
 * the opposite direction, therefore logical negative/down becomes +PWM. */
#define CAMERA_PAN_OUTPUT_SIGN              1
#define CAMERA_TILT_OUTPUT_SIGN              1
#define CAMERA_PAN_CENTER_OFFSET_DEG          0
#define CAMERA_TILT_CENTER_OFFSET_DEG         0
/* Values sent by OpenMV in MV,LOAD,...,<grip_state>,<valid>. */
#define CAR_GRIP_STATE_NOT_READY           0U
#define CAR_GRIP_STATE_APPROACH            1U
#define CAR_GRIP_STATE_READY               2U
#define CAR_GRIP_STATE_TOO_CLOSE           3U
#define CAR_UNLOAD_STATE_NOT_READY         0U
#define CAR_UNLOAD_STATE_APPROACH          1U
#define CAR_UNLOAD_STATE_READY             2U
#define CAR_UNLOAD_STATE_TOO_CLOSE         3U
#define CAR_LOAD_READY_AREA            6000U
#define CAR_UNLOAD_READY_AREA          4500U
#define CAR_YAW_DEADZONE_CD             500
#define CAR_VISION_RX_BUFFER_SIZE      64U
#define CAR_VISION_RX_RING_SIZE        256U
#define CAR_BLUETOOTH_RX_BUFFER_SIZE   48U
#define CAR_VISION_TIMEOUT_MS          300U
#define CAR_VISION_MODE_REPEAT_MS      500U
#define CAR_IMU_TIMEOUT_MS             300U

#define CAMERA_SCAN_LEFT_DEG           (-50)
#define CAMERA_SCAN_CENTER_DEG         10
#define CAMERA_SCAN_RIGHT_DEG          70
#define CAMERA_SCAN_STEP_MS            1200U
#define CAMERA_SCAN_TOP_DEG            25
#define CAMERA_SCAN_MIDDLE_DEG         10
#define CAMERA_SCAN_BOTTOM_DEG         (-25)
#define CAMERA_PAN_MIN_DEG             (-65)
#define CAMERA_PAN_MAX_DEG             85
#define CAMERA_PAN_INITIAL_DEG         10
#define CAMERA_TILT_MIN_DEG            (-35)
#define CAMERA_TILT_MAX_DEG            35
#define CAMERA_TRACK_DEADZONE_DEG      4
#define CAMERA_TRACK_GAIN_DIVISOR      3
#define CAMERA_TRACK_MAX_STEP_DEG      2
#define CAMERA_TRACK_UPDATE_MS         80U
#define CAMERA_SERVO_UPDATE_MS         20U
#define CAMERA_SERVO_SLEW_STEP_DEG     1
#define CAMERA_PAN_TRACK_SIGN          1
#define CAMERA_TILT_TRACK_SIGN         1
#define CAMERA_MANUAL_STEP_DEG         5

#define SERVO_MIN_PULSE_US             500U
#define SERVO_CENTER_PULSE_US          1500U
#define SERVO_MAX_PULSE_US             2500U
#define SERVO_FRAME_US                 20000U
#define SERVO_US_PER_DEG               10

#define CARGO_LEFT_MANUAL_STEP_DEG     30
#define CARGO_RIGHT_MANUAL_STEP_DEG    5
#define CARGO_LEFT_MIN_DEG             0
#define CARGO_LEFT_MAX_DEG             360
#define CARGO_RIGHT_MIN_DEG            20
#define CARGO_RIGHT_MAX_DEG            160
/* CALIBRATE: mirrored jaw angles after the linkage is installed. */
#define CARGO_LEFT_OPEN_DEG            120
#define CARGO_RIGHT_OPEN_DEG           120
#define CARGO_LEFT_CLOSED_DEG          200
#define CARGO_RIGHT_CLOSED_DEG         80

#define SERVO_CARGO_GRIP_PORT          GPIOE
#define SERVO_CARGO_GRIP_PIN           GPIO_PIN_9
#define SERVO_CARGO_LIFT_PORT          GPIOE
#define SERVO_CARGO_LIFT_PIN           GPIO_PIN_11
#define SERVO_CAMERA_PAN_PORT          GPIOE
#define SERVO_CAMERA_PAN_PIN           GPIO_PIN_13
#define SERVO_CAMERA_TILT_PORT         GPIOE
#define SERVO_CAMERA_TILT_PIN          GPIO_PIN_14

/* Startup gripper copied only from the supplied gripper reference project.
 * TIM1 CH1/CH2 (PE9/PE11) are dedicated to claw/lift in this version. */
#define GRAB_STARTUP_AUTO_PICKUP_ENABLE  0U
#define GRAB_CLAW_MIN_DEG               (-180)
#define GRAB_CLAW_MAX_DEG                 90
#define GRAB_CLAW_INIT_DEG               0
#define GRAB_CLAW_OPEN_DEG               90
/* The physical positions requested for v1.44 are swapped: pickup moves from
 * the new startup/open position -75 degrees to the old startup -38 degrees. */
#define GRAB_CLAW_CLOSE_DEG              -90
#define GRAB_LIFT_MIN_DEG               (-60)
#define GRAB_LIFT_MAX_DEG                 20
#define GRAB_LIFT_INIT_DEG                20
#define GRAB_LIFT_DOWN_DEG                15
#define GRAB_LIFT_UP_DEG                  -50
#define GRAB_CLAW_MANUAL_STEP_DEG          5
#define GRAB_LIFT_MANUAL_STEP_DEG          5
#define GRAB_POWER_STABILIZE_MS           200U
#define GRAB_INITIAL_MOVE_MS              500U
#define GRAB_ACTION_DELAY_MS              500U

/* CALIBRATE: independent left/right values compensate drivetrain mismatch.
 * The left motor is slower on the current car, so its defaults are 3 higher.
 * Change only these values during straight-line calibration. */
#define MOTOR_FORWARD_LEFT_SPEED       83
#define MOTOR_FORWARD_RIGHT_SPEED      70
#define MOTOR_SLOW_LEFT_SPEED          75
#define MOTOR_SLOW_RIGHT_SPEED         65
/* Shared low speed for approaching both the red load and black unload circle. */
#define MOTOR_LOAD_APPROACH_LEFT_SPEED  70
#define MOTOR_LOAD_APPROACH_RIGHT_SPEED 60
/* Independent wheel magnitudes for in-place turns. The left motor needs
 * more PWM to match the right motor's measured speed. */
#define MOTOR_TURN_LEFT_WHEEL_SPEED    92
#define MOTOR_TURN_RIGHT_WHEEL_SPEED   70
/* Line following starts from the measured straight-driving balance. */
#define MOTOR_LINE_LEFT_BASE_SPEED     MOTOR_FORWARD_LEFT_SPEED
#define MOTOR_LINE_RIGHT_BASE_SPEED    MOTOR_FORWARD_RIGHT_SPEED
/* Only active while ordinary line steering is correcting. A positive
 * correction accelerates the left outer wheel more; a negative correction
 * reduces the weaker left inner wheel less. Straight 83/70 is unchanged. */
#define MOTOR_LINE_LEFT_OUTER_GAIN_NUM   5
#define MOTOR_LINE_LEFT_OUTER_GAIN_DEN   4
#define MOTOR_LINE_LEFT_INNER_GAIN_NUM   3
#define MOTOR_LINE_LEFT_INNER_GAIN_DEN   4
#define MOTOR_LINE_SPEED_MAX          100
#define MOTOR_LINE_TURN_SPEED          MOTOR_LINE_SPEED_MAX
/* Lost-line in-place search uses a lower right-wheel magnitude. */
#define MOTOR_LINE_SEARCH_LEFT_SPEED   92
#define MOTOR_LINE_SEARCH_RIGHT_SPEED  60
#define MOTOR_SPEED_MAX                100
#define MOTOR_EFFECTIVE_MIN_SPEED      60
#define MOTOR_LINE_EFFECTIVE_MIN_SPEED 65
#define MOTOR_RAMP_STEP                5
#define MOTOR_RAMP_INTERVAL_MS         20U
#define MOTOR_PWM_FREQ_HZ              20000U
#define MOTOR_LEFT_DIRECTION_INVERT     0U
#define MOTOR_RIGHT_DIRECTION_INVERT    1U

#define MOTOR_LEFT_IN1_PORT            GPIOB
#define MOTOR_LEFT_IN1_PIN             GPIO_PIN_4
#define MOTOR_LEFT_IN2_PORT            GPIOB
#define MOTOR_LEFT_IN2_PIN             GPIO_PIN_5
#define MOTOR_LEFT_PWM_PORT            GPIOB
#define MOTOR_LEFT_PWM_PIN             GPIO_PIN_6

#define MOTOR_RIGHT_IN1_PORT           GPIOB
#define MOTOR_RIGHT_IN1_PIN            GPIO_PIN_1
#define MOTOR_RIGHT_IN2_PORT           GPIOC
#define MOTOR_RIGHT_IN2_PIN            GPIO_PIN_5
#define MOTOR_RIGHT_PWM_PORT           GPIOB
#define MOTOR_RIGHT_PWM_PIN            GPIO_PIN_0

#define MOTOR_PWM_PERIOD_COUNTS        800U

#endif /* CAR_CONFIG_H_ */
