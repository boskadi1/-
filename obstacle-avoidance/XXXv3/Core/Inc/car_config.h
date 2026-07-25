#ifndef CAR_CONFIG_H_
#define CAR_CONFIG_H_

#include "main.h"

#define CAR_SOFTWARE_VERSION           "4.00-PERIODIC-CHECK"

#define CAR_DEBUG_UART_BAUDRATE        9600U
#define CAR_PERIODIC_DEBUG_OUTPUT      0U
#define CAR_BLUETOOTH_UART_BAUDRATE    9600U
#define CAR_BLUETOOTH_TELEMETRY_OUTPUT 1U
#define CAR_BLUETOOTH_TELEMETRY_MS     500U
#define CAR_BLUETOOTH_START_HANDSHAKE  1U
#define CAR_BLUETOOTH_PROMPT_MS        1000U
#define CAR_BLUETOOTH_CMD_IDLE_MS      40U
#define CAR_BLUETOOTH_RX_BUDGET_BYTES  32U
#define CAR_IMU_UART_BAUDRATE          9600U
#define CAR_IMU_RX_BUDGET_BYTES        44U
#define CAR_IMU_RX_RING_SIZE           128U
#define CAR_IMU_ACCEL_CAL_SAMPLES      32U
#define CAR_IMU_FORWARD_X_SIGN          1
#define CAR_IMU_RIGHT_Y_SIGN            1
#define CAR_IMU_ACCEL_DEADBAND_MG      60
#define CAR_IMU_ACCEL_VELOCITY_MAX_MM_S 1000.0f
/* The installed JY61P increases its raw yaw during a physical left turn.
   Navigation uses clockwise/right as positive, so invert the sensor here.
   Bluetooth then negates this internal convention for the requested display:
   clockwise is negative and counter-clockwise is positive. */
#define CAR_IMU_YAW_SIGN               (-1)
#define CAR_OPENMV_UART_BAUDRATE       115200U
#define CAR_OPENMV_HEADING_TX_MS       50U

#define TAG_LOAD_ID                    1
#define TAG_UNLOAD_ID                  2

#define US_INVALID_CM                  0xFFFFU
#define US_FRONT_TURN_CM               25U
#define US_SIDE_LIMIT_CM               18U
#define US_CLEAR_CM                    40U
#define US_TIMEOUT_MS                  300U
#define US_ECHO_TIMEOUT_US             25000U
/* Quiet time between completed measurements; four modules are interleaved. */
#define US_TRIGGER_INTERVAL_MS         20U

/* Sensor beam directions in the robot frame; positive is robot-right. */
#define US_FRONT_LEFT_BEARING_DEG      (-15)
#define US_FRONT_RIGHT_BEARING_DEG       15
#define US_LEFT_BEARING_DEG            (-90)
#define US_RIGHT_BEARING_DEG             90

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

#define CAR_TASK_STATE_MS              20U
#define CAR_TASK_DEBUG_MS              250U
#define CAR_CARGO_ACTION_MS            1500U
/* CALIBRATE: mission timing and visual arrival criteria. */
#define CAR_TARGET_STABLE_MS            300U
#define CAR_ULTRASONIC_STARTUP_MS      1000U
#define CAR_LINE_OFFSET_DEADZONE         10
#define CAR_TARGET_ALIGN_DEADZONE_DEG     8
#define CAR_LOAD_READY_AREA            6000U
#define CAR_UNLOAD_READY_AREA          4500U
#define CAR_YAW_DEADZONE_CD             500
#define CAR_VISION_RX_BUFFER_SIZE      128U
#define CAR_VISION_RX_RING_SIZE        256U
#define CAR_BLUETOOTH_RX_BUFFER_SIZE   48U
#define CAR_VISION_TIMEOUT_MS          300U
#define CAR_IMU_TIMEOUT_MS             300U

/*
 * XXXv2 uses only the finite-state visual navigator defined in app_state.c.
 */
#define VISION_V2_AUTO_START                    1U
#define VISION_V2_TEST_ONLY                     1U
#define VISION_V2_BLUE_AREA_TRIGGER_PIXELS    5000U
#define VISION_V2_BLUE_AREA_CLEAR_PIXELS      3000U
#define VISION_V2_BLUE_BEARING_DEADBAND_DEG       3
#define VISION_V2_TURN_SMALL_BLUE_HOLD_MS       100U
#define VISION_V2_TURN_VISUAL_CLEAR_HOLD_MS    200U
#define VISION_V2_ZERO_HEADING_TOLERANCE_CD     800
#define VISION_V2_TURN_TOLERANCE_CD              500
#define VISION_V2_TURN_SETTLE_MS                 120U
#define VISION_V2_TURN_STALL_MS                 2000U
#define VISION_V2_TURN_STALL_MIN_CHANGE_CD       500
#define VISION_V2_TURN_COLLISION_BRAKE_MS         300U
#define VISION_V2_BYPASS_HEADING_DEG             90
#define VISION_V2_SCAN_LIMIT_CD                  9000
#define VISION_V2_TURN_SPEED                      72
#define VISION_V2_VISUAL_BIAS_DEG                 12
#define VISION_V2_FRONT_BRAKE_CM                  10U
#define VISION_V2_FRONT_CLEAR_CM                  45U
#define VISION_V2_PERIODIC_CHECK_MS             2000U
#define VISION_V2_FAST_CHECK_MS                 2000U
#define VISION_V2_PERIODIC_CHECK_DISTANCE_MM      250U
#define VISION_V2_CHECK_SETTLE_MS                200U
/* Internal yaw is positive clockwise/right. */
#define VISION_V2_DEFAULT_TURN_SIGN                1
#define VISION_V2_PRE_TURN_STOP_MS               300U
#define VISION_V2_MAX_TURN_SPEED                 38.0f
#define VISION_V2_HEADING_DEADBAND_DEG            3.0f
#define VISION_V2_HEADING_PID_KP                  0.55f
#define VISION_V2_HEADING_PID_KI                  0.08f
#define VISION_V2_HEADING_PID_KD                  0.08f
#define VISION_V2_HEADING_PID_I_LIMIT            50.0f
#define VISION_V2_HEADING_D_FILTER_ALPHA          0.25f
#define VISION_V2_PLANNER_SONAR_RANGE_CM          80U
#define VISION_V2_COLLISION_WINDOW_MS            500U
#define VISION_V2_COLLISION_IMU_MAX_MM             8L
#define VISION_V2_COLLISION_ENCODER_MAX_MM        12U
#define VISION_V2_COLLISION_IMPACT_MG           700
#define VISION_V2_COLLISION_SONAR_CM              45U
#define VISION_V2_COLLISION_HOLD_MS             1200U

/* Four quadrature encoders: A is counted on both edges, B gives direction. */
/* These values are hardware-dependent and MUST be measured on the real car. */
#define ENCODER_COUNTS_PER_WHEEL_REV   780.0f
#define ENCODER_WHEEL_DIAMETER_MM      65.0f
/* Wheel-center spacing = 145 mm inner gap + 27 mm tire width. */
#define ENCODER_TIRE_INNER_GAP_MM      145.0f
#define ENCODER_TIRE_WIDTH_MM          27.0f
#define ENCODER_TRACK_WIDTH_MM         172.0f
#define ENCODER_SPEED_SAMPLE_MS        20U
#define ENCODER_SPEED_FILTER_ALPHA     0.35f

#define ENCODER_FL_A_PORT              GPIOF
#define ENCODER_FL_A_PIN               GPIO_PIN_0
#define ENCODER_FL_B_PORT              GPIOF
#define ENCODER_FL_B_PIN               GPIO_PIN_1
#define ENCODER_RL_A_PORT              GPIOF
#define ENCODER_RL_A_PIN               GPIO_PIN_2
#define ENCODER_RL_B_PORT              GPIOF
#define ENCODER_RL_B_PIN               GPIO_PIN_3
#define ENCODER_FR_A_PORT              GPIOF
#define ENCODER_FR_A_PIN               GPIO_PIN_4
#define ENCODER_FR_B_PORT              GPIOF
#define ENCODER_FR_B_PIN               GPIO_PIN_5
#define ENCODER_RR_A_PORT              GPIOF
#define ENCODER_RR_A_PIN               GPIO_PIN_6
#define ENCODER_RR_B_PORT              GPIOF
#define ENCODER_RR_B_PIN               GPIO_PIN_7

/* Change an individual sign to -1 if that wheel counts down while moving forward. */
#define ENCODER_FL_SIGN                1
#define ENCODER_RL_SIGN                1
#define ENCODER_FR_SIGN                1
#define ENCODER_RR_SIGN                1

/* The physical pan servo is OpenMV Servo(1). STM32 sends yaw over USART3;
   OpenMV applies pan=-yaw. STM32 only owns the PE14 fixed-tilt servo. */
#define CAMERA_PAN_MIN_DEG             (-90)
#define CAMERA_PAN_MAX_DEG             90
#define CAMERA_PAN_INITIAL_DEG         0
#define CAMERA_FIXED_TILT_DEG          10
#define CAMERA_SERVO_UPDATE_MS         20U
#define CAMERA_PAN_MAX_SPEED_DEG_S     120U
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

#define MOTOR_LEFT_BASE_SPEED          97.0f
#define MOTOR_RIGHT_BASE_SPEED         70
#define MOTOR_LEFT_MIN_EFFECTIVE_SPEED 63
#define MOTOR_RIGHT_MIN_EFFECTIVE_SPEED 50
#define MOTOR_SPEED_MAX                117
/* Per-side physical PWM ceilings. Closed-loop correction, APF steering and
   escape manoeuvres must never drive either side above these limits. */
#define MOTOR_LEFT_SPEED_MAX           117
#define MOTOR_RIGHT_SPEED_MAX           90
#define MOTOR_PWM_FREQ_HZ              20000U
#define MOTOR_CONTROL_PERIOD_MS        20U
/* Initial closed-loop linear-speed target. Tune from measured encoder speed
   after the first floor test; it does not change the requested 97/70 feedforward. */
#define MOTOR_CRUISE_TARGET_MM_S       220.0f
#define MOTOR_SPEED_PI_KP              0.040f
#define MOTOR_SPEED_PI_KI              0.080f
#define MOTOR_SPEED_PI_I_LIMIT         18.0f
#define MOTOR_ENCODER_REVERSE_GUARD_MM_S 30.0f

/* Corrected wiring: left L298N channel B (IN3/IN4/ENB/OUT3/OUT4). */
#define MOTOR_LEFT_IN3_PORT            GPIOB
#define MOTOR_LEFT_IN3_PIN             GPIO_PIN_4
#define MOTOR_LEFT_IN4_PORT            GPIOB
#define MOTOR_LEFT_IN4_PIN             GPIO_PIN_5
#define MOTOR_LEFT_ENB_PORT            GPIOB
#define MOTOR_LEFT_ENB_PIN             GPIO_PIN_6

/* Corrected wiring: right L298N channel A (IN1/IN2/ENA/OUT1/OUT2). */
#define MOTOR_RIGHT_IN1_PORT           GPIOB
#define MOTOR_RIGHT_IN1_PIN            GPIO_PIN_1
#define MOTOR_RIGHT_IN2_PORT           GPIOC
#define MOTOR_RIGHT_IN2_PIN            GPIO_PIN_5
#define MOTOR_RIGHT_ENA_PORT           GPIOB
#define MOTOR_RIGHT_ENA_PIN            GPIO_PIN_0

#define MOTOR_PWM_PERIOD_COUNTS        800U

#endif /* CAR_CONFIG_H_ */
