#include "motor.h"
#include "car_config.h"
#include "car_data.h"

static int16_t s_left_applied;
static int16_t s_right_applied;
static uint32_t s_last_ramp_tick;

typedef enum
{
  LINE_SHARP_IDLE = 0,
  LINE_SHARP_BRAKE_BEFORE_TURN,
  LINE_SHARP_TURN,
  LINE_SHARP_BRAKE_BEFORE_FORWARD,
  LINE_SHARP_FORWARD
} LineSharpPhase_t;

static LineSharpPhase_t s_line_sharp_phase;
static int8_t s_line_sharp_direction;
static int16_t s_line_sharp_turn_boost;
static uint32_t s_line_sharp_tick;

static int16_t ClampSpeed(int16_t speed)
{
  if (speed > MOTOR_SPEED_MAX) return MOTOR_SPEED_MAX;
  if (speed < -MOTOR_SPEED_MAX) return -MOTOR_SPEED_MAX;
  return speed;
}

static int16_t ClampLineForwardSpeed(int32_t speed)
{
  if (speed > MOTOR_LINE_SPEED_MAX) return MOTOR_LINE_SPEED_MAX;
  if (speed < MOTOR_LINE_EFFECTIVE_MIN_SPEED) return MOTOR_LINE_EFFECTIVE_MIN_SPEED;
  return (int16_t)speed;
}

static int16_t ApplyMotorDeadband(int16_t speed)
{
  if ((speed > 0) && (speed < MOTOR_EFFECTIVE_MIN_SPEED)) return 0;
  if ((speed < 0) && (speed > -MOTOR_EFFECTIVE_MIN_SPEED)) return 0;
  return speed;
}

static uint32_t SpeedToCompare(int16_t speed)
{
  uint32_t magnitude = (speed < 0) ? (uint32_t)(-speed) : (uint32_t)speed;
  return (magnitude * MOTOR_PWM_PERIOD_COUNTS) / MOTOR_SPEED_MAX;
}

static int16_t ApproachSpeed(int16_t current, int16_t target)
{
  if (current < target)
  {
    current = (int16_t)(current + MOTOR_RAMP_STEP);
    if (current > target) current = target;
  }
  else if (current > target)
  {
    current = (int16_t)(current - MOTOR_RAMP_STEP);
    if (current < target) current = target;
  }
  return current;
}

static void Motor_HardwareInit(void)
{
  uint32_t timer_clock_hz = HAL_RCC_GetPCLK1Freq();
  uint32_t prescaler;

  if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U)
  {
    timer_clock_hz *= 2U;
  }

  prescaler = timer_clock_hz / (MOTOR_PWM_FREQ_HZ * MOTOR_PWM_PERIOD_COUNTS);
  if (prescaler == 0U) prescaler = 1U;

  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM4_CLK_ENABLE();

  TIM3->CR1 = 0U;
  TIM3->PSC = prescaler - 1U;
  TIM3->ARR = MOTOR_PWM_PERIOD_COUNTS - 1U;
  TIM3->CCR3 = 0U;
  TIM3->CCMR2 = TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
  TIM3->CCER = TIM_CCER_CC3E;
  TIM3->EGR = TIM_EGR_UG;
  TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

  TIM4->CR1 = 0U;
  TIM4->PSC = prescaler - 1U;
  TIM4->ARR = MOTOR_PWM_PERIOD_COUNTS - 1U;
  TIM4->CCR1 = 0U;
  TIM4->CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
  TIM4->CCER = TIM_CCER_CC1E;
  TIM4->EGR = TIM_EGR_UG;
  TIM4->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

static void Motor_ApplyLeft(int16_t speed)
{
  int16_t physical_speed;

  speed = ApplyMotorDeadband(ClampSpeed(speed));
#if MOTOR_LEFT_DIRECTION_INVERT
  physical_speed = (int16_t)(-speed);
#else
  physical_speed = speed;
#endif

  if (physical_speed > 0)
  {
    HAL_GPIO_WritePin(MOTOR_LEFT_IN1_PORT, MOTOR_LEFT_IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_LEFT_IN2_PORT, MOTOR_LEFT_IN2_PIN, GPIO_PIN_RESET);
  }
  else if (physical_speed < 0)
  {
    HAL_GPIO_WritePin(MOTOR_LEFT_IN1_PORT, MOTOR_LEFT_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_LEFT_IN2_PORT, MOTOR_LEFT_IN2_PIN, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(MOTOR_LEFT_IN1_PORT, MOTOR_LEFT_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_LEFT_IN2_PORT, MOTOR_LEFT_IN2_PIN, GPIO_PIN_RESET);
  }

  TIM4->CCR1 = SpeedToCompare(speed);
  gCar.motion.front_left_pwm = speed;
  gCar.motion.rear_left_pwm = speed;
  gCar.motion.left_pwm = speed;
}

static void Motor_ApplyRight(int16_t speed)
{
  int16_t physical_speed;

  speed = ApplyMotorDeadband(ClampSpeed(speed));
#if MOTOR_RIGHT_DIRECTION_INVERT
  physical_speed = (int16_t)(-speed);
#else
  physical_speed = speed;
#endif

  if (physical_speed > 0)
  {
    HAL_GPIO_WritePin(MOTOR_RIGHT_IN1_PORT, MOTOR_RIGHT_IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_RIGHT_IN2_PORT, MOTOR_RIGHT_IN2_PIN, GPIO_PIN_RESET);
  }
  else if (physical_speed < 0)
  {
    HAL_GPIO_WritePin(MOTOR_RIGHT_IN1_PORT, MOTOR_RIGHT_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_RIGHT_IN2_PORT, MOTOR_RIGHT_IN2_PIN, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(MOTOR_RIGHT_IN1_PORT, MOTOR_RIGHT_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_RIGHT_IN2_PORT, MOTOR_RIGHT_IN2_PIN, GPIO_PIN_RESET);
  }

  TIM3->CCR3 = SpeedToCompare(speed);
  gCar.motion.front_right_pwm = speed;
  gCar.motion.rear_right_pwm = speed;
  gCar.motion.right_pwm = speed;
}

void Motor_Init(void)
{
  Motor_HardwareInit();
  s_left_applied = 0;
  s_right_applied = 0;
  s_last_ramp_tick = HAL_GetTick();
  s_line_sharp_phase = LINE_SHARP_IDLE;
  s_line_sharp_direction = 0;
  s_line_sharp_turn_boost = 0;
  s_line_sharp_tick = HAL_GetTick();
  Motor_Stop();
}

void Motor_SetTarget(int16_t left_speed, int16_t right_speed)
{
  Motor_SetFourTarget(left_speed, left_speed, right_speed, right_speed);
}

void Motor_SetFourTarget(int16_t front_left_speed, int16_t rear_left_speed,
                         int16_t front_right_speed, int16_t rear_right_speed)
{
  gCar.motion.front_left_set_speed = front_left_speed;
  gCar.motion.rear_left_set_speed = rear_left_speed;
  gCar.motion.front_right_set_speed = front_right_speed;
  gCar.motion.rear_right_set_speed = rear_right_speed;
  gCar.motion.left_set_speed = (int16_t)(((int32_t)front_left_speed + rear_left_speed) / 2);
  gCar.motion.right_set_speed = (int16_t)(((int32_t)front_right_speed + rear_right_speed) / 2);
}

void Motor_Stop(void)
{
  gCar.motion.left_set_speed = 0;
  gCar.motion.right_set_speed = 0;
  gCar.motion.front_left_set_speed = 0;
  gCar.motion.rear_left_set_speed = 0;
  gCar.motion.front_right_set_speed = 0;
  gCar.motion.rear_right_set_speed = 0;
  s_left_applied = 0;
  s_right_applied = 0;
  Motor_ApplyLeft(0);
  Motor_ApplyRight(0);
}

static void ResetLineSharpManeuver(void)
{
  s_line_sharp_phase = LINE_SHARP_IDLE;
  s_line_sharp_direction = 0;
  s_line_sharp_turn_boost = 0;
  s_line_sharp_tick = HAL_GetTick();
}

static void Motor_ApplyImmediateTargets(int16_t left_speed,
                                        int16_t right_speed)
{
  Motor_SetTarget(left_speed, right_speed);
  s_left_applied = left_speed;
  s_right_applied = right_speed;
  Motor_ApplyLeft(s_left_applied);
  Motor_ApplyRight(s_right_applied);
  s_last_ramp_tick = HAL_GetTick();
}

/* Bluetooth L/R and automatic sharp turns share the same base targets.
 * Automatic sharp-turn retries may add boost. direction < 0: left turn;
 * direction > 0: right turn. */
static void Motor_GetInPlaceTurnTargets(int8_t direction, int16_t boost,
                                        int16_t *left_speed,
                                        int16_t *right_speed)
{
  int16_t left_magnitude = ClampSpeed(
      (int16_t)(MOTOR_TURN_LEFT_WHEEL_SPEED + boost));
  int16_t right_magnitude = ClampSpeed(
      (int16_t)(MOTOR_TURN_RIGHT_WHEEL_SPEED + boost));

  if (direction > 0)
  {
    *left_speed = left_magnitude;
    *right_speed = (int16_t)(-right_magnitude);
  }
  else
  {
    *left_speed = (int16_t)(-left_magnitude);
    *right_speed = right_magnitude;
  }
}

static void Motor_GetLineSearchTargets(int8_t direction,
                                       int16_t *left_speed,
                                       int16_t *right_speed)
{
  if (direction > 0)
  {
    *left_speed = MOTOR_LINE_SEARCH_LEFT_SPEED;
    *right_speed = (int16_t)(-MOTOR_LINE_SEARCH_RIGHT_SPEED);
  }
  else
  {
    *left_speed = (int16_t)(-MOTOR_LINE_SEARCH_LEFT_SPEED);
    *right_speed = MOTOR_LINE_SEARCH_RIGHT_SPEED;
  }
}

static void RunLineSharpManeuver(int16_t correction)
{
  uint32_t now = HAL_GetTick();
  uint32_t elapsed;
  int8_t direction = (correction > 0) ? 1 : -1;
  int16_t left_turn_speed;
  int16_t right_turn_speed;

  if ((s_line_sharp_phase == LINE_SHARP_IDLE) ||
      (direction != s_line_sharp_direction))
  {
    s_line_sharp_direction = direction;
    s_line_sharp_turn_boost = 0;
    s_line_sharp_phase = LINE_SHARP_BRAKE_BEFORE_TURN;
    s_line_sharp_tick = now;
    Motor_Stop();
    return;
  }

  Motor_GetInPlaceTurnTargets(direction, s_line_sharp_turn_boost,
                              &left_turn_speed,
                              &right_turn_speed);
  elapsed = now - s_line_sharp_tick;

  switch (s_line_sharp_phase)
  {
    case LINE_SHARP_BRAKE_BEFORE_TURN:
      Motor_Stop();
      if (elapsed >= CAR_LINE_SHARP_BRAKE_MS)
      {
        s_line_sharp_phase = LINE_SHARP_TURN;
        s_line_sharp_tick = now;
        Motor_ApplyImmediateTargets(left_turn_speed, right_turn_speed);
      }
      break;

    case LINE_SHARP_TURN:
      if (elapsed >= CAR_LINE_SHARP_TURN_MS)
      {
        s_line_sharp_phase = LINE_SHARP_BRAKE_BEFORE_FORWARD;
        s_line_sharp_tick = now;
        Motor_Stop();
      }
      else
      {
        Motor_SetTarget(left_turn_speed, right_turn_speed);
      }
      break;

    case LINE_SHARP_BRAKE_BEFORE_FORWARD:
      Motor_Stop();
      if (elapsed >= CAR_LINE_SHARP_BRAKE_MS)
      {
        s_line_sharp_phase = LINE_SHARP_FORWARD;
        s_line_sharp_tick = now;
        Motor_ApplyImmediateTargets(CAR_LINE_SHARP_FORWARD_LEFT_SPEED,
                                    CAR_LINE_SHARP_FORWARD_RIGHT_SPEED);
      }
      break;

    case LINE_SHARP_FORWARD:
      if (elapsed >= CAR_LINE_SHARP_FORWARD_MS)
      {
        int16_t lower_base_speed =
            (MOTOR_TURN_LEFT_WHEEL_SPEED < MOTOR_TURN_RIGHT_WHEEL_SPEED) ?
            MOTOR_TURN_LEFT_WHEEL_SPEED : MOTOR_TURN_RIGHT_WHEEL_SPEED;
        int16_t max_boost =
            (int16_t)(MOTOR_SPEED_MAX - lower_base_speed);

        if (s_line_sharp_turn_boost < max_boost)
        {
          s_line_sharp_turn_boost = (int16_t)(
              s_line_sharp_turn_boost + CAR_LINE_SHARP_TURN_RETRY_STEP);
          if (s_line_sharp_turn_boost > max_boost)
            s_line_sharp_turn_boost = max_boost;
        }
        s_line_sharp_phase = LINE_SHARP_BRAKE_BEFORE_TURN;
        s_line_sharp_tick = now;
        Motor_Stop();
      }
      else
      {
        Motor_SetTarget(CAR_LINE_SHARP_FORWARD_LEFT_SPEED,
                        CAR_LINE_SHARP_FORWARD_RIGHT_SPEED);
      }
      break;

    case LINE_SHARP_IDLE:
    default:
      ResetLineSharpManeuver();
      Motor_Stop();
      break;
  }
}

void Motor_Update(void)
{
  if ((gCar.sys.action != CAR_ACTION_LINE_FOLLOW) &&
      (s_line_sharp_phase != LINE_SHARP_IDLE))
  {
    ResetLineSharpManeuver();
  }

  switch (gCar.sys.action)
  {
    case CAR_ACTION_FORWARD:
      Motor_SetTarget(MOTOR_FORWARD_LEFT_SPEED,
                      MOTOR_FORWARD_RIGHT_SPEED);
      break;
    case CAR_ACTION_BACKWARD:
      if (gCar.sys.stage == CAR_STAGE_UNLOAD_ALIGN)
      {
        Motor_SetTarget(-MOTOR_LOAD_APPROACH_LEFT_SPEED,
                        -MOTOR_LOAD_APPROACH_RIGHT_SPEED);
      }
      else
      {
        Motor_SetTarget(-MOTOR_FORWARD_LEFT_SPEED,
                        -MOTOR_FORWARD_RIGHT_SPEED);
      }
      break;
    case CAR_ACTION_SLOW_FORWARD:
      if ((gCar.sys.stage == CAR_STAGE_LOAD_ALIGN) ||
          (gCar.sys.stage == CAR_STAGE_UNLOAD_ALIGN) ||
          (gCar.sys.stage == CAR_STAGE_UNLOAD_SEARCH))
      {
        Motor_SetTarget(MOTOR_LOAD_APPROACH_LEFT_SPEED,
                        MOTOR_LOAD_APPROACH_RIGHT_SPEED);
      }
      else
      {
        Motor_SetTarget(MOTOR_SLOW_LEFT_SPEED,
                        MOTOR_SLOW_RIGHT_SPEED);
      }
      break;
    case CAR_ACTION_TURN_LEFT:
    case CAR_ACTION_RESTORE_HEADING:
    {
      int16_t left_speed;
      int16_t right_speed;
      Motor_GetInPlaceTurnTargets(-1, 0, &left_speed, &right_speed);
      Motor_SetTarget(left_speed, right_speed);
      break;
    }
    case CAR_ACTION_TURN_RIGHT:
    {
      int16_t left_speed;
      int16_t right_speed;
      Motor_GetInPlaceTurnTargets(1, 0, &left_speed, &right_speed);
      Motor_SetTarget(left_speed, right_speed);
      break;
    }
    case CAR_ACTION_LINE_FOLLOW:
    {
      int16_t correction = gCar.motion.line_correction;
      int16_t left_speed;
      int16_t right_speed;

      if ((correction >= CAR_LINE_PIVOT_CORRECTION) ||
          (correction <= -CAR_LINE_PIVOT_CORRECTION))
      {
        RunLineSharpManeuver(correction);
        break;
      }

      ResetLineSharpManeuver();
      if (correction > 0)
      {
        left_speed = ClampLineForwardSpeed(
            (int32_t)MOTOR_LINE_LEFT_BASE_SPEED +
            ((int32_t)correction * MOTOR_LINE_LEFT_OUTER_GAIN_NUM) /
            MOTOR_LINE_LEFT_OUTER_GAIN_DEN);
      }
      else
      {
        left_speed = ClampLineForwardSpeed(
            (int32_t)MOTOR_LINE_LEFT_BASE_SPEED +
            ((int32_t)correction * MOTOR_LINE_LEFT_INNER_GAIN_NUM) /
            MOTOR_LINE_LEFT_INNER_GAIN_DEN);
      }
      right_speed = ClampLineForwardSpeed(
          (int32_t)MOTOR_LINE_RIGHT_BASE_SPEED - correction);

      /* Positive correction means the line is to the image right: make the
       * left wheels faster and the right wheels slower. Ordinary curves use
       * continuous differential steering. The weak left drivetrain gets an
       * asymmetric correction gain. Sharp curves continuously pivot above. */
      Motor_SetTarget(left_speed, right_speed);
      break;
    }
    case CAR_ACTION_SEARCH_LINE:
      if (gCar.motion.line_correction > 0)
      {
        int16_t left_speed;
        int16_t right_speed;
        Motor_GetLineSearchTargets(1, &left_speed, &right_speed);
        Motor_SetTarget(left_speed, right_speed);
      }
      else if (gCar.motion.line_correction < 0)
      {
        int16_t left_speed;
        int16_t right_speed;
        Motor_GetLineSearchTargets(-1, &left_speed, &right_speed);
        Motor_SetTarget(left_speed, right_speed);
      }
      else
      {
        Motor_Stop();
        return;
      }
      break;
    case CAR_ACTION_STOP:
    case CAR_ACTION_SCAN_TARGET:
    case CAR_ACTION_ALIGN_TARGET:
    case CAR_ACTION_LOAD_CARGO:
    case CAR_ACTION_UNLOAD_CARGO:
    case CAR_ACTION_MISSION_DONE:
    case CAR_ACTION_SENSOR_ERROR:
    default:
      Motor_Stop();
      return;
  }

  if ((HAL_GetTick() - s_last_ramp_tick) >= MOTOR_RAMP_INTERVAL_MS)
  {
    s_last_ramp_tick = HAL_GetTick();
    s_left_applied = ApproachSpeed(s_left_applied, gCar.motion.left_set_speed);
    s_right_applied = ApproachSpeed(s_right_applied, gCar.motion.right_set_speed);
    Motor_ApplyLeft(s_left_applied);
    Motor_ApplyRight(s_right_applied);
  }
}
