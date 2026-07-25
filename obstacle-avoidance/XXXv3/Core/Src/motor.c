#include "motor.h"
#include "car_config.h"
#include "car_data.h"
#include <math.h>

static float s_left_speed_i;
static float s_right_speed_i;
static int8_t s_left_command_sign;
static int8_t s_right_command_sign;
static uint32_t s_control_tick;
static uint8_t s_brake_active;
static float s_left_command_quant_error;
static float s_right_command_quant_error;

static int16_t ClampSpeed(int16_t speed)
{
  if (speed > MOTOR_SPEED_MAX) return MOTOR_SPEED_MAX;
  if (speed < -MOTOR_SPEED_MAX) return -MOTOR_SPEED_MAX;
  return speed;
}

static int16_t ApplyMotorDeadband(int16_t speed, int16_t minimum_speed)
{
  speed = ClampSpeed(speed);
  if ((speed > 0) && (speed < minimum_speed)) return minimum_speed;
  if ((speed < 0) && (speed > -minimum_speed)) return -minimum_speed;
  return speed;
}

static int16_t ClampSideSpeed(int16_t speed, int16_t maximum_speed)
{
  if (speed > maximum_speed) return maximum_speed;
  if (speed < -maximum_speed) return (int16_t)(-maximum_speed);
  return speed;
}

static float ClampFloat(float value, float low, float high)
{
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

static int16_t RoundToInt16(float value)
{
  value = ClampFloat(value, -32768.0f, 32767.0f);
  return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static int16_t QuantizeSpeedCommand(float speed, float *quant_error)
{
  float adjusted;
  int16_t command;

  speed = ClampFloat(speed, -(float)MOTOR_SPEED_MAX,
                     (float)MOTOR_SPEED_MAX);
  if (fabsf(speed) < 0.001f)
  {
    *quant_error = 0.0f;
    return 0;
  }

  /* Error diffusion makes 97.5 alternate between 98 and 97. Integer
     commands remain unchanged because their accumulated error is zero. */
  adjusted = speed + *quant_error;
  command = RoundToInt16(adjusted);
  *quant_error = adjusted - (float)command;
  return command;
}

static int8_t CommandSign(int16_t command)
{
  if (command > 0) return 1;
  if (command < 0) return -1;
  return 0;
}

static int16_t ClosedLoopSideOutput(int16_t command,
                                    float base_command,
                                    int16_t minimum_command,
                                    int16_t maximum_command,
                                    int32_t measured_mm_s,
                                    float dt_s,
                                    float *integral_pwm,
                                    int8_t *last_sign,
                                    int16_t *target_mm_s)
{
  const int16_t feedforward = ClampSideSpeed(
      ApplyMotorDeadband(command, minimum_command), maximum_command);
  const int8_t sign = CommandSign(feedforward);
  float target;
  float error;
  float candidate_i;
  float raw_output;
  float limited_output;
  uint8_t feedback_usable = gCar.encoder.speed_valid;

  if (sign == 0)
  {
    *integral_pwm = 0.0f;
    *last_sign = 0;
    *target_mm_s = 0;
    return 0;
  }

  if (sign != *last_sign)
  {
    *integral_pwm = 0.0f;
    *last_sign = sign;
  }

  target = ((float)feedforward / (float)base_command) *
           MOTOR_CRUISE_TARGET_MM_S;
  *target_mm_s = RoundToInt16(target);

  /* Wrong encoder polarity must not make the PI drive toward 100% PWM.
     Fall back to the known feedforward until the individual sign is fixed. */
  if (((target > 0.0f) &&
       ((float)measured_mm_s < -MOTOR_ENCODER_REVERSE_GUARD_MM_S)) ||
      ((target < 0.0f) &&
       ((float)measured_mm_s > MOTOR_ENCODER_REVERSE_GUARD_MM_S)))
  {
    feedback_usable = 0U;
  }

  if (feedback_usable == 0U)
  {
    *integral_pwm = 0.0f;
    return feedforward;
  }

  error = target - (float)measured_mm_s;
  candidate_i = ClampFloat(*integral_pwm + MOTOR_SPEED_PI_KI * error * dt_s,
                           -MOTOR_SPEED_PI_I_LIMIT,
                           MOTOR_SPEED_PI_I_LIMIT);
  raw_output = (float)feedforward + MOTOR_SPEED_PI_KP * error + candidate_i;

  if (sign > 0)
  {
    limited_output = ClampFloat(raw_output,
                                (float)minimum_command,
                                (float)maximum_command);
  }
  else
  {
    limited_output = ClampFloat(raw_output,
                                -(float)maximum_command,
                                -(float)minimum_command);
  }

  /* Conditional integration is the anti-windup path. */
  if (limited_output == raw_output)
  {
    *integral_pwm = candidate_i;
  }

  return RoundToInt16(limited_output);
}

static uint32_t SpeedToCompare(int16_t speed)
{
  uint32_t magnitude = (speed < 0) ? (uint32_t)(-speed) : (uint32_t)speed;
  return (magnitude * MOTOR_PWM_PERIOD_COUNTS) / MOTOR_SPEED_MAX;
}

static void Motor_HardwareInit(void)
{
  uint32_t timer_clock_hz = HAL_RCC_GetPCLK1Freq();
  uint32_t prescaler;
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, MOTOR_LEFT_IN3_PIN | MOTOR_LEFT_IN4_PIN |
                           MOTOR_RIGHT_IN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, MOTOR_RIGHT_IN2_PIN, GPIO_PIN_RESET);

  gpio.Pin = MOTOR_LEFT_IN3_PIN | MOTOR_LEFT_IN4_PIN | MOTOR_RIGHT_IN1_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);

  gpio.Pin = MOTOR_RIGHT_IN2_PIN;
  HAL_GPIO_Init(GPIOC, &gpio);

  gpio.Pin = MOTOR_LEFT_ENB_PIN;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(MOTOR_LEFT_ENB_PORT, &gpio);

  gpio.Pin = MOTOR_RIGHT_ENA_PIN;
  gpio.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(MOTOR_RIGHT_ENA_PORT, &gpio);

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
  speed = ClampSideSpeed(speed, MOTOR_LEFT_SPEED_MAX);
  speed = ApplyMotorDeadband(speed, MOTOR_LEFT_MIN_EFFECTIVE_SPEED);

  if (speed > 0)
  {
    /* Positive software speed must mean physical forward.  The installed
       left channel is opposite to the previous assumption: IN3=1, IN4=0. */
    HAL_GPIO_WritePin(MOTOR_LEFT_IN3_PORT, MOTOR_LEFT_IN3_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_LEFT_IN4_PORT, MOTOR_LEFT_IN4_PIN, GPIO_PIN_RESET);
  }
  else if (speed < 0)
  {
    HAL_GPIO_WritePin(MOTOR_LEFT_IN3_PORT, MOTOR_LEFT_IN3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_LEFT_IN4_PORT, MOTOR_LEFT_IN4_PIN, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(MOTOR_LEFT_IN3_PORT, MOTOR_LEFT_IN3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_LEFT_IN4_PORT, MOTOR_LEFT_IN4_PIN, GPIO_PIN_RESET);
  }

  TIM4->CCR1 = SpeedToCompare(speed);
  gCar.motion.front_left_pwm = speed;
  gCar.motion.rear_left_pwm = speed;
  gCar.motion.left_pwm = speed;
}

static void Motor_ApplyRight(int16_t speed)
{
  speed = ClampSideSpeed(speed, MOTOR_RIGHT_SPEED_MAX);
  speed = ApplyMotorDeadband(speed, MOTOR_RIGHT_MIN_EFFECTIVE_SPEED);

  if (speed > 0)
  {
    /* The corrected right-side motor wiring runs forward with IN1=0, IN2=1. */
    HAL_GPIO_WritePin(MOTOR_RIGHT_IN1_PORT, MOTOR_RIGHT_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_RIGHT_IN2_PORT, MOTOR_RIGHT_IN2_PIN, GPIO_PIN_SET);
  }
  else if (speed < 0)
  {
    HAL_GPIO_WritePin(MOTOR_RIGHT_IN1_PORT, MOTOR_RIGHT_IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_RIGHT_IN2_PORT, MOTOR_RIGHT_IN2_PIN, GPIO_PIN_RESET);
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

static void Motor_ApplyBrakeHardware(void)
{
  /* L298N dynamic braking: ENA/ENB remain high while both direction inputs
     on each bridge are equal. Both motor terminals are driven low, so the
     motor's back EMF produces braking torque instead of free coasting. */
  HAL_GPIO_WritePin(MOTOR_LEFT_IN3_PORT, MOTOR_LEFT_IN3_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_LEFT_IN4_PORT, MOTOR_LEFT_IN4_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_RIGHT_IN1_PORT, MOTOR_RIGHT_IN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_RIGHT_IN2_PORT, MOTOR_RIGHT_IN2_PIN, GPIO_PIN_RESET);
  TIM4->CCR1 = MOTOR_PWM_PERIOD_COUNTS;
  TIM3->CCR3 = MOTOR_PWM_PERIOD_COUNTS;

  gCar.motion.front_left_pwm = 0;
  gCar.motion.rear_left_pwm = 0;
  gCar.motion.front_right_pwm = 0;
  gCar.motion.rear_right_pwm = 0;
  gCar.motion.left_pwm = 0;
  gCar.motion.right_pwm = 0;
}

void Motor_Init(void)
{
  Motor_HardwareInit();
  s_left_speed_i = 0.0f;
  s_right_speed_i = 0.0f;
  s_left_command_sign = 0;
  s_right_command_sign = 0;
  s_brake_active = 0U;
  s_left_command_quant_error = 0.0f;
  s_right_command_quant_error = 0.0f;
  s_control_tick = HAL_GetTick() - MOTOR_CONTROL_PERIOD_MS;
  Motor_Stop();
}

void Motor_SetTarget(float left_speed, float right_speed)
{
  const int16_t left_command =
      QuantizeSpeedCommand(left_speed, &s_left_command_quant_error);
  const int16_t right_command =
      QuantizeSpeedCommand(right_speed, &s_right_command_quant_error);

  Motor_SetFourTarget(left_command, left_command,
                      right_command, right_command);
}

void Motor_SetFourTarget(int16_t front_left_speed, int16_t rear_left_speed,
                         int16_t front_right_speed, int16_t rear_right_speed)
{
  s_brake_active = 0U;
  gCar.motion.front_left_set_speed = front_left_speed;
  gCar.motion.rear_left_set_speed = rear_left_speed;
  gCar.motion.front_right_set_speed = front_right_speed;
  gCar.motion.rear_right_set_speed = rear_right_speed;
  gCar.motion.left_set_speed = (int16_t)(((int32_t)front_left_speed + rear_left_speed) / 2);
  gCar.motion.right_set_speed = (int16_t)(((int32_t)front_right_speed + rear_right_speed) / 2);
}

void Motor_Brake(void)
{
  gCar.motion.left_set_speed = 0;
  gCar.motion.right_set_speed = 0;
  gCar.motion.front_left_set_speed = 0;
  gCar.motion.rear_left_set_speed = 0;
  gCar.motion.front_right_set_speed = 0;
  gCar.motion.rear_right_set_speed = 0;
  gCar.motion.left_target_mm_s = 0;
  gCar.motion.right_target_mm_s = 0;
  s_left_speed_i = 0.0f;
  s_right_speed_i = 0.0f;
  s_left_command_sign = 0;
  s_right_command_sign = 0;
  s_left_command_quant_error = 0.0f;
  s_right_command_quant_error = 0.0f;
  s_brake_active = 1U;
  Motor_ApplyBrakeHardware();
}

void Motor_Stop(void)
{
  s_brake_active = 0U;
  gCar.motion.left_set_speed = 0;
  gCar.motion.right_set_speed = 0;
  gCar.motion.front_left_set_speed = 0;
  gCar.motion.rear_left_set_speed = 0;
  gCar.motion.front_right_set_speed = 0;
  gCar.motion.rear_right_set_speed = 0;
  gCar.motion.left_target_mm_s = 0;
  gCar.motion.right_target_mm_s = 0;
  s_left_speed_i = 0.0f;
  s_right_speed_i = 0.0f;
  s_left_command_sign = 0;
  s_right_command_sign = 0;
  s_left_command_quant_error = 0.0f;
  s_right_command_quant_error = 0.0f;
  Motor_ApplyLeft(0);
  Motor_ApplyRight(0);
}

void Motor_Update(void)
{
  uint32_t now;
  uint32_t elapsed_ms;
  float dt_s;
  int16_t left_pwm;
  int16_t right_pwm;

  switch (gCar.sys.action)
  {
    case CAR_ACTION_FORWARD:
      Motor_SetTarget(MOTOR_LEFT_BASE_SPEED, MOTOR_RIGHT_BASE_SPEED);
      break;
    case CAR_ACTION_BACKWARD:
      Motor_SetTarget(-MOTOR_LEFT_BASE_SPEED, -MOTOR_RIGHT_BASE_SPEED);
      break;
    case CAR_ACTION_SLOW_FORWARD:
      Motor_SetTarget(MOTOR_LEFT_MIN_EFFECTIVE_SPEED,
                      MOTOR_RIGHT_MIN_EFFECTIVE_SPEED);
      break;
    case CAR_ACTION_TURN_LEFT:
    case CAR_ACTION_RESTORE_HEADING:
      Motor_SetTarget(-VISION_V2_TURN_SPEED,
                       VISION_V2_TURN_SPEED);
      break;
    case CAR_ACTION_TURN_RIGHT:
      Motor_SetTarget(VISION_V2_TURN_SPEED,
                     -VISION_V2_TURN_SPEED);
      break;
    case CAR_ACTION_SEARCH_LINE:
      Motor_SetTarget(-MOTOR_LEFT_MIN_EFFECTIVE_SPEED,
                       MOTOR_RIGHT_MIN_EFFECTIVE_SPEED);
      break;
    case CAR_ACTION_VISION_DRIVE:
      /* AppState_Update() has already written continuous left/right targets. */
      break;
    case CAR_ACTION_STOP:
    case CAR_ACTION_SCAN_TARGET:
    case CAR_ACTION_ALIGN_TARGET:
    case CAR_ACTION_LOAD_CARGO:
    case CAR_ACTION_UNLOAD_CARGO:
    case CAR_ACTION_MISSION_DONE:
    case CAR_ACTION_SENSOR_ERROR:
    default:
      if (s_brake_active != 0U)
      {
        Motor_ApplyBrakeHardware();
      }
      else
      {
        Motor_Stop();
      }
      return;
  }

  now = HAL_GetTick();
  elapsed_ms = now - s_control_tick;
  if (elapsed_ms < MOTOR_CONTROL_PERIOD_MS)
  {
    return;
  }
  s_control_tick = now;
  dt_s = (float)elapsed_ms / 1000.0f;

  left_pwm = ClosedLoopSideOutput(gCar.motion.left_set_speed,
                                  MOTOR_LEFT_BASE_SPEED,
                                  MOTOR_LEFT_MIN_EFFECTIVE_SPEED,
                                  MOTOR_LEFT_SPEED_MAX,
                                  gCar.encoder.left_speed_mm_s,
                                  dt_s,
                                  &s_left_speed_i,
                                  &s_left_command_sign,
                                  &gCar.motion.left_target_mm_s);
  right_pwm = ClosedLoopSideOutput(gCar.motion.right_set_speed,
                                   MOTOR_RIGHT_BASE_SPEED,
                                   MOTOR_RIGHT_MIN_EFFECTIVE_SPEED,
                                   MOTOR_RIGHT_SPEED_MAX,
                                   gCar.encoder.right_speed_mm_s,
                                   dt_s,
                                   &s_right_speed_i,
                                   &s_right_command_sign,
                                   &gCar.motion.right_target_mm_s);

  /* Closed-loop update only; no artificial acceleration/deceleration ramp. */
  Motor_ApplyLeft(left_pwm);
  Motor_ApplyRight(right_pwm);
}
