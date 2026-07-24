#include "encoder.h"
#include "car_config.h"
#include "car_data.h"
#include "main.h"

static volatile uint32_t s_wheel_pulses[4];
static volatile uint8_t s_wheel_seen[4];
static volatile uint32_t s_left_last_pulse_tick;
static volatile uint32_t s_right_last_pulse_tick;
static uint32_t s_last_sample_tick;

static uint32_t CombineSideCounts(uint32_t first, uint32_t second,
                                  uint8_t first_seen, uint8_t second_seen)
{
  if ((first_seen != 0U) && (second_seen != 0U))
  {
    return (first + second) / 2U;
  }
  if (first_seen != 0U) return first;
  if (second_seen != 0U) return second;
  return 0U;
}

static int16_t CountsToFeedback(uint32_t counts, int16_t target)
{
  int32_t feedback;

  feedback = (int32_t)(counts * 100U) /
             (int32_t)ENCODER_COUNTS_PER_SAMPLE_AT_FULL_SPEED;
  if (feedback > ENCODER_FEEDBACK_MAX_PERCENT)
    feedback = ENCODER_FEEDBACK_MAX_PERCENT;
  if (target < 0) feedback = -feedback;
  return (int16_t)feedback;
}

void Encoder_Init(void)
{
  uint8_t i;

  for (i = 0U; i < 4U; ++i)
  {
    s_wheel_pulses[i] = 0U;
    s_wheel_seen[i] = 0U;
  }
  s_left_last_pulse_tick = 0U;
  s_right_last_pulse_tick = 0U;
  s_last_sample_tick = HAL_GetTick();

  gCar.motion.left_feedback_speed = 0;
  gCar.motion.right_feedback_speed = 0;
  gCar.motion.left_encoder_counts = 0U;
  gCar.motion.right_encoder_counts = 0U;
  gCar.motion.left_encoder_valid = 0U;
  gCar.motion.right_encoder_valid = 0U;
  gCar.motion.encoder_sample_tick = s_last_sample_tick;

  /* PF0..PF7 are already configured as rising-edge EXTI inputs in main.c. */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 6U, 0U);
  HAL_NVIC_SetPriority(EXTI1_IRQn, 6U, 0U);
  HAL_NVIC_SetPriority(EXTI2_IRQn, 6U, 0U);
  HAL_NVIC_SetPriority(EXTI3_IRQn, 6U, 0U);
  HAL_NVIC_SetPriority(EXTI4_IRQn, 6U, 0U);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 6U, 0U);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void Encoder_Update(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t pulses[4];
  uint32_t left_counts;
  uint32_t right_counts;
  uint32_t primask;
  uint8_t i;

  if ((now - s_last_sample_tick) < ENCODER_SAMPLE_MS) return;
  s_last_sample_tick = now;

  primask = __get_PRIMASK();
  __disable_irq();
  for (i = 0U; i < 4U; ++i)
  {
    pulses[i] = s_wheel_pulses[i];
    s_wheel_pulses[i] = 0U;
  }
  if (primask == 0U) __enable_irq();

  left_counts = CombineSideCounts(pulses[0], pulses[1],
                                   s_wheel_seen[0], s_wheel_seen[1]);
  right_counts = CombineSideCounts(pulses[2], pulses[3],
                                    s_wheel_seen[2], s_wheel_seen[3]);
  if (left_counts > 65535U) left_counts = 65535U;
  if (right_counts > 65535U) right_counts = 65535U;

  gCar.motion.left_encoder_counts = (uint16_t)left_counts;
  gCar.motion.right_encoder_counts = (uint16_t)right_counts;
  gCar.motion.left_feedback_speed =
      CountsToFeedback(left_counts, gCar.motion.left_set_speed);
  gCar.motion.right_feedback_speed =
      CountsToFeedback(right_counts, gCar.motion.right_set_speed);
  gCar.motion.left_encoder_valid =
      ((s_left_last_pulse_tick != 0U) &&
       ((now - s_left_last_pulse_tick) <=
        ENCODER_SIGNAL_TIMEOUT_MS)) ? 1U : 0U;
  gCar.motion.right_encoder_valid =
      ((s_right_last_pulse_tick != 0U) &&
       ((now - s_right_last_pulse_tick) <=
        ENCODER_SIGNAL_TIMEOUT_MS)) ? 1U : 0U;
  gCar.motion.encoder_sample_tick = now;
}

void Encoder_GpioExtiCallback(uint16_t gpio_pin)
{
  uint32_t now = HAL_GetTick();

  if ((gpio_pin == ENCODER_FRONT_LEFT_A_PIN) ||
      (gpio_pin == ENCODER_FRONT_LEFT_B_PIN))
  {
    ++s_wheel_pulses[0];
    s_wheel_seen[0] = 1U;
    s_left_last_pulse_tick = now;
  }
  else if ((gpio_pin == ENCODER_REAR_LEFT_A_PIN) ||
           (gpio_pin == ENCODER_REAR_LEFT_B_PIN))
  {
    ++s_wheel_pulses[1];
    s_wheel_seen[1] = 1U;
    s_left_last_pulse_tick = now;
  }
  else if ((gpio_pin == ENCODER_FRONT_RIGHT_A_PIN) ||
           (gpio_pin == ENCODER_FRONT_RIGHT_B_PIN))
  {
    ++s_wheel_pulses[2];
    s_wheel_seen[2] = 1U;
    s_right_last_pulse_tick = now;
  }
  else if ((gpio_pin == ENCODER_REAR_RIGHT_A_PIN) ||
           (gpio_pin == ENCODER_REAR_RIGHT_B_PIN))
  {
    ++s_wheel_pulses[3];
    s_wheel_seen[3] = 1U;
    s_right_last_pulse_tick = now;
  }
}
