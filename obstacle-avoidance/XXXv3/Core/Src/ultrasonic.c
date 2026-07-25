#include "ultrasonic.h"
#include "main.h"
#include "car_config.h"
#include "car_data.h"

/*
 * Four HC-SR04 modules are triggered one at a time to reduce acoustic
 * cross-talk. Echo waiting is a non-blocking state machine: a disconnected
 * module or a stuck ECHO level can invalidate only that sample and can never
 * stop Bluetooth, motor control or the 20 ms navigation task.
 */

typedef struct
{
  GPIO_TypeDef *trig_port;
  uint16_t trig_pin;
  GPIO_TypeDef *echo_port;
  uint16_t echo_pin;
} UltrasonicHw_t;

typedef enum
{
  US_PHASE_IDLE = 0,
  US_PHASE_WAIT_RISE,
  US_PHASE_WAIT_FALL
} UltrasonicPhase_t;

static const UltrasonicHw_t s_hw[4] =
{
  {US_FL_TRIG_PORT, US_FL_TRIG_PIN, US_FL_ECHO_PORT, US_FL_ECHO_PIN},
  {US_FR_TRIG_PORT, US_FR_TRIG_PIN, US_FR_ECHO_PORT, US_FR_ECHO_PIN},
  {US_LEFT_TRIG_PORT, US_LEFT_TRIG_PIN, US_LEFT_ECHO_PORT, US_LEFT_ECHO_PIN},
  {US_RIGHT_TRIG_PORT, US_RIGHT_TRIG_PIN, US_RIGHT_ECHO_PORT, US_RIGHT_ECHO_PIN}
};

static uint8_t s_measure_index;
static uint32_t s_last_trigger_tick;
static UltrasonicPhase_t s_phase;
static uint32_t s_phase_cycle;
static uint32_t s_phase_tick;
static uint32_t s_pulse_start_cycle;
static uint32_t s_cycles_per_us;

static uint32_t ElapsedUs(uint32_t start_cycle)
{
  if (s_cycles_per_us == 0U) return 0U;
  return (DWT->CYCCNT - start_cycle) / s_cycles_per_us;
}

/*
 * Only the fixed 3/10 us trigger pulse is busy-waited. A loop-count guard
 * guarantees return even if DWT->CYCCNT is unexpectedly stopped.
 */
static void DwtDelayUsBounded(uint32_t us)
{
  const uint32_t start = DWT->CYCCNT;
  const uint32_t target = us * s_cycles_per_us;
  uint32_t guard = target * 4U + 64U;

  while (((DWT->CYCCNT - start) < target) && (guard > 0U))
  {
    guard--;
  }
}

static uint16_t PulseWidthToDistance(uint32_t pulse_width_us)
{
  uint16_t distance_cm = (uint16_t)((pulse_width_us + 29U) / 58U);

  if (distance_cm > 400U) return US_INVALID_CM;
  /* A very close echo is dangerous, not "no obstacle". */
  if (distance_cm < 2U) return 2U;
  return distance_cm;
}

static void StoreMeasure(uint8_t index, uint16_t value)
{
  const uint32_t now = HAL_GetTick();
  const uint8_t valid = (value != US_INVALID_CM) ? 1U : 0U;

  switch (index)
  {
    case 0U:
      gCar.us.front_left_cm = value;
      gCar.us.front_left_valid = valid;
      gCar.us.front_left_tick = now;
      break;
    case 1U:
      gCar.us.front_right_cm = value;
      gCar.us.front_right_valid = valid;
      gCar.us.front_right_tick = now;
      break;
    case 2U:
      gCar.us.left_cm = value;
      gCar.us.left_valid = valid;
      gCar.us.left_tick = now;
      break;
    case 3U:
      gCar.us.right_cm = value;
      gCar.us.right_valid = valid;
      gCar.us.right_tick = now;
      break;
    default:
      break;
  }
}

static void RefreshTimeoutFlags(void)
{
  const uint32_t now = HAL_GetTick();

  if ((now - gCar.us.front_left_tick) > US_TIMEOUT_MS)
    gCar.us.front_left_valid = 0U;
  if ((now - gCar.us.front_right_tick) > US_TIMEOUT_MS)
    gCar.us.front_right_valid = 0U;
  if ((now - gCar.us.left_tick) > US_TIMEOUT_MS)
    gCar.us.left_valid = 0U;
  if ((now - gCar.us.right_tick) > US_TIMEOUT_MS)
    gCar.us.right_valid = 0U;
}

static void CompleteMeasure(uint16_t value)
{
  HAL_GPIO_WritePin(s_hw[s_measure_index].trig_port,
                    s_hw[s_measure_index].trig_pin, GPIO_PIN_RESET);
  StoreMeasure(s_measure_index, value);
  s_measure_index = (uint8_t)((s_measure_index + 1U) & 0x03U);
  s_last_trigger_tick = HAL_GetTick();
  s_phase = US_PHASE_IDLE;
}

static void BeginMeasure(uint8_t index)
{
  const UltrasonicHw_t *hw = &s_hw[index];

  /* Re-enable the cycle counter defensively. The bounded trigger delay and
     HAL tick timeout below still prevent a lockup if it does not advance. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  HAL_GPIO_WritePin(hw->trig_port, hw->trig_pin, GPIO_PIN_RESET);
  DwtDelayUsBounded(3U);
  HAL_GPIO_WritePin(hw->trig_port, hw->trig_pin, GPIO_PIN_SET);
  DwtDelayUsBounded(10U);
  HAL_GPIO_WritePin(hw->trig_port, hw->trig_pin, GPIO_PIN_RESET);

  s_phase_cycle = DWT->CYCCNT;
  s_phase_tick = HAL_GetTick();
  s_phase = US_PHASE_WAIT_RISE;
}

static void RequestMeasure(uint8_t index)
{
  if (s_phase != US_PHASE_IDLE) return;
  s_measure_index = (uint8_t)(index & 0x03U);
  BeginMeasure(s_measure_index);
}

static void ConfigurePins(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOD, US_FL_TRIG_PIN | US_FR_TRIG_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOG, US_LEFT_TRIG_PIN | US_RIGHT_TRIG_PIN, GPIO_PIN_RESET);

  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Pin = US_FL_TRIG_PIN | US_FR_TRIG_PIN;
  HAL_GPIO_Init(GPIOD, &gpio);
  gpio.Pin = US_LEFT_TRIG_PIN | US_RIGHT_TRIG_PIN;
  HAL_GPIO_Init(GPIOG, &gpio);

  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  gpio.Pin = US_FL_ECHO_PIN | US_FR_ECHO_PIN;
  HAL_GPIO_Init(GPIOD, &gpio);
  gpio.Pin = US_LEFT_ECHO_PIN;
  HAL_GPIO_Init(GPIOG, &gpio);
  gpio.Pin = US_RIGHT_ECHO_PIN;
  HAL_GPIO_Init(GPIOC, &gpio);
}

void Ultrasonic_Init(void)
{
  const uint32_t now = HAL_GetTick();

  ConfigurePins();
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  s_cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000U;
  if (s_cycles_per_us == 0U) s_cycles_per_us = 1U;

  gCar.us.front_left_cm = US_INVALID_CM;
  gCar.us.front_right_cm = US_INVALID_CM;
  gCar.us.left_cm = US_INVALID_CM;
  gCar.us.right_cm = US_INVALID_CM;
  gCar.us.front_left_valid = 0U;
  gCar.us.front_right_valid = 0U;
  gCar.us.left_valid = 0U;
  gCar.us.right_valid = 0U;
  gCar.us.front_left_tick = now;
  gCar.us.front_right_tick = now;
  gCar.us.left_tick = now;
  gCar.us.right_tick = now;

  s_measure_index = 0U;
  s_phase = US_PHASE_IDLE;
  s_phase_cycle = DWT->CYCCNT;
  s_phase_tick = now;
  s_pulse_start_cycle = DWT->CYCCNT;
  s_last_trigger_tick = now - US_TRIGGER_INTERVAL_MS;
}

void Ultrasonic_MeasureFrontLeft(void)
{
  RequestMeasure(0U);
}

void Ultrasonic_MeasureFrontRight(void)
{
  RequestMeasure(1U);
}

void Ultrasonic_MeasureLeft(void)
{
  RequestMeasure(2U);
}

void Ultrasonic_MeasureRight(void)
{
  RequestMeasure(3U);
}

void Ultrasonic_Update(void)
{
  const uint32_t now = HAL_GetTick();
  const UltrasonicHw_t *hw = &s_hw[s_measure_index];

  if (s_phase == US_PHASE_IDLE)
  {
    if ((now - s_last_trigger_tick) >= US_TRIGGER_INTERVAL_MS)
    {
      BeginMeasure(s_measure_index);
    }
  }
  else if (s_phase == US_PHASE_WAIT_RISE)
  {
    if (HAL_GPIO_ReadPin(hw->echo_port, hw->echo_pin) == GPIO_PIN_SET)
    {
      s_pulse_start_cycle = DWT->CYCCNT;
      s_phase_cycle = s_pulse_start_cycle;
      s_phase_tick = now;
      s_phase = US_PHASE_WAIT_FALL;
    }
    else if ((ElapsedUs(s_phase_cycle) >= US_ECHO_TIMEOUT_US) ||
             ((now - s_phase_tick) >=
              ((US_ECHO_TIMEOUT_US + 999U) / 1000U + 2U)))
    {
      CompleteMeasure(US_INVALID_CM);
    }
  }
  else
  {
    if (HAL_GPIO_ReadPin(hw->echo_port, hw->echo_pin) == GPIO_PIN_RESET)
    {
      CompleteMeasure(PulseWidthToDistance(
          ElapsedUs(s_pulse_start_cycle)));
    }
    else if ((ElapsedUs(s_phase_cycle) >= US_ECHO_TIMEOUT_US) ||
             ((now - s_phase_tick) >=
              ((US_ECHO_TIMEOUT_US + 999U) / 1000U + 2U)))
    {
      CompleteMeasure(US_INVALID_CM);
    }
  }

  RefreshTimeoutFlags();
}
