#include "ultrasonic.h"
#include "main.h"
#include "car_config.h"
#include "car_data.h"

typedef struct
{
  GPIO_TypeDef *trig_port;
  uint16_t trig_pin;
  GPIO_TypeDef *echo_port;
  uint16_t echo_pin;
} UltrasonicHw_t;

static uint8_t s_measure_index;
static uint32_t s_last_trigger_tick;

static const UltrasonicHw_t s_hw[4] =
{
  {US_FL_TRIG_PORT, US_FL_TRIG_PIN, US_FL_ECHO_PORT, US_FL_ECHO_PIN},
  {US_FR_TRIG_PORT, US_FR_TRIG_PIN, US_FR_ECHO_PORT, US_FR_ECHO_PIN},
  {US_LEFT_TRIG_PORT, US_LEFT_TRIG_PIN, US_LEFT_ECHO_PORT, US_LEFT_ECHO_PIN},
  {US_RIGHT_TRIG_PORT, US_RIGHT_TRIG_PIN, US_RIGHT_ECHO_PORT, US_RIGHT_ECHO_PIN}
};

static void DwtDelayUs(uint32_t us)
{
  uint32_t cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000U;
  uint32_t start = DWT->CYCCNT;
  uint32_t target = us * cycles_per_us;

  while ((DWT->CYCCNT - start) < target)
  {
  }
}

static uint16_t MeasureOne(const UltrasonicHw_t *hw)
{
  const uint32_t cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000U;
  uint32_t wait_start;
  uint32_t pulse_start;
  uint32_t pulse_width_us;

  HAL_GPIO_WritePin(hw->trig_port, hw->trig_pin, GPIO_PIN_RESET);
  DwtDelayUs(2U);
  HAL_GPIO_WritePin(hw->trig_port, hw->trig_pin, GPIO_PIN_SET);
  DwtDelayUs(10U);
  HAL_GPIO_WritePin(hw->trig_port, hw->trig_pin, GPIO_PIN_RESET);

  wait_start = DWT->CYCCNT;
  while (HAL_GPIO_ReadPin(hw->echo_port, hw->echo_pin) == GPIO_PIN_RESET)
  {
    if (((DWT->CYCCNT - wait_start) / cycles_per_us) > US_ECHO_TIMEOUT_US)
    {
      return US_INVALID_CM;
    }
  }

  pulse_start = DWT->CYCCNT;
  while (HAL_GPIO_ReadPin(hw->echo_port, hw->echo_pin) == GPIO_PIN_SET)
  {
    if (((DWT->CYCCNT - pulse_start) / cycles_per_us) > US_ECHO_TIMEOUT_US)
    {
      return US_INVALID_CM;
    }
  }

  pulse_width_us = (DWT->CYCCNT - pulse_start) / cycles_per_us;
  return (uint16_t)(pulse_width_us / 58U);
}

static void StoreMeasure(uint8_t index, uint16_t value)
{
  uint32_t now = HAL_GetTick();
  uint8_t valid = (value == US_INVALID_CM) ? 0U : 1U;

  switch (index)
  {
    case 0:
      gCar.us.front_left_cm = value;
      gCar.us.front_left_valid = valid;
      gCar.us.front_left_tick = now;
      break;
    case 1:
      gCar.us.front_right_cm = value;
      gCar.us.front_right_valid = valid;
      gCar.us.front_right_tick = now;
      break;
    case 2:
      gCar.us.left_cm = value;
      gCar.us.left_valid = valid;
      gCar.us.left_tick = now;
      break;
    case 3:
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
  uint32_t now = HAL_GetTick();

  if ((now - gCar.us.front_left_tick) > US_TIMEOUT_MS) gCar.us.front_left_valid = 0U;
  if ((now - gCar.us.front_right_tick) > US_TIMEOUT_MS) gCar.us.front_right_valid = 0U;
  if ((now - gCar.us.left_tick) > US_TIMEOUT_MS) gCar.us.left_valid = 0U;
  if ((now - gCar.us.right_tick) > US_TIMEOUT_MS) gCar.us.right_valid = 0U;
}

void Ultrasonic_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  s_measure_index = 0U;
  s_last_trigger_tick = HAL_GetTick();
}

void Ultrasonic_MeasureFrontLeft(void)
{
  StoreMeasure(0U, MeasureOne(&s_hw[0]));
}

void Ultrasonic_MeasureFrontRight(void)
{
  StoreMeasure(1U, MeasureOne(&s_hw[1]));
}

void Ultrasonic_MeasureLeft(void)
{
  StoreMeasure(2U, MeasureOne(&s_hw[2]));
}

void Ultrasonic_MeasureRight(void)
{
  StoreMeasure(3U, MeasureOne(&s_hw[3]));
}

void Ultrasonic_Update(void)
{
  if ((HAL_GetTick() - s_last_trigger_tick) >= US_TRIGGER_INTERVAL_MS)
  {
    s_last_trigger_tick = HAL_GetTick();
    StoreMeasure(s_measure_index, MeasureOne(&s_hw[s_measure_index]));
    s_measure_index = (uint8_t)((s_measure_index + 1U) & 0x03U);
  }

  RefreshTimeoutFlags();
}
