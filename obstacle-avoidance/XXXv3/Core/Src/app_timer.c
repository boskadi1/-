#include "app_timer.h"
#include "car_config.h"

static volatile uint32_t s_tick_ms;
static volatile uint8_t s_state_flag;
static volatile uint8_t s_debug_flag;

void AppTimer_Init(void)
{
  s_tick_ms = 0U;
  s_state_flag = 0U;
  s_debug_flag = 0U;
}

void AppTimer_OnSysTick(void)
{
  s_tick_ms++;

  if ((s_tick_ms % CAR_TASK_STATE_MS) == 0U)
  {
    s_state_flag = 1U;
  }

  if ((s_tick_ms % CAR_TASK_DEBUG_MS) == 0U)
  {
    s_debug_flag = 1U;
  }
}

uint8_t AppTimer_ConsumeStateFlag(void)
{
  uint8_t flag = s_state_flag;
  s_state_flag = 0U;
  return flag;
}

uint8_t AppTimer_ConsumeDebugFlag(void)
{
  uint8_t flag = s_debug_flag;
  s_debug_flag = 0U;
  return flag;
}
