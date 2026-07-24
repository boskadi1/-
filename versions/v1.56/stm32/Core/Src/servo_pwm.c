#include "servo_pwm.h"
#include "main.h"
#include "car_config.h"

static uint16_t ClampPulse(uint32_t pulse_us)
{
  if (pulse_us < SERVO_MIN_PULSE_US) return SERVO_MIN_PULSE_US;
  if (pulse_us > SERVO_MAX_PULSE_US) return SERVO_MAX_PULSE_US;
  return (uint16_t)pulse_us;
}

void ServoPwm_Init(void)
{
  uint32_t timer_clock_hz = HAL_RCC_GetPCLK2Freq();
  uint32_t prescaler;

  if ((RCC->CFGR & RCC_CFGR_PPRE2) != 0U)
  {
    timer_clock_hz *= 2U;
  }

  prescaler = timer_clock_hz / 1000000U;
  if (prescaler == 0U) prescaler = 1U;

  __HAL_RCC_TIM1_CLK_ENABLE();
  TIM1->CR1 = 0U;
  TIM1->PSC = prescaler - 1U;
  TIM1->ARR = SERVO_FRAME_US - 1U;
  TIM1->CCR1 = SERVO_CENTER_PULSE_US;
  TIM1->CCR2 = SERVO_CENTER_PULSE_US;
  TIM1->CCR3 = SERVO_CENTER_PULSE_US;
  TIM1->CCR4 = SERVO_CENTER_PULSE_US;
  TIM1->CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 |
                 TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
  TIM1->CCMR2 = TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 |
                 TIM_CCMR2_OC4PE | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
  TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
  TIM1->BDTR = TIM_BDTR_MOE;
  TIM1->EGR = TIM_EGR_UG;
  TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

void ServoPwm_SetPulseUs(ServoChannel_t channel, uint16_t pulse_us)
{
  uint16_t pulse = ClampPulse(pulse_us);

  switch (channel)
  {
    case SERVO_CHANNEL_GRAB_CLAW: TIM1->CCR1 = pulse; break;
    case SERVO_CHANNEL_GRAB_LIFT: TIM1->CCR2 = pulse; break;
    case SERVO_CHANNEL_CAMERA_PAN: TIM1->CCR3 = pulse; break;
    case SERVO_CHANNEL_CAMERA_TILT: TIM1->CCR4 = pulse; break;
    default: break;
  }
}

void ServoPwm_SetScaledAngle(ServoChannel_t channel, int16_t angle_deg,
                             int16_t full_scale_deg)
{
  uint32_t pulse;

  if (full_scale_deg <= 0) return;
  if (angle_deg < 0) angle_deg = 0;
  if (angle_deg > full_scale_deg) angle_deg = full_scale_deg;
  pulse = SERVO_MIN_PULSE_US +
          ((uint32_t)angle_deg * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) /
          (uint32_t)full_scale_deg;
  ServoPwm_SetPulseUs(channel, (uint16_t)pulse);
}

void ServoPwm_SetAbsoluteAngle(ServoChannel_t channel, int16_t angle_deg)
{
  ServoPwm_SetScaledAngle(channel, angle_deg, 180);
}

void ServoPwm_SetCenteredAngle(ServoChannel_t channel, int16_t angle_deg)
{
  int32_t pulse = (int32_t)SERVO_CENTER_PULSE_US +
                  (int32_t)angle_deg * SERVO_US_PER_DEG;
  ServoPwm_SetPulseUs(channel, ClampPulse((pulse < 0) ? 0U : (uint32_t)pulse));
}
