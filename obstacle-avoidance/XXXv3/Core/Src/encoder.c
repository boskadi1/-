#include "encoder.h"
#include "car_config.h"
#include "car_data.h"
#include <math.h>

#define PI_F 3.14159265359f
#define RAD_TO_CENTIDEG 5729.577951f

static volatile int32_t s_fl_count;
static volatile int32_t s_rl_count;
static volatile int32_t s_fr_count;
static volatile int32_t s_rr_count;

static int32_t s_prev_fl;
static int32_t s_prev_rl;
static int32_t s_prev_fr;
static int32_t s_prev_rr;
static int32_t s_speed_prev_fl;
static int32_t s_speed_prev_rl;
static int32_t s_speed_prev_fr;
static int32_t s_speed_prev_rr;
static uint32_t s_speed_sample_tick;
static float s_left_speed_mm_s;
static float s_right_speed_mm_s;
static float s_yaw_cd;
static float s_center_mm;
static float s_path_mm;

static int32_t RoundToInt32(float value)
{
  return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static float NormalizeYawCd(float yaw_cd)
{
  while (yaw_cd > 18000.0f) yaw_cd -= 36000.0f;
  while (yaw_cd < -18000.0f) yaw_cd += 36000.0f;
  return yaw_cd;
}

static int32_t QuadratureStep(GPIO_TypeDef *a_port, uint16_t a_pin,
                              GPIO_TypeDef *b_port, uint16_t b_pin,
                              int32_t sign)
{
  const GPIO_PinState a = HAL_GPIO_ReadPin(a_port, a_pin);
  const GPIO_PinState b = HAL_GPIO_ReadPin(b_port, b_pin);
  return ((a == b) ? 1 : -1) * sign;
}

void Encoder_GpioExtiCallback(uint16_t gpio_pin)
{
  if (gpio_pin == ENCODER_FL_A_PIN)
  {
    s_fl_count += QuadratureStep(ENCODER_FL_A_PORT, ENCODER_FL_A_PIN,
                                 ENCODER_FL_B_PORT, ENCODER_FL_B_PIN,
                                 ENCODER_FL_SIGN);
  }
  else if (gpio_pin == ENCODER_RL_A_PIN)
  {
    s_rl_count += QuadratureStep(ENCODER_RL_A_PORT, ENCODER_RL_A_PIN,
                                 ENCODER_RL_B_PORT, ENCODER_RL_B_PIN,
                                 ENCODER_RL_SIGN);
  }
  else if (gpio_pin == ENCODER_FR_A_PIN)
  {
    s_fr_count += QuadratureStep(ENCODER_FR_A_PORT, ENCODER_FR_A_PIN,
                                 ENCODER_FR_B_PORT, ENCODER_FR_B_PIN,
                                 ENCODER_FR_SIGN);
  }
  else if (gpio_pin == ENCODER_RR_A_PIN)
  {
    s_rr_count += QuadratureStep(ENCODER_RR_A_PORT, ENCODER_RR_A_PIN,
                                 ENCODER_RR_B_PORT, ENCODER_RR_B_PIN,
                                 ENCODER_RR_SIGN);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
  Encoder_GpioExtiCallback(gpio_pin);
}

void Encoder_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOF_CLK_ENABLE();

  gpio.Pin = ENCODER_FL_A_PIN | ENCODER_RL_A_PIN |
             ENCODER_FR_A_PIN | ENCODER_RR_A_PIN;
  gpio.Mode = GPIO_MODE_IT_RISING_FALLING;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOF, &gpio);

  gpio.Pin = ENCODER_FL_B_PIN | ENCODER_RL_B_PIN |
             ENCODER_FR_B_PIN | ENCODER_RR_B_PIN;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &gpio);

  HAL_NVIC_SetPriority(EXTI0_IRQn, 6U, 0U);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  HAL_NVIC_SetPriority(EXTI2_IRQn, 6U, 0U);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  HAL_NVIC_SetPriority(EXTI4_IRQn, 6U, 0U);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 6U, 0U);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  s_fl_count = 0;
  s_rl_count = 0;
  s_fr_count = 0;
  s_rr_count = 0;
  s_prev_fl = 0;
  s_prev_rl = 0;
  s_prev_fr = 0;
  s_prev_rr = 0;
  s_speed_prev_fl = 0;
  s_speed_prev_rl = 0;
  s_speed_prev_fr = 0;
  s_speed_prev_rr = 0;
  s_speed_sample_tick = HAL_GetTick();
  s_left_speed_mm_s = 0.0f;
  s_right_speed_mm_s = 0.0f;
  s_yaw_cd = 0.0f;
  s_center_mm = 0.0f;
  s_path_mm = 0.0f;
  gCar.encoder.left_speed_mm_s = 0;
  gCar.encoder.right_speed_mm_s = 0;
  gCar.encoder.speed_valid = 0U;
  gCar.encoder.valid = 1U;
}

void Encoder_ResetHeading(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  s_prev_fl = s_fl_count;
  s_prev_rl = s_rl_count;
  s_prev_fr = s_fr_count;
  s_prev_rr = s_rr_count;
  s_speed_prev_fl = s_fl_count;
  s_speed_prev_rl = s_rl_count;
  s_speed_prev_fr = s_fr_count;
  s_speed_prev_rr = s_rr_count;
  if (primask == 0U) __enable_irq();

  s_yaw_cd = 0.0f;
  s_center_mm = 0.0f;
  s_path_mm = 0.0f;
  s_speed_sample_tick = HAL_GetTick();
  s_left_speed_mm_s = 0.0f;
  s_right_speed_mm_s = 0.0f;
  gCar.encoder.yaw_cd = 0;
  gCar.encoder.center_distance_mm = 0;
  gCar.encoder.path_distance_mm = 0U;
  gCar.encoder.left_speed_mm_s = 0;
  gCar.encoder.right_speed_mm_s = 0;
  gCar.encoder.speed_valid = 0U;
}

void Encoder_Update(void)
{
  int32_t fl;
  int32_t rl;
  int32_t fr;
  int32_t rr;
  int32_t dfl;
  int32_t drl;
  int32_t dfr;
  int32_t drr;
  float mm_per_count;
  float left_mm;
  float right_mm;
  uint32_t now;
  uint32_t speed_elapsed_ms;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  fl = s_fl_count;
  rl = s_rl_count;
  fr = s_fr_count;
  rr = s_rr_count;
  if (primask == 0U) __enable_irq();

  dfl = fl - s_prev_fl;
  drl = rl - s_prev_rl;
  dfr = fr - s_prev_fr;
  drr = rr - s_prev_rr;
  s_prev_fl = fl;
  s_prev_rl = rl;
  s_prev_fr = fr;
  s_prev_rr = rr;

  mm_per_count = (PI_F * ENCODER_WHEEL_DIAMETER_MM) /
                 ENCODER_COUNTS_PER_WHEEL_REV;
  left_mm = 0.5f * (float)(dfl + drl) * mm_per_count;
  right_mm = 0.5f * (float)(dfr + drr) * mm_per_count;

  /* Positive yaw is clockwise/right, matching the OpenMV bearing sign. */
  s_yaw_cd += ((left_mm - right_mm) / ENCODER_TRACK_WIDTH_MM) *
              RAD_TO_CENTIDEG;
  s_yaw_cd = NormalizeYawCd(s_yaw_cd);
  s_center_mm += 0.5f * (left_mm + right_mm);
  s_path_mm += 0.5f * (fabsf(left_mm) + fabsf(right_mm));

  gCar.encoder.front_left_count = fl;
  gCar.encoder.rear_left_count = rl;
  gCar.encoder.front_right_count = fr;
  gCar.encoder.rear_right_count = rr;
  gCar.encoder.yaw_cd = RoundToInt32(s_yaw_cd);
  gCar.encoder.center_distance_mm = RoundToInt32(s_center_mm);
  gCar.encoder.path_distance_mm = (uint32_t)RoundToInt32(s_path_mm);

  now = HAL_GetTick();
  speed_elapsed_ms = now - s_speed_sample_tick;
  if (speed_elapsed_ms >= ENCODER_SPEED_SAMPLE_MS)
  {
    const float speed_left_mm =
        0.5f * (float)((fl - s_speed_prev_fl) + (rl - s_speed_prev_rl)) *
        mm_per_count;
    const float speed_right_mm =
        0.5f * (float)((fr - s_speed_prev_fr) + (rr - s_speed_prev_rr)) *
        mm_per_count;
    const float raw_left_mm_s = speed_left_mm * 1000.0f /
                                (float)speed_elapsed_ms;
    const float raw_right_mm_s = speed_right_mm * 1000.0f /
                                 (float)speed_elapsed_ms;

    if (gCar.encoder.speed_valid == 0U)
    {
      s_left_speed_mm_s = raw_left_mm_s;
      s_right_speed_mm_s = raw_right_mm_s;
      gCar.encoder.speed_valid = 1U;
    }
    else
    {
      s_left_speed_mm_s += ENCODER_SPEED_FILTER_ALPHA *
                           (raw_left_mm_s - s_left_speed_mm_s);
      s_right_speed_mm_s += ENCODER_SPEED_FILTER_ALPHA *
                            (raw_right_mm_s - s_right_speed_mm_s);
    }

    gCar.encoder.left_speed_mm_s = RoundToInt32(s_left_speed_mm_s);
    gCar.encoder.right_speed_mm_s = RoundToInt32(s_right_speed_mm_s);
    s_speed_prev_fl = fl;
    s_speed_prev_rl = rl;
    s_speed_prev_fr = fr;
    s_speed_prev_rr = rr;
    s_speed_sample_tick = now;
  }
}
