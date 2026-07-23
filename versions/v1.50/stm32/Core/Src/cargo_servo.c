#include "cargo_servo.h"
#include "main.h"
#include "car_config.h"
#include "car_data.h"
#include "servo_pwm.h"

typedef enum
{
  CARGO_OPERATION_NONE = 0,
  CARGO_OPERATION_LOAD,
  CARGO_OPERATION_UNLOAD
} CargoOperation_t;

static CargoOperation_t s_operation;
static uint32_t s_operation_tick;
static int16_t s_left_deg;
static int16_t s_right_deg;

static int16_t ClampAngle(int16_t angle, int16_t min_angle, int16_t max_angle)
{
  if (angle < min_angle) return min_angle;
  if (angle > max_angle) return max_angle;
  return angle;
}

static void SetLeftAngle(int16_t angle_deg)
{
  s_left_deg = ClampAngle(angle_deg, CARGO_LEFT_MIN_DEG, CARGO_LEFT_MAX_DEG);
  ServoPwm_SetScaledAngle(SERVO_CHANNEL_CARGO_LEFT, s_left_deg,
                          CARGO_LEFT_MAX_DEG);
}

static void SetRightAngle(int16_t angle_deg)
{
  s_right_deg = ClampAngle(angle_deg, CARGO_RIGHT_MIN_DEG, CARGO_RIGHT_MAX_DEG);
  ServoPwm_SetAbsoluteAngle(SERVO_CHANNEL_CARGO_RIGHT, s_right_deg);
}

void CargoServo_Open(void)
{
  SetLeftAngle(CARGO_LEFT_OPEN_DEG);
  SetRightAngle(CARGO_RIGHT_OPEN_DEG);
}

void CargoServo_Close(void)
{
  SetLeftAngle(CARGO_LEFT_CLOSED_DEG);
  SetRightAngle(CARGO_RIGHT_CLOSED_DEG);
}

void CargoServo_AdjustLeft(int16_t delta_deg)
{
  SetLeftAngle((int16_t)(s_left_deg + delta_deg));
}

void CargoServo_AdjustRight(int16_t delta_deg)
{
  SetRightAngle((int16_t)(s_right_deg + delta_deg));
}

void CargoServo_Init(void)
{
  gCar.cargo.busy = 0U;
  gCar.cargo.loaded = 0U;
  s_operation = CARGO_OPERATION_NONE;
  s_operation_tick = HAL_GetTick();
  s_left_deg = CARGO_LEFT_OPEN_DEG;
  s_right_deg = CARGO_RIGHT_OPEN_DEG;
  CargoServo_Open();
}

void CargoServo_Load(void)
{
  if (s_operation != CARGO_OPERATION_LOAD)
  {
    s_operation = CARGO_OPERATION_LOAD;
    s_operation_tick = HAL_GetTick();
    gCar.cargo.busy = 1U;
    CargoServo_Close();
  }

  if ((HAL_GetTick() - s_operation_tick) >= 500U)
  {
    gCar.cargo.loaded = 1U;
  }
}

void CargoServo_Unload(void)
{
  if (s_operation != CARGO_OPERATION_UNLOAD)
  {
    s_operation = CARGO_OPERATION_UNLOAD;
    s_operation_tick = HAL_GetTick();
    gCar.cargo.busy = 1U;
    CargoServo_Open();
  }

  if ((HAL_GetTick() - s_operation_tick) >= 500U)
  {
    gCar.cargo.loaded = 0U;
  }
}

void CargoServo_Update(void)
{
  if (gCar.sys.action == CAR_ACTION_LOAD_CARGO)
  {
    CargoServo_Load();
  }
  else if (gCar.sys.action == CAR_ACTION_UNLOAD_CARGO)
  {
    CargoServo_Unload();
  }
  else
  {
    gCar.cargo.busy = 0U;
    s_operation = CARGO_OPERATION_NONE;
  }
}
