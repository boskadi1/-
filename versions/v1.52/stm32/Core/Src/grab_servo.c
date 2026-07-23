#include "grab_servo.h"
#include "car_config.h"
#include "car_data.h"
#include "main.h"
#include "servo_pwm.h"
#include <string.h>

typedef enum
{
  GRAB_STATE_IDLE = 0,
  GRAB_STATE_PICKUP_CLAMP,
  GRAB_STATE_PICKUP_LIFT,
  GRAB_STATE_DROPOFF_LOWER,
  GRAB_STATE_DROPOFF_OPEN
} GrabState_t;

static GrabState_t s_state;
static uint32_t s_state_tick;

static int16_t ClampAngle(int16_t angle, int16_t minimum, int16_t maximum)
{
  if (angle < minimum) return minimum;
  if (angle > maximum) return maximum;
  return angle;
}

static void SetClawAngle(int16_t angle)
{
  angle = ClampAngle(angle, GRAB_CLAW_MIN_DEG, GRAB_CLAW_MAX_DEG);
  ServoPwm_SetCenteredAngle(SERVO_CHANNEL_GRAB_CLAW, angle);
  gCar.grab.claw_deg = angle;
}

static void SetLiftAngle(int16_t angle)
{
  angle = ClampAngle(angle, GRAB_LIFT_MIN_DEG, GRAB_LIFT_MAX_DEG);
  ServoPwm_SetCenteredAngle(SERVO_CHANNEL_GRAB_LIFT, angle);
  gCar.grab.lift_deg = angle;
}

void GrabServo_Init(void)
{
  memset(&gCar.grab, 0, sizeof(gCar.grab));
  s_state = GRAB_STATE_IDLE;
  s_state_tick = HAL_GetTick();

  /* Reference-project power-up sequence: center first, then move to the
   * calibrated initial pose after the servo supply has stabilized. */
  ServoPwm_SetPulseUs(SERVO_CHANNEL_GRAB_CLAW, SERVO_CENTER_PULSE_US);
  ServoPwm_SetPulseUs(SERVO_CHANNEL_GRAB_LIFT, SERVO_CENTER_PULSE_US);
  HAL_Delay(GRAB_POWER_STABILIZE_MS);

  SetClawAngle(GRAB_CLAW_INIT_DEG);
  SetLiftAngle(GRAB_LIFT_INIT_DEG);
  HAL_Delay(GRAB_INITIAL_MOVE_MS);
}

void GrabServo_StartPickup(void)
{
  if ((s_state != GRAB_STATE_IDLE) || (gCar.grab.has_object != 0U)) return;

  s_state = GRAB_STATE_PICKUP_CLAMP;
  s_state_tick = HAL_GetTick();
  gCar.grab.busy = 1U;
  SetClawAngle(GRAB_CLAW_CLOSE_DEG);
}

void GrabServo_StartDropoff(void)
{
  if ((s_state != GRAB_STATE_IDLE) || (gCar.grab.has_object == 0U)) return;

  s_state = GRAB_STATE_DROPOFF_LOWER;
  s_state_tick = HAL_GetTick();
  gCar.grab.busy = 1U;
  SetLiftAngle(GRAB_LIFT_DOWN_DEG);
}

void GrabServo_AdjustClaw(int16_t delta_deg)
{
  if (gCar.grab.busy != 0U) return;
  SetClawAngle((int16_t)(gCar.grab.claw_deg + delta_deg));
}

void GrabServo_AdjustLift(int16_t delta_deg)
{
  if (gCar.grab.busy != 0U) return;
  SetLiftAngle((int16_t)(gCar.grab.lift_deg + delta_deg));
}

uint8_t GrabServo_IsBusy(void)
{
  return gCar.grab.busy;
}

uint8_t GrabServo_HasObject(void)
{
  return gCar.grab.has_object;
}

void GrabServo_Update(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t elapsed;

  if (s_state == GRAB_STATE_IDLE)
  {
    gCar.grab.busy = 0U;
    if ((gCar.sys.action == CAR_ACTION_LOAD_CARGO) &&
        (gCar.grab.has_object == 0U))
    {
      GrabServo_StartPickup();
    }
    else if ((gCar.sys.action == CAR_ACTION_UNLOAD_CARGO) &&
             (gCar.grab.has_object != 0U))
    {
      GrabServo_StartDropoff();
    }
    return;
  }

  elapsed = now - s_state_tick;
  switch (s_state)
  {
    case GRAB_STATE_PICKUP_CLAMP:
      if (elapsed >= GRAB_ACTION_DELAY_MS)
      {
        gCar.grab.has_object = 1U;
        s_state = GRAB_STATE_PICKUP_LIFT;
        s_state_tick = now;
        SetLiftAngle(GRAB_LIFT_UP_DEG);
      }
      break;

    case GRAB_STATE_PICKUP_LIFT:
      if (elapsed >= GRAB_ACTION_DELAY_MS)
      {
        gCar.grab.busy = 0U;
        s_state = GRAB_STATE_IDLE;
      }
      break;

    case GRAB_STATE_DROPOFF_LOWER:
      if (elapsed >= GRAB_ACTION_DELAY_MS)
      {
        s_state = GRAB_STATE_DROPOFF_OPEN;
        s_state_tick = now;
        SetClawAngle(GRAB_CLAW_OPEN_DEG);
      }
      break;

    case GRAB_STATE_DROPOFF_OPEN:
      if (elapsed >= GRAB_ACTION_DELAY_MS)
      {
        gCar.grab.has_object = 0U;
        gCar.grab.busy = 0U;
        s_state = GRAB_STATE_IDLE;
      }
      break;

    case GRAB_STATE_IDLE:
    default:
      gCar.grab.busy = 0U;
      s_state = GRAB_STATE_IDLE;
      break;
  }
}
