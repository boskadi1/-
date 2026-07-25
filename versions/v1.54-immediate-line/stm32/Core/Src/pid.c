#include "pid.h"

static float Limit(float value, float min_value, float max_value)
{
  if (value > max_value) return max_value;
  if (value < min_value) return min_value;
  return value;
}

void Pid_Init(PidController_t *pid, float kp, float ki, float kd, float out_min, float out_max)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
  pid->output_min = out_min;
  pid->output_max = out_max;
}

float Pid_Update(PidController_t *pid, float setpoint, float measurement, float dt_s)
{
  float error = setpoint - measurement;
  float derivative = 0.0f;
  float output;

  if (dt_s > 0.0f)
  {
    derivative = (error - pid->prev_error) / dt_s;
  }

  pid->integral += error * dt_s;
  output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
  pid->prev_error = error;
  return Limit(output, pid->output_min, pid->output_max);
}
