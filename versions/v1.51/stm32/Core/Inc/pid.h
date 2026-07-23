#ifndef PID_H_
#define PID_H_

typedef struct
{
  float kp;
  float ki;
  float kd;
  float integral;
  float prev_error;
  float output_min;
  float output_max;
} PidController_t;

void Pid_Init(PidController_t *pid, float kp, float ki, float kd, float out_min, float out_max);
float Pid_Update(PidController_t *pid, float setpoint, float measurement, float dt_s);

#endif /* PID_H_ */
