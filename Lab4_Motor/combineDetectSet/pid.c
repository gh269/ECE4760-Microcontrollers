
#include "pid.h"
#include "stdint.h"

void pid_Init(int16_t p_factor, int16_t i_factor, int16_t d_factor, struct PID_DATA *pid)
// Set up PID controller parameters
{
 
  // Start values for PID controller
  pid->sumError = 0;
  pid->lastProcessValue = 0;
  // Tuning constants for PID loop
  pid->P_Factor = p_factor;
  pid->I_Factor = i_factor;
  pid->D_Factor = d_factor;
  // Limits to avoid overflow
  pid->maxError = MAX_INT / (pid->P_Factor + 1);
  pid->maxSumError = MAX_I_TERM / (pid->I_Factor + 1);
}


double pid_Controller(int16_t setPoint, int16_t processValue, struct PID_DATA *pid_st)
{
  double error;
  double p_term;
  double d_term;
  double i_term, ret, temp;

  error = setPoint - processValue;

  // Calculate Pterm and limit error overflow
  if (error > pid_st->maxError){
    p_term = MAX_INT;
  }
  else if (error < -pid_st->maxError){
    p_term = -MAX_INT;
  }
  else{
    p_term = pid_st->P_Factor * error;
  }

  // Calculate Iterm and limit integral runaway
  temp = pid_st->sumError + error;
  if(temp > pid_st->maxSumError){
    i_term = MAX_I_TERM;
    pid_st->sumError = pid_st->maxSumError;
  }
  else if(temp < -pid_st->maxSumError){
    i_term = -MAX_I_TERM;
    pid_st->sumError = -pid_st->maxSumError;
  }
  else{
    pid_st->sumError = temp;
    i_term = ((double)pid_st->I_Factor * pid_st->sumError);
  }

  // Calculate Dterm
  d_term = ((double)pid_st->D_Factor * (pid_st->lastProcessValue - processValue));

  pid_st->lastProcessValue = processValue;
	
  ret = (p_term + i_term + d_term) / SCALING_FACTOR;

  if(ret > MAX_INT){
    ret = MAX_INT;
  }
  else if(ret < -MAX_INT){
    ret = -MAX_INT;
  }

  return((int16_t)ret);
} 

void pid_Reset_Integrator(pidData_t *pid_st)
{
  pid_st->sumError = 0;
}
