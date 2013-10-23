#ifndef PID_H
#define PID_H

#include "stdint.h"

#define SCALING_FACTOR  128.0

/*! \brief PID Status
 *
 * Setpoints and data used by the PID control algorithm
 */
typedef struct PID_DATA{
  //! Last process value, used to find derivative of process value.
  int16_t lastProcessValue;
  //! Summation of errors, used for integrate calculations
  int32_t sumError;
  //! The Proportional tuning constant, multiplied with SCALING_FACTOR
  double P_Factor;
  //! The Integral tuning constant, multiplied with SCALING_FACTOR
  double I_Factor;
  //! The Derivative tuning constant, multiplied with SCALING_FACTOR
  double D_Factor;
  //! Maximum allowed error, avoid overflow
  int16_t maxError;
  //! Maximum allowed sumerror, avoid overflow
  int32_t maxSumError;
} pidData_t;

/*! \brief Maximum values
 *
 * Needed to avoid sign/overflow problems
 */
// Maximum value of variables
#define MAX_INT         INT8_MAX
#define MAX_LONG        INT16_MAX
#define MAX_I_TERM      (MAX_LONG / 2)

// Boolean values
#define FALSE           0
#define TRUE            1

void pid_Init(double p_factor, double i_factor, double d_factor, struct PID_DATA *pid);
double pid_Controller(int16_t setPoint, int16_t processValue, struct PID_DATA *pid_st);
void pid_Reset_Integrator(pidData_t *pid_st);

#endif
