/********************************************************************************
* pid.h: Contains drivers for user friendly PID controllers.
********************************************************************************/
#ifndef PID_H_
#define PID_H_

/* Include directives: */
#include <stdio.h>
#include "header.hpp"

/********************************************************************************
* pid: Struct for implementation of PID controllers with adjustable parameters, 
*      selectable minimum and maximum output values etc.
********************************************************************************/
struct pid
{
   double target;     /* Desired output value. */
   double output;     /* Real output value. */
   double input;      /* Input value from sensor (used for printing only). */
   double output_min; /* Minimum output value. */
   double output_max; /* Maximum output value. */
   double last_error; /* Last measured error. */
   double integrate;  /* Integral value, multiplied with ki when setting new output. */
   double derivate;   /* Delta value, multiplied with kd when setting new output. */
   double kp;         /* Proportional constant. */
   double ki;         /* Integrate constant. */
   double kd;         /* Derivate constant */
};

/********************************************************************************
* pid_init: Initiates PID controller with specified parameters.
*
*           - self      : Reference to the PID controller.
*           - target    : Desired output value.
*           - output_min: Minimum output value.
*           - output_max: Maximum output value .
*           - kp        : Proportional constant.
*           - ki        : Integrate constant.
*           - kd        : Derivate constant.
********************************************************************************/
void pid_init(struct pid* self,
              const double target,
              const double output_min,
              const double output_max,
              const double kp,
              const double ki,
              const double kd);

/********************************************************************************
* pid_regulate: Regulates output value of PID controller on the basis of  
*               specified new input value.
*
*               - self     : Reference to the PID controller.
*               - new_input: New input value of PID controller.
********************************************************************************/
void pid_regulate(struct pid* self,
                  const double new_input);

/********************************************************************************
* set_pid: Sets new pid values.
********************************************************************************/
void set_pid(struct pid* self, double kp, double ki, double kd);

#endif /* PID_H_ */
