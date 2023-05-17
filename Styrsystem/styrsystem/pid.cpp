/********************************************************************************
* pid.c: Contains drivers for user friendly PID controllers.
********************************************************************************/
#include "header.hpp"

/* Static functions: */
static inline void pid_check_output(struct pid* self);

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
              const double kd)
{
   self->target = target;
   self->output_min = output_min;
   self->output_max = output_max;
   self->kp = kp;
   self->ki = ki;
   self->kd = kd;

   self->output = 0;
   self->input = 0;
   self->last_error = 0;
   self->integrate = 0;
   self->derivate = 0;
   return;
}

/********************************************************************************
* pid_regulate: Regulates output value of PID controller on the basis of
*               specified new input value.
*
*               - self     : Reference to the PID controller.
*               - new_input: New input value of PID controller.
********************************************************************************/
void pid_regulate(struct pid* self,
                  const double new_input)
{
   const double error = self->target - new_input;
   self->input = new_input;

   self->derivate = error - self->last_error;
   self->integrate += error;
   self->output = self->target + self->kp * error + 
                  self->ki * self->integrate + 
                  self->kd * self->derivate;

   pid_check_output(self);
   self->last_error = error;
   return;
}

/********************************************************************************
* pid_check_output: Checks if the PID controller output value is within
*                   specified minimum and maximum output value. If the output
*                   value is out of this range, the value is set to nearest
*                   boundary.
*
*                   - self: Reference to the PID controller.
********************************************************************************/
static inline void pid_check_output(struct pid* self)
{
   if (self->output < self->output_min)
   {
      self->output = self->output_min;
   }
   else if (self->output > self->output_max)
   {
      self->output = self->output_max;
   }
   return;
}


/*********************************************************************************
* pid_print: Prints target value, input, output and last measured error for
*            PID controller. The output is printed in the terminal as default.
*            All floating point values are printed with one decimal.
*
*            - self   : Reference to the PID controller.
*            - ostream: Reference to output stream used (default = stdout).
*********************************************************************************/
void pid_print(const struct pid* self,
               FILE* ostream)
{
   if (!ostream) ostream = stdout;
   fprintf(ostream, "--------------------------------------------------------------------------------\n");
   fprintf(ostream, "Target value:\t\t%.1f\n", self->target);
   fprintf(ostream, "Input value:\t\t%.1f\n", self->input);
   fprintf(ostream, "Output value:\t\t%.1f\n", self->output);
   fprintf(ostream, "Last measured error:\t%.1f\n", self->last_error);
   fprintf(ostream, "--------------------------------------------------------------------------------\n\n");
   return;
}

void set_pid(struct pid* self, double kp, double ki, double kd)
{
  self->kp = kp;
  self->ki = ki;
  self->kd = kd;
  return;
}
