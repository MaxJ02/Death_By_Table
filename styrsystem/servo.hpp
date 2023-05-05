/********************************************************************************
* servo.h: Includes drivers for PID controlled servos with TOF (Time Of Flight)
*          sensors to read the relative angle, followed by regulating the
*          servo angle towards the target.
********************************************************************************/
#ifndef SERVO_HPP_
#define SERVO_HPP_

/* Include directives: */
#include "header.h"

/********************************************************************************
* servo: Struct for implementation of PID controlled servos.
*        Two TOF (Time Of Flight) sensors are used to read the relative
*        angle of the servo. The PID controller regulates the servo angle
*        according to the sensor input to steer towards specified target.
********************************************************************************/
struct servo
{
   struct pid pid;       /* PID controller for regulating the servo angle. */
   struct tof tof_left;  /* Left TOF sensor, indicates relative distance to the left. */
   struct tof tof_right; /* Right TOF sensor, indicates relative distance to the right.  */
};

/********************************************************************************
* servo_init: Initiates servo with specified parameters. For instance, a target
*             of 90 degrees is center, a range of 0 - 180 degrees means all the 
*             way to the left and to the right (0 = left, 180 = right). 
* 
*             - self        : Reference to the servo.
*             - target_angle: Target angle for servo.
*             - angle_min   : Minimum servo angle.
*             - angle_max   : Maximum servo angle.
*             - input_min   : Minimum input value for sensors.
*             - input_max   : Maximum input value for sensors.
*             - kp          : Proportional constant for PID controller.
*             - ki          : Integrate constant for PID controller.
*             - kd          : Derivate constant for PID controlle.
********************************************************************************/
void servo_init(struct servo* self,
                const double target_angle,
                const double angle_min,
                const double angle_max,
                const double input_min,
                const double input_max,
                const double kp,
                const double ki,
                const double kd); 

/********************************************************************************
* servo_print: Prints target value, input and output for servo, along with servo 
*              angle relative to the target. The output is printed with one
*              decimal per parameter. As default, the output is printed in
*              the terminal.
* 
*              - self   : Reference to the servo.
*              - ostream: Reference to output stream used (default = stdout).
********************************************************************************/
void servo_print(const struct servo* self,
                 FILE* ostream);
  
/********************************************************************************
* servo_run: Reads input values for left and right TOF sensor, regulates the
*            servo angle according to the input and prints the result in the 
*            terminal.
* 
*            - self: Reference to the servo.
********************************************************************************/
void servo_run(struct servo* self);

/********************************************************************************
* servo_target: Returns the target angle of the servo.
*
*               - self: Reference to the servo.
********************************************************************************/
static inline double servo_target(const struct servo* self)
{
   return self->pid.target;
}

/********************************************************************************
* servo_output: Returns the current output angle of the servo.
*
*               - self: Reference to the servo.
********************************************************************************/
static inline double servo_output(const struct servo* self)
{
   return self->pid.output;
}

/********************************************************************************
* servo_input_range: Returns the range of the servo input values, i.e. the
*                    difference between specified max and min values.
*
*                    - self: Reference to the servo.
********************************************************************************/
static inline double servo_input_range(const struct servo* self)
{
   return tof_input_range(&self->tof_left);
}

/********************************************************************************
* servo_input_difference: Returns the difference between the input signals
*                         of left and right TOF sensor.
*
*                         For example, if the left sensor reads 500 while the
*                         right sensor reads 700, the difference between the
*                         input signals is 500 - 700 = -200, which is returned.
*
*                         - self: Reference to the servo.
********************************************************************************/
static inline double servo_input_difference(const struct servo* self)
{
   return self->tof_left.val - self->tof_right.val;
}

/********************************************************************************
* servo_input_ratio: Returns the ratio of the difference between the input values
*                    for left and right sensor as a number between 0 to 1.
*
*                    For example, assume the input min and max are set to
*                    0 and 1023 respectively and the left sensor reads 500 while
*                    the right sensor reads 700. Then the difference between the
*                    input signals is 500 - 700 = -200 while the input range is
*                    1023 - 0 = 1023.
*
*                    To scale the difference from a value between [-1023, 1023]
*                    to [0, 1023], the input range is added with the difference.
*                    The calculated sum is then divided by 2. Therefore, the
*                    scaled input is (-200 + 1023) / 2 = 411.5.
*
*                    Finally the ratio of the scaled input is calculated by
*                    dividing the scaled input value with the input range.
*                    Therefore, the input ratio is 411.5 / 1023 = 0.4, i.e
*                    40 % of max. This ratio is returned after calculation.
*
*                    - self: Reference to the servo.
********************************************************************************/
static inline double servo_input_ratio(const struct servo* self)
{
   const double scaled_input = (servo_input_difference(self) + 
                                servo_input_range(self)) / 2.0;
   return scaled_input / servo_input_range(self);
}

/********************************************************************************
* servo_input_mapped: Returns relative input measured between values of left and
*                     right TOF sensor, mapped to scale with the servo angle and
*                     centered to the target.
*
*                     - self: Reference to the servo.
********************************************************************************/
static inline double servo_input_mapped(const struct servo* self)
{
   return servo_input_ratio(self) * servo_target(self) * 2;
}
  
#endif /* SERVO_HPP_ */