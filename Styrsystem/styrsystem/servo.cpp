/********************************************************************************
* servo.c: Includes drivers for PID controlled servos with TOF (Time Of Flight)
*          sensors to read the relative angle, followed by regulating the
*          servo angle towards the target.
********************************************************************************/
#include "header.hpp"
#include <VL53L1X.h>

/* Static functions: */
static inline void servo_print_relative_angle(const struct servo* self,
                                              FILE* ostream);

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
                const double kd)
{
   pid_init(&self->pid, target_angle, angle_min, angle_max, kp, ki, kd);
   tof_init(&self->tof_left, input_min, input_max);
   tof_init(&self->tof_right, input_min, input_max);
   return;
}


