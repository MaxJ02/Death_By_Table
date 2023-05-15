/********************************************************************************
* tof.h: Include drivers for TOF (Time Of Flight) sensors with adjustable min 
*        and max values. The sensor values is entered via the terminal. 
*        As default, the min and max sensor values are set to  0 and 1023 
*        respectively.
********************************************************************************/
#ifndef TOF_H_
#define TOF_H_

#include "header.hpp"

/* Macro definitions: */
#define TOF_DEFAULT_MIN 0.0    /* Default minimum TOF sensor value. */
#define TOF_DEFAULT_MAX 1023.0 /* Default maximum TOF sensor value. */

/********************************************************************************
* tof: Struct for implementation of TOF (Time Of Flight) sensors with 
*      adjustable min and max values.
********************************************************************************/
struct tof
{
   double val; /* Input sensor value. */
   double min; /* Minimum sensor value. */
   double max; /* Maximum sensor value. */
};

/********************************************************************************
* tof_init: Initiates TOF sensor with specified minimum and maximum sensor
*           values, provided that the maximum value is higher than the
*           minimum value.
*
*           - self      : Reference to the TOF sensor.
*           - sensor_min: Minimum sensor value.
*           - sensor_max: Maximum sensor value.
********************************************************************************/
static inline void tof_init(struct tof* self,
                            const double sensor_min,
                            const double sensor_max)
{
   self->val = 0;

   if (sensor_max > sensor_min)
   {
      self->min = sensor_min;
      self->max = sensor_max;
   }
   else
   {
      self->min = TOF_DEFAULT_MIN;
      self->max = TOF_DEFAULT_MAX;
   }
   return;
}

/********************************************************************************
* tof_check_sensor_value: Checks if specified TOF sensor value is within set 
*                         minimum and maximum. If the sensor value is out of 
*                         this range, the value is set to nearest boundary.
* 
*                         - self: Reference to the TOF sensor.
********************************************************************************/
static inline void tof_check_sensor_value(struct tof* self)
{
   if (self->val < self->min)
   {
      self->val = self->min;
   }
   else if (self->val > self->max)
   {
      self->val = self->max;
   }
   return;
}

/********************************************************************************
* tof_input_range: Returns the range of the input values, i.e. the difference
*                  between specified max and min values, for TOF sensor.
* 
*                  - self: Reference to the TOF sensor.
********************************************************************************/
static inline double tof_input_range(const struct tof* self)
{
   return self->max - self->min;
}

#endif /* TOF_H_ */