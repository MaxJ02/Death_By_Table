#include "header.h"

// Servo variables
Servo myServo;
double angle;
struct servo servo1;

// Sensor variables
const uint8_t sensorCount = 3;
const uint8_t xshutPins[sensorCount] = { 5, 6, 7 };
VL53L1X sensors[sensorCount];
int distance[sensorCount];
int totalDistance = 0;
int averageDistance = 0;

/********************************************************************************
* servo_run: Reads input values for left and right TOF sensor, regulates the
*            servo angle according to the input and prints the result in the
*            terminal.
*
*            - self: Reference to the servo.
********************************************************************************/
void servo_run(struct servo* self)
{
   self->tof_left.val = sensors[1].read();
   self->tof_right.val = sensors[2].read();

   pid_regulate(&self->pid, servo_input_mapped(self));
   angle = self->pid.output;
   //angle = map(angle,0,1023,0,180);
   myServo.write(angle);
   return;
}

void setup()
{
  // Servo setup
  myServo.attach(9);

  // Sensor setup
  while (!Serial) {}
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  // Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }

  // Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    // Stop driving this sensor's XSHUT low. This should allow the carrier
    // board to pull it high. (We do NOT want to drive XSHUT high since it is
    // not level shifted.) Then wait a bit for the sensor to start up.
    pinMode(xshutPins[i], INPUT);
    delay(10);

    sensors[i].setTimeout(500);
    if (!sensors[i].init())
    {
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i);
      while (1);
    }

    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    sensors[i].setAddress(0x2A + i);

    sensors[i].startContinuous(50);
  }
  //PID initiering på servo
  servo_init(&servo1,90,45,135,0,500,1,001,01);

}


void loop()
{

  
  //Run servo and pid regulation
  while(1)
  {
  servo_run(&servo1);
  Serial.print("Left: ");
  Serial.print(servo1.tof_left.val);
  Serial.print('\t');
  Serial.print("Right: ");
  Serial.print(servo1.tof_right.val);
  Serial.print('\t');
  Serial.print("Angle: ");
  Serial.print(servo1.pid.output);
  Serial.print('\t');
  Serial.print("Mapped input: ");
  Serial.print(servo_input_mapped(&servo1));
  Serial.println();
  }
}