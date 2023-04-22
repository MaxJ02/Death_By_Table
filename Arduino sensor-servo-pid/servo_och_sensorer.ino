#include "header.h"

#define RUN_SIGNAL 10
#define REVERSE 8
#define REVERSE_COUNT 30 //3 sekunder
#define RESTORE_COUNT 20 //2 sekunder

// Servo variables
Servo myServo;
double angle;
struct servo servo1;

// Sensor variables
const uint8_t sensorCount = 3;
const uint8_t xshutPins[sensorCount] = { 5, 6, 7 };
VL53L1X sensors[sensorCount];
int distance[sensorCount];
bool reverse = false;
uint16_t timer_counter = 0;



/********************************************************************************
* servo_run: Reads input values for left and right TOF sensor, regulates the
*            servo angle according to the input and prints the result in the
*            terminal.
*
*            - self: Reference to the servo.
********************************************************************************/
void servo_run(struct servo* self)
{
   pid_regulate(&self->pid, servo_input_mapped(self));
   angle = self->pid.output;
   //angle = map(angle,0,180,0,1023);
   myServo.write(angle);
   return;
}

/********************************************************************************
* check_reverse: Om fram signalen har varit lägre än 200mm i 3 sekunder sätts 
*                reverse variabeln till true.            
********************************************************************************/
void check_reverse()
{
  if(reverse == true)
  {
    timer_counter++;
    if(timer_counter >= RESTORE_COUNT)
    {
      timer_counter = 0;
      reverse = false;
      digitalWrite(REVERSE, 0);
    }
  }
  else if(sensors[2].read() <200 )
  {
    timer_counter++;
    if(timer_counter >= REVERSE_COUNT)
    {
      timer_counter = 0;
      reverse = true;
      digitalWrite(REVERSE, 1);
    }
  }
  else timer_counter = 0;
}

/********************************************************************************
* print_values: Skriver ut värden på sensorer och vilken vinkeln som servon
*               är inställd mot. Skriver även ut om bil är i reverse.
********************************************************************************/
void print_values()
{
  Serial.print("Left: ");
  Serial.print(servo1.tof_left.val);
  Serial.print('\t');
  Serial.print("Right: ");
  Serial.print(servo1.tof_right.val);
  Serial.print('\t');
  Serial.print("Angle: ");
  Serial.print(servo1.pid.output);
  Serial.print('\t');
  Serial.print("Front distance: ");
  Serial.print(sensors[2].read());
  Serial.print('\t');
  if(reverse)
  {
    Serial.print("Reverse? Yes \t");
  }
  else Serial.print("Reverse? No \t");
  Serial.print(timer_counter);
  Serial.println();
}


void setup()
{
  //Start signal
  pinMode(RUN_SIGNAL, INPUT);

 // Servo setup
  myServo.attach(9);

  //Reverse signal
  pinMode(REVERSE, OUTPUT);

  // Sensor setup
  while (!Serial) {}
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  for (uint8_t i = 0; i < sensorCount; i++)
  {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }

  for (uint8_t i = 0; i < sensorCount; i++)
  {
    pinMode(xshutPins[i], INPUT);
    delay(10);

    sensors[i].setTimeout(500);
    if (!sensors[i].init())
    {
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i);
      while (1);
    }

    sensors[i].setAddress(0x2A + i);

    sensors[i].startContinuous(50);
  }

  //PID initiering på servo
  servo_init(&servo1,90,35,135,0,500,1,0,0.1);
}



void loop()
{
  while(1)
  {
  check_reverse();
  if(reverse)
  {
    servo1.tof_left.val = sensors[1].read();
    servo1.tof_right.val = sensors[0].read();
    servo_run(&servo1);
    print_values();
  }
  else
  {
  servo1.tof_left.val = sensors[0].read();
  servo1.tof_right.val = sensors[1].read();
  servo_run(&servo1);
  print_values();
  }
  }
}
