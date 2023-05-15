#include "header.hpp"

#define RUN_SIGNAL 4
#define REVERSE 3
#define BOOST 2
#define REVERSE_COUNT 30 //3 sekunder
#define RESTORE_COUNT 20 //2 sekunder
#define BOOST_COUNT 10

// Servo variables
Servo myServo;
double angle;
struct servo servo1;

// Sensor variables
// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPins[] = {5,6,7};

// The number of sensors in your system.
const uint8_t sensorCount = sizeof(xshutPins) / sizeof(uint8_t);

VL53L1X sensors[sensorCount];
double reverse_direction[10];
bool reverse = false;
bool boost = false;
uint16_t timer_counter = 0;
uint16_t boost_counter = 0;




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
  else if(sensors[1].read() <200 )
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

void check_boost()
{
    double frontdistance = sensors[1].read();

    if(boost == true)
    {
    boost_counter++;
    if(boost_counter >= 2)
    {
      boost_counter = 0;
      boost = false;
      digitalWrite(BOOST, LOW);
    }
    }
    else if(frontdistance < 450 && frontdistance > 100)
    {
      boost_counter++;
      if(boost_counter >= BOOST_COUNT)
      {
       boost_counter = 0;
       boost = true;
       digitalWrite(BOOST, HIGH);
      }
    }
  else boost_counter = 0;
  // Serial.print("Frontlow: ");
  // Serial.print(frontdistance);
  // Serial.print('\t');
}

void store_direction()
{
  static int direction_counter;
  if(direction_counter > sizeof(reverse_direction))
  {
    direction_counter = 0;
  }
  reverse_direction[direction_counter] = angle;
  direction_counter++;
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
  Serial.print("Frontlow: ");
  Serial.print(sensors[1].read());
  // Serial.print('\t');
  // Serial.print("Fronthigh: ");
  // Serial.print(sensors[3].read());
  // Serial.print('\t');
  Serial.println();
}


void setup()
{
  //Kommunikations pinnar till andra arduinon.
  pinMode(RUN_SIGNAL, INPUT);
  pinMode(REVERSE, OUTPUT);
  pinMode(BOOST, OUTPUT);

 // Servo setup
  myServo.attach(9);

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

    sensors[i].startContinuous(100);
  }

  //PID initiering på servo
  servo_init(&servo1,95,         // servons "mitt" Ändra inte.
              60,130,     // Min & max vinklar
              0,1024,     // min max sensor avstånd
              0.65,0.005, 1); // PID default är 0.75,0.01, 0.6 // 0.7
}

void loop()
{
  if(digitalRead(RUN_SIGNAL)==1)
  {
      // if(reverse = false)
      // {
      servo1.tof_left.val = sensors[2].read();
      servo1.tof_right.val = sensors[0].read();
      check_reverse();
      servo_run(&servo1);
      //print_values();
  //     }
  //     else
  //     {
  //     servo1.tof_left.val = sensors[0].read();
  //     servo1.tof_right.val = sensors[2].read();
  //     check_reverse();
  //     servo_run(&servo1);
  //     }
   }
  if(digitalRead(RUN_SIGNAL)==0)
  {
    myServo.write(95);
  }
}
