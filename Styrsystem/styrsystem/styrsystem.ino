#include "header.hpp"

#define RUN_SIGNAL 4
#define BOOST 3
#define TURN 2
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
bool boost = false;
bool turn = false;
uint16_t timer_counter = 0;
uint16_t boost_counter = 0;
static double frontdistance = 0;




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

   return;
}

void check_boost()
{
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
    else if(frontdistance < 400 && frontdistance > 200)
    {
       boost = true;
       digitalWrite(BOOST, HIGH);
    }
  else boost_counter = 0;
  // Serial.print("Frontlow: ");
  // Serial.print(frontdistance);
  // Serial.print('\t');
}

void get_front_distance()
{
  frontdistance = sensors[1].read();
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
  // Serial.print("Turn?");
  // Serial.print(turn);

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
  pinMode(BOOST, OUTPUT);
  pinMode(TURN, OUTPUT);

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
              0.50,0.001, 1.11); // PID default är 0.75,0.01, 0.6 // 0.7  ////// 0.65,0.005, 1

  wdt_disable();  /* Disable the watchdog and wait for more than 2 seconds */
  delay(3000);  /* Done so that the Arduino doesn't keep resetting infinitely in case of wrong configuration */
  wdt_enable(WDTO_2S);
}

// 0.55,0.005, 1.15 Funkar sådär
//0.55,0.001, 1.15
//0.50,0.001, 1.11 // HASTIGHET 35

void loop()
{
  while(digitalRead(RUN_SIGNAL)==1)
  {
      servo1.tof_left.val = sensors[2].read();
      servo1.tof_right.val = sensors[0].read();
      servo_run(&servo1);
      if(angle < 85 || angle > 105) // 1 varv if(angle < 80 || angle > 110) HASTIGHET 38 //// if(angle < 85 || angle > 105) hastighet 35/23
      {
        digitalWrite(TURN, HIGH);
        //turn = true;
        set_pid(&servo1.pid, 0.60,0.005, 1);
        get_front_distance();
        check_reverse();
      }
      else
      {
        digitalWrite(TURN, LOW);
        //turn = false;
        set_pid(&servo1.pid, 0.50, 0.001, 1.11);
        get_front_distance();
        check_boost();
        check_reverse();
      }
      myServo.write(angle);
      wdt_reset();
      //print_values();
    //}
  }
  myServo.write(95);
}
