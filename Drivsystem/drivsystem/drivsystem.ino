// connect motor controller pins to Arduino digital pins
// motor one
#define KillSwitch 2 // Pin nummer för killswitch på startmodul
#define RemoteStart 3 // Pin nummer för start modulens remote start
#define StartSignal A0 // Pin nummer som signalerar till styr arduino att starta
#define BOOST A1     // pin nummer för back-funktion.
#define TURN A2       // pin nummer för boost funktion
#define MOTOR_FORWARD  10 // pin nummer för motor_forward pin
#define MOTOR_BACKWARD 11  // pin nummer för motor_backward pin

static inline void motor_forward(const uint8_t pwm_value)
{
  analogWrite(MOTOR_FORWARD, pwm_value);
  digitalWrite(MOTOR_BACKWARD, 0);
}

static inline void motor_backward(const uint8_t pwm_value)
{
  digitalWrite(MOTOR_FORWARD, 0);
  analogWrite(MOTOR_BACKWARD, pwm_value);
}

static inline void motor_wait_for_start(void)
{
  while ((digitalRead(RemoteStart) && digitalRead(KillSwitch)) == 0);
  digitalWrite(StartSignal, 1);
  return;
}

static inline void boost(int speed)
{
  set_speed(speed);
  delay(500);
}

void setup()
{
  // Initierar pinnar för motorstyrning samt startmodul.
  pinMode(RemoteStart, INPUT);
  pinMode(KillSwitch, INPUT);
  pinMode(StartSignal, OUTPUT);
}

//Sätter motorhastighet  i procent mellan 0-100%
void set_speed(uint8_t speed)
{
  speed = map(speed, 0,100,0,255);
  analogWrite(MOTOR_FORWARD, speed);
  digitalWrite(MOTOR_BACKWARD, 0);
}

void demo()
{
  for (int i = 0; i < 200; i++)
  {
    motor_forward(i);
    delay(20);
  } 
}
void loop()
{
  //demo();
  motor_wait_for_start(); 
  // set_speed(50);
  // delay(100);
  while((digitalRead(RemoteStart) && digitalRead(KillSwitch)) == 1)
  {
    if((digitalRead(BOOST) == 1))
    {
      // if(digitalRead(TURN)==1)
      // {
      //   motor_backward(80);
      //   delay(500);
      // }
      // else
      //{ 
       set_speed(80);
       delay(50);
      //}   
    }
    else if(digitalRead(TURN)==1)
    {
      set_speed(20);
    }
    else
    {
      set_speed(35);// 70 var default med svagare LIPO, 60 Nuvarande default med 11v LIPO. Ny hastighet defualt 20. //37
    }
  }
  while((digitalRead(RemoteStart) && digitalRead(KillSwitch)) == 0)
  {
    digitalWrite(MOTOR_BACKWARD, 0);
    digitalWrite(MOTOR_FORWARD, 0);
    digitalWrite(StartSignal, 0);
  }
}