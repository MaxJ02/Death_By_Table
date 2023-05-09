// connect motor controller pins to Arduino digital pins.
// motor one
#define KillSwitch 2 // Pin nummer för killswitch på startmodul
#define RemoteStart 3 // Pin nummer för start modulens remote start
#define StartSignal 6 // Pin nummer som signalerar till styr arduino att starta

#define MOTOR_FORWARD  10
#define MOTOR_BACKWARD 11 

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

void setup()
{
  // Initierar pinnar för motorstyrning samt startmodul.
  pinMode(RemoteStart, INPUT);
  pinMode(KillSwitch, INPUT);
  pinMode(StartSignal, OUTPUT);
  motor_wait_for_start();
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
  set_speed(80); // 70 är default
  while((digitalRead(RemoteStart) && digitalRead(KillSwitch)) == 1)
  {

  }
  while(1)
  {
    digitalWrite(MOTOR_BACKWARD, 0);
    digitalWrite(MOTOR_FORWARD, 0);
    digitalWrite(StartSignal, 0);
  }
}
