// connect motor controller pins to Arduino digital pins
// motor one
#define KillSwitch 8 // Pin nummer för killswitch på startmodul
#define RemoteStart 9 // Pin nummer för start modulens remote start

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
  return;
}

void setup()
{
  // Initierar pinnar för motorstyrning samt startmodul.
  pinMode(RemoteStart, INPUT);
  pinMode(KillSwitch, INPUT);
  motor_wait_for_start();
}

void demo()
{
  // this function will run the motors across the range of possible speeds
  // note that maximum speed is determined by the motor itself and the operating voltage
  // the PWM values sent by analogWrite() are fractions of the maximum speed possible 
  // by your hardware
  // turn on motor
  // accelerate from zero to maximum speed
  for (int i = 0; i < 255; i++)
  {
    motor_forward(i);
    delay(20);
  } 
}
void loop()
{
  demo();
  while((digitalRead(RemoteStart) && digitalRead(KillSwitch)) == 1)
  {

  }
  while(1)
  {

    digitalWrite(MOTOR_BACKWARD, 0);
    digitalWrite(MOTOR_FORWARD, 0);
  }
}
