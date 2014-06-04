#include <Servo.h>

// Arduino pins for the shift register
#define MOTORLATCH 12
#define MOTORCLK 4
#define MOTORENABLE 7
#define MOTORDATA 8

// 8-bit bus after the 74HC595 shift register 
// (not Arduino pins)
// These are used to set the direction of the bridge driver.
#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR3_A 5
#define MOTOR3_B 7
#define MOTOR4_A 0
#define MOTOR4_B 6

// Arduino pins for the PWM signals.
#define MOTOR1_PWM 11
#define MOTOR2_PWM 3
#define MOTOR3_PWM 6
#define MOTOR4_PWM 5
#define SERVO1_PWM 10
#define SERVO2_PWM 9

// Codes for the motor function.
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3

// Declare classes for Servo connectors of the MotorShield.
Servo servo_claw;
Servo servo_tilt;

void setup()
{
  Serial.begin(9600);
  Serial.println("Claw & Tilt using Adafruit Motor Shield");

  // Attach the pin number to the servo library.
  // This might also set the servo in the middle position.
  servo_claw.attach(SERVO1_PWM);
  servo_tilt.attach(SERVO2_PWM);
}

void exec(char *cmd)
{
  Servo *servo = 0;
  int ixStart = 0;
  //Serial.println(*cmd);
  
  while(*cmd == ' ' || *cmd == '\t') cmd++;
  
  switch(*cmd)
  {
    case 'c': servo = &servo_claw; break;
    case 't': servo = &servo_tilt; break;
    default:
      Serial.println("error: unknown command");
      return;
  }
  
  int angle = atoi(cmd+1);
  if(angle < 0) angle = 0;
  if(angle > 180) angle = 180;
  
  if(servo == 0) return;
  servo->write(angle);
  Serial.println(angle);
}

const int BUFSIZE = 20;
char buf[BUFSIZE+1];
int ixBuf = 0;

void loop()
{
  while(Serial.available())
  {
      char c = Serial.read();
      //Serial.println(buf);
      
      if(c == '\n')
      {
        buf[ixBuf] = '\0';
        exec(buf);
        ixBuf = 0;
      }
      else if(ixBuf < BUFSIZE)
      {
        buf[ixBuf++] = c;
      }
  }
}
