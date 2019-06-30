// Trying PD for smoother motor control

// Testing how to make robot drive straight
// using proportional control

#include <external_MPU6050_6Axis_MotionApps20.h>
#include <external_MPU6050_I2Cdev.h>
#include <external_I2CIO.h>
#include <Wire.h>
#include <math.h> //math library

// Pins for H-bridge control
int leftEnableDC = 10; //pwm
int rightEnableDC = 11; //pwm 
int in1 = 9; //digital
int in2 = 8; //digital
int in3 = 12; //digital
int in4 = 13; //digital

// Encoder
const byte encoder0pinA = 2; //left motor
const byte encoder0pinB = 1;
const byte encoder1pinA = 3; //right motor
const byte encoder1pinB = 4;

byte encoder0PinALast;
byte encoder1PinALast;

int leftCount;//the number of the pulses
int rightCount;//the number of the pulses
boolean DirectionR;
boolean DirectionL;

#define MAX_SPEED 80

//function for the motor controller
void motor(int pin1,int pin2,int rightSpeed, int leftSpeed)
{  
  if (pin1 == 1) {
    analogWrite(leftEnableDC,leftSpeed);
    digitalWrite(in1,HIGH); //forward 
    digitalWrite(in2,LOW);  
  }
  else
  {
    analogWrite(leftEnableDC,leftSpeed);
    digitalWrite(in1,LOW); 
    digitalWrite(in2,HIGH);
  }

  if (pin2 == 1) {
    analogWrite(rightEnableDC,rightSpeed);
    digitalWrite(in3,HIGH); 
    digitalWrite(in4,LOW);  
  }
  else
  {
    analogWrite(rightEnableDC,rightSpeed);
    digitalWrite(in3,LOW); 
    digitalWrite(in4,HIGH);  
  }
}

// ISR for encoders
void Lencoder()
{
  int Lstate = digitalRead(encoder0pinA);
  if((encoder0PinALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoder0pinB);
    if(val == LOW && DirectionL)
    {
      DirectionL = false; //Reverse
    }
    else if(val == HIGH && !DirectionL)
    {
      DirectionL = true;  //Forward
    }
  }
  encoder0PinALast = Lstate;

  if(!DirectionL)  leftCount++;
  else  leftCount--;
}

void Rencoder()
{
  int Lstate = digitalRead(encoder1pinA);
  if((encoder1PinALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoder1pinB);
    if(val == LOW && DirectionR)
    {
      DirectionR = false; //Reverse
    }
    else if(val == HIGH && !DirectionR)
    {
      DirectionR = true;  //Forward
    }
  }
  encoder1PinALast = Lstate;

  if(!DirectionR)  rightCount++;
  else  rightCount--;
}

void setup() {
  
  Serial.begin(38400);
  Wire.begin();

  //pins for motor controller
  pinMode(leftEnableDC,OUTPUT);
  pinMode(rightEnableDC,OUTPUT);
 
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);  

  pinMode(encoder0pinB,INPUT);
  pinMode(encoder1pinB,INPUT);
  attachInterrupt(digitalPinToInterrupt(2), Lencoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), Rencoder, CHANGE);

  delay(3000);
}
 
char motor_command[36];
int setPoint_Speed = 60;
unsigned long motor_time;

int leftPWM = setPoint_Speed;
int rightPWM = setPoint_Speed-20;  

float targetRticks = 4700.0;
float targetLticks = 4700.0;

float lError = 0.0;
float rError = 0.0;
float prev_errorR = 0.0;
float prev_errorL = 0.0;

// PD
float k_p = 0.03; // 1/16 = 1/ CPR
float k_d = 0.02;

void loop()
{ 

  motor(1,0,rightPWM,leftPWM);
  
  // Error from encoder
  rError = targetRticks - rightCount;
 
  //PID
  //rightPWM += (k_p *rError) + (k_d * prev_errorR) + (k_i * errorR_sum); 
  rightPWM += (k_p *rError) + (k_d * prev_errorR);
  
  if (rightPWM > MAX_SPEED) rightPWM = MAX_SPEED; 

  //Serial.print(leftCount); Serial.print(" | "); Serial.println(rightCount); //debug controller output
  // Output to motors
  //sprintf(motor_command,"L: %d | R: %d\n\n",leftPWM, rightPWM);
  //Serial.print(motor_command);

  if (millis() - motor_time > 2000) //adjust tuning for distance that is one cell
  {
    motor_time = millis();
    motor(0,0,0,0);
    leftCount = 0;
    rightCount = 0;
    prev_errorR = rError;
    prev_errorL = lError; 
    delay(500);
  }
 
  delay(15);  
}
