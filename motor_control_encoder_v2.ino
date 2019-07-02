// Trying PD for smoother motor control + 
// in-depth encoder feedback loop

// Trying PD for smoother motor control

// Testing how to make robot drive straight
// using proportional control

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

#define MAX_SPEED 60.0 //Percentage PWM OR duty cycle

//function for the motor controller
void motor(int pin1,int pin2,float percRight,float percLeft)
{  
  if (pin1 == 1) {
    analogWrite(leftEnableDC,(percLeft/100.0)*255);
    digitalWrite(in1,HIGH); //forward 
    digitalWrite(in2,LOW);  
  }
  else
  {
    analogWrite(leftEnableDC,(percLeft/100.0)*255);
    digitalWrite(in1,LOW); 
    digitalWrite(in2,HIGH);
  }

  if (pin2 == 1) {
    analogWrite(rightEnableDC,(percRight/100.0)*255);
    digitalWrite(in3,HIGH); 
    digitalWrite(in4,LOW);  
  }
  else
  {
    analogWrite(rightEnableDC,(percRight/100)*255);
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
  
  Serial.begin(115200);

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
unsigned long motor_time;

// Motor/Wheel parameters
#define COUNT_PER_REV 1920.0 // 16 CPR * 120:1 gear ratio
#define CIRCUM 240.0 //mm

void robotForward(float distance, float setSpeedPerc)
{
  long targetCount;
  float numRev;

  long lDiff, rDiff;  // diff between current encoder count and previous count

  float leftPWM = setSpeedPerc;
  float rightPWM = setSpeedPerc;

  // variables for tracking the left and right encoder counts
  long prevlCount, prevrCount;
  
  // variable used to offset motor power on right vs left to keep straight.
  double offset = 0.4;  // offset amount to compensate Right vs. Left drive

  numRev = distance / CIRCUM;  // calculate the target # of rotations
  targetCount = numRev * COUNT_PER_REV;    // calculate the target count

  Serial.print("Target Rev: ");
  Serial.print(numRev, 3);
  Serial.print(" Target Count: ");
  Serial.println(targetCount);
  
  // Reset encoders
  leftCount = 0;
  rightCount = 0;
  delay(50);

  motor(1,0,rightPWM,leftPWM);

  while (abs(rightCount) < targetCount)
  {
    motor(1,0,rightPWM,leftPWM);

    Serial.print(abs(leftCount));
    Serial.print("\t");
    Serial.print(abs(rightCount));
    Serial.print("\t");
    Serial.println(targetCount);

    // calculate the rotation "speed" as a difference in the count from previous cycle.
    lDiff = (abs(leftCount) - prevlCount);
    rDiff = (abs(rightCount) - prevrCount);

    // store the current count as the "previous" count for the next cycle.
    prevlCount = -leftCount;
    prevrCount = -rightCount;

    // if left is faster than the right, slow down the left / speed up right
    if (lDiff > rDiff) 
    {
      leftPWM -= offset;
      rightPWM += offset;
    }
    // if right is faster than the left, speed up the left / slow down right
    else if (lDiff < rDiff) 
    {
      leftPWM += offset;  
      rightPWM -= offset;    }
    delay(50);  // short delay to give motors a chance to respond.
  }
  
  motor(0,0,0.0,0.0);
  delay(1000);

}

// PD Parameters
float k_p = 0.03; // 1/16 = 1/ CPR
float k_d = 0.02;

// Error terms for PD controller
float lError = 0.0;
float rError = 0.0;
float prev_errorR = 0.0;
float prev_errorL = 0.0;

void loop()
{ 

  robotForward(250.0,30.0);

//  motor(1,0,rightPWM,leftPWM);
//  
//  // Error from encoder
//  rError = targetRticks - rightCount;
//   
//  //PD
//  rightPWM += (k_p *rError) + (k_d * prev_errorR);
//  
//  //Serial.print(leftCount); Serial.print(" | "); Serial.println(rightCount); //debug controller output
//  // Output to motors
//  //sprintf(motor_command,"L: %d | R: %d\n\n",leftPWM, rightPWM);
//  //Serial.print(motor_command);
//
//  if (millis() - motor_time > 2000)
//  {
//    motor_time = millis();
//    motor(0,0,0,0);
//    leftCount = 0;
//    rightCount = 0;
//    prev_errorR = rError;
//    prev_errorL = lError; 
//    delay(500);
//  }
 
  delay(15);  
}

