// Testing H-brige using pure Arduino
// Will try and reverse engineer it with API class stuff

#include <Servo.h>

// Defining pins for hardware
#define RC_ON_OFF 6   //RC receiver on and off channel pin
#define MOTOR_PWM 5   //Motor for propulsion
#define SERVO_PWM 3   //Servo for steering. 0 - 120 (to calibrate)
#define SERVO_PWM_TOP 1900
#define SERVO_PWM_BOTTOM 1100

Servo driveMotor;
Servo steerServo;

// Pins for H-bridge control

int leftEnableDC = 9; //pwm
int rightEnableDC = 3; //pwm 
int in1 = 2; //digital
int in2 = 4; //digital
int in3 = 8; //digital
int in4 = 10; //digital

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

void setup() {
  
  Serial.begin(115200);

  //set up motor control
  pinMode(MOTOR_PWM,OUTPUT);
  pinMode(SERVO_PWM,OUTPUT);
  pinMode(RC_ON_OFF,INPUT);

  //pins for motor controller
  pinMode(leftEnableDC,OUTPUT);
  pinMode(rightEnableDC,OUTPUT);
 
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);

  driveMotor.attach(MOTOR_PWM);
  steerServo.attach(SERVO_PWM);

  delay(2000);
}

// State Machine setup
#define RC_IDLE              (0x0000) // Powered on but no motion yet
#define RC_RUN               (0x0001) // During race + receive commands from NUC?
#define RC_OFF               (0x0002) // Disarm ESC after finishing race
int system_mode = RC_RUN;

int motorPWM = 1000;
int servoPWM = 1500;
char commands[64];
int motorSpeed_R = 50;


void loop() {

  char RC_Serial; //to be replaced by NUC commands or TX stick position
  
  if (Serial.available() > 0) RC_Serial = Serial.read();
  else RC_Serial = -1;

  switch (system_mode)
  {
    /*
    case (RC_IDLE):  

    // Starts RC car  
    if (RC_Serial == 's' )
    //if (pulseIn(RC_ON_OFF, HIGH, 20000) > 1800) //in reality using TX
    {
      //send arming signal to ESC
      driveMotor.write(1000);
      delay(1000);
      system_mode = RC_RUN;
      Serial.println("Armed -> Go!");
    }
    break;
  */
   case (RC_RUN):
      
      // Throttle ESC
      // Need some method to have serial commands from NUC interacting with hardware
      switch (RC_Serial)
      {
        /*
        case('l'): // left steer
          servoPWM += 100;
          if (servoPWM >= SERVO_PWM_TOP)
          {
            servoPWM = SERVO_PWM_TOP;
          }
          steerServo.writeMicroseconds(servoPWM);
          break;

        case('r'): // right steer
          servoPWM -= 100;
          if (servoPWM <= SERVO_PWM_BOTTOM)
          {
            servoPWM = SERVO_PWM_BOTTOM;
          }
          steerServo.writeMicroseconds(servoPWM);
          break;
        */
        case('a'): // accelerate
          motorSpeed_R += 50;
          break;

        case('d'): // decelerate
          motorSpeed_R -= 50;
          break;

        default:
         break;        
      }

      /*
      // Stops RC car if transmitter sends stop command (left stick to bottom)
      // Need to do so <5m after finish line
      
      if (RC_Serial == 'e' )
      //if (pulseIn(RC_ON_OFF, HIGH, 30000) < 1500 && pulseIn(RC_ON_OFF, HIGH, 30000) > 1000) //in reality using TX
      {        
        system_mode = RC_OFF;
        Serial.println("RC car STOPPED!");
      }  
      break;
      */

   case (RC_OFF):
    
    driveMotor.writeMicroseconds(1000); 
    steerServo.writeMicroseconds(1500); 
    motorPWM = 1000;
    servoPWM = 1500;     
    break;
 
   default:
    break;
 }   

  //sprintf(commands, "%d | %d\n", motorPWM, servoPWM);
  motor(1,1,motorSpeed_R,motorSpeed_R);
  Serial.println(motorSpeed_R);
  
  delay(50);
  
}
