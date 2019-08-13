// Combining Explore + Drive Code
// Merged on 30/7

// Combining Driving and Exploration 
// TODO: calibrate encoder for right turns - might need to increase
//encoder counts were changed for right turns
//

// Including API Libraries
#include <units.h>
#include <hardware.h>
#include <hardware_definition.h>
#include <Wire.h>
#include <Sensors.h>
#include <external_DFRobot_LCD.h>

// Avoids prepending scope operator to all functionsS
#define SIDEMAX 130
#define SIDEMIN 5
#define FRONTMAX 130
#define FRONTMIN 5

// Avoids prepending scope operator to all functions
using namespace hardware;

// Motor/Wheel parameters
#define COUNT_PER_REV       1500.0  // 16 CPR * 120:1 gear ratio
#define CIRCUM              240.0 // mm
#define STRAIGHT_DISTANCE   215.0 // mm
#define LIDAR_MAX_SETPOINT  85.0 // mm
#define STRAIGHT_SPEED      22.0
#define SPEEDRUN_SPEED      22.0
#define SPEEDRUN_DISTANCE   200.0
#define COUNT_PER_REV_TURN  1600.0  // 16 CPR * 120:1 gear ratio
#define SPEED_OFFSET 5.00 //straight (with no lidar control)
#define L_SPEED_OFFSET 6.0
#define RIGHT_WALL 50.0
#define LEFT_WALL 60.0 

#define ROW 6  //number of row and col to print out = num of cells + 1
#define COL 10
#define SPEED_RUN 53

bool robot_start = false; //will be set to true once LED sequence is finished
int movement_num = 0;

//arrays for map
int Hmap[10][10];  //global maze map horizontal extended
int Vmap[10][10];  //global maze map vertical extended
int Hmap2[10][10];  //global maze map horizontal extended
int Vmap2[10][10];  //global maze map vertical extended
int tempMap[10][10];  //temp map to transfer map details 

int prows;
int pcols;
int pheading;
int pgoalx;
int pgoaly;
int tempGoalx;
int tempGoaly;
bool SExplored = false;
bool swapdir;

//global cases for flood fill map
int values[9][9];
int ConvertToWalls[9][9];
int pathOne[40];
int pathTwo[40];

DFRobot_LCD lcd(16,2);  //16 characters and 2 lines of show

void setUpUltrasonic()
{
  //Ultrasonic set up
  pinMode(TRIGGER_PIN, OUTPUT); // Sets the TRIGGER_PIN as an Output
  pinMode(ECHO_PIN, INPUT); // Sets the ECHO_PIN as an Input
  //Serial3.println("Ultrasonic initialised");
}

void setUpLidars()
{
  //Setting address for lidars
  pinMode(LIDARONE,OUTPUT);
  pinMode(LIDARTWO,OUTPUT);

  digitalWrite(LIDARONE, HIGH);//enable lidar two
  digitalWrite(LIDARTWO, LOW); //shut down lidar one before chaning address
  delay(100);
  lidarOne.setAddress(0x30);
  delay(10);
  digitalWrite(LIDARTWO, HIGH);//enable lidar one 
  
  lidarOne.init();
  //Serial3.println("Lidar One initialised");
  lidarOne.configureDefault();
  lidarOne.setTimeout(500);
  
  lidarTwo.init();              //lidar one uses default address
  //Serial3.println("Lidar Two initialised");
  lidarTwo.configureDefault();
  lidarTwo.setTimeout(500);
    
}

void setupimu() {
  pinMode(INTERRUPT_PIN, INPUT);
  imu.initialize();
  //Serial3.println(imu.testConnection() ? "IMU initialised" : "imu6050 connection failed");
  devStatus = imu.dmpInitialize();
  
  //Serial3.println("Starting offset");
  SetOffsets(TheOffsets);
  //Serial3.println("Offsets done");
  
  //set up for dmp
  if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      //Serial3.println(F("Enabling DMP..."));
      imu.setDMPEnabled(true);
  
      // enable Arduino interrupt detection
      attachInterrupt(digitalPinToInterrupt(18), dmpDataReady, RISING);
      imuIntStatus = imu.getIntStatus();
  
      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      dmpReady = true;
  
      // get expected DMP packet size for later comparison
      packetSize = imu.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      //Serial3.print(F("DMP Initialization failed (code "));
  }
}

float ultrasonicdist() {
  long duration;
  float distance;
  
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = (float(duration)/(29.155*2));
  distance = distance*10;

  if (distance > 1000) {  //setting max distance at 1000
    distance = 1000;
  }
  return distance;
}

//--------------MOTOR CONTROL HERE----------------------

// Encoder variables for Phase A
volatile int eCountR = 0; 
volatile int eCountL = 0; 
volatile byte pinAcountR;
volatile byte pinAcountL;
boolean DirectionR = true; // Rotation direction for right motor
boolean DirectionL = true; // Rotation direction for left motor

// Control Parameters
double error_P_lidar = 0.0;
double error_P_encoder = 0.0;

double error_D_lidar = 0.0;
double error_D_encoder = 0.0;

double prev_error_encoder = 0.0;
double prev_error_lidar = 0.0;

double totalError = 0.0;
double sumError = 0.0;

//-------------------PID Parameters---------------------//
// Constants for Lidar
double K_p_lidar = 1.25;
double K_d_lidar = 0.5;

// Constants for Encoder
double K_p_encoder = 0.07;
double K_d_encoder = 0.04;
double K_i_encoder = 0.01;
//-----------------------------------------------------//

// Motor variables (to match hardware_definitions.h)
int leftEnableDC = 10; //pwm
int rightEnableDC = 11; //pwm 
int in1 = 9; //digital
int in2 = 8; //digital
int in3 = 12; //digital
int in4 = 13; //digital

//function for the motor controller
void motorControl(int pin1,int pin2,float percRight,float percLeft)
{  
  if (pin1 == 1) {
    analogWrite(leftEnableDC,(percLeft/100)*255);
    digitalWrite(in1,HIGH); //forward 
    digitalWrite(in2,LOW);  
  }
  else
  {
    analogWrite(leftEnableDC,(percLeft/100)*255);
    digitalWrite(in1,LOW); 
    digitalWrite(in2,HIGH);
  }

  if (pin2 == 1) {
    analogWrite(rightEnableDC,(percRight/100)*255);
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

void resetAllEncoders()
{
  eCountL = 0;
  eCountR = 0;
  delay(10);
}

void robotForward(float distance, float setSpeedPerc)
{
  long targetCount;
  float numRev;

  long lDiff, rDiff;  // diff between current encoder count and previous count

  float leftPWM = setSpeedPerc;
  float rightPWM = setSpeedPerc;

  float prevlCount, prevrCount;
 
  // Bias - variable used to offset motor power on right vs left to keep straight.
  double offset = 0.25;  // offset amount to compensate Right vs. Left drive

  numRev = distance / CIRCUM;  // calculate the target # of rotations
  targetCount = numRev * (COUNT_PER_REV);    // calculate the target count
 
  // Reset encoders
  eCountL = 0;
  eCountR = 0;
  delay(15);

  motorControl(1,0,rightPWM,leftPWM);

  while (abs(eCountR) < targetCount)
  {
    motorControl(1,0,rightPWM,leftPWM+offset);

    // Error count from encoder 'ticks'
    lDiff = (abs(eCountL) - prevlCount);
    rDiff = (abs(eCountR) - prevrCount);

    // Have walls on either side (Part 3 of Phase B)
    // L1 = 70. L2 = 60. Bias = 10
    // L1 = RIGHT and L2 = LEFT
    //if (use_lidar == true)
    // readings around 

    if ((lidarOne.readRangeSingleMillimeters() < LIDAR_MAX_SETPOINT && lidarTwo.readRangeSingleMillimeters() < LIDAR_MAX_SETPOINT))
    {
        error_P_lidar = (lidarOne.readRangeSingleMillimeters() - lidarTwo.readRangeSingleMillimeters() - 10.0);
        error_D_lidar = error_P_lidar - prev_error_lidar;
  
        // left of a cell
        if (lidarOne.readRangeSingleMillimeters() > lidarTwo.readRangeSingleMillimeters())
        {
            leftPWM += totalError;
            rightPWM -= totalError;

          if (leftPWM > setSpeedPerc + L_SPEED_OFFSET || rightPWM > setSpeedPerc + L_SPEED_OFFSET)
          { 
            leftPWM = setSpeedPerc + L_SPEED_OFFSET*1.20;
            rightPWM = setSpeedPerc + L_SPEED_OFFSET*0.80; 
          }
        }
        //right of a cell
        else if (lidarOne.readRangeSingleMillimeters() < lidarTwo.readRangeSingleMillimeters())
        {
            leftPWM -= totalError;
            rightPWM += totalError;

          if (leftPWM >= setSpeedPerc + L_SPEED_OFFSET || rightPWM >= setSpeedPerc + L_SPEED_OFFSET)
          {
            leftPWM = setSpeedPerc + L_SPEED_OFFSET*0.80;
            rightPWM = setSpeedPerc + L_SPEED_OFFSET*1.20; 
          }
        }    
     }

    //if wall on right but not on left
    else if ((lidarOne.readRangeSingleMillimeters() < LIDAR_MAX_SETPOINT && lidarTwo.readRangeSingleMillimeters() > LIDAR_MAX_SETPOINT))
    { 
        
        error_P_lidar = RIGHT_WALL - (lidarOne.readRangeSingleMillimeters());
        error_D_lidar = error_P_lidar - prev_error_lidar;
  
        //K_p and K_d (for lidar)  
        totalError = (K_p_lidar * error_P_lidar) + (K_d_lidar * error_D_lidar);
        prev_error_lidar = error_P_lidar;

        if (lidarOne.readRangeSingleMillimeters() > RIGHT_WALL) // too far from right
        {
          
          leftPWM += totalError;
          rightPWM -= totalError;

          if (leftPWM > setSpeedPerc + L_SPEED_OFFSET || rightPWM > setSpeedPerc + L_SPEED_OFFSET)
          { 
            leftPWM = setSpeedPerc + L_SPEED_OFFSET*1.20;
            rightPWM = setSpeedPerc + L_SPEED_OFFSET*0.80; 
          }
          
        }

        if (lidarOne.readRangeSingleMillimeters() < RIGHT_WALL) // too close to right
        {
          
          leftPWM -= totalError;
          rightPWM += totalError;

          if (leftPWM > setSpeedPerc + L_SPEED_OFFSET || rightPWM > setSpeedPerc + L_SPEED_OFFSET)
          { 
            leftPWM = setSpeedPerc + L_SPEED_OFFSET*0.80;
            rightPWM = setSpeedPerc + L_SPEED_OFFSET*1.20; 
          }
          
        }

    }

    //if wall on left but not on right
    else if ((lidarOne.readRangeSingleMillimeters()> LIDAR_MAX_SETPOINT && lidarTwo.readRangeSingleMillimeters() < LIDAR_MAX_SETPOINT))
    { 
        
        error_P_lidar = LEFT_WALL - (lidarTwo.readRangeSingleMillimeters());
        error_D_lidar = error_P_lidar - prev_error_lidar;
  
        //K_p and K_d (for lidar)  
        totalError = (K_p_lidar * error_P_lidar) + (K_d_lidar * error_D_lidar);
        prev_error_lidar = error_P_lidar;

        if (lidarTwo.readRangeSingleMillimeters() > LEFT_WALL) // too far from left
        {
          
          leftPWM -= totalError;
          rightPWM += totalError;

          if (leftPWM > setSpeedPerc + L_SPEED_OFFSET || rightPWM > setSpeedPerc + L_SPEED_OFFSET)
          { 
            leftPWM = setSpeedPerc + L_SPEED_OFFSET*0.80;
            rightPWM = setSpeedPerc + L_SPEED_OFFSET*1.20; 
          }
          
        }

        if (lidarTwo.readRangeSingleMillimeters() < LEFT_WALL) // too close to left
        {
          
          leftPWM += totalError;
          rightPWM -= totalError;

          if (leftPWM > setSpeedPerc + L_SPEED_OFFSET || rightPWM > setSpeedPerc + L_SPEED_OFFSET)
          { 
            leftPWM = setSpeedPerc + L_SPEED_OFFSET*1.20;
            rightPWM = setSpeedPerc + L_SPEED_OFFSET*0.80; 
          }
          
        }
    }

    // no walls
    else {
      
      double e_error = (lDiff - rDiff); //encoder error between left and right wheel
      sumError += e_error;      
      double dError = (lDiff - rDiff) - prev_error_encoder;

      totalError = (K_p_encoder * e_error) - (K_d_encoder * setSpeedPerc) + 
      (K_i_encoder * sumError);

      // Adjust speed just on encoder   
      leftPWM += totalError;  
      rightPWM -= totalError;
  
      if (leftPWM >= setSpeedPerc + SPEED_OFFSET || rightPWM >= setSpeedPerc + SPEED_OFFSET)
      {
        leftPWM = setSpeedPerc + SPEED_OFFSET;
        rightPWM = setSpeedPerc + SPEED_OFFSET; 
      }

      prev_error_lidar = e_error; //store previous error for next cycle-    

    }
      // Store previous count  
      prevlCount = abs(eCountL);
      prevrCount = abs(eCountR);
  }    
  robotStop();
  totalError = 0;
  delay(10);
  resetAllEncoders();
}

void robotStop()
{
  right_motor::stop();
  left_motor::stop();
  delay(15);
}

// ----------NEW FUNCTION to replace existing turn function--------------
// robot turn with IMU
void robotTurnIMU(int directionVal)
{

  double setSpeedPercTurn = 25.0;
  float currYaw; //get current reading from IMU
  while (!gyro()) 
    gyro();
  
  if (gzf >= -45 && gzf <= 45) {
    currYaw = 0;
  } else if (gzf > 45 && gzf <= 135)
    currYaw = 90;
  else if (gzf > 135 || gzf < -135) {
    currYaw = 180; 
  } else if (gzf >= -135 && gzf <= -45) {
    currYaw = -90;
  }
    
  // 1 = Turn CW (to right)
  if (directionVal == 1)
  {
      while (gzf <= currYaw + 90) // yaw value should increase up to the desired value
      {
        
      // is this a reset for gyro? or a check whether its busy?
      while (!gyro()) 
        gyro(); //obtain the gzf value from this function call.  
              
        motorControl(1,1,setSpeedPercTurn,setSpeedPercTurn); 
      } 
  }
 
  // 0 = Turn CCW (to left)
  else
  {
    while (gzf >= currYaw - 90) // yaw value should decrease up to the desired value
    {
      if (!gyro()) 
        gyro(); //obtain the gzf value from this function call.
      
      motorControl(0,0,setSpeedPercTurn,setSpeedPercTurn); 
    }
  }  
}

void robotTurn(int directionVal)
{

  float setSpeedPerc = 25.0;

  resetEncoderR();
  resetEncoderL();

  if (directionVal == 1)  //Turn to left (CCW)
  {
    //Right + and Left -

    while (abs(eCountR) <= COUNT_PER_REV_TURN/3.85)
    {
      motorControl(1,1,setSpeedPerc,setSpeedPerc);    
    }
    
    resetEncoderR();
    resetEncoderL();
    robotStop();
    delay(200);
  }

  else
  {
    //-ve so turn to right (CW)
    //Right - and Left +

    while (abs(eCountR) <= COUNT_PER_REV_TURN/2.85)
    {
      motorControl(0,0,setSpeedPerc,setSpeedPerc);  
    }

  resetEncoderR();
  resetEncoderL(); 
  robotStop();    
  delay(200); 
  }  
  delay(50);
}

void robotTurnv2(int directionVal)
{

   float currYaw; //get current reading from IMU
   while (!gyro()) 
  {
    delay(10);
  }
  
  if (gzf >= -45 && gzf <= 45) {
    currYaw = 0;
  } else if (gzf > 45 && gzf <= 135)
    currYaw = 90;
  else if (gzf > 135 || gzf < -135) {
    currYaw = 180; 
  } else if (gzf >= -135 && gzf <= -45) {
    currYaw = -90;
  }

  float val = abs(gzf - currYaw);

  float setSpeedPerc = 25.0;

  resetEncoderR();
  resetEncoderL();

  if (directionVal == 1)  //Turn to left (CCW)
  {
    //Right + and Left -

    while (abs(eCountR) <= (COUNT_PER_REV_TURN/3.70 *(val/100 +1)))
    {
      motorControl(1,1,setSpeedPerc,setSpeedPerc);    
    }
    
    resetEncoderR();
    resetEncoderL();
    robotStop();
    delay(200);
  }

  else
  {
    //-ve so turn to right (CW)
    //Right - and Left +

    while (abs(eCountR) <= COUNT_PER_REV_TURN/2.65*(val/100 +1))
    {
      motorControl(0,0,setSpeedPerc,setSpeedPerc);  
    }

  resetEncoderR();
  resetEncoderL(); 
  robotStop();    
  delay(200); 
  }  
  delay(50);
}

void robotLeft()
{
  resetAllEncoders();   
  robotTurn(0); //turn CCW on spot
  //robotTurnv2(0);
  resetAllEncoders();
  delay(200);
  robotForward(STRAIGHT_DISTANCE,STRAIGHT_SPEED); //go forward on cell
  resetAllEncoders();  
  delay(100);
}

void robotRight()
{
  resetAllEncoders();
  robotTurn(1); //turn CW on spot
  //robotTurnv2(1);
  resetAllEncoders();
  delay(200);
  robotForward(STRAIGHT_DISTANCE,STRAIGHT_SPEED); //go forward one cell
  resetAllEncoders();  
  delay(100);
}

void robotLeftSpeed()
{
  resetAllEncoders();   
  robotTurn(0); //turn CCW on spot
  resetAllEncoders();
  delay(100);
  robotForward(SPEEDRUN_DISTANCE,SPEEDRUN_SPEED); //go forward on cell
  resetAllEncoders();  
  delay(100);
}

void robotRightSpeed()
{
  resetAllEncoders();
  robotTurn(1); //turn CW on spot
  resetAllEncoders();
  delay(100);
  robotForward(SPEEDRUN_DISTANCE,SPEEDRUN_SPEED); //go forward one cell
  resetAllEncoders();  
  delay(100);
}

void robotReverse() {
  resetAllEncoders();
  robotTurn(1); //turn CW on spot
  delay(200);
  resetAllEncoders();
  robotTurn(1);
  resetAllEncoders();
  delay(200);
  robotForward(STRAIGHT_DISTANCE,STRAIGHT_SPEED); //go forward one cell
  delay(200);
  resetAllEncoders();
}

//--------------MOTOR CONTROL HERE----------------------

// ISR for encoder interrupt at Right Motor
void encoderCountR()
{
    encoder_count LstateR = int(pins::right_encoder_a::read());

    if((pinAcountR == LOW) && LstateR == HIGH)
    {
        encoder_count encoder_bR = encoder_count(pins::right_encoder_b::read());

        if(encoder_bR == LOW && DirectionR)
        {
          DirectionR = false; //Reverse

        }
        else if(encoder_bR == HIGH && !DirectionR)
        {
          DirectionR = true;  //Forward
        }
    }

    pinAcountR = LstateR;

    if(!DirectionR)  eCountR++;
    else  eCountR--;
}

// ISR for encoder interrupt at Left Motor
void encoderCountL()
{
    encoder_count LstateL = encoder_count(pins::left_encoder_a::read());

    if((pinAcountL == LOW) && LstateL == HIGH)
    {
        encoder_count encoder_bL = encoder_count(pins::left_encoder_b::read());

        if(encoder_bL == LOW && DirectionL)
        {
          DirectionL = false; //Reverse

        }
        else if(encoder_bL == HIGH && !DirectionL)
        {
          DirectionL = true;  //Forward
        }
    }

    pinAcountL = LstateL;

    if(!DirectionL)  eCountL++;
    else  eCountL--;
}

void resetEncoderR()
{
  eCountR = 0;
}

void resetEncoderL()
{
  eCountL = 0;
}


void setupDigitalPins()
{
  led::config_io_mode(io_mode::output);
  statusRed::config_io_mode(io_mode::output);
  statusGreen::config_io_mode(io_mode::output);
}

void setupMotor()
{  
  right_motor::enable();
  left_motor::enable();
}

void setupEncoderWheel()
{
  left_encoder::enable();
  right_encoder::enable();
  
  // Pure Arduino Interrupt Setup
  attachInterrupt(digitalPinToInterrupt(3), encoderCountR, CHANGE); //ISR for Right Motor
  attachInterrupt(digitalPinToInterrupt(2), encoderCountL, CHANGE); //ISR for Left Motor 
}

void startLEDSequence(){
  //Green ON and Red OFF
  statusGreen::write(logic_level::high);
  statusRed::write(logic_level::low);   
}

void speedRunLED()
{
  statusGreen::write(logic_level::high);
  statusRed::write(logic_level::low); 
  delay(50);
  statusGreen::write(logic_level::low);
  statusRed::write(logic_level::high);
  delay(50);
  statusGreen::write(logic_level::high);
  statusRed::write(logic_level::low); 
  delay(50);
  statusGreen::write(logic_level::low);
  statusRed::write(logic_level::high);
  delay(1000);
     
}

// Timing for hardware tests
unsigned long explore_test = 0;
unsigned long led_timer = 0;
logic_level ledLogic = logic_level::low;
char drivemode;

//////////////////////////////////////////////////////////PART 2//////////////////////////////////////////////////////
//start condition
void startphaseb() {
  int count = 0;  //count for seeing object
  int ucount = 0; //count for not seeing object
  float distance;
  bool seen = false;
  while (1) {
    distance = ultrasonicdist();
    
    //calibrate distance according to placement of robot in cell
    //if anything is within 50mm, increase count
    //if object is within 50mm for 10 samples - set flag
    if (distance <= 40) {
      count += 1;
      if (count == 10) {
        seen = true;
        //Serial3.println("seen");
      }
    } else if (seen == true) {
      ucount += 1;
      //Serial3.print("unseen");
      if (ucount == 10) {
        break;
      }
    } else {
      seen = false;
      count = 0;
    }
    
  }
  //Serial3.println("starting");
  robot_start = true;
}

//check if wall in front of ultrasonic
//between 30 and 60mm - needs calibration
bool wall(float distance, int maxv, int minv) {
  bool wall = false;
  if (distance >= minv && distance <= maxv) {
    wall = true;   
  }
  return wall;
}

//returns compass direction of robot
int compass() {
  int temp;
  if (!gyro()) 
    gyro();
    
  if (gzf >= -45 && gzf <= 45) {
      temp = NORTH;
    } else if (gzf > 45 && gzf <= 135)
      temp = EAST;
    else if (gzf > 135 || gzf < -135) {
      temp = SOUTH; 
    } else if (gzf >= -135 && gzf <= -45) {
      temp = WEST;
    }
  return temp;
}

//contains some imu functions

void SetOffsets(int TheOffsets[6]) {
  imu.setXAccelOffset(TheOffsets [0]);
  imu.setYAccelOffset(TheOffsets [1]);
  imu.setZAccelOffset(TheOffsets [2]);
  imu.setXGyroOffset (TheOffsets [3]);
  imu.setYGyroOffset (TheOffsets [4]);
  imu.setZGyroOffset (TheOffsets [5]);
} // SetOffsets


//returns 1 if successfully updated values
bool gyro() {
  bool success;
  int failcount = 0;
  if (!dmpReady) return;

    // wait for imu interrupt or extra packet(s) available
    while (!imuInterrupt && fifoCount < packetSize) {
      if (++failcount >= 100) {
        success = false;
        break;
      }
    }

    // reset interrupt flag and get INT_STATUS byte
    imuInterrupt = false;
    imuIntStatus = imu.getIntStatus();

    // get current FIFO count
    fifoCount = imu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((imuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        imu.resetFIFO();
        success = false;
        //Serial3.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (imuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = imu.getFIFOCount();

        // read a packet from FIFO
        imu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
        // display Euler angles in degrees
        imu.dmpGetQuaternion(&q, fifoBuffer);
        imu.dmpGetGravity(&gravity, &q);
        imu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        gxf = ypr[1]*180/M_PI;
        gyf = ypr[2]*180/M_PI;
        gzf = ypr[0]*180/M_PI;

        success = true;
    }
    return success;
}

//checks case if we need to flip east and west
void checkflip(short i,short j,short compassd,bool &firstopen,bool &swapdir) {
  float distance;
  distance = 0;
  if (i == 0 || j == 0 && firstopen == false) {
    if (compassd == 0) {
        //check if there is an open wall on the right - do multiple reads
        for (int temp = 0; temp !=5; temp++) {
          distance+= lidarOne.readRangeSingleMillimeters();
        }
        distance = distance/5;
        
        //if no wall on the right case - flip east and west
        if(!wall(distance, SIDEMAX, SIDEMIN)) {
          swapdir = true;
          firstopen = true;
        }
        //handles case if initial direction is dead end and you need to go other way
    } else if (compassd == 1||compassd == 3) {
        for (int temp = 0; temp !=5; temp++) {
          distance+= lidarTwo.readRangeSingleMillimeters();
        }
        distance = distance/5;
  
        if(!wall(distance, SIDEMAX, SIDEMIN)) {
          swapdir = true;
          firstopen = true;
        }
    }
     //otherwise if not case - set all to false except for firstopen 
  } else if (i != 0 && j != 0) {
          swapdir = false;
          firstopen = true;
  }
}

void transfermap(int rows_wall, int cols_wall) {
 //copy Vmap across to tempmap since only vertical walls need to be flipped
 for (int i = 0; i < rows_wall+1; i++) {
    for (int j = 0; j < cols_wall+1; j++) {
      tempMap[i][j] = Vmap[i][j]; 
    }
  }

 for (int x = 0; x < rows_wall; x++) {
  for (int y = 0; y < cols_wall; y+=2) {
    Vmap[x][y] = tempMap[x][y+1];
    Vmap[x][y+1] = tempMap[x][y];  
  }
 }
}

//handles case where start pos is different
short othersp(bool swapdir, short compassd) {
  if (swapdir) {
    if (compassd == EAST)
      compassd = WEST;
    else if (compassd == WEST)
      compassd = EAST;
  }
  return compassd;
}

void readAndstore(short i, short j, short compassd, bool swapdir) {

  //taking average ultrasonic distance values
  float distance;
  for (int count = 0; count != 5; count++) {
    distance += ultrasonicdist();
  }
  distance = distance/5;
  
  bool ultrawall = wall(distance, FRONTMAX, FRONTMIN);

  distance = 0;   //reset distance
  for (int temp = 0; temp !=5; temp++) {
    distance+= lidarOne.readRangeSingleMillimeters();
  }
  distance = distance/5;
  bool L1wall = wall(distance, SIDEMAX, SIDEMIN);
  //Serial3.print("L1wall = "); Serial3.println(L1wall);

  distance = 0;   //reset distance
  for (int tempa = 0; tempa !=5; tempa++) {
    distance+= lidarTwo.readRangeSingleMillimeters();
  }
  distance = distance/5;
  bool L2wall = wall(distance, SIDEMAX, SIDEMIN);
  //Serial3.print("L2wall = "); Serial3.println(L2wall);
  storemaze(compassd, i, j, L1wall, L2wall, ultrawall, swapdir);
}

void storemaze(short compassd, short i, short j, int L1wall, int L2wall, int ultrawall, bool swapdir) {
    if (!swapdir) {
      switch(compassd) {
      case NORTH:
        Hmap[i+1][j] = ultrawall;
        Vmap[i][j+1] = L2wall; //assume l2 is on the left when facing north
        Vmap[i][j] = L1wall;
        break;
      case EAST:
        Vmap[i][j] = ultrawall;
        Hmap[i+1][j] = L2wall;
        Hmap[i][j] = L1wall;
        break;
      case SOUTH:
        Hmap[i][j] = ultrawall;
        Vmap[i][j+1] = L1wall;
        Vmap[i][j] = L2wall;
        break;
      case WEST:
        Vmap[i][j+1] = ultrawall;
        Hmap[i+1][j] = L1wall;
        Hmap[i][j] = L2wall;
        break;
      }
   } else {
    switch(compassd) {
      case NORTH:
        Hmap[i+1][j] = ultrawall;
        Vmap[i][j] = L2wall; //assume l2 is on the left when facing north
        Vmap[i][j+1] = L1wall;
        break;
      case EAST:
        Vmap[i][j] = ultrawall;
        Hmap[i][j] = L2wall;
        Hmap[i+1][j] = L1wall;
        break;
      case SOUTH:
        Hmap[i][j] = ultrawall;
        Vmap[i][j] = L1wall;
        Vmap[i][j+1] = L2wall;
        break;
      case WEST:
        Vmap[i][j+1] = ultrawall;
        Hmap[i][j] = L1wall;
        Hmap[i+1][j] = L2wall;
        break;
    }
  }
}

short findnextdir(short &i, short &j, short &oldi, short &oldj, int rows_wall, int cols_wall) {
  short nextdir = 5;

  //check north
  if (j != 0 && Vmap[i][j] != 1) {                                                     //check east
    if (values[i][j] == (values[i][j-1]+1) && oldj != (j-1)) 
      nextdir = EAST;
  }
  if (i !=0 && Hmap[i][j] != 1) {                                                       //check south
    if (values[i][j] == (values[i-1][j]+1) && oldi != (i-1))
      nextdir = SOUTH;
  }
  if (j != (cols_wall-1) && Vmap[i][j+1] != 1) {                                         //check west
    if (values[i][j] == (values[i][j+1]+1) && oldj != (j+1))
      nextdir = WEST;
   }

  if (i != (rows_wall-1) && Hmap[i+1][j] != 1) {
    if (values[i][j] == (values[i+1][j]+1) && oldi != (i+1)){
      nextdir = NORTH;
    }
  }
  
  if (nextdir == 5) {
    //next set of preferences if only available path has been previously travelled
    if (j != 0 && Vmap[i][j] != 1) {                                                        //check east
      if (values[i][j] == (values[i][j-1]+1))
        nextdir = EAST;
    } 
    if (i !=0 && Hmap[i][j] != 1) {                                                       //check south
      if (values[i][j] == (values[i-1][j]+1))
        nextdir = SOUTH;
    } 
    if (j != (cols_wall - 1) && Vmap[i][j+1] != 1) {                                               //check west
      if (values[i][j] == (values[i][j+1]+1))
        nextdir = WEST;
    }
    if (i != (rows_wall-1) && Hmap[i+1][j] != 1) {
      if (values[i][j] == (values[i+1][j]+1))
        nextdir = NORTH;
    }
  }
  return nextdir;
}

short findnextdir2(short &i, short &j, short &oldi, short &oldj, int rows_wall, int cols_wall) {
  short nextdir = 5;

  //check north
  if (j != 0 && Vmap2[i][j] != 1) {                                                     //check east
    if (ConvertToWalls[i][j] == (ConvertToWalls[i][j-1]+1) && oldj != (j-1)) 
      nextdir = EAST;
  }
  if (i !=0 && Hmap2[i][j] != 1) {                                                       //check south
    if (ConvertToWalls[i][j] == (ConvertToWalls[i-1][j]+1) && oldi != (i-1))
      nextdir = SOUTH;
  }
  if (j != (cols_wall-1) && Vmap2[i][j+1] != 1) {                                         //check west
    if (ConvertToWalls[i][j] == (ConvertToWalls[i][j+1]+1) && oldj != (j+1))
      nextdir = WEST;
   }

  if (i != (rows_wall-1) && Hmap2[i+1][j] != 1) {
    if (ConvertToWalls[i][j] == (ConvertToWalls[i+1][j]+1) && oldi != (i+1)){
      nextdir = NORTH;
    }
  }
  
  if (nextdir == 5) {
    //next set of preferences if only available path has been previously travelled
    if (j != 0 && Vmap2[i][j] != 1) {                                                        //check east
      if (ConvertToWalls[i][j] == (ConvertToWalls[i][j-1]+1))
        nextdir = EAST;
    } 
    if (i !=0 && Hmap2[i][j] != 1) {                                                       //check south
      if (ConvertToWalls[i][j] == (ConvertToWalls[i-1][j]+1))
        nextdir = SOUTH;
    } 
    if (j != (cols_wall - 1) && Vmap2[i][j+1] != 1) {                                               //check west
      if (ConvertToWalls[i][j] == (ConvertToWalls[i][j+1]+1))
        nextdir = WEST;
    }
    if (i != (rows_wall-1) && Hmap2[i+1][j] != 1) {
      if (ConvertToWalls[i][j] == (ConvertToWalls[i+1][j]+1))
        nextdir = NORTH;
    }
  }
  return nextdir;
}

//prints Hmaze and Vmaze combined
void printMaze(short initialheading, int rows_wall, int cols_wall, int goalx, int goaly) {
  for(int i = 0;i < 2*(rows_wall)+1; i++) {
    for(int j = 0;j < 2*(cols_wall)+1 ;j++) {
        //Add Horizontal Walls
        if(i%2 == 0 && j%2 == 1)
        {
            if(Hmap[i/2][j/2] == true)
              Serial3.print(" ---");
            else if (Hmap[i/2][j/2] == false)
              Serial3.print("    ");
            else if (Hmap[i/2][j/2] == 2)
              Serial3.print(" ***");
        }

        //Add Vertical Walls
        if(i%2 == 1 && j%2 == 0)
        {
            if (i == 1 && j == 0) 
            {
              if (initialheading == WEST)
                Serial3.print("| S "); 
              else if (initialheading == SOUTH)
                Serial3.print("| S ");
              else
                Serial3.print("| S ");
            } else if (i == (tempGoalx*2 +1) && j == (tempGoaly*2)) {
              if (Vmap[i/2][j/2] == true) 
                Serial3.print("| X ");
              if (Vmap[i/2][j/2] == false)
                Serial3.print("  X ");  
            } else if(Vmap[i/2][j/2] == true)
              Serial3.print("|   ");
            else if (Vmap[i/2][j/2] == false)
              Serial3.print("    ");
            else if (Vmap[i/2][j/2] == 2)
              Serial3.print("*   ");
          }
    delay(10);
    }
    Serial3.println();
  }
}

void printMaze2(short initialheading, int rows_wall, int cols_wall, int goalx, int goaly) {
  for(int i = 0;i < 2*(rows_wall)+1; i++) {
    for(int j = 0;j < 2*(cols_wall)+1 ;j++) {
        //Add Horizontal Walls
        if(i%2 == 0 && j%2 == 1)
        {
            if(Hmap2[i/2][j/2] == true)
              Serial3.print(" ---");
            else if (Hmap2[i/2][j/2] == false)
              Serial3.print("    ");
            else if (Hmap2[i/2][j/2] == 2)
              Serial3.print(" ***");
        }

        //Add Vertical Walls
        if(i%2 == 1 && j%2 == 0)
        {
            if (i == 1 && j == 0) 
            {
              if (initialheading == WEST)
                Serial3.print("| S "); 
              else if (initialheading == SOUTH)
                Serial3.print("| S ");
              else
                Serial3.print("| S ");
            } else if (i == (tempGoalx*2 +1) && j == (tempGoaly*2)) {
              if (Vmap2[i/2][j/2] == true) 
                Serial3.print("| X ");
              if (Vmap2[i/2][j/2] == false)
                Serial3.print("  X ");  
            } else if(Vmap2[i/2][j/2] == true)
              Serial3.print("|   ");
            else if (Vmap2[i/2][j/2] == false)
              Serial3.print("    ");
            else if (Vmap2[i/2][j/2] == 2)
              Serial3.print("*   ");
          }
    delay(10);
    }
    Serial3.println();
  }
}


void setup() {
  Wire.begin(); 
  Serial3.begin(9600);
  setUpLidars();
  setUpUltrasonic();
  setupimu();
  setupDigitalPins();
  setupMotor(); 
  setupEncoderWheel(); 
  lcd.init(); //initialise lcd
  lcd.setBacklight(0);
  pinMode(SPEED_RUN,INPUT);
  pinMode(13,OUTPUT); 
  delay(100);
  Serial3.println("Started");
}

// State Machine
#define MODE_OFF 1
#define MODE_EXPLORE 2
#define MODE_GOAL 3 
#define MODE_SPEEDRUN 4
int system_mode = MODE_OFF;

void loop()
{
  switch(system_mode) {
    case(MODE_OFF):
      Serial3.println("Start Condition");
      statusGreen::write(logic_level::low);
      statusRed::write(logic_level::high); 
      resetAllEncoders(); 
      
      startphaseb();

      // after sufficient exploration, ready for speed run mode
      // not explore mode
      if (SExplored == true)
      {
        system_mode = MODE_SPEEDRUN;
        delay(3000);
        statusGreen::write(logic_level::high);
        statusRed::write(logic_level::low);
        break;
      }
      //else
      
      statusGreen::write(logic_level::high);
      statusRed::write(logic_level::low);
      //filling up wall maps
      for (int a =0; a <= 9; a++) {
        for (int b =0; b <= 9; b++) {
          Hmap[a][b] = 2;
          Vmap[a][b] = 2;
        }
      }
      Serial3.println("Starting");
      delay(500);
      
      system_mode = MODE_EXPLORE; 
    break;
    
    case(MODE_EXPLORE):
      short compassd; //contains compass direction
      short tcompass; //temp compass
      bool ultrawall; //ultrasonic wall
      bool L1wall;  //lidar one wall
      bool L2wall;  //lidar two wall
      short initialheading;
      bool goal;
      short i;  //row
      short j;  //col
      short oldi;
      short oldj;
      bool firstopen;
      short nextdir;
      bool sizedet;
      int moveCount;
      float distance;
      SExplored = false;

      //initial flood fill values
      tempGoalx = 5;
      tempGoaly = 5;
      compassd = 0;
      i = 0;
      j = 0;
      oldj = 0;  //arbitary values
      oldi = 0;
      firstopen = false;
      swapdir = false;
      sizedet = false;
      int rows_wall;
      int cols_wall;
      cols_wall = 9;
      rows_wall = 9;
      distance = 0;
      
      while(!sizedet){
        
        while (!gyro()) {
          delay(50);
          gyro();     
        }
        
        compassd = compass();
        tcompass = compassd;  //store value into temp just in case
  
        //determine if need to swap east and west 
        if (!firstopen) 
        { 
          checkflip(i,j,compassd,firstopen,swapdir);
          
          if (swapdir) {
            //flip all previous values
            transfermap(rows_wall,cols_wall);
          }
        }
  
        compassd = othersp(swapdir, compassd);
        
        readAndstore(i, j, compassd, swapdir);  
        //printMaze(1, rows_wall, cols_wall, tempGoalx, tempGoaly);
        flood_fill(tempGoalx,tempGoaly,rows_wall, cols_wall);
        
        nextdir = findnextdir(i,j, oldi, oldj, rows_wall, cols_wall);

        //if you can't find a direction then redo
        while (nextdir == 5) {
          readAndstore(i,j,compassd, swapdir);
          flood_fill(tempGoalx,tempGoaly, rows_wall, cols_wall);
          nextdir = findnextdir(i,j,oldi,oldj, rows_wall, cols_wall);
          ////lcd.clear();
          Serial3.println("Processing...");
        }
        
        //update oldi and oldj
        oldi = i;
        oldj = j;
        
        newpos(i,j,nextdir);  //updates north count as well
       
        //adjusting output next direction for flip if needed
        if (swapdir) {
          if (nextdir == EAST) {
            nextdir = WEST;
          } else if (nextdir == WEST) {
            nextdir = EAST;
          }
        }

        compassd = tcompass;  //compassd is now = to real heading that was stored
       
       moveCount++; //update movecount before moving since starting at 0;
       moveRobot(compassd, nextdir, moveCount);

        //check if goal has been reached
        if (i >= tempGoalx) {
          initialheading = SOUTH;
          pheading = initialheading;
          sizedet = true;
          Serial3.print("Goal 1.1 Reached");
          Serial3.print("i = "); Serial3.println(i);
          Serial3.print("j = "); Serial3.println(j);
          //lcd.clear();
          //delay(1000);
          break;
        } else if (j >= tempGoaly) {
          initialheading = WEST;
          pheading = initialheading;
          sizedet = true;
          //lcd.clear();
          Serial3.print("GOAL 1.2 REACHED");
          Serial3.print("i = "); Serial3.println(i);
          Serial3.print("j = "); Serial3.println(j);
          //delay(1000);
          break;
        }
    }
    //printMaze(1, rows_wall, cols_wall, tempGoalx, tempGoaly);
    //////lcd.clear();
    Serial3.println("---NEW GOAL---");

    //reset size and goal
    switch(initialheading){
      case(WEST):
        rows_wall = 5;
        cols_wall = 9;
        prows = rows_wall;
        pcols = cols_wall;
        tempGoalx = 2;
        tempGoaly = 4;
        pgoalx = tempGoalx;
        pgoaly = tempGoaly;
        bool centre;
        centre = false;
  
        for (int a =0; a <= 9; a++) {
          for (int b =0; b <= 9; b++) {
            Hmap2[a][b] = Hmap[a][b];
            Vmap2[a][b] = Vmap[a][b];
          }
        }
        while(!SExplored) {
          
          while (!gyro()) {
            delay(50);
            gyro();     
          }
        
          compassd = compass(); //update compass
          tcompass = compassd; //store value into temp        
          compassd = othersp(swapdir, compassd);
          readAndstore(i, j, compassd, swapdir);  

                  //determine if need to swap east and west 

          //compare paths
          //if explored break, if not continue exploring
          //find a new goal based on unexplored arrays 
          //then feed the new goal to flood fill and head there

          Serial3.println("Not SExplored _ West");

          if (i == pgoalx && j == pgoaly && centre == false) 
          {
            centre = true; 
            Serial3.println("centre found");
            statusRed::write(logic_level::high); 
            statusGreen::write(logic_level::high);
            delay(1000);
            statusRed::write(logic_level::low);
            
          }

          //must find centre first before being able to break out
          if (centre == true && compare_path()) 
          {
            SExplored = true;
            //lcd.clear();
            Serial3.println("");
            Serial3.println("READY for SPEED RUN");
            statusRed::write(logic_level::high); 
            statusGreen::write(logic_level::low); 
            delay(10000);
            //speedRunLED();
            system_mode = MODE_OFF;
            break;
          } else {
            if (i == tempGoalx && j == tempGoaly) 
            {
              //lcd.clear();
              Serial3.print("new goal");
              if (tempGoalx == pgoalx) 
              {
                tempGoalx = 0;
                tempGoaly = 0;
                
              }
              else 
              {
                tempGoalx = pgoalx;
                tempGoaly = pgoaly;
              }
            }
          }
          
          //apply flood fill for 5 by 9
          flood_fill(tempGoalx,tempGoaly, rows_wall, cols_wall);
          //next dir to 5 by 9
          nextdir = findnextdir(i,j, oldi, oldj, rows_wall,cols_wall);
          
          //if you can't find a direction then redo
          while (nextdir == 5) {
            readAndstore(i,j,compassd, swapdir);
            flood_fill(tempGoalx,tempGoaly, rows_wall, cols_wall);
            nextdir = findnextdir(i,j,oldi,oldj, rows_wall, cols_wall);
            ////lcd.clear();
            Serial3.print("Processing...1");
          }
          
          //update oldi and oldj
          oldi = i;
          oldj = j;
          
          newpos(i,j,nextdir);  //updates north count as well

          //reflip to reflect real one
          if (swapdir) {
            if (nextdir == EAST) {
              nextdir = WEST;
            } else if (nextdir == WEST) {
              nextdir = EAST;
            }
          }
          compassd = tcompass;  //let compassd = real heading again;

          moveCount++;
          moveRobot(compassd, nextdir, moveCount);
        }
      break;

      case(SOUTH):
        rows_wall = 9;
        cols_wall = 5;
        prows = rows_wall;
        pcols = cols_wall;
        tempGoalx = 4;
        tempGoaly = 2;  
        pgoalx = tempGoalx;
        pgoaly = tempGoaly;
        bool centre2;
        centre2 = false;
        //lcd.clear();
        Serial3.print("South Case");
        delay(300);
        while(!SExplored) {

          while (!gyro()) {
            delay(50);
            gyro();     
          }
        
          compassd = compass(); //update compass
          tcompass = compassd;          
          compassd = othersp(swapdir, compassd);
          readAndstore(i, j, compassd, swapdir);  
          //lcd.clear();
          Serial3.println("Not SExplored _ South");

          if (i == pgoalx && j == pgoaly && centre2 == false) 
          {
            centre2 = true; 
            Serial3.println("centre found");
            statusRed::write(logic_level::high); 
            statusGreen::write(logic_level::high);
            delay(1000);
            statusRed::write(logic_level::low);
            
          }

          //compare paths
          //if explored break, if not continue exploring
          //find a new goal based on unexplored arrays 
          //then feed the new goal to flood fill and head there
          if (centre2 == true && compare_path()) 
          {
            SExplored = true;
            Serial3.println("READY for SPEED RUN");
            statusRed::write(logic_level::high); 
            statusGreen::write(logic_level::low);  
            delay(10000);
            //speedRunLED();
            system_mode = MODE_OFF;
            break;
          } else {
            if (i == tempGoalx && j == tempGoaly) 
            {
              //lcd.clear();
              Serial3.print("new goal");

              if (tempGoalx == pgoalx) 
              {
                tempGoalx = 0;
                tempGoaly = 0;
                Serial.println("centre found");
                statusRed::write(logic_level::high); 
                statusGreen::write(logic_level::high);
                delay(1000);
                statusRed::write(logic_level::low);
              }
              else 
              {
                tempGoalx = pgoalx;
                tempGoaly = pgoaly;
              }
            }
          }

          //lcd.clear();
          Serial3.println("not sufficiently explored");
          //apply flood fill for 9 by 5
          flood_fill(tempGoalx,tempGoaly, rows_wall, cols_wall);
 
          //next dir to 9 by 5
          nextdir = findnextdir(i,j, oldi, oldj, rows_wall,cols_wall);
          
         //if you can't find a direction then redo
          while (nextdir == 5) {
            readAndstore(i,j,compassd, swapdir);
            flood_fill(tempGoalx,tempGoaly, rows_wall, cols_wall);
            nextdir = findnextdir(i,j,oldi,oldj, rows_wall, cols_wall);
            //lcd.clear();
            Serial3.print("Processing...2");
          }

          // Update position
          //update oldi and oldj
          oldi = i;
          oldj = j;
          
          newpos(i,j,nextdir);  //updates north count as well
          
          //adjust direction to reflect reality
          if (swapdir) {
            if (nextdir == EAST) {
              nextdir = WEST;
            } else if (nextdir == WEST) {
              nextdir = EAST;
            }
          }
          compassd = tcompass;

          // next direction is decided a this point (after floodfill)
          // store into an array
          // nextdir variable
          
          moveCount++;
          
          // iterate over pathOne/pathTwo arrays for sequence from Start to End
          moveRobot(compassd, nextdir, moveCount);
        }
      break;
      
      default:
      break;
    }
    
    break;

    case(MODE_SPEEDRUN):
      goal = false;
      i = 0;  //assuming starting back at the start
      j = 0;
      oldi = 0;
      oldj = 0;
      nextdir = 6;
      for (int w = 0; w < 40; w++) {
        Serial3.print(pathTwo[w]); 
      }
      
      //flood_fill(pgoalx,pgoaly, rows_wall, cols_wall);
      
      // make a finalised path array that you go through
      // TODO: MAKE RobotMove()
      Serial3.print("Doing speed run");
      while(!goal) 
      {

        for (int j = 0; j < sizeof(pathTwo); j++)
        {
        while (!gyro()) 
        {
            delay(50);
            gyro();     
        }

        compassd = compass(); //update compass 
        nextdir = pathTwo[j];
          if (nextdir == 6) 
          {
            robotStop();
            break;
          }
          moveRobotSpeed(compassd, nextdir);
          if (digitalRead(SPEED_RUN) == LOW) 
          {
            break;
          }
                  
        }
        Serial3.println("Complete");
        //Serial3.print(nextdir); Serial3.println(" ");
        //moveRobot(compassd, pathTwo[moveCount], moveCount);
        
        goal = true; //at end of sequence should be at goal anyway.
        break; //end LED sequence
      }

      if (goal == true && digitalRead(SPEED_RUN) == HIGH) 
      {
      // LED RED turns on to indicate goal is reached (for speed run - Explore)
      statusGreen::write(logic_level::low);
      statusRed::write(logic_level::high); 
      system_mode = 10;
      } else 
      {
        system_mode = MODE_OFF;
        delay(7000);
      }
      
    break;

    default:
    break;
  }
}
  
void newpos(short &i, short &j, short &compassd) {
  if (compassd == 0) 
    i++;
  else if (compassd == 2)
    i--;        
  else if (compassd == 1) 
    j--;
  else if (compassd == 3) 
    j++;
}

void moveRobotSpeed(short compassd, short nextdir) {
  //Serial3.print("compass ="); Serial3.println(compassd);
  if (compassd == nextdir) {
    Serial3.println("-Forward");
    resetAllEncoders();
    delay(200);
    robotForward(SPEEDRUN_DISTANCE,SPEEDRUN_SPEED);
    delay(50);
    resetAllEncoders();
    //lcd.clear();
  } else if (nextdir - compassd == 1) {
    Serial3.println("-Right");
    resetAllEncoders();
    delay(50);
    robotRightSpeed();
    delay(50);
    resetAllEncoders();
  } else if (compassd - nextdir == 1) {
    Serial3.println("-Left");
    resetAllEncoders();
    delay(50);
    robotLeftSpeed();
    delay(50);
    resetAllEncoders();
  } else if ((nextdir - compassd == 2) || (compassd - nextdir == 2)) {
    Serial3.println("-Reverse (Left)");
    resetAllEncoders();
    delay(50);
    robotReverse();
    delay(50);
    resetAllEncoders();
  } else if (nextdir - compassd == 3) {
    Serial3.println("-Left");
    resetAllEncoders();
    delay(50);
    robotLeftSpeed();
    delay(50);
    resetAllEncoders();
  } else if (compassd - nextdir == 3) {
    Serial3.println("-Right");
    resetAllEncoders();
    delay(50);
    robotRightSpeed();
    delay(50);
    resetAllEncoders();
  } 
  delay(50);
}

void moveRobot(short compassd, short nextdir, int moveCount) {
  //Serial3.print("compass ="); Serial3.println(compassd);
  if (compassd == nextdir) {
    //lcd.clear();
    //Serial3.print(moveCount);
    Serial3.println("-Forward");
    resetAllEncoders();
    delay(200);
    robotForward(STRAIGHT_DISTANCE,STRAIGHT_SPEED); //go forward one cell
    delay(50);
    resetAllEncoders();
    //lcd.clear();
  } else if (nextdir - compassd == 1) {
    ////lcd.clear();
    //Serial3.print(moveCount);
    Serial3.println("-Right");
    resetAllEncoders();
    delay(50);
    robotRight();
    delay(50);
    resetAllEncoders();
    //lcd.clear();
   // Serial3.println("Stop");
  } else if (compassd - nextdir == 1) {
    //lcd.clear();
    //Serial3.print(moveCount);
    Serial3.println("-Left");
    resetAllEncoders();
    delay(50);
    robotLeft();
    delay(50);
    resetAllEncoders();
    //lcd.clear();
    //Serial3.println("Stop");
  } else if ((nextdir - compassd == 2) || (compassd - nextdir == 2)) {
    //lcd.clear();
    //Serial3.print(moveCount);
    Serial3.println("-Reverse (Left)");
    resetAllEncoders();
    delay(50);
    robotReverse();
    delay(50);
    resetAllEncoders();
    //lcd.clear();
    //Serial3.println("Stop");
  } else if (nextdir - compassd == 3) {
    //lcd.clear();
    //Serial3.print(moveCount);
    Serial3.println("-Left");
    resetAllEncoders();
    delay(50);
    robotLeft();
    delay(50);
    resetAllEncoders();
    //lcd.clear();
    //Serial3.print("Stop");
  } else if (compassd - nextdir == 3) {
    //lcd.clear();
    //Serial3.print(moveCount);
    Serial3.println("-Right");
    resetAllEncoders();
    delay(50);
    robotRight();
    delay(50);
    resetAllEncoders();
    //lcd.clear();
    //Serial3.println("Stop");
  } 
  delay(100);
}

//different flood fills
void flood_fill(int mouseSRow, int mouseSColumn, int rows_wall, int cols_wall) {

  int mazeValueChanged = 1;
  int CurrentExploredValue = 0;
  int N = (rows_wall*cols_wall)-1;
  
  //Set start cell to 0
  for(int row = 0;row < rows_wall; row++){
    for(int col = 0; col < cols_wall; col++){
       if(row == mouseSRow && col == mouseSColumn){
         values[mouseSRow][mouseSColumn] = 0;
       }else {
         values[row][col] = N;
       }
    }
  }
 
//  //Do floodfill
  while(mazeValueChanged != 0){
    mazeValueChanged = 0;
    for(int FloodR = 0; FloodR < rows_wall; FloodR++){
        for(int FloodC = 0; FloodC < cols_wall; FloodC++){
            if(values[FloodR][FloodC] == CurrentExploredValue){
                //check NORTH
                if(FloodR != rows_wall-1){
                    if((Hmap[FloodR+1][FloodC] != 1) && (values[FloodR+1][FloodC] == N)){
                        values[FloodR+1][FloodC] = values[FloodR][FloodC] + 1;
                        mazeValueChanged = 1;
                    }
                } 
                
                if(FloodC != 0){ //check EAST
                    if((Vmap[FloodR][FloodC] != 1) && (values[FloodR][FloodC-1] == N)){
                        values[FloodR][FloodC-1] = values[FloodR][FloodC]+1;
                        mazeValueChanged = 1;
                    }
                }
                
                if(FloodR != 0){ //checl SOUTH
                    if((Hmap[FloodR][FloodC] != 1) && (values[FloodR-1][FloodC] == N)){
                        values[FloodR-1][FloodC] = values[FloodR][FloodC]+1;
                        mazeValueChanged = 1;
                    }
                }
                
                if(FloodC != cols_wall - 1){ //check WEST
                    if((Vmap[FloodR][FloodC+1] != 1)&& (values[FloodR][FloodC+1] == N)){
                        values[FloodR][FloodC+1] = values[FloodR][FloodC]+1;
                        mazeValueChanged = 1;                 
                    }
                }   
            }          
        }
    }
    CurrentExploredValue += 1;
  } 
}


//different flood fills
void flood_fill2(int mouseSRow, int mouseSColumn, int rows_wall, int cols_wall) {
  Serial.println("printing map2");
  //printMaze2(1, rows_wall, cols_wall, tempGoalx, tempGoaly);
  int mazeValueChanged = 1;
  int CurrentExploredValue = 0;
  int N = (rows_wall*cols_wall)-1;
  
  //Set start cell to 0
  for(int row = 0;row < rows_wall; row++){
    for(int col = 0; col < cols_wall; col++){
       if(row == mouseSRow && col == mouseSColumn){
         ConvertToWalls[mouseSRow][mouseSColumn] = 0;
       }else {
         ConvertToWalls[row][col] = N;
       }
    }
  }
 
//  //Do floodfill
  while(mazeValueChanged != 0){
    mazeValueChanged = 0;
    for(int FloodR = 0; FloodR < rows_wall; FloodR++){
        for(int FloodC = 0; FloodC < cols_wall; FloodC++){
            if(ConvertToWalls[FloodR][FloodC] == CurrentExploredValue){
                //check NORTH
                if(FloodR != rows_wall-1){
                    if((Hmap2[FloodR+1][FloodC] != 1) && (ConvertToWalls[FloodR+1][FloodC] == N)){
                        ConvertToWalls[FloodR+1][FloodC] = ConvertToWalls[FloodR][FloodC] + 1;
                        mazeValueChanged = 1;
                    }
                } 
                
                if(FloodC != 0){ //check EAST
                    if((Vmap2[FloodR][FloodC] != 1) && (ConvertToWalls[FloodR][FloodC-1] == N)){
                        ConvertToWalls[FloodR][FloodC-1] = ConvertToWalls[FloodR][FloodC]+1;
                        mazeValueChanged = 1;
                    }
                }
                
                if(FloodR != 0){ //checl SOUTH
                    if((Hmap2[FloodR][FloodC] != 1) && (ConvertToWalls[FloodR-1][FloodC] == N)){
                        ConvertToWalls[FloodR-1][FloodC] = ConvertToWalls[FloodR][FloodC]+1;
                        mazeValueChanged = 1;
                    }
                }
                
                if(FloodC != cols_wall - 1){ //check WEST
                    if((Vmap2[FloodR][FloodC+1] != 1)&& (ConvertToWalls[FloodR][FloodC+1] == N)){
                        ConvertToWalls[FloodR][FloodC+1] = ConvertToWalls[FloodR][FloodC]+1;
                        mazeValueChanged = 1;                 
                    }
                }   
            }          
        }
    }
    CurrentExploredValue += 1;
  } 
}

void printflood(int rows_wall, int cols_wall) {
  for (int i = 0; i < rows_wall; i++) {
    for (int j = 0; j < cols_wall; j++) {
      Serial3.print(values[i][j]); 
      Serial3.print("  ");
    }
    Serial3.println();
  }
}

void printflood2(int rows_wall, int cols_wall) {
  for (int i = 0; i < rows_wall; i++) {
    for (int j = 0; j < cols_wall; j++) {
      Serial3.print(ConvertToWalls[i][j]); 
      Serial3.print("  ");
    }
    Serial3.println();
  }
}

//call this after finding orientation of maze
//isolating fastest path going from sstart to goal
//note all these directions are relative to the matrix and not reality
//flip east and west if need be
void pathOneCalc() {
  
short CurrRowCheck = 0;
short CurrColCheck = 0;
short prevCurrRowCheck = 0;
short prevCurrColCheck = 0;
short nextdir = 5;
int count = 0;
for (int i = 0; i < 40; i++) {
  pathOne[i] = 6;
}

flood_fill(pgoalx, pgoaly,prows, pcols);

while (CurrRowCheck != pgoalx || CurrColCheck != pgoaly) {
  //pathOne[CurrRowCheck][CurrColCheck] = values[CurrRowCheck][CurrColCheck];
  // might be issue with next dir
  nextdir = findnextdir(CurrRowCheck, CurrColCheck, prevCurrRowCheck, prevCurrColCheck, prows, pcols);
  pathOne[count] = nextdir;
  count++;
  newpos(CurrRowCheck, CurrColCheck, nextdir);
}

Serial3.println("Path 1");
for (int t = 0; t < 40; t++) {
  Serial3.print(pathOne[t]);
}
Serial3.println("");
}

//isolate second path where unexplored is walls
void pathTwoCalc() {

for (int i = 0; i <prows+1; i++) {
  for (int j = 0; j<pcols+1; j++) {
    Hmap2[i][j] = Hmap[i][j];
    Vmap2[i][j] = Vmap[i][j];
    if (Hmap2[i][j] == 2) 
      Hmap2[i][j] = 1;
    if (Vmap2[i][j] == 2)
      Vmap2[i][j] = 1;
  }
}
//printMaze(1, prows,pcols, tempGoalx, tempGoaly);

//flood fill where unexplored is walls
flood_fill2(pgoalx, pgoaly, prows, pcols);
//printflood2(prows, pcols);
Serial3.println("Flood 2 printed");

short CurrRowCheck = 0;
short CurrColCheck = 0;
short prevCurrRowCheck = 0;
short prevCurrColCheck = 0;
short nextdir = 5;
int count = 0;

for (int i = 0; i < 40; i++) {
  pathTwo[i] = 6;
}

while (CurrRowCheck != pgoalx || CurrColCheck != pgoaly) {
  //pathTwo[CurrRowCheck][CurrColCheck] = ConvertToWalls[CurrRowCheck][CurrColCheck];
  nextdir = findnextdir2(CurrRowCheck, CurrColCheck, prevCurrRowCheck, prevCurrColCheck, prows, pcols);
  pathTwo[count] = nextdir;
  count++;
  newpos(CurrRowCheck, CurrColCheck, nextdir);
  if (nextdir == 5)
    break;
}
Serial3.println("Path 2");
for (int w = 0; w < 40; w++) {
  Serial3.print(pathTwo[w]); 
}
Serial3.println("");

}

//returns 0 if not sufficiently explored
//returns 1 if exploring is done
bool compare_path() {
  printMaze(1, prows, pcols, tempGoalx, tempGoaly);
  bool doneExplore = true;

  //update the isolated paths
  pathOneCalc();
  pathTwoCalc();
  //Serial3.println("Path One");
  //printflood(prows, pcols);
  ////lcd.clear();
  Serial3.println("Comparing Path Function Check");
  
  //compare the two paths
  for (int i = 0; i<40; i++) {
    if (pathOne[i] != pathTwo[i])
    {
      doneExplore = false; //lower flag
    }
  }
  
  Serial.println("");
  
  return doneExplore;
}
