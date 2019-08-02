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
#define SIDEMAX 110
#define SIDEMIN 10
#define FRONTMAX 110
#define FRONTMIN 10

// Avoids prepending scope operator to all functions
using namespace hardware;

// Motor/Wheel parameters
#define COUNT_PER_REV       1700.0  // 16 CPR * 120:1 gear ratio
#define CIRCUM              240.0 // mm
#define STRAIGHT_DISTANCE   200.0 // mm
#define LIDAR_MAX_SETPOINT  75.0 // mm
#define STRAIGHT_SPEED      25.0
#define COUNT_PER_REV_TURN  1650.0  // 16 CPR * 120:1 gear ratio

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
bool SExplored;

//global cases for flood fill map
int values[9][9];
int ConvertToWalls[9][9];
int pathOne[9][9];
int pathTwo[9][9];


// Encoder variables for Phase A
volatile int eCountR = 0; 
volatile int eCountL = 0; 
volatile byte pinAcountR;
volatile byte pinAcountL;
boolean DirectionR = true; // Rotation direction for right motor
boolean DirectionL = true; // Rotation direction for left motor

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
double K_p_lidar = 0.03;
double K_d_lidar = 0.00;

// Constants for Encoder
double K_p_encoder = 0.06;
double K_d_encoder = 0.03;
double K_i_encoder = 0.01;
//------------------------------------------------------//

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
  double offset = 2.0;  // offset amount to compensate Right vs. Left drive

  numRev = distance / CIRCUM;  // calculate the target # of rotations
  targetCount = numRev * (COUNT_PER_REV);    // calculate the target count
 
  // Reset encoders
  eCountL = 0;
  eCountR = 0;
  delay(15);

  motorControl(1,0,rightPWM,leftPWM);

  while (abs(eCountL) < targetCount)
  {
    motorControl(1,0,rightPWM,leftPWM);

    // Error count from encoder 'ticks'
    lDiff = (abs(eCountL) - prevlCount)/3;
    rDiff = (abs(eCountR) - prevrCount)/3;

    // Have walls on either side (Part 3 of Phase B)
    // L1 = 65. L2 = 45. Bias = 15
    // L1 = RIGHT and L2 = LEFT
    //if (use_lidar == true)
    // readings around 

    if ((lidarOne.readRangeSingleMillimeters() <= LIDAR_MAX_SETPOINT && lidarTwo.readRangeSingleMillimeters() <= LIDAR_MAX_SETPOINT))
    {
        error_P_lidar = (lidarOne.readRangeSingleMillimeters() - lidarTwo.readRangeSingleMillimeters() - 5.0);
        error_D_lidar = error_P_lidar - prev_error_lidar;
  
        //K_p and K_d (for lidar)  
        totalError = (K_p_lidar * error_P_lidar) + (K_d_lidar * error_D_lidar);
        prev_error_lidar = error_P_lidar;
  
        if (lidarOne.readRangeSingleMillimeters() > lidarTwo.readRangeSingleMillimeters())
        {
          leftPWM += 2.0;
          rightPWM -= 2.0;

          if (leftPWM > setSpeedPerc || rightPWM > setSpeedPerc)
          {
            leftPWM = setSpeedPerc;
            rightPWM = setSpeedPerc; 
          }
        }
        else if (lidarOne.readRangeSingleMillimeters() < lidarTwo.readRangeSingleMillimeters())
        {
          leftPWM -= 2.0;
          rightPWM += 2.0;

          if (leftPWM >= setSpeedPerc || rightPWM >= setSpeedPerc)
          {
            leftPWM = setSpeedPerc;
            rightPWM = setSpeedPerc; 
          }
        }     
     }
    
      double e_error = (lDiff - rDiff); //encoder error between left and right wheel
      sumError += e_error;      
      double dError = (lDiff - rDiff) - prev_error_encoder;

      totalError = (K_p_encoder * e_error) - (K_d_encoder * setSpeedPerc) + 
      (K_i_encoder * sumError);

      leftPWM += totalError/2.5;  
      rightPWM -= totalError/2.5;

      if (leftPWM >= setSpeedPerc || rightPWM >= setSpeedPerc)
      {
        leftPWM = setSpeedPerc;
        rightPWM = setSpeedPerc; 
      }

      prev_error_lidar = e_error; //store previous error for next cycle-        
      // Store previous count  
      prevlCount = abs(eCountL);
      prevrCount = abs(eCountR);

  }
  robotStop();
  delay(10);
  resetAllEncoders();
}

void robotStop()
{
  right_motor::stop();
  left_motor::stop();
  delay(15);
}

void robotTurn(int directionVal)
{

  float setSpeedPerc = 25.0;

  resetEncoderR();
  resetEncoderL();

  if (directionVal == 1)  //Turn to left (CCW)
  {
    //Right + and Left -

    while (abs(eCountR) <= COUNT_PER_REV_TURN/3.50)
    {
      motorControl(0,0,setSpeedPerc,setSpeedPerc);    
    }
    
    resetEncoderR();
    resetEncoderL();
    robotStop();
    delay(200);
  }

  else //directionVal is 0
  {
    //-ve so turn to right (CW)
    //Right - and Left +

    while (abs(eCountR) <= COUNT_PER_REV_TURN/3.10)
    {
      motorControl(1,1,setSpeedPerc,setSpeedPerc);  
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
  resetAllEncoders();
  delay(100);
  robotForward(STRAIGHT_DISTANCE,STRAIGHT_SPEED); //go forward on cell
  resetAllEncoders();  
}

void robotRight()
{
  resetAllEncoders();
  robotTurn(1); //turn CW on spot
  resetAllEncoders();
  delay(100);
  robotForward(STRAIGHT_DISTANCE,STRAIGHT_SPEED); //go forward one cell
  resetAllEncoders();  
}

void robotReverse() {
  resetAllEncoders();
  robotTurn(1); //turn CW on spot
  delay(250);
  resetAllEncoders();
  robotTurn(1);
  resetAllEncoders();
  delay(250);
  robotForward(STRAIGHT_DISTANCE,STRAIGHT_SPEED); //go forward one cell
  delay(250);
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
  Serial3.print("Ultrawall = "); Serial3.println(ultrawall);

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
            } else if (i == (goalx*2 +1) && j == (goaly*2)) {
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
      bool swapdir;
      short nextdir;
      bool sizedet;
      int goalx;  //goal in a 9 by 9
      int goaly;
      int moveCount;
      float distance;
      SExplored = false;

      //initial flood fill values
      goalx = 5;
      goaly = 5;
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
        //Serial3.print("Current dir = "); Serial3.println(compassd); 

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
        
        flood_fill(goalx,goaly,rows_wall, cols_wall);
        
        nextdir = findnextdir(i,j, oldi, oldj, rows_wall, cols_wall);

        //if you can't find a direction then redo
        while (nextdir == 5) {
          readAndstore(i,j,compassd, swapdir);
          flood_fill(goalx,goaly, rows_wall, cols_wall);
          nextdir = findnextdir(i,j,oldi,oldj, rows_wall, cols_wall);
          lcd.clear();
          lcd.print("Processing...");
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
        if (i >= goalx) {
          initialheading = SOUTH;
          pheading = initialheading;
          sizedet = true;
          //lcd.clear();
          //lcd.print("GOAL 1.1 REACHED");
          //delay(1000);
          break;
        } else if (j >= goaly) {
          initialheading = WEST;
          pheading = initialheading;
          sizedet = true;
          //lcd.clear();
          //lcd.print("GOAL 1.2 REACHED");
          //delay(1000);
          break;
        }
       
    }

    //lcd.clear();
    //lcd.print("NEW GOAL");

    //reset size and goal
    switch(initialheading){
      case(WEST):
        rows_wall = 5;
        cols_wall = 9;
        prows = rows_wall;
        pcols = cols_wall;
        goalx = 2;
        goaly = 4;
        pgoalx = goalx;
        pgoaly = goaly;

        while(!SExplored) {
          
          while (!gyro()) {
            delay(50);
            gyro();     
          }
          compassd = compass(); //update compass
          tcompass = compassd; //store value into temp
          compassd = othersp(swapdir, compassd);
          readAndstore(i, j, compassd, swapdir);  

          //compare paths
          //if explored break, if not continue exploring
          //find a new goal based on unexplored arrays 
          //then feed the new goal to flood fill and head there
          if (compare_path()) 
          {
            SExplored = true;
            system_mode = MODE_SPEEDRUN;
            break;
          } else {
            //calculate new goal
          }
          
          //apply flood fill for 5 by 9
          flood_fill(goalx,goaly, rows_wall, cols_wall);
         
          //next dir to 5 by 9
          nextdir = findnextdir(i,j, oldi, oldj, rows_wall,cols_wall);
          
          //if you can't find a direction then redo
          while (nextdir == 5) {
            readAndstore(i,j,compassd, swapdir);
            flood_fill(goalx,goaly, rows_wall, cols_wall);
            nextdir = findnextdir(i,j,oldi,oldj, rows_wall, cols_wall);
            lcd.clear();
            lcd.print("Processing...1");
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
        goalx = 4;
        goaly = 2;  
        pgoalx = goalx;
        pgoaly = goaly;

        while(!SExplored) {

          while (!gyro()) {
            delay(50);
            gyro();     
          }
          compassd = compass(); //update compas
          tcompass = compassd;
          compassd = othersp(swapdir, compassd);
          readAndstore(i, j, compassd, swapdir);  

          //compare paths
          //if explored break, if not continue exploring
          //find a new goal based on unexplored arrays 
          //then feed the new goal to flood fill and head there
          if (compare_path()) 
          {
            SExplored = true;
            system_mode = MODE_SPEEDRUN;
            break;
          } else {
            //calculate new goal
          }
          
          //apply flood fill for 9 by 5
          flood_fill(goalx,goaly, rows_wall, cols_wall);
 
          //next dir to 9 by 5
          nextdir = findnextdir(i,j, oldi, oldj, rows_wall,cols_wall);
          
         //if you can't find a direction then redo
          while (nextdir == 5) {
            readAndstore(i,j,compassd, swapdir);
            flood_fill(goalx,goaly, rows_wall, cols_wall);
            nextdir = findnextdir(i,j,oldi,oldj, rows_wall, cols_wall);
            lcd.clear();
            lcd.print("Processing...2");
          }
          
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

          moveCount++;
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
      
      bool startSpeed = false;
      while(startSpeed == false) 
      {

        //flick of switch to break out of loop and start the speed run  
        if (digitalRead(SPEED_RUN) == HIGH)
        {
          startSpeed = true;
        }      
      }

      // make a finalised path array that you go through
      // TODO: MAKE RobotMove() WITH FASTER PWM
      
      while(!goal) 
      {
        if (i == pgoalx && j == pgoaly) 
        {
          goal = true;
          break;  
        }
        
        compassd = compass(); //update compass

        //path in values array will be the same as path in isolated arrays - does not prioritise straight though
        nextdir = findnextdir(i,j, oldi, oldj, prows,pcols);
        oldi = i;
        oldj = j;
        newpos(i,j,nextdir);
        moveCount++;          //no longer needed if not using LCD
        moveRobot(compassd, nextdir, moveCount);
      }
 
      statusGreen::write(logic_level::low);
      statusRed::write(logic_level::high); 
      Serial3.println("printing");
      //printMaze(pheading, prows, pcols, pgoalx, pgoaly);

      system_mode = 10;
      
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


void moveRobot(short compassd, short nextdir, int moveCount) {
  //Serial3.print("compass ="); Serial3.println(compassd);
  if (compassd == nextdir) {
    //lcd.clear();
    //lcd.print(moveCount);
    //lcd.print("-Forward");
    resetAllEncoders();
    delay(50);
    robotForward(STRAIGHT_DISTANCE,STRAIGHT_SPEED); //go forward one cell
    delay(50);
    resetAllEncoders();
    //lcd.clear();
    //lcd.write("Stop");
  } else if (nextdir - compassd == 1) {
    //lcd.clear();
    //lcd.print(moveCount);
    //lcd.print("-Right");
    resetAllEncoders();
    delay(50);
    robotRight();
    delay(50);
    resetAllEncoders();
    //lcd.clear();
    //lcd.print("Stop");
  } else if (compassd - nextdir == 1) {
    //lcd.clear();
    //lcd.print(moveCount);
    //lcd.print("-Left");
    resetAllEncoders();
    delay(50);
    robotLeft();
    delay(50);
    resetAllEncoders();
    //lcd.clear();
    //lcd.print("Stop");
  } else if ((nextdir - compassd == 2) || (compassd - nextdir == 2)) {
    //lcd.clear();
    //lcd.print(moveCount);
    //lcd.print("-Reverse (Left)");
    resetAllEncoders();
    delay(50);
    robotReverse();
    delay(50);
    resetAllEncoders();
    //lcd.print("Stop");
  } else if (nextdir - compassd == 3) {
    //lcd.clear();
    //lcd.print(moveCount);
    //lcd.print("-Left");
    resetAllEncoders();
    delay(50);
    robotLeft();
    delay(50);
    resetAllEncoders();
    //lcd.clear();
    //lcd.print("Stop");
  } else if (compassd - nextdir == 3) {
    //lcd.clear();
    //lcd.print(moveCount);
    //lcd.print("-Right");
    resetAllEncoders();
    delay(50);
    robotRight();
    delay(50);
    resetAllEncoders();
    //lcd.clear();
    //lcd.print("Stop");
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

for (int i = 0; i < prows; i++) {
  for (int j = 0; j < pcols; j++) {
    pathOne[i][j] = prows*pcols+1;
  }
}

flood_fill(pgoalx, pgoaly,prows, pcols);

while (CurrRowCheck != prows && CurrColCheck != pcols) {
  pathOne[CurrRowCheck][CurrColCheck] = values[CurrRowCheck][CurrColCheck];
  nextdir = findnextdir(CurrRowCheck, CurrColCheck, prevCurrRowCheck, prevCurrColCheck, prows, pcols);
  newpos(CurrRowCheck, CurrColCheck, nextdir);
}

}

//isolate second path where unexplored is walls
void pathTwoCalc() {

for (int i = 0; i <prows+1; i++) {
  for (int j = 0; j<pcols+1; j++) {
    Hmap2[i][j] = Hmap[i][j];
    Vmap2[i][j] = Vmap[i][j];
    if (Hmap2[i][j] == 2) 
      Hmap2[i][j] = 1;
    if (Vmap2[i][j] = 2)
      Vmap2[i][j] = 1;
  }
}
//flood fill where unexplored is walls
flood_fill2(pgoalx, pgoaly, prows, pcols);

short CurrRowCheck = 0;
short CurrColCheck = 0;
short prevCurrRowCheck = 0;
short prevCurrColCheck = 0;
short nextdir = 5;

for (int i = 0; i < prows; i++) {
  for (int j = 0; j < pcols; j++) {
    pathTwo[i][j] = prows*pcols+1;
  }
}

while (CurrRowCheck != prows && CurrColCheck != pcols) {
  pathTwo[CurrRowCheck][CurrColCheck] = ConvertToWalls[CurrRowCheck][CurrColCheck];
  nextdir = findnextdir2(CurrRowCheck, CurrColCheck, prevCurrRowCheck, prevCurrColCheck, prows, pcols);
  newpos(CurrRowCheck, CurrColCheck, nextdir);
}

}

//returns 0 if not sufficiently explored
//returns 1 if exploring is done
bool compare_path() {
  bool doneExplore = true;

  //update the isolated paths
  pathTwoCalc();
  pathOneCalc();
  
  //compare the two paths
  for (int i = 0; i < prows; i++) {
    for (int j = 0; j < pcols; j++) {
      if (pathOne[i][j] != pathTwo[i][j])
        doneExplore = false; 
    }
  }
  return doneExplore;
}
