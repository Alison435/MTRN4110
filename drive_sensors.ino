// Combining Driving and Exploration 
// TODO: include logic for wall detection and move decisions (EXPLORATION)

// Including API Libraries
#include <units.h>
#include <hardware.h>
#include <hardware_definition.h>
#include <Wire.h>
#include <Sensors.h>

// Avoids prepending scope operator to all functions
using namespace hardware;

// Motor/Wheel parameters
#define COUNT_PER_REV       1500.0  // 16 CPR * 120:1 gear ratio
#define CIRCUM              240.0 // mm
#define CELL_LENGTH         250.0 // mm
#define STRAIGHT_DISTANCE   200.0

// wall detection
#define SIDEMAX 50
#define SIDEMIN 20
#define FRONTMAX 60
#define FRONTMIN 30


#define ROW 6  //number of row and col to print out = num of cells + 1
#define COL 10


// Test Defines
//#define USING_LIDAR
//#define USING_MOTORS

bool robot_start = false; //will be set to true once LED sequence is finished
int movement_num = 0;

//arrays for map
int Hmap[10][10];  //global maze map horizontal extended
int Vmap[10][10];  //global maze map vertical extended
int Hmaze[ROW][COL];  //truncated maze
int Vmaze[ROW][COL];
int printonce = 0;

// Encoder variables for Phase A
volatile int eCountR = 0; 
volatile int eCountL = 0; 
volatile byte pinAcountR;
volatile byte pinAcountL;
boolean DirectionR = true; // Rotation direction for right motor
boolean DirectionL = true; // Rotation direction for left motor

void setUpUltrasonic()
{
  //Ultrasonic set up
  pinMode(TRIGGER_PIN, OUTPUT); // Sets the TRIGGER_PIN as an Output
  pinMode(ECHO_PIN, INPUT); // Sets the ECHO_PIN as an Input
  //Serial.println("Ultrasonic initialised");
}

void setUpLidars()
{
 // #ifdef USING_LIDAR
     
  pinMode(LIDARONE,OUTPUT);
  pinMode(LIDARTWO,OUTPUT);
  digitalWrite(LIDARTWO, HIGH);//enable lidar two
  digitalWrite(LIDARONE, LOW); //shut down lidar one before chaning address
  delay(100);
  lidarTwo.setAddress(0x30);
  delay(10);
  digitalWrite(LIDARONE, HIGH);//enable lidar one 
  
  lidarTwo.init();
  Serial.println("Lidar Two initialised");
  lidarTwo.configureDefault();
  lidarTwo.setTimeout(500);
  
  lidarOne.init();              //lidar one uses default address
  Serial.println("Lidar One initialised");
  lidarOne.configureDefault();
  lidarOne.setTimeout(500);
    
// #endif USING_LIDAR

}

void setupimu() {
  pinMode(INTERRUPT_PIN, INPUT);
  imu.initialize();
  Serial.println(imu.testConnection() ? "IMU initialised" : "imu6050 connection failed");
  devStatus = imu.dmpInitialize();
  
  Serial.println("Starting offset");
  int TheOffsets[6] = {-4091, -587, 2145, 299, -26, 43};
  SetOffsets(TheOffsets);
  Serial.println("Offsets done");

  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    imu.setDMPEnabled(true);
  
    // enable Arduino interrupt detection
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
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
    Serial.print(F("DMP Initialization failed (code "));
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
  double offset = 0.35;  // offset amount to compensate Right vs. Left drive

  numRev = distance / CIRCUM;  // calculate the target # of rotations
  targetCount = numRev * COUNT_PER_REV;    // calculate the target count
 
  // Reset encoders
  eCountL = 0;
  eCountR = 0;
  delay(50);

  motorControl(1,0,rightPWM,leftPWM);

  while (abs(eCountR) < targetCount)
  {
    motorControl(1,0,rightPWM,leftPWM+0.5);

    // calculate the rotation "speed" as a difference in the count from previous cycle.
    lDiff = (abs(eCountL) - prevlCount);
    rDiff = (abs(eCountR) - prevrCount);

    // store the current count as the "previous" count for the next cycle.
    prevlCount = -eCountL;
    prevrCount = -eCountR;

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
  
  robotStop();
  delay(1000);
}

// Functions for motor control and turning
// Implementing Proportional Controller for differential drive system

void robotStop()
{
  right_motor::stop();
  left_motor::stop();
}

void resetAllEncoders()
{
  eCountL = 0;
  eCountR = 0;
  delay(10);
}

void robotTurn(int directionVal)
{

  float setSpeedPerc = 30.0;

  resetEncoderR();
  resetEncoderL();

  if (directionVal == 0)  //Turn to left (CCW)
  {
    //Right + and Left -

    while (abs(eCountR) <= COUNT_PER_REV/3.10)
    {
      motorControl(0,0,setSpeedPerc,setSpeedPerc);    
    }
    
    resetEncoderR();
    resetEncoderL();
    robotStop();
    delay(500);
  }

  else //directionVal is 1
  {
    //-ve so turn to right (CW)
    //Right - and Left +

    while (abs(eCountR) <= COUNT_PER_REV/3.50)
    {
      motorControl(1,1,setSpeedPerc,setSpeedPerc);  
    }

  resetEncoderR();
  resetEncoderL(); 
  robotStop();    
  delay(1000); 
  }  
}

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

void startLEDSequence()
{
  //Green ON and Red OFF
  statusGreen::write(logic_level::high);
  statusRed::write(logic_level::low);   
}

void robotLeft()
{
  resetAllEncoders();   
  robotTurn(0); //turn CCW on spot
  resetAllEncoders();
  robotForward(STRAIGHT_DISTANCE,30.0); //go forward on cell
  resetAllEncoders();  
}

void robotRight()
{
  resetAllEncoders();
  robotTurn(1); //turn CW on spot
  resetAllEncoders();
  robotForward(STRAIGHT_DISTANCE,30.0); //go forward one cell
  resetAllEncoders();  
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
    //need to test accuracy of measurements
    Serial.println(distance);
    if (distance <= 40) {
      count += 1;
      if (count == 5) {
        seen = true;
        Serial.println("seen");
      }
    } else if (seen == true) {
      ucount += 1;
      Serial.print("unseen");
      if (ucount == 5) {
        break;
      }
    }
    
  }
  Serial.println("starting");
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
  if (gzf >= -45 && gzf <= 45) {
      temp = NORTH;
    } else if (gzf > 45 && gzf <= 135)
      temp = EAST;
    else if (gzf > 135 || gzf < -135) {
      temp = SOUTH; 
    } else if (gzf >= -135 && gxf <= -45) {
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
  bool success = false;
  if (!dmpReady) return;

    // wait for imu interrupt or extra packet(s) available
    while (!imuInterrupt && fifoCount < packetSize) {

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
        Serial.println(F("FIFO overflow!"));

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
//        imu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        gxf = ypr[1]*180/M_PI;
        gyf = ypr[2]*180/M_PI;
        gzf = ypr[0]*180/M_PI;

        success = true;
    }
    return success;
}

//return the overall number of times you have gone north
//controls storage etc
int explore() {
  bool moved = true;    //for initial reading of start position
  short compassd; //contains compass direction
  int northCount = 0;
  bool ultrawall; //ultrasonic wall
  bool L1wall;  //lidar one wall
  bool L2wall;  //lidar two wall
  bool goal = 0;
  short i = 0;  //row
  short j = 0;  //col
  int walls[4];

  //Maze* my_maze;    /* maze for keeping track of flood values and walls */
  //Stack* my_stack;  /* stack used for flood fill */
  short directionw = NORTH;    /* keeps track of direction that mouse is moving in */
  //short x, y; /* keeps track of current row, col value mouse is in within maze */
  
  //my_maze = new_Maze();    /* Initialize new maze */
  //my_stack = new_Stack();  /* Initialize new stack */
  
  while (!goal) {

    //call gyro again if overflow occurs
    if (!gyro()) 
      gyro();
      
    //only run this loop if you have moved to another cell
    if (moved == true) {
      //record values
      //increment i or j depending on command sent
  
      //update compass and check if we have gone north or south
      compassd = compass();
      if (compassd == 0) 
        northCount++;
      else if (compassd = 2)
        northCount--; //decrease if you went south
  
      //returns 1 if wall and 0 if not based on max and min values
      ultrawall = wall(ultrasonicdist(), FRONTMAX, FRONTMIN);
      L1wall = wall(lidarOne.readRangeSingleMillimeters(), SIDEMAX, SIDEMIN);
      L2wall = wall(lidarTwo.readRangeSingleMillimeters(), SIDEMAX, SIDEMIN);
  
      //walls in array format - N E W S
      memset(walls, 0, sizeof(walls));  //reset the array before reallocating
      walls[compassd] = ultrawall;
      if (compassd == WEST) {
        walls[0] = L2wall;
        walls[compassd-1] = L1wall;
      } else if(compassd == NORTH) {
        walls[3] = L1wall;
        walls[compassd+1] = L2wall;
      }
  
      storemaze(compassd, i, j, L1wall, L2wall, ultrawall);
        //send new command and increment i and j based on the command 
      moved == false;
        //calculate where to move
      }
  }
}

void storemaze(short compassd, short i, short j, int L1wall, int L2wall, int ultrawall) {
    switch(compassd) {
    case NORTH:
      Hmap[i][j] = ultrawall;
      Vmap[i][j+1] = L2wall; //assume l2 is on the right when facing north
      Vmap[i][j] = L1wall;
      break;
    case EAST:
      Vmap[i][j+1] = ultrawall;
      Hmap[i][j] = L1wall;
      Hmap[i+1][j] = L2wall;
      break;
    case SOUTH:
      Hmap[i+1][j] = ultrawall;
      Vmap[i][j+1] = L1wall;
      Vmap[i][j] = L2wall;
      break;
    case WEST:
      Vmap[i][j] = ultrawall;
      Hmap[i][j] = L2wall;
      Hmap[i+1][j] = L1wall;
      break;
  }
}

//extract 5 by 9
void extract5by9() {
  for (int i = 0; i < ROW; i++) {
    for (int j = 0; j < COL; j++) {

      //set all outer edges to 1 since enclosed
      if (i == 0 || i == (ROW-1) || j == 0 || j == (COL -1)) {
        Vmaze[i][j] = 1;
        Hmaze[i][j] = 1;
      } else {
        Vmaze[i][j] = Vmap[i][j];
        Hmaze[i][j] = Hmap[i][j];  
      }
    }
  }
}

void extract9by5() {
  //go through columns first
  for (int j = 0; j < ROW; j++) {
    for (int i = 0; i < COL; j++) {

      //set outside walls
      if (i == 0 || i == (ROW-1) || j == 0 || j == (COL -1)) {
        Vmaze[i][j] = 1;
        Hmaze[i][j] = 1;
      } else {
        Vmaze[i][j] = Vmap[i][j];
        Hmaze[i][j] = Hmap[i][j];  
      }
    }
  }
}

//prints Hmaze and Vmaze combined
void printMaze(int northCount) {
  for(int i = 0;i < 2*(ROW-1)+1; i++) {
    for(int j = 0;j < 2*(COL-1)+1 ;j++)
    {
        //Add Horizontal Walls
        if(i%2 == 0 && j%2 == 1)
        {
            if(Hmap[i/2][j/2] == true)
              Serial.print(" ---");
            else if (Hmap[i/2][j/2] == false)
              Serial.print("    ");
            else if (Hmap[i/2][j/2] == 2)
              Serial.print(" ***");
        }

        //Add Vertical Walls
        if(i%2 == 1 && j%2 == 0)
        {
            if (i == 1 && j == 0) 
            {
              if (northCount == 5)
                Serial.print("| W "); 
              else if (northCount == 3)
                Serial.print("| S ");
            }
              
            else if(Vmap[i/2][j/2] == true)
              Serial.print("|   ");
            else if (Vmap[i/2][j/2] == false)
              Serial.print("    ");
            else if (Vmap[i/2][j/2] == 2)
              Serial.print("*   ");
          }
    }
    Serial.print("\n");
  }
}


void setup() {

  Serial.begin(9600);
  Serial1.begin(9600);  
  setupDigitalPins();
  setupMotor(); 
  setupEncoderWheel(); 
  setUpLidars();
  setUpUltrasonic();
  pinMode(13,OUTPUT); 
  delay(100);
}

// State Machine
#define MODE_OFF 1
#define MODE_EXPLORE 2
#define MODE_GOAL 3
int system_mode = MODE_OFF;

float distance;  

void loop()
{
  short compassd; //contains compass direction
  int northCount = 0;
  bool ultrawall; //ultrasonic wall
  bool L1wall;  //lidar one wall
  bool L2wall;  //lidar two wall
  bool goal = 0;
  short i = 0;  //row
  short j = 0;  //col
  
  // Start LED sequence:
  statusGreen::write(logic_level::low);
  statusRed::write(logic_level::high); 
  resetAllEncoders();

  if (!robot_start){
    Serial.println("robot starting phase");
    startphaseb();
  }
  
  if (robot_start) {
    statusGreen::write(logic_level::high);
    statusRed::write(logic_level::low);
  }
  
  // Using Bluetooth Module (COM31) on TX and RX (Serial Port 0)
  // Manual testing of exploration logic and actuation
  while (!goal) {
    if (Serial.available() > 0)   
    {
      drivemode = Serial.read();  
  
      // Physical motion after each decision is made
      // TODO: synthesise actuation with logic autonomously
      switch (drivemode)
      {
            
        case 's':
          startLEDSequence();     
          robotForward(STRAIGHT_DISTANCE,30.0); //go forward one cell         
          break;
  
        case 'l':
          startLEDSequence(); 
          robotLeft();
          break;
              
        case 'r':
          startLEDSequence();   
          robotRight();
          break;
  
        //reverse if dead end found
        case 'b':
          startLEDSequence();
          resetAllEncoders(); 
          robotTurn(0); //turn CCW on spot
          resetAllEncoders();
          delay(15);
          robotTurn(0); //turn CCW on spot;
          resetAllEncoders();
          break;

        //break from loop
        case 'd':
          goal = true;
          break;
          
        default:
          break;           
       }
  
       //we have gone straight
        compassd = compass();
      if (compassd == 0) 
      {
        northCount++; 
        i++;
      } else if (compassd == 2){
        northCount--; //decrease if you went south
        i--;        
      } else if (compassd == 1)
        j++;
        else if (compassd == 3)
        j--;
        
      //returns 1 if wall and 0 if not based on max and min values
      ultrawall = wall(ultrasonicdist(), FRONTMAX, FRONTMIN);
      L1wall = wall(lidarOne.readRangeSingleMillimeters(), SIDEMAX, SIDEMIN);
      L2wall = wall(lidarTwo.readRangeSingleMillimeters(), SIDEMAX, SIDEMIN);
      storemaze(compassd, i, j, L1wall, L2wall, ultrawall);
  
  
    //Check which direction to go next
    //Lidar/Ultrasonic -> forward, reverse, right or left
     }     
  }
  if (printonce == 0)
  {
    printMaze(northCount);
    printonce++;
  }
}               
