

//2xLidars, Ultrasonic and IMU sensors - Phase A - 2
//Alison Truong

//using imu6050 offset-finder 2016-10-19 by Robert R. Fenichel (bob@fenichel.net)

//lidar one is on the right and lidar two is on the left of the board

#include <external_MPU6050_6Axis_MotionApps20.h>
#include <external_MPU6050_I2Cdev.h>
#include <external_I2CIO.h>
#include <external_VL6180X.h>

#include <Wire.h>

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

//uno - sda: A4 scl:A5
//mega - sda:20 scl:21

int trigPin = 11;  //change for mega
int echoPin = 12;  //change for mega
int lidarone = 13;
int lidartwo = 7;
float imuAscale = 16384.0;  //from datasheet
float imuGscale = 131.0;    //from datasheet
int16_t ax, ay, az;
uint8_t gx, gy, gz;

float axf, ayf, azf;
float gxf, gyf, gzf;

uint32_t oldtime;
uint32_t currtime;

int input;

//variables for imu calibration
const char LBRACKET = '[';
const char RBRACKET = ']';
const char COMMA    = ',';
const char BLANK    = ' ';
const char PERIOD   = '.';

const int iAx = 0;
const int iAy = 1;
const int iAz = 2;
const int iGx = 3;
const int iGy = 4;
const int iGz = 5;

const int usDelay = 3150;   // empirical, to hold sampling to 200 Hz
const int NFast =  1000;    // the bigger, the better (but slower)
const int NSlow = 10000;    // ..
const int LinesBetweenHeaders = 5;
      int LowValue[6];
      int HighValue[6];
      int Smoothed[6];
      int LowOffset[6];
      int HighOffset[6];
      int Target[6];
      int LinesOut;
      int N;
      

//variables for imu gyro/accel
// imu control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t imuIntStatus;   // holds actual interrupt status byte from imu
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool imuInterrupt = false;     // indicates whether imu interrupt pin has gone high



//instantiate object
VL6180X lidarOne;
VL6180X lidarTwo;
MPU6050 imu;

void dmpDataReady() {
    imuInterrupt = true;
}

void setup() {
Serial.begin(9600); // Starts the serial communication 115200 for mega
Wire.begin();

//Ultrasonic set up
pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
pinMode(echoPin, INPUT); // Sets the echoPin as an Input
Serial.println("Ultrasonic set up");

//Setting address for lidars
pinMode(lidarone,OUTPUT);
pinMode(lidartwo,OUTPUT);
digitalWrite(lidartwo, HIGH);//enable lidar two
digitalWrite(lidarone, LOW); //shut down lidar one before chaning address
delay(100);
lidarTwo.setAddress(0x30);
delay(10);
digitalWrite(lidarone, HIGH);//enable lidar one 

lidarTwo.init();
Serial.println("Lidar Two initialised");
lidarTwo.configureDefault();
lidarTwo.setTimeout(500);

lidarOne.init();              //lidar one uses default address
Serial.println("Lidar One initialised");
lidarOne.configureDefault();
lidarOne.setTimeout(500);


//IMU set up
pinMode(INTERRUPT_PIN, INPUT);
imu.initialize();
Serial.println(imu.testConnection() ? "imu6050 connection successful" : "imu6050 connection failed");
devStatus = imu.dmpInitialize();

Serial.println("Starting offset");
int TheOffsets[6] = {-4091, -587, 2145, 299, -26, 43};
SetOffsets(TheOffsets);
Serial.println("Offsets done");

//imuCalibrate();
//imustartv();  //set initial values

//set up for dmp
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

Serial.println("Ready!");
/*
Serial.println("offsets: AX, Ay, Az");
Serial.print(imu.getXAccelOffset()); Serial.print("\t");Serial.print(imu.getYAccelOffset()); Serial.print("\t");Serial.println(imu.getZAccelOffset());
Serial.print(imu.getXGyroOffset());Serial.print("\t"); Serial.print(imu.getYGyroOffset()); Serial.print("\t"); Serial.println(imu.getZGyroOffset());
*/
}

void loop() {
  if (Serial.available() > 0) {
    input = Serial.read();
    if ((input <= '4')  && (input >= '1')) {
      switch(input) {
      case '1':
        displayAll();
        break;
      case '2':
        casetwo();
        break;
      case '3':
        casethree();
        break;
      case '4':
        casefour();
        break;
      }
      
    } else {
      Serial.println("Invalid input");
    }  
  }
  gyro();
}

void casefour() {
  //right lidar is lidar one
  int mini = 40;
  int maxi = 80;
  int temp = 0;
  int holder = 0;
  uint32_t timer = millis();
  uint8_t distance;
  float udist;
  int walls[4]; //North, South, East and West
  Serial.print("case 4");
  
  while (1) {
    gyro();   //update gyro readings

    if (millis() - timer >= 500) {
      timer = millis();
      Serial.print(gzf); Serial.print("\t");
      
      if (gzf >= -45 && gzf <= 45) {
        temp = 0;
      } else if (gzf > 45 && gzf <= 135)
        temp = 1;
      else if (gzf > 135 || gzf < -135) {
        temp = 2; 
      } else if (gzf >= -135 && gxf <= -45) {
        temp = 3;
      }
      Serial.print("temp"); Serial.println(temp);
  
      distance = lidarTwo.readRangeSingleMillimeters();
      holder = temp - 1;  //lidar two is left
        if (holder == -1)   //wrap around
          holder = 3;
      if (distance <= maxi && distance >= mini) {
        walls[holder] = 1;
      } else {
        walls[holder] = 0;
      }
        
      distance = lidarOne.readRangeSingleMillimeters();
      holder = temp + 1;  //lidar two is left
        if (holder == 4)   //wrap around
          holder = 0;
      if (distance <= maxi && distance >= mini) {
        walls[holder] = 1;
      } else {
        walls[holder] = 0;
      }
  
      udist = ultrasonicdist();
      if (udist <= maxi && udist >= mini)
        walls[temp] = 1;
      else 
        walls[temp] = 0;

      for (int i = 0; i < 4; i++) {
        Serial.print(walls[i]); Serial.print("\t"); 
      }
    }
    //break from loop by pressing 's'
    if (Serial.available() > 0) {
      input = Serial.read();
      if (input == 's'){
       Serial.println("Stopping");
        break;
      }
    } 
  }  
}


void casethree() {
//infinite loop checking for wall readings at 60mm +/- 5mm - allowing for error
  oldtime = millis();
  while (1) {
    currtime = millis();
    if (currtime - oldtime >= 500) {
      casethreesub();
      oldtime = currtime;
    }
    
    //break out of loop by pressing 's'
    if (Serial.available() > 0) {
      input = Serial.read();
      if (input == 's')
        break;
    }
  }
}

void casethreesub() {
  bool wall = false;
  uint8_t lidard;
  float udist;
  int mini = 55;
  int maxi = 65;

  wall = false;
    lidard = lidarOne.readRangeSingleMillimeters();
    if (lidard >= mini && lidard <= maxi) 
      wall = true;
    Serial.print(wall); Serial.print("\t");
    
    wall = false;
    udist = ultrasonicdist();
    Serial.print(udist); Serial.print(" ");
    if (udist >= mini && udist <= maxi) 
      wall = true;
    Serial.print(wall); Serial.print("\t");
  
    wall = false;
    lidard = lidarTwo.readRangeSingleMillimeters();
    Serial.print(lidard); Serial.print(" ");
      if (lidard >= mini && lidard <= maxi) 
        wall = true;
    Serial.println(wall); //Serial.println();

}

void casetwo() {
  bool flag = 0;  //set to 1 after first reading of ~50mm
  uint32_t start = millis(); //record time at start of loop
  uint32_t count = 0; //record time when first reading of ~50mm appears
  int atdistcount = 0;
  float distance;
  
  while ((millis() - start) < 5000) {
    //Serial.println("1");
    distance = ultrasonicdist();
    Serial.println(distance);
    if (distance >= 40 && distance <= 60) { //distance limit between 45 and 55mm
      if (!flag) { //if in region for >3s, restart counter
        count = millis();
        flag = 1;
      } else if ((millis() - count) > 3500) {
        Serial.println("Too long, restarting count...");
        count = millis();  
      }
    } else if ((millis() - count) <= 3500 && (millis() - count) >= 1600) {
      Serial.println("Start");
      break;
    } else {
      if (flag) {
        Serial.println("Not long enough");
        flag = 0;     //restart count
      }
    }
 }

  if (!flag && (millis() - start) >= 5000)
    Serial.println("Timeout");
  
}

void displayAll() {  
  float distance = ultrasonicdist();
  Serial.print("Ultrasonic:"); Serial.print(distance); Serial.print("\t");
  Serial.print("Lidar 1:"); Serial.print(lidarOne.readRangeSingleMillimeters()); Serial.print("\t");
  Serial.print("Lidar 2:"); Serial.print(lidarTwo.readRangeSingleMillimeters()); Serial.print("\t");

  imuprint();
  Serial.print("AX:"); Serial.print(axf); Serial.print("\t");
  Serial.print("AY:"); Serial.print(ayf); Serial.print("\t");
  Serial.print("AZ:"); Serial.print(azf); Serial.print("\t");
  Serial.print("pitch:"); Serial.print(gxf); Serial.print("\t");
  Serial.print("roll:"); Serial.print(gyf); Serial.print("\t");
  Serial.print("yaw:"); Serial.println(gzf);
}

float ultrasonicdist() {
  long duration;
  float distance;
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (float(duration)/(29.155*2));
  distance = distance*10;

  if (distance > 1000) {  //setting max distance at 1000
    distance = 1000;
  }
  return distance;
}

void imuprint() {
  imu.getAcceleration(&ax, &ay, &az);

  //type cast to float for accuracy
  axf = float(ax)/imuAscale;
  ayf = float(ay)/imuAscale;
  azf = float(az)/imuAscale;
}


void gyro() {
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
        imu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        gxf = ypr[1]*180/M_PI;
        gyf = ypr[2]*180/M_PI;
        gzf = ypr[0]*180/M_PI;

        //Serial.print("ypr\t");
        //Serial.print(ypr[0] * 180/M_PI);
        //Serial.print("\t");
        //Serial.print(ypr[1] * 180/M_PI);
        //Serial.print("\t");
        //Serial.println(ypr[2] * 180/M_PI);
    }
}


//imu calibration functions 
// imu6050 offset-finder, based on Jeff Rowberg's imu6050_RAW
// 2016-10-19 by Robert R. Fenichel (bob@fenichel.net)

void imuCalibrate() { 
    for (int i = iAx; i <= iGz; i++)
      { // set targets and initial guesses
        Target[i] = 0; // must fix for ZAccel 
        HighOffset[i] = 0;
        LowOffset[i] = 0;
      } // set targets and initial guesses
    Target[iAz] = 16384;
    SetAveraging(NFast);
    
    PullBracketsOut();
    PullBracketsIn();
}


void ForceHeader()
  { LinesOut = 99; }
    
void GetSmoothed()
  { int16_t RawValue[6];
    int i;
    long Sums[6];
    for (i = iAx; i <= iGz; i++)
      { Sums[i] = 0; }
//    unsigned long Start = micros();

    for (i = 1; i <= N; i++)
      { // get sums
        imu.getMotion6(&RawValue[iAx], &RawValue[iAy], &RawValue[iAz], 
                             &RawValue[iGx], &RawValue[iGy], &RawValue[iGz]);
        if ((i % 500) == 0)
          Serial.print(PERIOD);
        delayMicroseconds(usDelay);
        for (int j = iAx; j <= iGz; j++)
          Sums[j] = Sums[j] + RawValue[j];
      } // get sums
//    unsigned long usForN = micros() - Start;
//    Serial.print(" reading at ");
//    Serial.print(1000000/((usForN+N/2)/N));
//    Serial.println(" Hz");
    for (i = iAx; i <= iGz; i++)
      { Smoothed[i] = (Sums[i] + N/2) / N ; }
  } // GetSmoothed

void SetOffsets(int TheOffsets[6])
  { imu.setXAccelOffset(TheOffsets [iAx]);
    imu.setYAccelOffset(TheOffsets [iAy]);
    imu.setZAccelOffset(TheOffsets [iAz]);
    imu.setXGyroOffset (TheOffsets [iGx]);
    imu.setYGyroOffset (TheOffsets [iGy]);
    imu.setZGyroOffset (TheOffsets [iGz]);
  } // SetOffsets

void ShowProgress()
  { if (LinesOut >= LinesBetweenHeaders)
      { // show header
        Serial.println("\tXAccel\t\t\tYAccel\t\t\t\tZAccel\t\t\tXGyro\t\t\tYGyro\t\t\tZGyro");
        LinesOut = 0;
      } // show header
    Serial.print(BLANK);
    for (int i = iAx; i <= iGz; i++)
      { Serial.print(LBRACKET);
        Serial.print(LowOffset[i]),
        Serial.print(COMMA);
        Serial.print(HighOffset[i]);
        Serial.print("] --> [");
        Serial.print(LowValue[i]);
        Serial.print(COMMA);
        Serial.print(HighValue[i]);
        if (i == iGz)
          { Serial.println(RBRACKET); }
        else
          { Serial.print("]\t"); }
      }
    LinesOut++;
  } // ShowProgress

void PullBracketsIn()
  { boolean AllBracketsNarrow;
    boolean StillWorking;
    int NewOffset[6];
  
    Serial.println("\nclosing in:");
    AllBracketsNarrow = false;
    ForceHeader();
    StillWorking = true;
    while (StillWorking) 
      { StillWorking = false;
        if (AllBracketsNarrow && (N == NFast))
          { SetAveraging(NSlow); }
        else
          { AllBracketsNarrow = true; }// tentative
        for (int i = iAx; i <= iGz; i++)
          { if (HighOffset[i] <= (LowOffset[i]+1))
              { NewOffset[i] = LowOffset[i]; }
            else
              { // binary search
                StillWorking = true;
                NewOffset[i] = (LowOffset[i] + HighOffset[i]) / 2;
                if (HighOffset[i] > (LowOffset[i] + 10))
                  { AllBracketsNarrow = false; }
              } // binary search
          }
        SetOffsets(NewOffset);
        GetSmoothed();
        for (int i = iAx; i <= iGz; i++)
          { // closing in
            if (Smoothed[i] > Target[i])
              { // use lower half
                HighOffset[i] = NewOffset[i];
                HighValue[i] = Smoothed[i];
              } // use lower half
            else
              { // use upper half
                LowOffset[i] = NewOffset[i];
                LowValue[i] = Smoothed[i];
              } // use upper half
          } // closing in
        ShowProgress();
      } // still working
   
  } // PullBracketsIn

void PullBracketsOut()
  { boolean Done = false;
    int NextLowOffset[6];
    int NextHighOffset[6];

    Serial.println("expanding:");
    ForceHeader();
 
    while (!Done)
      { Done = true;
        SetOffsets(LowOffset);
        GetSmoothed();
        for (int i = iAx; i <= iGz; i++)
          { // got low values
            LowValue[i] = Smoothed[i];
            if (LowValue[i] >= Target[i])
              { Done = false;
                NextLowOffset[i] = LowOffset[i] - 1000;
              }
            else
              { NextLowOffset[i] = LowOffset[i]; }
          } // got low values
      
        SetOffsets(HighOffset);
        GetSmoothed();
        for (int i = iAx; i <= iGz; i++)
          { // got high values
            HighValue[i] = Smoothed[i];
            if (HighValue[i] <= Target[i])
              { Done = false;
                NextHighOffset[i] = HighOffset[i] + 1000;
              }
            else
              { NextHighOffset[i] = HighOffset[i]; }
          } // got high values
        ShowProgress();
        for (int i = iAx; i <= iGz; i++)
          { LowOffset[i] = NextLowOffset[i];   // had to wait until ShowProgress done
            HighOffset[i] = NextHighOffset[i]; // ..
          }
     } // keep going
  } // PullBracketsOut

void SetAveraging(int NewN)
  { N = NewN;
    Serial.print("averaging ");
    Serial.print(N);
    Serial.println(" readings each time");
   } // SetAveraging
