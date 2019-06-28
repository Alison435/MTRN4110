//2xLidars, Ultrasonic and IMU sensors - Phase A - 2
//using imu6050 offset-finder 2016-10-19 by Robert R. Fenichel (bob@fenichel.net)
//using dmp measurements - jeff rowlberg
//Alison Truong
//lidar one is on the right and lidar two is on the left of the board

#include <external_MPU6050_6Axis_MotionApps20.h>
#include <external_MPU6050_I2Cdev.h>
#include <external_I2CIO.h>
#include <external_VL6180X.h>

#include <Wire.h>

#define INTERRUPT_PIN 2  // use pin 2 on uno - mega 18

//uno - sda: A4 scl:A5
//mega - sda:20 scl:21

int trigPin = 11;  //change for mega pin 32
int echoPin = 12;  //change for mega pin 33
int lidarone = 13; //shutdown pins for lidar on mega - 30
int lidartwo = 7; //mega - 31
float imuAscale = 16384.0;  //from datasheet
float imuGscale = 131.0;    //from datasheet
int16_t ax, ay, az;
uint8_t gx, gy, gz;

float axf, ayf, azf;
float gxf, gyf, gzf;

uint32_t oldtime;
uint32_t currtime;

int input;

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
Serial.println(imu.testConnection() ? "IMU initialised" : "imu6050 connection failed");
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
        caseone();
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

void caseone() {
  while(1) {
    displayAll();
    if (Serial.available() > 0) {
      input = Serial.read();
      if (input == 's'){
       Serial.println("Stopping");
        break;
      }
  }
  
}
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
  memset(walls,0,sizeof(walls));
  
  Serial.print("case 4");
  
  while (1) {
    gyro();   //update gyro readings
    
    if (millis() - timer >= 500) {
      timer = millis();
      //Serial.print(gzf); Serial.print("\t");
      
      //reset walls
      //memset(walls,0,sizeof(walls));
        
      if (gzf >= -45 && gzf <= 45) {
        temp = 0;
      } else if (gzf > 45 && gzf <= 135)
        temp = 1;
      else if (gzf > 135 || gzf < -135) {
        temp = 2; 
      } else if (gzf >= -135 && gxf <= -45) {
        temp = 3;
      }
      
      //Serial.print("temp"); Serial.println(temp);
  
      distance = lidarTwo.readRangeSingleMillimeters();
      holder = temp - 1;  //lidar two is left
        if (holder == -1)   //wrap around
          holder = 3;
      if (distance <= maxi && distance >= mini) {
        walls[holder] = 1;
      } 
        
      distance = lidarOne.readRangeSingleMillimeters();
      holder = temp + 1;  //lidar two is left
        if (holder == 4)   //wrap around
          holder = 0;
      if (distance <= maxi && distance >= mini) {
        walls[holder] = 1;
      }

  
      udist = ultrasonicdist();
      if (udist <= maxi && udist >= mini){
        walls[temp] = 1;
      }
      
      for (int i = 0; i < 4; i++) {
        Serial.print(walls[i]); 
        Serial.print("\t"); 
      }
        Serial.println("");
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
      if (input == 's') {
        Serial.println("stopping");
        break;
      }
    }
  }
}

void casethreesub() {
  bool wall = false;
  uint8_t lidard;
  float udist;
  int mini = 50;
  int maxi = 70;

  wall = false;
  lidard = lidarTwo.readRangeSingleMillimeters();
  if (lidard >= mini && lidard <= maxi) 
    wall = true;
  Serial.print(wall); Serial.print("\t");
  
  wall = false;
  udist = ultrasonicdist();
  //Serial.print(udist); Serial.print(" ");
  if (udist >= mini && udist <= maxi) 
    wall = true;
  Serial.print(wall); Serial.print("\t");

  wall = false;
  lidard = lidarOne.readRangeSingleMillimeters();
  //Serial.print(lidard); Serial.print(" ");
    if (lidard >= mini && lidard <= maxi) 
      wall = true;
  Serial.println(wall); //Serial.println();

}

void casetwo() {
  bool flag = 0;  //set to 1 after first reading of ~50mm
  uint32_t start = millis(); //record time at start of loop
  uint32_t count = 0; //record time when first reading of ~50mm appears
  int countout = 0;
  int atdistcount = 0;
  float distance;
  
  while (1) {
    distance = ultrasonicdist();
    Serial.println(distance);
    if (distance >= 40 && distance <= 60) { //distance limit between 45 and 55mm
      if (!flag) { //if in region for >3s, restart counter
        count = millis();
        flag = 1;
      }
      
    } else if ((millis() - count) >= 1600 && flag == 1 && (millis() - count) <= 4000) { 
      //else if ((millis() - count) <= 3500 && (millis() - count) >= 1600) {
      Serial.println("Start");
      break;
    } else if (flag && countout > 3) {
        Serial.println("Not long enough");
        flag = 0;     //restart count
    }
    if (Serial.available() > 0) {
      input = Serial.read();
      if (input == 's'){
       Serial.println("Stopping");
        break;
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
  gyro();
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
       // Serial.println(F("FIFO overflow!"));

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

void SetOffsets(int TheOffsets[6]) {
  imu.setXAccelOffset(TheOffsets [0]);
  imu.setYAccelOffset(TheOffsets [1]);
  imu.setZAccelOffset(TheOffsets [2]);
  imu.setXGyroOffset (TheOffsets [3]);
  imu.setYGyroOffset (TheOffsets [4]);
  imu.setZGyroOffset (TheOffsets [5]);
} // SetOffsets
