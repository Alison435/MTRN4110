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

#define INTERRUPT_PIN 18  // use pin 2 on uno - mega 18

//uno - sda: A4 scl:A5
//mega - sda:20 scl:21

int trigPin = 11;  //change for mega pin 32
int echoPin = 12;  //change for mega pin 33
int lidarone = 30; //shutdown pins for lidar on mega - 30
int lidartwo = 31; //mega - 31
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
Serial.println("Ultrasonic initialised");

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

}

void loop() {
  float distance;
  
  //gyro() updates gyro values stored in gx,gy and gz
  //returns 1 if successfully updates
  //returns 0 if overflow occurs - can just call again
  if (gyro() == false)
    gyro();

  //updates acceleration values stored in ax,ay and az
  imuprint();

  //returns distance in millimeters
  distance = lidarOne.readRangeSingleMillimeters();
  distance = lidarTwo.readRangeSingleMillimeters();

  //returns distance in millimeters
  distance = ultrasonicdist();

  
  
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

        success = true;
    }
    return success;
}

void SetOffsets(int TheOffsets[6]) {
  imu.setXAccelOffset(TheOffsets [0]);
  imu.setYAccelOffset(TheOffsets [1]);
  imu.setZAccelOffset(TheOffsets [2]);
  imu.setXGyroOffset (TheOffsets [3]);
  imu.setYGyroOffset (TheOffsets [4]);
  imu.setZGyroOffset (TheOffsets [5]);
} // SetOffsets
