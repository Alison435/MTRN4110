// Testing how to make robot drive straight -> 2 Methods
// 1. Using IMU for P control (TRYING THIS ONE ATM)

// Testing how to make robot drive straight
// using proportional control

#include <external_MPU6050_6Axis_MotionApps20.h>
#include <external_MPU6050_I2Cdev.h>
#include <external_I2CIO.h>
#include <Wire.h>

// Pins for H-bridge control

int leftEnableDC = 10; //pwm
int rightEnableDC = 11; //pwm 
int in1 = 9; //digital
int in2 = 8; //digital
int in3 = 12; //digital
int in4 = 13; //digital

#define MAX_SPEED 100

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

// IMU Functions

#define INTERRUPT_PIN 2  // use pin 2 on uno - mega 18

//uno - sda: A4 scl:A5
//mega - sda:20 scl:21

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
//VL6180X lidarOne;
//VL6180X lidarTwo;
MPU6050 imu;

void dmpDataReady() {
    imuInterrupt = true;
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
  
  delay(5000);
}

char motor_command[36];
int setPoint_Speed = 80.0;
int leftPWM = setPoint_Speed;
int rightPWM = setPoint_Speed;
double k_p = 0.3;
double initial_yaw = 82.0;

void loop()
{
  
  // Use IMU readings for proportional control
  leftPWM = setPoint_Speed - (k_p * ((gzf)-initial_yaw));
  rightPWM = setPoint_Speed + (k_p *((gzf)-initial_yaw)); 
 
  // imuprint(); Not needed as this is only for acceleration
  gyro();
  Serial.println(gzf);
  // Output to motors
  sprintf(motor_command,"L: %d, R: %d\n\n",leftPWM, rightPWM);
  Serial.print(motor_command); 
  motor(1,0,rightPWM, leftPWM);
  delay(20);
  
}


