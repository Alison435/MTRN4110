#pragma once
#include "external_MPU6050_6Axis_MotionApps20.h"
#include "external_MPU6050_I2Cdev.h"
#include "external_I2CIO.h"
#include "external_VL6180X.h"

#include <Wire.h>

#define TRIGGER_PIN 32
#define ECHO_PIN 33
#define LIDARONE 30
#define LIDARTWO 31
#define INTERRUPT_PIN 18


float imuAscale = 16384.0;
float imuGscale = 131.0;
int TheOffsets[6] = { -4091, -587, 2145, 299, -26, 43 };
int16_t ax, ay, az;
int8_t gx, gy, gz;

float axf, ayf, azf;
float gxf, gyf, gzf;


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
VL6180X lidarOne = VL6180X();
VL6180X lidarTwo = VL6180X();
MPU6050 imu;



void dmpDataReady() {
	imuInterrupt = true;
}