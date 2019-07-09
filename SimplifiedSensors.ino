//2xLidars, Ultrasonic and IMU sensors - Phase A - 2
//using imu6050 offset-finder 2016-10-19 by Robert R. Fenichel (bob@fenichel.net)
//using dmp measurements - jeff rowlberg
//Alison Truong
//lidar one is on the right and lidar two is on the left of the board

#include <Sensors.h>


//uno - sda: A4 scl:A5
//mega - sda:20 scl:21

void setup() {
Serial.begin(9600); // Starts the serial communication 115200 for mega
Wire.begin();

//Ultrasonic set up
pinMode(TRIGGER_PIN, OUTPUT); // Sets the trigPin as an Output
pinMode(ECHO_PIN, INPUT); // Sets the echoPin as an Input
Serial.println("Ultrasonic initialised");

//Setting address for lidars
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


//IMU set up
pinMode(INTERRUPT_PIN, INPUT);
imu.initialize();
Serial.println(imu.testConnection() ? "IMU initialised" : "imu6050 connection failed");
devStatus = imu.dmpInitialize();

Serial.println("Starting offset");
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
