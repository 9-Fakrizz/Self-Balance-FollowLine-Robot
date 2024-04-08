//Viral Science www.youtube.com/c/viralscience  www.viralsciencecreativity.com
//Self Balancing Robot
#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 #include "Wire.h"
#endif

#define MIN_ABS_SPEED 30
#define IR_SENSOR_RIGHT 12
#define IR_SENSOR_LEFT 11
#define MOTOR_SPEED 30

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

//PID
// double originalSetpoint = 172.50;
double originalSetpoint = 171.40;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;

//adjust these values to fit your own design
double Kp = 60;   
double Kd = 2.2;
double Ki = 270;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.68;
double motorSpeedFactorRight = 0.58;


//MOTOR CONTROLLER
int ENA = 5;
int IN1 = 6;
int IN2 = 7;
int IN3 = 9;
int IN4 = 8;
int ENB = 10;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
 mpuInterrupt = true;
}


void setup()
{
 // join I2C bus (I2Cdev library doesn't do this automatically)
 #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 Wire.begin();
 TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
 #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
 Fastwire::setup(400, true);
 #endif
  //Serial.begin(115200);
  // set pinmode
  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

 mpu.initialize();

 devStatus = mpu.dmpInitialize();

 // supply your own gyro offsets here, scaled for min sensitivity
 mpu.setXGyroOffset(198);
 mpu.setYGyroOffset(82);
 mpu.setZGyroOffset(-91);
 mpu.setZAccelOffset(588); // 1688 factory default for my test chip

 // make sure it worked (returns 0 if so)
 if (devStatus == 0)
 {
 // turn on the DMP, now that it's ready
 mpu.setDMPEnabled(true);

 // enable Arduino interrupt detection
 attachInterrupt(0, dmpDataReady, RISING);
 mpuIntStatus = mpu.getIntStatus();

 // set our DMP Ready flag so the main loop() function knows it's okay to use it
 dmpReady = true;

 // get expected DMP packet size for later comparison
 packetSize = mpu.dmpGetFIFOPacketSize();
 
 //setup PID
 pid.SetMode(AUTOMATIC);
 pid.SetSampleTime(10);
 pid.SetOutputLimits(-255, 255); 
 }
 else
 {
 // ERROR!
 // 1 = initial memory load failed
 // 2 = DMP configuration updates failed
 // (if it's going to break, usually the code will be 1)
 Serial.print(F("DMP Initialization failed (code "));
 Serial.print(devStatus);
 Serial.println(F(")"));
 }
}


void loop()
{
  int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
  int leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);

  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize)
  {
    //no mpu data - performing PID calculations and output to motors 
    pid.Compute();
    //motorController.move(output, MIN_ABS_SPEED);
    //Serial.println("Output is "+String(output));
    if (rightIRSensorValue == LOW && leftIRSensorValue == LOW)
    {
      // Both sensors off the line, move forward
      setpoint = 185.50;
      motorController.move(output+(output*0.12), MIN_ABS_SPEED);

    }
    else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW)
    {
      // Right sensor on the line, turn left
      //motorController.move(50, MIN_ABS_SPEED);
      digitalWrite(ENA,0);
      setpoint = 175.50;
      motorSpeedFactorLeft = 0.8;
      motorSpeedFactorRight = 0.7;
      //Serial.println("left");
      motorController.move(output-(output*0.015), MIN_ABS_SPEED);
    }
    else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH)
    {
      // Left sensor on the line, turn right
      //motorController.move(-50, MIN_ABS_SPEED);
      digitalWrite(ENB,0);
      setpoint = 175.50;
      motorSpeedFactorLeft = 0.8;
      motorSpeedFactorRight = 0.7;
      //Serial.println("right");
      motorController.move(output-(output*0.015), MIN_ABS_SPEED);
    }
    setpoint = 171.68;
    motorSpeedFactorLeft = 0.68;
    motorSpeedFactorRight = 0.58;
    motorController.move(output+(output*0.23), MIN_ABS_SPEED);
    

  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    input = ypr[1] * 180 / M_PI + 180;
  }
}
