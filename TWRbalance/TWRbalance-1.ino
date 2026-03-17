

/*
Pin settings ========================================================================
***L293D pins in () after description***
D0 = ---- (Rx)                      D6 = Left motor IN2 (to L293D 7)        
D1 = ---- (Tx)                      D7 = Right motor IN3 (to L293D 15)        
D2 = Left encoder Yellow            D8 = Right motor IN4 (to L293D 10)                      
D3 = Right  encoder Yellow          D9 = Left motor PWM A (to L293D 1)      
D4 = Left encoder White             D10 = Right motor PWM B (to L293D 9)    
D5 = Left motor IN1 (to L293D 2)    D11 = Right encoder White               

L293D 5V = pin 16
L293D 12V = pin 8
L293D ground = 4, 13 
L293D pin = 3, Left Motor Red Driver +
L293D pin = 6, Left Motor Black Driver -
L293D pin = 14, Right Motor Red Driver +
L293D pin = 11, Right Motor Black Driver -


Motors =============================================================================

JGA25           : 
type            : geared motor
gearboxRatio    : 1:34
no load speed   : 126 RPM, 12V (94.5 RPM at 9 V???)
ticksRevolution : 12*34 = 408 https://www.openimpulse.com/blog/products-page/25d-gearmotors/jga25-371-dc-gearmotor-encoder-126-rpm-12-v-2/
torque          : 

See alsoL: https://www.pololu.com/product/4822; this has comparable motor description

wheel diameter  : 7.3 cm = 0.073 m 
wheel perimeter : =3.14*0.073 = 0.22934 m
wheel distance  : ? m  (center wheels)


IMU ================================================================================

The MPU6050 is used along with library of Rowberg

*/


#include <Encoder.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "math.h"
#include <avr/wdt.h>
#include "PID_v1.h"

// PINS ==============================================================
const unsigned int leftMotorPin1 = 5;
const unsigned int leftMotorPin2 = 6;
const unsigned int rightMotorPin1 = 7;
const unsigned int rightMotorPin2 = 8;

const unsigned int leftMotorPWMPin = 9;
const unsigned int rightMotorPWMPin = 10;


/********** PID **********/
#include <PID_v1.h>

/*double kP = 50;
double kI = 12;
double kD = 7;
*/

double kP = 150;
double kI = 400;
double kD = 5;

double setpoint, input, output;   // PID variables
PID pid(&input, &output, &setpoint, kP, kI, kD, DIRECT); // PID setup



/********** MPU **********/
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
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
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float pitch;
long velocity;

float trimPot;
float trimAngle;
double pwmLeft = 0.0, pwmRight = 0.0;

int IMUdataReady = 0;
volatile bool mpuInterrupt = false;

/********** SETUP **********/
void setup() {
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(leftMotorPWMPin, OUTPUT);
  pinMode(rightMotorPWMPin, OUTPUT);
   
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);

    // initialize device
    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXAccelOffset(-1136);
  mpu.setYAccelOffset(-212);
  mpu.setZAccelOffset(902);
  mpu.setXGyroOffset(49);
  mpu.setYGyroOffset(-12);
  mpu.setZGyroOffset(-66);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(-255,255);
    pid.SetSampleTime(10);
    
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
 
  //trimPot = analogRead(A3)*-1; // read dial on robot and *-1 for direction

  trimAngle = -4.3;//(trimPot/100) + 5; // adjust to degrees

  getAngles();

  pitch = (ypr[1] * 180/M_PI); // adjust to degrees

  // PID vars
  setpoint = trimAngle; 
  input = pitch;

  pid.Compute();

  // set motor speed with adjusted turn values
//  pwmLeft = output;
//  pwmRight = output;
//  Serial.print("pwmL: "); Serial.println(pwmLeft);
//  Serial.print("pwmR: "); Serial.println(pwmRight);
  
  /*if (pitch > 25 || pitch < -25) { // angle threshold
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0); 
  } else {
    leftMotor.setSpeed(abs(speedLeft));
    rightMotor.setSpeed(abs(speedRight));
  }

  // move motors
  if (speedLeft < 0) { 
    leftMotor.forward();
  } else {
    leftMotor.backward();
  }
  
  if (speedRight < 0) { 
    rightMotor.forward();
  } else {
    rightMotor.backward();
  }
  */
  // print some control info
  Serial.print("pitch: ");
  Serial.println(pitch);

  setPower();
}

inline void forwardLeft()
{
	digitalWrite(leftMotorPin1, LOW);  digitalWrite(leftMotorPin2, HIGH); // left motor
}

inline void forwardRight()
{
	digitalWrite(rightMotorPin1, HIGH); digitalWrite(rightMotorPin2, LOW);  // right motor
}

inline void backwardLeft()
{
	digitalWrite(leftMotorPin1, HIGH);  digitalWrite(leftMotorPin2, LOW); // left motor backwards
}

inline void backwardRight()
{
	digitalWrite(rightMotorPin1, LOW); digitalWrite(rightMotorPin2, HIGH);  // right motor backwards
}

void stop()
{
	analogWrite(leftMotorPWMPin, 0); analogWrite(rightMotorPWMPin, 0);
}

inline void setPower()
{	//Set direction and adjust for motor stiction and backlash
  pwmLeft = output;
  pwmRight = output;

	if (pwmLeft < 0)  { forwardLeft();   pwmLeft *= -1;   }
	else              { backwardLeft();                   }
	if (pwmRight < 0) { forwardRight();  pwmRight *= -1;  }
	else              { backwardRight();                  }

	//Round-up to integer and cap to range
	int pwmL = int(pwmLeft + 0.5);  pwmL = constrain(pwmL, 0, 255);
	int pwmR = int(pwmRight + 0.5); pwmR = constrain(pwmR, 0, 255);
  Serial.print("pwmL: "); Serial.println(pwmL);
  Serial.print("pwmR: "); Serial.println(pwmR);
	//Throttle the motors
	analogWrite(leftMotorPWMPin, pwmL);
	analogWrite(rightMotorPWMPin, pwmR);
}

inline void getAngles()
{
	fifoCount = mpu.getFIFOCount();
	while (fifoCount < packetSize) { fifoCount = mpu.getFIFOCount(); }

	mpu.getFIFOBytes(fifoBuffer, packetSize);
	fifoCount -= packetSize;
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

	//pitch = ypr[1] * RAD_TO_DEG;
	//yawCurrent   = ypr[0] * RAD_TO_DEG;

	//if (yawCurrent < 0) { yawCurrent += 360; }        //adjust to (forward) clockwise range 0-360 

  //Serial.print("Pitch: ");
  //Serial.print(pitch);
  //Serial.print(" yaw: ");
  //Serial.println(yawCurrent);

	mpu.resetFIFO();
}
