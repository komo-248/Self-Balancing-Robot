/*  PINOUTS
***L293D pins in () after description***
D0 = ---- (Rx)                      D6 = R IN4      
D1 = ---- (Tx)                      D7 = L OutA1       
D2 = ----                           D8 = L OutB1                      
D3 = L IN1                          D9 = R OutA2     
D4 = L IN2                          D10 = R OutB2   
D5 = R IN3                          D11 = L PWM
                                    D12 = R PWM               

    MOTORS
JGA25           : 
type            : geared motor
gearboxRatio    : 1:34
no load speed   : 126 RPM, 12V (94.5 RPM at 9 V???)
ticksRevolution : 12*34 = 408 
  https://www.openimpulse.com/blog/products-page/25d-gearmotors/jga25-371-dc-gearmotor-encoder-126-rpm-12-v-2/
torque          : 
  See also: https://www.pololu.com/product/4822 (this has comparable motor description)
wheel diameter  : 7.3 cm = 0.073 m 
wheel perimeter : =3.14*0.073 = 0.22934 m
wheel distance  : ? m  (center wheels)

    IMU
The MPU6050 is used along with library of Rowberg*/






// LIBRARIES
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "math.h"
#include <avr/wdt.h>
#include "PID_v1.h"

// PIN ASSIGNMENTS
const unsigned int leftMotorPin1 = 3;
const unsigned int leftMotorPin2 = 4;
const unsigned int rightMotorPin1 = 5;
const unsigned int rightMotorPin2 = 6;
const unsigned int leftMotorPWMPin = 11;
const unsigned int rightMotorPWMPin = 12;

// PID
#include <PID_v1.h>

double kP = 30; // [balance];   Ranges:   32-45,      40,     32
double kI = 90; //                        90--------------------->
double kD = .5; // [judder];              0.65-0.75,  0.75,   0.55

double setpoint, input, output;   // variables
PID pid(&input, &output, &setpoint, kP, kI, kD, DIRECT); // setup

// MPU
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu; // addy is 0x68 or 0x69

// control/status varibless
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion varibless
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
float trimAngle = 2.85;
double pwmLeft = 0.0, pwmRight = 0.0;
int IMUdataReady = 0;
volatile bool mpuInterrupt = false;






// SETUP
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
    // gyro offsets
  mpu.setXAccelOffset(-1934);
  mpu.setYAccelOffset(994);
  mpu.setZAccelOffset(677);
  mpu.setXGyroOffset(-21);
  mpu.setYGyroOffset(66);
  mpu.setZGyroOffset(2);
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);
        // enable Arduino interrupt detection
        // attachInterrupt(0, dmpDataReady, RISING);
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
}

unsigned long t = 0; // Variable to store current time
unsigned long p = 0; // Variable to store previous time






void loop() {
    getAngles();
    pitch = (ypr[1] * 180 / M_PI);
    // PID variables
    input = pitch;
    pid.Compute();
    
    unsigned long currentTime = millis(); // Update the current time
    
    // Run setPower2 for 100 milliseconds
    moveForward();
    if (currentTime - p < 1000) {
        // Continue executing moveForward()
    } else {
        // Stop moveForward() and proceed to the next step
        standStill();
        p = currentTime; // Update the previous time
    }

    // Run setPower for 1 second
    if (currentTime - p < 2000) {
        // Continue executing standStill()
    } else {
        // Stop standStill() and reset the timing
        p = currentTime; // Update the previous time
    }
}






// Function to make the robot stand still
void standStill() {  
    setpoint = trimAngle;
    setPower(); // Call setPower function to update motor PWM signals with the new setpoint
}

// Function to move the robot forward
void moveForward() {
    setpoint = trimAngle; // Increase the setpoint by the specified angle increase
    setPower2(); // Call setPower2 function to update motor PWM signals with the new setpoint
}
inline void forwardLeft() {
	digitalWrite(leftMotorPin1, LOW);  digitalWrite(leftMotorPin2, HIGH); // left motor
}

inline void forwardRight() {
	digitalWrite(rightMotorPin1, HIGH); digitalWrite(rightMotorPin2, LOW);  // right motor
}

inline void backwardLeft() {
	digitalWrite(leftMotorPin1, HIGH);  digitalWrite(leftMotorPin2, LOW); // left motor backwards
}

inline void backwardRight() {
	digitalWrite(rightMotorPin1, LOW); digitalWrite(rightMotorPin2, HIGH);  // right motor backwards
}






inline void setPower() {	//Set direction and adjust for motor stiction and backlash
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

inline void setPower2() {	// Attempt at locking the motors in a state of forward movement. Function is called in the loop
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
	analogWrite(leftMotorPWMPin, 100);
	analogWrite(rightMotorPWMPin, 100);
}

inline void getAngles() {
	fifoCount = mpu.getFIFOCount();
	while (fifoCount < packetSize) { fifoCount = mpu.getFIFOCount(); }
	mpu.getFIFOBytes(fifoBuffer, packetSize);
	fifoCount -= packetSize;
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
	pitch = ypr[1] * RAD_TO_DEG;
	mpu.resetFIFO();
}