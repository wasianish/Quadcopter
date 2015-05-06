// NOTES:
//   This is optimized for the DUE.  This means that:
//     - 32 bit
//     - pin mode masks do not work, must use pinMode()
//   There is a 5 us handicap for the ping calculation to account for runtime
//   The main loop is split into tasks, named func1-3.
//     - func1: pull data from mpu, make slight adjustments

#include<Wire.h>
#include<Servo.h>

// Pin setup for the ultrassonic sensor
uint32_t triggerPin = 8;
uint32_t echoPin = 9;

// Register and bitmask setup for manual triggering of the ultrasonic sensor
volatile uint32_t * triggerOutput;
volatile uint32_t * echoInput;
uint32_t triggerBitmask;
uint32_t echoBitmask;

// 3 basic functions
uint32_t funcClocks[3] = {10000, 20000, 50000};
uint32_t funcCounters[3] = {0, 0, 0};

// Raw data from the mpu
int32_t AcX, AcY, AcZ, GyX, GyY, GyZ, temp;
// The time between ping and response
uint32_t pingTime;
// maxPingTime + micros()
uint32_t maxPing;
// Response timeout
uint32_t maxPingTime = 1000;
// Start of ping timeout
uint32_t pingTimeout = 18000;

// Integral of mpu data, used to perform trajectory corrections
int32_t VelX, VelY, VelZ, RotX, RotY, RotZ;

// Timestamp of last mpu read, used to calculate double integral
uint32_t lastDataTimestamp = 0;

// Mode, used to make decisions
//   mode = 0 --> hover
//   mode = 1 --> follow set of velocity and rotation targets sent by controller
//   mode = 2 --> target gps position
uint32_t mode = 0;

// I2C address of mpu
const int MPU6050 = 0x86;

void setup() {
  // Being the I2C
  Wire.begin();
  // Setup up the mpu
  setupMPU6050();
  // Setup the registers and bitmasks for the ultrasonic sensor
  setupSonar();
  // Set up serial to rf here
}

void loop(){
  
  
  
  if(funcCounters[0] > funcClock[0]) {
    readMPU6050();
    calcVelAndRot();
    lastDataTimestamp = micros();
    
  }
  if(funcCounters[1] > funcCounters[1]) {
    
  }
  if(funcCounters[2] > funcCounters[2]) {
    
  }
  
  // Increment counters
  funcCounters[0] += micros();
  funcCounters[1] += micros();
  funcCounters[2] += micros();
}

// Sets up the sonar
void setupSonar() {
  triggerOutput = portOutputRegister(digitalPinToPort(triggerPin));
  echoInput = portInputRegister(digitalPinToPort(echoPin));
  triggerBitmask = digitalPinToBitMask(triggerPin);
  echoBitmask = digitalPinToBitMask(echoPin);
  pinMode(echoPin, INPUT);
}

// Pings the sonar, the ping in microseconds is stored in pingTime
void pingSonar() {
  // Trigger the ping
  pinMode(triggerPin, OUTPUT);
  *triggerOutput &= !triggerBitmask;
  delayMicroseconds(4);
  *triggerOutput |= triggerBitmask;
  delayMicroseconds(10);
  *triggerOutput &= !triggerBitmask;
  pinMode(triggerPin, INPUT);
  
  // Wait for echo pin to reset, may want to add infinite loop correction
  maxPing = micros() + pingTimeout;
  while(*echoInput & echoBitmask && micros() < maxPing) {}
  while(!(*echoInput & echoBitmask)) {
    if(micros() > maxPing) {
      pingTime = 0;
      return;
    }
  }
  maxPing = micros() + maxPingTime;
  
  // Wait for ping
  while(*echoInput & echoBitmask) {
    if (micros() > maxPing) {
      pingTime = 0;
      return;
    }
  }
  // Calculate ping time
  pingTime = (micros() - (maxPing - maxPingTime) - 5);
}

// Wakes the MPU6050
void setupMPU6050() {
  Wire.beginTransmission(MPU6050);
  // Wake MPU
  Wire.write(0x6B);
  Wire.write(0x00);
  // Setup Fifo
  Wire.write(0x23);
  // Temp, gyro, accel
  Wire.write(0b11111000);
  // Pull config reg
  Wire.write(0x1A);
  Wire.requestFrom(MPU6050, 3, true);
  uint8_t configReg = Wire.read();
  uint8_t gyroConfigReg = Wire.read();
  uint8_t accelConfigReg = Wire.read();
  // Setup DHPF
  Wire.write(0x1A);
  Wire.write(configReg|0b00000001);
  // Gyro config (set range and start self test)
  Wire.write(0x1B);
  Wire.write(gyroConfigReg|0b11100000);
  // Accel config (set range and start self test)
  Wire.write(0x1C);
  Wire.write(accelConfigReg|0b11100000);
  Wire.endTransmission(true);
}

// Reads data from the MPU6050
// Currently only needs to read the accel and gyro
void readMPU6050() {
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);
  Wire.requestFrom(MPU6050,14,true);
  AcX = (uint32_t)(Wire.read()) << 8 + Wire.read();
  AcY = (uint32_t)(Wire.read()) << 8 + Wire.read();
  AcZ = (uint32_t)(Wire.read()) << 8 + Wire.read();
  temp = (uint32_t)(Wire.read()) << 8 + Wire.read();
  GyX = (uint32_t)(Wire.read()) << 8 + Wire.read();
  GyY = (uint32_t)(Wire.read()) << 8 + Wire.read();
  GyZ = (uint32_t)(Wire.read()) << 8 + Wire.read();
  Wire.endTransmission(true);
}

void calcVelAndRot() {
  
}
