#include <PID_v1.h>
#include <MPU6050.h>
#include <I2Cdev.h>
#include <microsmooth.h>
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <printf.h>
#include "Wire.h"
#include "Kalman.h" 

#define MAX_SIZE 6
#define CE_PIN 7
#define CSN_PIN 8
#define FC_NODE 1
#define CONTROLLER_NODE 0
#define TYPE_ANGLE 0 
#define TYPE_CONTROL 1
#define TYPE_PID 2
#define PIN_FRONT 3
#define PIN_BACK 5
#define PIN_LEFT 6
#define PIN_RIGHT 9

RF24 radio(CE_PIN,CSN_PIN);
RF24Network network(radio);
struct payload_t
{
  double dataVector[MAX_SIZE];
};

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

Kalman kalmanX;
Kalman kalmanY;

uint32_t timer,gyroTimer,radioTimer;
double kalAngleX, kalAngleY;
double compAngleRoll, compAnglePitch;
double consKp=1, consKi=0.05, consKd=0.25;
double PIDOutputX,PIDOutputY;
double setPointX,setPointY;
double angleVec[10]; //X - Y
double PIDVec[10]; // X - Y 
int throttle = 0;
int sampleTime,radioTime,currentSample = 0;
PID PIDX(&kalAngleX, &PIDOutputX, &setPointX, consKp, consKi, consKd, DIRECT);
PID PIDY(&kalAngleY, &PIDOutputY, &setPointY, consKp, consKi, consKd, DIRECT);
void setup() {
  pinMode(PIN_FRONT,OUTPUT);
  pinMode(PIN_BACK,OUTPUT);
  pinMode(PIN_LEFT,OUTPUT);
  pinMode(PIN_RIGHT,OUTPUT);
  Wire.begin();

  Serial.begin(115200);    
  printf_begin();
  SPI.begin();
  radio.begin();
  network.begin(/*channel*/ 90, /*node address*/ FC_NODE);

  accelgyro.initialize();

  accelgyro.setXGyroOffset(3);
  accelgyro.setYGyroOffset(-9);
  accelgyro.setZGyroOffset(2);
  accelgyro.setXAccelOffset(482);
  accelgyro.setYAccelOffset(2742);
  accelgyro.setZAccelOffset(1489);


  delay(100);
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  double roll  = atan2(ay, az) * RAD_TO_DEG;
  double pitch = atan(-ax / sqrt((double) ay * ay + (double) az * az)) * RAD_TO_DEG;
  compAngleRoll  = roll;
  compAnglePitch = pitch;
  kalmanX.setAngle(roll); 
  kalmanY.setAngle(pitch);
  setPointX = setPointY = 0;
  sampleTime = 10;
  radioTime = 10;
  PIDX.SetSampleTime(sampleTime);
  PIDX.SetMode(AUTOMATIC);
  PIDX.SetOutputLimits(-10,10);

  PIDY.SetSampleTime(sampleTime);
  PIDY.SetMode(AUTOMATIC);
  PIDY.SetOutputLimits(-10,10);

}

void loop() {
  if(millis() - gyroTimer > sampleTime)
  {
    readGyroKalmann();
    gyroTimer = millis();
  }
  if(millis() - radioTimer > radioTime)
  {
    sendData();
    radioTimer = millis();
  }
  network.update();
  if( network.available() )
  {
     readData(); 
  }
//  if(throttle > 90)
//  {  
//     analogWrite(PIN_LEFT,throttle+PIDOutputY);
//     analogWrite(PIN_RIGHT,throttle-PIDOutputY);
//  }
//  else
//  {
//    analogWrite(PIN_LEFT,throttle);
//    analogWrite(PIN_RIGHT,throttle);
//  }
    analogWrite(PIN_FRONT,throttle);
    analogWrite(PIN_BACK,throttle);
    analogWrite(PIN_LEFT,throttle);
    analogWrite(PIN_RIGHT,throttle);
}
void sendData()
{
  network.update();
  payload_t payload;

  payload.dataVector[0] = angleVec[0];  
  payload.dataVector[1] = angleVec[1]; 
  payload.dataVector[2] = PIDVec[0];
  payload.dataVector[3] = PIDVec[1];
  RF24NetworkHeader header(/*to node*/ CONTROLLER_NODE,TYPE_ANGLE);
  bool ok = network.write(header,&payload,sizeof(payload));
}


void readData()
{
  RF24NetworkHeader header;
  payload_t payload;
  network.read(header,&payload,sizeof(payload));
  switch (header.type)
  {
  case TYPE_CONTROL:
    {
      throttle = payload.dataVector[0];
      Serial.println(throttle);
      break;
    }
  case TYPE_PID:
    {
      consKp=payload.dataVector[0];
      consKi=payload.dataVector[1]; 
      consKd=payload.dataVector[2];
      Serial.println(consKp);
      Serial.println(consKi);
      Serial.println(consKd);
      PIDX.SetTunings(consKp,consKi,consKd);
      PIDY.SetTunings(consKp,consKi,consKd);
      break;
    }
  }
}

void readGyroKalmann()
{
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
  double roll  = atan2((double)  ay,(double)  az) * RAD_TO_DEG;
  double pitch = atan(-ax / sqrt( (double)  ay * ay +  (double)  az * az)) * RAD_TO_DEG;
  double gyroXrate = (double) gx / 131.0; // Convert to deg/s
  double gyroYrate = (double) gy / 131.0; // Convert to deg/s
  
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    kalAngleX = roll;
  } 
  else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  PIDX.Compute();
  PIDY.Compute();
  angleVec[0] = kalAngleX;
  angleVec[1] = kalAngleY;
  PIDVec[0] = PIDOutputX;
  PIDVec[1] = PIDOutputY;
}
void printGyro()
{
  Serial.print(compAngleRoll); 
  Serial.print("\t");
  Serial.print(compAnglePitch); 
  Serial.print("\t");
  Serial.print(PIDOutputX); 
  Serial.print("\t");
  Serial.print(PIDOutputY); 
  Serial.print("\t");
  Serial.println();
}




