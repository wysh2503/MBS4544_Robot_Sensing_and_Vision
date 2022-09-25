#include "MPU9250.h"
#include <math.h>

// MPU9250 object on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;

float aX, aY, aZ;
float gX, gY, gZ;
float mX, mY, mZ;
float thetaMea, thetaMeaDeg, thetaFilOld=0, thetaFilNew, thetaG=0;
float theta=0, thetaDeg;
float phiMea, phiMeaDeg, phiFilOld=0, phiFilNew, phiG=0;
float phi=0, phiDeg;
float Xm, Ym, psi, psiDeg;
float dt;
unsigned long millisOld;

void setup() {
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  // start communication with IMU 
  status = IMU.begin();

  //IMU.calibrateAccel(); //calibrate the accelerometer
  //IMU.calibrateGyro(); //calibrate the gyroscope
  //IMU.calibrateMag(); //calibrate the magnetometer
  millisOld = millis();
}

void loop() {
  IMU.readSensor(); //read the sensor
  aX = IMU.getAccelX_mss();
  aY = IMU.getAccelY_mss();
  aZ = IMU.getAccelZ_mss();
  gX = IMU.getGyroX_rads();
  gY = IMU.getGyroY_rads();
  gZ = IMU.getGyroZ_rads();
  mX = IMU.getMagX_uT();
  mY = IMU.getMagY_uT();
  mZ = IMU.getMagZ_uT();

  thetaMea = atan2(aX,aZ); // pitch measured by accelerometer
  thetaMeaDeg = thetaMea/2/3.14*360;
  phiMea = atan2(aY,aZ); // roll measured by accelerometer
  phiMeaDeg = phiMea/2/3.14*360;
  phiFilNew = 0.9*phiFilOld + 0.1*phiMeaDeg; // simple LPF
  thetaFilNew = 0.9*thetaFilOld + 0.1*thetaMeaDeg; // simple LPF

  dt = (millis() - millisOld)/1000.;
  millisOld = millis();
  
  // Getting pitch and roll from gyroscope
  thetaG = (thetaG + gY*dt);
  phiG = (phiG + gX*dt);

  // Complimentary filter
  theta = (theta + gY*dt)*0.95 + thetaMea*0.05;
  thetaDeg = theta/2/3.14*360;
  phi = (phi + gX*dt)*0.95 + phiMea*0.05;
  phiDeg = phi/2/3.14*360;

  // Calculate Yaw
  Xm = mX*cos(theta)-mY*sin(phi)*sin(theta)+mZ*cos(phi)*sin(theta);
  Ym = mY*cos(phi)+mZ*sin(phi);
  psi = atan2(Ym,Xm);
  psiDeg = psi/(2*3.14)*360;
  
  // display the data
  //Serial.print("AccelX: ");
  Serial.print(aX,3);
  Serial.print(",");
  //Serial.print("AccelY: ");
  Serial.print(aY,3);
  Serial.print(",");
  //Serial.print("AccelZ: ");
  Serial.print(aZ,3);
  Serial.print(",");
  Serial.print(gX,3);
  Serial.print(",");
  Serial.print(gY,3);
  Serial.print(",");
  Serial.print(gZ,3);
  Serial.print(",");
  Serial.print(mX,3); 
  Serial.print(",");
  Serial.print(mY,3);
  Serial.print(",");
  Serial.print(mZ,3);
  Serial.print(",");
  Serial.print(thetaMeaDeg);
  Serial.print(",");
  Serial.print(phiMeaDeg);
  Serial.print(",");
  Serial.print(thetaFilNew);
  Serial.print(",");
  Serial.print(phiFilNew);
  Serial.print(",");
  Serial.print(thetaG);
  Serial.print(",");
  Serial.print(phiG);
  Serial.print(",");
  Serial.print(thetaDeg);
  Serial.print(",");
  Serial.print(phiDeg);
  Serial.print(",");
  Serial.print(psiDeg);
  Serial.print(",");
  Serial.println(IMU.getTemperature_C(),3);

  thetaFilOld = thetaFilNew;
  phiFilOld = phiFilNew;
  
  delay(100);
}
