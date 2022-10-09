#include "Wire.h"
//#include "math.h"

const int IMU_ADDR = 0x68; //ADO=5V -> 0x69

// Declare variables.
// NOTE: theta = pitch, phi = roll, psi = yaw
int16_t accel_X, accel_Y, accel_Z, temp_raw, gyro_X, gyro_Y, gyro_Z;
double aX, aY, aZ, theta_acc, phi_acc; //accel values and theta and phi calculated from accelerometer
double theta_acc_previous = 0, theta_acc_LP, phi_acc_previous = 0, phi_acc_LP;
double gX, gY, gZ, theta_gyro=0, phi_gyro=0, psi_gyro;
unsigned long previousTime, elapseTime;
double dt;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  Wire.beginTransmission(IMU_ADDR); //communicate with IMU
  Wire.write(0x6B); //PWR_MGMT_1 register
  Wire.write(0x0);
  Wire.endTransmission(true);

  Wire.beginTransmission(IMU_ADDR); //communicate with IMU
  Wire.write(0x1C); //ACCEL_CONFIG register
  Wire.write(0b00000000); //Bit3 & Bit 4 -> 0,0 -> +/-2g
  Wire.endTransmission(true);

  Wire.beginTransmission(IMU_ADDR); //communicate with IMU
  Wire.write(0x1B); //GYRO_CONFIG register
  Wire.write(0b00000000); //Bit3 & Bit 4 -> 0,0 -> +/-250deg/s
  Wire.endTransmission(true);

  previousTime = millis();
}

void loop() {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x3B); // start from ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(IMU_ADDR, 14, true); //request a total of 14 bytes

  // read registers from IMU
  accel_X = Wire.read()<<8 | Wire.read(); //read registers 0x3B & 0x3C
  accel_Y = Wire.read()<<8 | Wire.read(); //read registers 0x3D & 0x3E
  accel_Z = Wire.read()<<8 | Wire.read(); //read registers 0x3F & 0x40
  temp_raw = Wire.read()<<8 | Wire.read(); //read registers 0x41 & 0x42
  gyro_X = Wire.read()<<8 | Wire.read(); //read registers 0x43 & 0x44
  gyro_Y = Wire.read()<<8 | Wire.read(); //read registers 0x45 & 0x46
  gyro_Z = Wire.read()<<8 | Wire.read(); //read registers 0x47 & 0x48
  
  Wire.endTransmission(true);

//  Serial.print("aX_raw: "); Serial.print(accel_X);Serial.print(", ");
//  Serial.print("aY_raw: "); Serial.print(accel_Y);Serial.print(", ");
//  Serial.print("aZ_raw: "); Serial.print(accel_Z);Serial.print(", ");
//  Serial.print("gX_raw: "); Serial.print(gyro_X);Serial.print(", ");
//  Serial.print("gY_raw: "); Serial.print(gyro_Y);Serial.print(", ");
//  Serial.print("gZ_raw: "); Serial.print(gyro_Z);Serial.print(", ");
//  Serial.print("Temp_raw: "); Serial.print(temp_raw);
//  Serial.print("aX: "); Serial.print(accel_X/16384.0);Serial.print(", ");
//  Serial.print("aY: "); Serial.print(accel_Y/16384.0);Serial.print(", ");
//  Serial.print("aZ: "); Serial.print(accel_Z/16384.0);
//  Serial.println();
  
  aX = accel_X/16384.0; // convert the raw data to g value. 1g = 9.8 m/s2
  aY = accel_Y/16384.0;
  aZ = accel_Z/16384.0;

 //--------------Getting Roll(phi) and Pitch(theta) from accelerometer readings only-------------------// 
  theta_acc = atan2(aX, aZ)*(180/PI); // approximation of theta (pitch). use either this or the eauation right below.
  // theta_acc = atan2(aX, sqrt(pow(aY,2)+pow(aZ,2)))*(180/PI); // another approximation on theta (pitch). Either use this or the equation right above.
  phi_acc = atan2(aY, aZ)*(180/PI); // approximation of phi (roll). use either this or the eauation right below.
  // phi_acc = atan2(aY, sqrt(pow(aX,2)+pow(aZ,2)))*(180/PI);// another approximation on phi (roll). Either use this or the equation right above.

  // simple Low Pass Filter (LPF) added for theta(pitch) and phi(roll)
  theta_acc_LP = 0.7*theta_acc_previous + 0.3*theta_acc; // adjust the ratio of preivous theta value and the newly measured value
  phi_acc_LP = 0.7*phi_acc_previous + 0.3*phi_acc;

  theta_acc_previous = theta_acc_LP;
  phi_acc_previous = phi_acc_LP;
//-------------------------------------------------------------------------------------//

//--------------Getting Roll(phi) and Pitch(theta) from gyroscope readings only-------------------//
  gX = gyro_X/131.0 +2.2 ; // convert the raw data to rad/s. "+2.2" is the gyro error
  gY = gyro_Y/131.0 + 0.5;
  gZ = gyro_Z/131.0;

  elapseTime = millis() - previousTime;
  dt = elapseTime / 1000.0;
  previousTime = millis();

  theta_gyro = theta_gyro + gY * dt; // pitch angle from gyro
  phi_gyro = phi_gyro + gX * dt; // roll angle from gyro
  psi_gyro = psi_gyro + gZ * dt; // yaw angle from gyro
//------------------------------------------------------------------------------------------------//

//  Print all data to serial plot 
  Serial.print(theta_acc);
  Serial.print(",");
  Serial.print(phi_acc);
  Serial.print(",");
  Serial.print(theta_acc_LP); // theta value from accel with LPF
  Serial.print(",");
  Serial.print(phi_acc_LP);
//
  Serial.print(",");
  Serial.print(theta_gyro);
  Serial.print(",");
  Serial.print(phi_gyro);
  Serial.print(",");
  Serial.print(psi_gyro);
  Serial.println();
  
  delay(50);
}
