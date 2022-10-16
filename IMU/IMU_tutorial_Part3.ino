/* IMU - MPU9250: a 6-axis Accelerometer and Gyroscope + integrated magnetometer (AK8963)
 * Reading raw data from MPU9250 IMU sensor via I2C
 * Convert raw data into Roll, Pitch, Yaw angles
 * created by: Winston Yeung
 * revision date: 16 Oct 2022
 */

#include "Wire.h"

const int IMU_ADDR = 0x68; // I2C address for IMU. ADO=5V -> 0x69
const int MAG_ADDR = 0x0C; //Magnetometer (AK8963) address

// Declare variables.
// NOTE: theta = pitch, phi = roll, psi = yaw
int16_t accel_X, accel_Y, accel_Z, temp_raw, gyro_X, gyro_Y, gyro_Z; // variables for raw sensor data
double aX, aY, aZ, theta_acc, phi_acc; //accel values and theta and phi calculated from accelerometer
double theta_acc_previous = 0, theta_acc_LP, phi_acc_previous = 0, phi_acc_LP;
double gX, gY, gZ, theta_gyro=0, phi_gyro=0, psi_gyro;
double theta_compli, phi_compli; //theta and phi calculated from accelerometer and gyroscope
unsigned long previousTime, elapseTime;
double dt;

//@@@@@@@@@ Declare variables for magnetometer @@@@@@@@
int8_t Device_ID; // device ID for magnetometer
uint8_t mag_XL, mag_XH, mag_YL, mag_YH, mag_ZL, mag_ZH; //0x03 (HXL), 0x04 (HXH),0x05 (HYL), 0x06 (HYH),0x07 (HZL), 0x08 (HZH)
int16_t mag_X, mag_Y, mag_Z;
double mX, mY; // compensated magnetometer X and Y
int8_t Msens_X,Msens_Y, Msens_Z;
float d2r; // degree to radian

int8_t control_1; //0x0A (CNTL)
int8_t status_1; //0x02 (ST1)
int8_t status_2; //0x09 (ST2)
float psi;
float asaX, asaY, asaZ; //0x10 (ASAX), 0x11, 0x12
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

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

  Wire.beginTransmission(IMU_ADDR); //communicate with IMU
  Wire.write(0x37);   //  IMU INT PIN CONFIG    
  Wire.write(0x02);   //  0x02 activate bypass in order to communicate with magnetometer
  Wire.endTransmission(true);
  delay(100);

  // ************ Magnetometer Registers*************************************
  Wire.beginTransmission(MAG_ADDR); 
  Wire.write(0x0B);   //  CONTROL 2
  Wire.write(0b00000001);   //  0 NORMAL OR 1 RESET
  Wire.endTransmission(true);
  delay(100);

  Wire.beginTransmission(MAG_ADDR);   //SLEEP MODE
  Wire.write(0x0A);   //  CONTROL 1
  Wire.write(0b00010000);   // 1 for 16 bit or 0 for 14 bit output, 0000 SLEEP MODE
  Wire.endTransmission(true);
  delay(100);

  Wire.beginTransmission(MAG_ADDR);   //ROM WRITE MODE
  Wire.write(0x0A);   //  CONTROL 1
  Wire.write(0b00011111); // 1 for 16 bit or 0 for 14 bit output, 1111 FUSE ROM ACCESS MODE
  Wire.endTransmission(true);
  delay(100);

  Wire.beginTransmission(MAG_ADDR);   //GET MAGNETIC SENSITIVITY DATA FOR CONVERTING RAW DATA
  Wire.write(0x10);     //  ASAX  
  Wire.endTransmission(false);
  Wire.requestFrom(MAG_ADDR, 3 , true);  //GET SENSITIVITY ADJUSMENT VALUES STARTS AT ASAX
  Msens_X = Wire.read();    //GET X SENSITIVITY ADJUSMENT VALUE
  Msens_Y = Wire.read();    //GET Y SENSITIVITY ADJUSMENT VALUE
  Msens_Z = Wire.read();    //GET Z SENSITIVITY ADJUSMENT VALUE
//  Serial.println(Msens_X);
//  Serial.println(Msens_Y);
//  Serial.println(Msens_Z);
  Wire.endTransmission(true);
  asaX = (((Msens_X-128))/256.0f)+1.0f;
  asaY = (((Msens_Y-128))/256.0f)+1.0f;
  asaZ = (((Msens_Z-128))/256.0f)+1.0f;
//  Serial.print("Mx Sensitivity: ");  Serial.println(asaX);
//  Serial.print("My Sensitivity: ");  Serial.println(asaY);
//  Serial.print("Mz Sensitivity: ");  Serial.println(asaZ); 
  delay(100);
  
//  Wire.beginTransmission(MAG_ADDR);   //SLEEP MODE
//  Wire.write(0x0A);   //  CONTROL 1
//  Wire.write(0b00010000);  // 1 for 16 bit or 0 for 14 bit output, 0000 SLEEP MODE
//  Wire.endTransmission(true);
//  delay(100);
    
  Wire.beginTransmission(MAG_ADDR);   //CONT MODE 2
  Wire.write(0x0A);
  Wire.write(0b00010110); // 1 for 16 bit or 0 for 14 bit output, 0110 FOR CONT MODE 2
  Wire.endTransmission(true);
  delay(100);
  // **********************************************************
  previousTime = millis();
}

void loop() {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x3B); // start from ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(IMU_ADDR, 14, true); //request a total of 14 bytes

  // read registers from IMU
  accel_X = Wire.read()<<8 | Wire.read(); //read registers 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  accel_Y = Wire.read()<<8 | Wire.read(); //read registers 0x3D & 0x3E
  accel_Z = Wire.read()<<8 | Wire.read(); //read registers 0x3F & 0x40
  temp_raw = Wire.read()<<8 | Wire.read(); //read registers 0x41 & 0x42
  gyro_X = Wire.read()<<8 | Wire.read(); //read registers 0x43  (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L) 
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
//------------end of Roll(phi) and Pitch(theta) from accelerometer readings---------//

//--------------Getting Roll(phi) and Pitch(theta) from gyroscope readings only-------------------//
  gX = gyro_X/131.0 +2.2 ; // convert the raw data to rad/s. "+2.2" is the gyro error
  gY = gyro_Y/131.0 + 0.5;
  gZ = gyro_Z/131.0;

  elapseTime = millis() - previousTime;
  dt = elapseTime / 1000.0;
  previousTime = millis();

  theta_gyro = theta_gyro - gY * dt; // pitch angle from gyro
  phi_gyro = phi_gyro + gX * dt; // roll angle from gyro
  psi_gyro = psi_gyro + gZ * dt; // yaw angle from gyro (not quite useful, only incremental value)
//-----------end of Roll(phi) and Pitch(theta) from gyroscope readings --------------------//

// ------- Complimentary filter from accelerometer and gyroscope data ---------//
// Complimentary filter uses a mix of two sensor data with 
//a specific ratio which depend on which one you trust better
  theta_compli = 0.95*(theta_compli - gY * dt) + 0.05*(theta_acc);
  phi_compli = 0.95*(phi_compli + gX * dt) + 0.05*(phi_acc);
// ------- end of Complimentary filter from accelerometer and gyroscope data -----//

//********** Get data from magnetometer *****************//
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x00); // 0x00 (WIA) - Device ID
  Wire.endTransmission(false);
  Wire.requestFrom(MAG_ADDR, 1 , true);   
  Device_ID = Wire.read();
//  Serial.print("Device_ID: "); Serial.println(Device_ID,DEC);  
  Wire.endTransmission(true);
 
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x0A); // Access Control byte
  Wire.endTransmission(false);
  Wire.requestFrom(MAG_ADDR, 1 , true);  
  control_1 = Wire.read();  // check DRDY bit if ready to read
//  Serial.print("control_1: "); Serial.println(control_1,BIN);  
  Wire.endTransmission(true);

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x02); // Access Status 1 byte
  Wire.endTransmission(false);
  Wire.requestFrom(MAG_ADDR, 1 , true);   
  status_1 = Wire.read();
//  Serial.print("Status 1: "); Serial.println(status_1,BIN);  
  Wire.endTransmission(true);

  if(status_1 == 0b00000011) {
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(0x03); //Start reading measurement data from address 0x03 (HXL)
    Wire.endTransmission(false);
    Wire.requestFrom(MAG_ADDR, 7 , true);

    mag_XL = Wire.read(); //first data is HXL - low byte of mag_X
    mag_XH = Wire.read();
    mag_YL = Wire.read();
    mag_YH = Wire.read();
    mag_ZL = Wire.read();
    mag_ZH = Wire.read();
    mag_X = ((int16_t)mag_XH << 8) | mag_XL; // combine two bytes in correct format
    mag_Y = ((int16_t)mag_YH << 8) | mag_YL; 
    mag_Z = ((int16_t)mag_ZH << 8) | mag_ZL; 
   
    status_2 = Wire.read();   // check if there is a magnetic sensor overflow (i.e. HOFL=1)
//    Serial.print("Status 2: "); Serial.println(status_2,BIN);  
    
    Wire.endTransmission(true);
    
    if(status_2 != 0x08){
//      Serial.print("  | mX = "); Serial.print(mag_X*asaX*0.15); Serial.print(" [uT]"); // "0.15" is a scale factor with 16-bit resolution
//      Serial.print("  | mY = "); Serial.print(mag_Y*asaY*0.15); Serial.print(" [uT]"); // [uT] is a unit in microTesla to measure magnetic field
//      Serial.print("  | mZ = "); Serial.print(mag_Z*asaZ*0.15); Serial.println(" [uT]");
      
     // psi = atan2(mag_Y,mag_X)*(180/PI); // Yaw angle in degree from magnetometer data
      
// For real situation, yaw angle must be compensated when the IMU is tilting in some angles while rotating
      d2r = PI/180.0; // degree to radian conversion
      mX = mag_Y*cos(theta_compli*d2r) - mag_X*sin(phi_compli*d2r)*sin(theta_compli*d2r) + mag_Z*cos(phi_compli*d2r)*sin(theta_compli*d2r);
      mY = mag_X*cos(phi_compli*d2r) + mag_Z*sin(phi_compli*d2r);
      psi = atan2(mY, mX)*(180./PI);
//
      
//      Serial.println(Yaw); 
    }
  }    
//****** end of magnetometer manipulation *************//

// === Print all data to serial plot ===//
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
  Serial.print(",");
  Serial.print(theta_compli);
  Serial.print(",");
  Serial.print(phi_compli);
  Serial.print(",");
  Serial.print(psi);
  Serial.println();
// === end of Printing data to serial port ===//
  
  delay(100);
}
