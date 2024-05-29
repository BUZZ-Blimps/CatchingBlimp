/*
 BerryIMU_v3.cpp 
*/

#include "BerryIMU_v3.h"
#include <Wire.h> //I2C Library
#include "LSM6DSL.h"
#include "LIS3MDL.h"
#include "BM388.h"
#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>

void BerryIMU_v3::BerryIMU_v3_Setup(){
  Wire.begin();        // Initialise i2c
  Wire.setClock(100000);  //Change i2c bus speed to 400kHz
  // Serial.begin(115200); //Start serial for output set to 115200 Baud Rate
  ref_pressure_found = true;

  //Initialize the accelerometer
  //LSM6DSL_CTRL1_XL is the linear acceleration sensor control register 1
  //0b10011111 -> 0b says there is a binary input (example with +/- 8g
  //           -> 1001 is the ODR register settings ODR 3.33 kHz high performance
  //           -> 11 translated is the accelerometer full scale FS selection +/- 8g
  //           -> 1 is the LPF1 (digital filter) selection 0 disabled 1 enabled
  //           -> 1 is the analog chain bandwidth BW = 400hz
  //extra info the digital filter is run first then goes through the next composite filter
  //Uncomment writeTo with desired sensitivity
  //writeTo(LSM6DSL_ADDRESS,LSM6DSL_CTRL1_XL,0b10010011);        // ODR 3.33 kHz, +/- 2g , BW = 400hz
  //writeTo(LSM6DSL_ADDRESS,LSM6DSL_CTRL1_XL,0b10011011);        // ODR 3.33 kHz, +/- 4g , BW = 400hz
  writeTo(LSM6DSL_ADDRESS, LSM6DSL_CTRL1_XL, 0b10011111);      // ODR 3.33 kHz, +/- 8g , BW = 400hz
  //writeTo(LSM6DSL_ADDRESS,LSM6DSL_CTRL1_XL,0b10010111);        // ODR 3.33 kHz, +/- 16g , BW = 400hz

  //LSM6DSL_CTRL8_XL is the linear acceleration sensor control register 8
  //0b11001000 -> 0b says there is a binary input
  //           -> 1 low pass filter LPF2 selection LPF2_XL_EN (0 is low pass path 1 /  low pass path 2)
  //           -> 10 LPF2 and HP filter configuration and cutoof setting
  //           -> 0 HP filter reference mode (0 disabled / 1 enabled)
  //           -> 1 input composite (0 ODR/2 low pass filtered sent to composite filter (default) / 1 ODR/4 low pass filtered sent to composite filter)
  //           -> 0 HP_Slope_XL_EN (0 low pass pass / 1 high pass path)
  //           -> 0 always 0
  //           -> 0 LOW_PASS_ON_6D
  writeTo(LSM6DSL_ADDRESS, LSM6DSL_CTRL8_XL, 0b11001000);      // Low pass filter enabled, BW9, composite filter

  //LSM6DSL_CTRL3_C is the control register 3
  //0b01000100 -> 0b says there is a binary input
  //           -> 0 BOOT
  //           -> 1 BDU
  //           -> 0 H_LACTIVE
  //           -> 0 PP_OD
  //           -> 0 SIM
  //           -> 1 IF_INC
  //           -> 0 BLE
  //           -> 0 SW_RESET
  // Keep these as they are most are default settings
  writeTo(LSM6DSL_ADDRESS, LSM6DSL_CTRL3_C, 0b01000100);       // Enable Block Data update, increment during multi byte read

  //**********************************************************************************************************************************************************

  //Initialize the gyroscope
  //LSM6DSL_CTRL2_G is the angular rate sensor control register 2
  //0b10011100 -> 0b says there is a binary input
  //           -> 1001 ODR gyroscope output data rate selection
  //           -> 11 FS full scale selection (00: 250 dps; 01: 500 dps; 10: 1000 dps; 11: 2000 dps)
  //           -> 0 FS_125 Gyroscope full-scale at 125 dps (0: disabled; 1: enabled)
  //           -> 0 always 0
  writeTo(LSM6DSL_ADDRESS, LSM6DSL_CTRL2_G, 0b10011100);       // ODR 3.3 kHz, 2000 dps
  
  //Can add a high pass filter to the gyro via the CTRL7_G Angular rate sensor control register 7
  //writeTo(LSM6DSL_ADDRESS, LSM6DSL_CTRL7_G, 0b01000000);      // High Pass Enabled, HP filter cutoff 16mHz

  //Can add a low pass filter to the gyro via the CTRL6_C Angular rate sensor control register 6
  //writeTo(LSM6DSL_ADDRESS, LSM6DSL_CTRL6_C, 0b00000000);      // Not in use

  //**********************************************************************************************************************************************************

  //Initialize the magnetometer
  //Will do later
  writeTo(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG1, 0b11011100);     // Temp sesnor enabled, High performance, ODR 80 Hz, FAST ODR disabled and Selft test disabled.
  writeTo(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG2, 0b00100000);     // +/- 8 gauss
  writeTo(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG3, 0b00000000);     // Continuous-conversion mode

  //**********************************************************************************************************************************************************

  //Initialize the Barometer BMP388 and Temperature Sensor

  //bits read in the following order (bit 7 - bit 0)
  //ends with bit 0 starts with bit 7

  //Drone preset configuration
  //writeTo(BM388_ADDRESS, PWR_CTRL, 0b00110011);    // Enables pressure sensor, Enables temperature sensor, Normal mode
  //writeTo(BM388_ADDRESS, OSR, 0b00000011);    // x8 oversampling pressure measurement, x1 oversampling temp measurement
  //writeTo(BM388_ADDRESS, ODR, 0x02);    // Output data rate 50Hz
  //writeTo(BM388_ADDRESS, CONFIG, 0b00000010);     // IIR filter coefficient of 3

  //Indoor navigation configuration
  writeTo(BM388_ADDRESS,PWR_CTRL, 0b00110011);     // Enables pressure sensor, Enables temperature sensor, Normal mode
  writeTo(BM388_ADDRESS,OSR, 0b00001100);                 // x16 oversampling pressure measurement, x2 oversampling temp measurement
  writeTo(BM388_ADDRESS,ODR, 0x03);                 // Output data rate 25Hz
  writeTo(BM388_ADDRESS,CONFIG, 0b00000110);      // IIR filter coefficient of 63

  //Loading the calibration values
  readFrom(BM388_ADDRESS, NVM_PAR_T1_LSB, 21, buff_calib);
  float NVM_PAR_T1_val = (uint16_t)(buff_calib[0] | (buff_calib[1] << 8));
  float NVM_PAR_T2_val = (uint16_t)(buff_calib[2] | (buff_calib[3] << 8));
  float NVM_PAR_T3_val = (int8_t)buff_calib[4];
  float NVM_PAR_P1_val = (int16_t)(buff_calib[5] | (buff_calib[6] << 8));
  float NVM_PAR_P2_val = (int16_t)(buff_calib[7] | (buff_calib[8] << 8));
  float NVM_PAR_P3_val = (int8_t)buff_calib[9];
  float NVM_PAR_P4_val = (int8_t)buff_calib[10];
  float NVM_PAR_P5_val = (uint16_t)(buff_calib[11] | (buff_calib[12] << 8));
  float NVM_PAR_P6_val = (uint16_t)(buff_calib[13] | (buff_calib[14] << 8));
  float NVM_PAR_P7_val = (int8_t)buff_calib[15];
  float NVM_PAR_P8_val = (int8_t)buff_calib[16];
  float NVM_PAR_P9_val = (int16_t)(buff_calib[17] | (buff_calib[18] << 8));
  float NVM_PAR_P10_val = (int8_t)buff_calib[19];
  float NVM_PAR_P11_val = (int8_t)buff_calib[20];

  //Floating point compensation
  PAR_T1 = NVM_PAR_T1_val / pow(2, -8);
  PAR_T2 = NVM_PAR_T2_val / pow(2, 30);
  PAR_T3 = NVM_PAR_T3_val / pow(2, 48);
  PAR_P1 = (NVM_PAR_P1_val - pow(2, 14)) / pow(2, 20);
  PAR_P2 = (NVM_PAR_P2_val - pow(2, 14)) / pow(2, 29);
  PAR_P3 = NVM_PAR_P3_val / pow(2, 32);
  PAR_P4 = NVM_PAR_P4_val / pow(2, 37);
  PAR_P5 = NVM_PAR_P5_val / pow(2, -3);
  PAR_P6 = NVM_PAR_P6_val / pow(2, 6);
  PAR_P7 = NVM_PAR_P7_val / pow(2, 8);
  PAR_P8 = NVM_PAR_P8_val / pow(2, 15);
  PAR_P9 = NVM_PAR_P9_val / pow(2, 48);
  PAR_P10 = NVM_PAR_P10_val / pow(2, 48);
  PAR_P11 = NVM_PAR_P11_val / pow(2, 65);

  //**********************************************************************************************************************************************************
}

void BerryIMU_v3::IMU_read(){
  //Read the measurements from the sensors, combine and convert to correct values
  //The values are expressed in 2’s complement (MSB for the sign and then 15 bits for the value)
  //Start at OUT_X_L_A and read 6 bytes.

  //---------------------------------------------------------------------------------------
  //Accelerometer Output
  readFrom(LSM6DSL_ADDRESS, LSM6DSL_OUT_X_L_XL, 6, buff);
  accRaw[0] = (int)(buff[0] | (buff[1] << 8));
  accRaw[1] = (int)(buff[2] | (buff[3] << 8));
  accRaw[2] = (int)(buff[4] | (buff[5] << 8));
  // Bit shift done by 256^2 = 65536 and 65536/2 = 32768
  if (accRaw[0] >= 32768) accRaw[0] = accRaw[0] - 65536;
  if (accRaw[1] >= 32768) accRaw[1] = accRaw[1] - 65536;
  if (accRaw[2] >= 32768) accRaw[2] = accRaw[2] - 65536;

  //  //Convert Accel raw to G's when FS is +/- 2g
  //  AccXraw = (accRaw[0]* 0.061)/1000;
  //  AccYraw = (accRaw[1]* 0.061)/1000;
  //  AccZraw = (accRaw[2]* 0.061)/1000;

  //  //Convert Accel raw to G's when FS is +/- 4g
  //  AccXraw = (accRaw[0]* 0.122)/1000;
  //  AccYraw = (accRaw[1]* 0.122)/1000;
  //  AccZraw = (accRaw[2]* 0.122)/1000;

  //Convert Accel raw to G's when FS is +/- 8g
  AccYraw = (accRaw[0] * 0.244) / 1000.0;
  AccXraw = -(accRaw[1] * 0.244) / 1000.0;
  AccZraw = (accRaw[2] * 0.244) / 1000.0;

  //  //Convert Accel raw to G's when FS is +/- 16g
  //  AccXraw = (accRaw[0]* 0.488)/1000;
  //  AccYraw = (accRaw[1]* 0.488)/1000;
  //  AccZraw = (accRaw[2]* 0.488)/1000;

  //---------------------------------------------------------------------------------------
  //Magnetometer Output
  readFrom(LIS3MDL_ADDRESS, 0x80 | LIS3MDL_OUT_X_L, 6, buff);
  magRaw[0] = (int)(buff[0] | (buff[1] << 8));
  magRaw[1] = (int)(buff[2] | (buff[3] << 8));
  magRaw[2] = (int)(buff[4] | (buff[5] << 8));
  if (magRaw[0] >= 32768) magRaw[0] = magRaw[0] - 65536;
  if (magRaw[1] >= 32768) magRaw[1] = magRaw[1] - 65536;
  if (magRaw[2] >= 32768) magRaw[2] = magRaw[2] - 65536;

  //convert?
  MagYraw = magRaw[0];
  MagXraw = -magRaw[1];
  MagZraw = magRaw[2];

  //---------------------------------------------------------------------------------------
  //Gyroscope Output
  readFrom(LSM6DSL_ADDRESS, LSM6DSL_OUT_X_L_G, 6, buff);
  gyrRaw[0] = (int)(buff[0] | (buff[1] << 8));
  gyrRaw[1] = (int)(buff[2] | (buff[3] << 8));
  gyrRaw[2] = (int)(buff[4] | (buff[5] << 8));
  if (gyrRaw[0] >= 32768) gyrRaw[0] = gyrRaw[0] - 65536;
  if (gyrRaw[1] >= 32768) gyrRaw[1] = gyrRaw[1] - 65536;
  if (gyrRaw[2] >= 32768) gyrRaw[2] = gyrRaw[2] - 65536;

  //Convert Gyro raw to degrees per second updated (deg/s)
  gyr_rateYraw = (gyrRaw[0] * 70) / 1000.0;
  gyr_rateXraw = -(gyrRaw[1] * 70) / 1000.0;
  gyr_rateZraw = (gyrRaw[2] * 70) / 1000.0;

  //---------------------------------------------------------------------------------------
  //Barometer and Temperature Sensor Output
  //Starts with the PRESS_XLSB_7_0 output register then will read the remainig five
  readFrom(BM388_ADDRESS, PRESS_XLSB_7_0, 6, buff);

  // Last 3 bytes are the temperature XLSB, LSB, MSB
  float tempRaw = (int)(buff[3] | (buff[4] << 8) | (buff[5] << 16));
  comp_temp = temp_compensation(tempRaw);
  //Serial.println(comp_temp); //Temperature in deg C

  // First 3 bytes are the pressure XLSB, LSB, MSB
  // Bit shift done by 256^3 = 16777216 and 16777216/3 = 5592405
  pressRaw = (int)(buff[0] | (buff[1] << 8) | (buff[2] << 16));
  comp_press = press_compensation(pressRaw, comp_temp);
  // Serial.println(comp_press); //Pressure in Pa

  //Altitude (in meters)
  //https://www.circuitbasics.com/set-bmp180-barometric-pressure-sensor-arduino/
  //Sets the reference pressure (therefore setting the reference height)
  if(ref_pressure_found){
    ref_ground_press = comp_press;
    ref_pressure_found = false;
  }
  alt = 44330 * (1 - pow((comp_press / ref_ground_press), (1 / 5.255))); //In meters

  // alt = comp_press;  // plus something from base station

  //---------------------------------------------------------------------------------------
}

void BerryIMU_v3::IMU_ROTATION(float rotation_angle){  //current: 180 degrees z axis rotation
      BLA::Matrix<3, 3> Rz = {cosf(rotation_angle/180*PI),-sinf(rotation_angle/180*PI),0,
                              sinf(rotation_angle/180*PI),cosf(rotation_angle/180*PI),0,
                              0,0,1}; // 3X3 Matrix
      BLA::Matrix<3, 1> gyr_rate= {gyr_rateXraw,
                                   gyr_rateYraw,
                                   gyr_rateZraw};
      BLA::Matrix<3, 1> Acc_raw = {AccXraw,
                                   AccYraw,
                                   AccZraw};
      BLA::Matrix<3, 1> corrected_gyr_rate = Rz*gyr_rate;
      BLA::Matrix<3, 1> corrected_Acc_raw = Rz*Acc_raw;

      this->AccXraw = corrected_Acc_raw(0);
      this->AccYraw = corrected_Acc_raw(1);
      this->AccZraw = corrected_Acc_raw(2);

      this->gyr_rateXraw = corrected_gyr_rate(0);
      this->gyr_rateYraw = corrected_gyr_rate(1);
      this->gyr_rateZraw = corrected_gyr_rate(2);
}


float BerryIMU_v3::temp_compensation(float raw_temperature) {
  float partial_data1 = raw_temperature - PAR_T1;
  float partial_data2 = partial_data1 * PAR_T2;
  float comp_temp = partial_data2 + (partial_data1 * partial_data1) * PAR_T3;
  return comp_temp;
}


float BerryIMU_v3::press_compensation(float raw_pressure, float comp_temp) {
  float partial_data1 = PAR_P6 * comp_temp;
  float partial_data2 = PAR_P7 * (comp_temp * comp_temp);
  float partial_data3 = PAR_P8 * (comp_temp * comp_temp * comp_temp);
  float partial_out1 = PAR_P5 + partial_data1 + partial_data2 + partial_data3;
  partial_data1 = PAR_P2 * comp_temp;
  partial_data2 = PAR_P3 * (comp_temp * comp_temp);
  partial_data3 = PAR_P4 * (comp_temp * comp_temp * comp_temp);
  float partial_out2 = raw_pressure * (PAR_P1 + partial_data1 + partial_data2 + partial_data3);
  partial_data1 = raw_pressure * raw_pressure;
  partial_data2 = PAR_P9 + PAR_P10 * comp_temp;
  partial_data3 = partial_data1 * partial_data2;
  float partial_data4 = partial_data3 + (raw_pressure * raw_pressure * raw_pressure) * PAR_P11;
  float comp_press = partial_out1 + partial_out2 + partial_data4;

  return comp_press;
}

void BerryIMU_v3::writeTo(int device, byte address, byte val) {
  Wire.beginTransmission(device); //start transmission to device
  Wire.write(address);        // send register address
  Wire.write(val);        // send value to write (writing byte to control register to change sensor parameters)
  Wire.endTransmission(); //end transmission
}

//Used for Accelerometer,Gyroscope, Magnetometer, and Barometer
void BerryIMU_v3::readFrom(int device, byte address, int num, byte buff[]) {
  Wire.beginTransmission(device); //start transmission to device
  Wire.write(address);        //sends address to read from
  Wire.endTransmission(); //end transmission

  Wire.beginTransmission(device); //start transmission to device (initiate again)
  Wire.requestFrom(device, num);    // request 6 bytes from device (2 for each axis's MSB and LSB)

  int i = 0;
  while (Wire.available())   //device may send less than requested (abnormal)
  {
    buff[i] = Wire.read(); // receive a byte
    i++;
  }
  Wire.endTransmission(); //end transmission
}
