/*
Helpful commands:

To make a file executable: chmod 755 <filename>

To run the copy script: ./copyCodeToPi.sh <Orange Pi's IP>
- For Orange Pi number #, the IP address will be: 192.168.0.10#
- All laptops should also have the hostnames saved as opi#

To ssh into Orange Pi number #: ssh opi@opi#

To save an ssh password for Orange Pi number #: ssh-copy-id opi@opi#
*/

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <stdbool.h>
#include "LSM6DSL.h"
#include "LIS3MDL.h"
#include "BM388.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <stdint.h>
// #include <linux/i2c-dev.h>
// #include <i2c/smbus.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>
// #include <wiringPiI2C.c>
// I2C definitions

#define I2C_SLAVE	0x0703
#define I2C_SMBUS	0x0720	/* SMBus-level access */

#define I2C_SMBUS_READ	1
#define I2C_SMBUS_WRITE	0

// SMBus transaction types

#define I2C_SMBUS_QUICK		    0
#define I2C_SMBUS_BYTE		    1
#define I2C_SMBUS_BYTE_DATA	    2 
#define I2C_SMBUS_WORD_DATA	    3
#define I2C_SMBUS_PROC_CALL	    4
#define I2C_SMBUS_BLOCK_DATA	    5
#define I2C_SMBUS_I2C_BLOCK_BROKEN  6
#define I2C_SMBUS_BLOCK_PROC_CALL   7		/* SMBus 2.0 */
#define I2C_SMBUS_I2C_BLOCK_DATA    8

// SMBus messages

#define I2C_SMBUS_BLOCK_MAX	32	/* As specified in SMBus standard */	
#define I2C_SMBUS_I2C_BLOCK_MAX	32	/* Not specified but we use same structure */

// Structures used in the ioctl() calls

union i2c_smbus_data
{
  uint8_t  byte ;
  uint16_t word ;
  uint8_t  block [I2C_SMBUS_BLOCK_MAX + 2] ;	// block [0] is used for length + one more for PEC
} ;

struct i2c_smbus_ioctl_data
{
  char read_write ;
  uint8_t command ;
  int size ;
  union i2c_smbus_data *data ;
} ;

static inline int i2c_smbus_access_bl (int fd, char rw, uint8_t command, int size, union i2c_smbus_data *data)
{
  struct i2c_smbus_ioctl_data args ;

  args.read_write = rw ;
  args.command    = command ;
  args.size       = size ;
  args.data       = data ;
  return ioctl (fd, I2C_SMBUS, &args) ;
}

// int wiringPiI2CReadRegBlock (int fd, int reg)
// {
//   union i2c_smbus_data data;

//   if (i2c_smbus_access_bl (fd, I2C_SMBUS_READ, reg, I2C_SMBUS_WORD_DATA, &data))
//     return -1 ;
//   else
//     return data.word & 0xFFFF ;
// }

int wiringPiI2CReadRegBlock (int fd, int reg, uint8_t *buff)
{
  union i2c_smbus_data data;
  if (i2c_smbus_access_bl (fd, I2C_SMBUS_READ, reg, 6, &data))
    return -1 ;
  else
	for (int i = 0; i < 6; i++) {
		buff[i] = data.block[i + 1];
	}
	// buff[0] = data.block[1];
	// buff[1] = data.block[2];
	// buff[2] = data.block[3];
	// buff[3] = data.block[4];
	// buff[4] = data.block[5];
	// buff[5] = data.block[6];
    return data.block[0];
//   if (i2c_smbus_read_i2c_block_data(fd, I2C_SMBUS_I2C_BLOCK_DATA, reg, 6, &data))
//     return -1 ;
//   else
//     return data.word & 0xFFFF ;
}

int main (int argc, char *argv[]){
	const char *device;
	device = "/dev/i2c-3"; 

	int LIS3MDL = wiringPiI2CSetupInterface(device, LIS3MDL_ADDRESS);
	printf("i2c LIS3MDL fd: %d\n", LIS3MDL);
	// delay(100);

	int LSM6DSL = wiringPiI2CSetupInterface(device, LSM6DSL_ADDRESS);
	printf("i2c LSM6DSL fd: %d\n", LSM6DSL);

	int BM388 = wiringPiI2CSetupInterface(device, BM388_ADDRESS);
	printf("i2c LSM6DSL fd: %d\n", BM388);
	// delay(100);

	int accRaw[3];
	uint8_t buff[6];
	float AccXraw; 
    float AccYraw; 
    float AccZraw;

	// printf("i2c read LSM6DSL: %d\n", wiringPiI2CRead(LSM6DSL));
	
	wiringPiI2CWriteReg8(LSM6DSL, LSM6DSL_CTRL1_XL, 0b10011111);
	wiringPiI2CWriteReg8(LSM6DSL, LSM6DSL_CTRL8_XL, 0b11001000);
	wiringPiI2CWriteReg8(LSM6DSL, LSM6DSL_CTRL3_C, 0b01000100);
	wiringPiI2CWriteReg8(LSM6DSL, LSM6DSL_CTRL2_G, 0b10011100);

	wiringPiI2CWriteReg8(LIS3MDL, LIS3MDL_CTRL_REG1, 0b11011100);     // Temp sesnor enabled, High performance, ODR 80 Hz, FAST ODR disabled and Selft test disabled.
	wiringPiI2CWriteReg8(LIS3MDL, LIS3MDL_CTRL_REG2, 0b00100000);     // +/- 8 gauss
	wiringPiI2CWriteReg8(LIS3MDL, LIS3MDL_CTRL_REG3, 0b00000000); 

	wiringPiI2CWriteReg8(BM388,PWR_CTRL, 0b00110011);     // Enables pressure sensor, Enables temperature sensor, Normal mode
	wiringPiI2CWriteReg8(BM388,OSR, 0b00001100);                 // x16 oversampling pressure measurement, x2 oversampling temp measurement
	wiringPiI2CWriteReg8(BM388,ODR, 0x03);                 // Output data rate 25Hz
	wiringPiI2CWriteReg8(BM388,CONFIG, 0b00000110);      // IIR filter coefficient of 63 

	while(true) {
		int out = wiringPiI2CReadRegBlock(LSM6DSL, LSM6DSL_OUT_X_L_XL, buff); 
		accRaw[0] = (int)(buff[0] | (buff[1] << 8));
  		accRaw[1] = (int)(buff[2] | (buff[3] << 8));
  		accRaw[2] = (int)(buff[4] | (buff[5] << 8));

		if (accRaw[0] >= 32768) accRaw[0] = accRaw[0] - 65536;
		if (accRaw[1] >= 32768) accRaw[1] = accRaw[1] - 65536;
		if (accRaw[2] >= 32768) accRaw[2] = accRaw[2] - 65536;

		AccYraw = (accRaw[0] * 0.244) / 1000.0;
		AccXraw = -(accRaw[1] * 0.244) / 1000.0;
		AccZraw = (accRaw[2] * 0.244) / 1000.0;

		printf("RAW: %d\n", out);
		printf("Test: %d\n", buff[0]);
		printf("AccYraw: %f\n", AccYraw);
		printf("AccXraw: %f\n", AccXraw);
		printf("AccZraw: %f\n", AccZraw);


		delay(100);
	}

	



}
