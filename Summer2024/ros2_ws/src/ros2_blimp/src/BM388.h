//Device adress
#define BM388_ADDRESS       0x77

//Power modes
#define PWR_CTRL            0x1B

//Oversampling setting pressure/temperature
#define OSR                 0x1C

//Output data rates
#define ODR                 0x1D

//IIR filter
#define CONFIG              0x1F

//Calibration data
#define NVM_PAR_T1_LSB      0x31
#define NVM_PAR_T1_MSB      0x32
#define NVM_PAR_T2_LSB      0x33
#define NVM_PAR_T2_MSB      0x34
#define NVM_PAR_T3          0x35
#define NVM_PAR_P1_LSB      0x36
#define NVM_PAR_P1_MSB      0x37
#define NVM_PAR_P2_LSB      0x38
#define NVM_PAR_P2_MSB      0x39
#define NVM_PAR_P3          0x3A
#define NVM_PAR_P4          0x3B
#define NVM_PAR_P5_LSB      0x3C
#define NVM_PAR_P5_MSB      0x3D
#define NVM_PAR_P6_LSB      0x3E
#define NVM_PAR_P6_MSB      0x3F
#define NVM_PAR_P7          0x40
#define NVM_PAR_P8          0x41
#define NVM_PAR_P9_LSB      0x42
#define NVM_PAR_P9_MSB      0x43
#define NVM_PAR_P10         0x44
#define NVM_PAR_P11         0x45

//Pressure data
#define PRESS_XLSB_7_0      0x04
#define PRESS_LSB_15_8      0x05
#define PRESS_MSB_23_16     0x06

//Temperature data
#define TEMP_XLSB_7_0       0x07
#define TEMP_LSB_8_15       0x08
#define TEMP_MSB_23_16      0x09