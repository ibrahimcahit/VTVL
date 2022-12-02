/*
 * mpu6050.c
 *
 *  Created on: Mar 18, 2022
 *      Author: Ibrahim Ozdemir
 *      GitHub: ibrahimcahit   
 */

//Includes
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "mpu6050.h"

// Set initial input parameters
enum Ascale_MPU6050 {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale_MPU6050 {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

//Specify sensor full scale
uint8_t Gscale_MPU6050 = GFS_2000DPS;
uint8_t Ascale_MPU6050 = AFS_8G;

int16_t accelRaw_MPU6050[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroRaw_MPU6050[3];   // Stores the 16-bit signed gyro sensor output

float aRes_MPU6050, gRes_MPU6050;      // scale resolutions per LSB for the sensors
float gyroBias_MPU6050[3] = {0, 0, 0}, accelBias_MPU6050[3] = {0, 0, 0};      // Bias corrections for gyro and accelerometer
float SelfTest_MPU6050[12];    // holds results of gyro and accelerometer self test

// Main Init function
uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx, MPU6050_t*DataStruct){

	// Init user variables
	DataStruct->CALIBRATIN_OK_u8 = 0;

	uint8_t readData;

	//read MPU6050 WHOAMI
	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDRESS, WHO_AM_I_MPU6050, 1, &readData, 1, i2c_timeout);

	if (readData == 104) {

		//Start by performing self test and reporting values
		MPU6050SelfTest(I2Cx, SelfTest_MPU6050);

		//Calibrate gyro and accelerometers, load biases in bias registers
		calibrateMPU6050(I2Cx, DataStruct, gyroBias_MPU6050, accelBias_MPU6050);
		HAL_Delay(1000);

		//init Gyro and Accelerometer
		initMPU6050(I2Cx);
		HAL_Delay(1000);

		getMPU6050Ares();
		getMPU6050Gres();

		HAL_Delay(100);

		return 0;
	}
	return 1; // Loop forever if communication doesn't happen
}

// Data read function
void readMPU6050(I2C_HandleTypeDef *I2Cx, MPU6050_t*DataStruct) {

	uint8_t Data;
	// If intPin goes high, all data registers have new data
	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDRESS, INT_STATUS, 1, &Data, 1, i2c_timeout);

	if (Data & 0x01) {  // On interrupt, check if data ready interrupt

		readMPU6050AccelData(I2Cx, accelRaw_MPU6050);  // Read the accelerometer x/y/z adc values
		readMPU6050GyroData(I2Cx, gyroRaw_MPU6050);  // Read the gyro x/y/z adc values

		// Now we'll calculate the accleration value into actual g's
		DataStruct->MPU6050_Accel_f32[0] = (float)accelRaw_MPU6050[0]*aRes_MPU6050; //- accelBias_MPU6050[0];  // get actual g value, this depends on scale being set
		DataStruct->MPU6050_Accel_f32[1] = (float)accelRaw_MPU6050[1]*aRes_MPU6050; //- accelBias_MPU6050[1];
		DataStruct->MPU6050_Accel_f32[2] = (float)accelRaw_MPU6050[2]*aRes_MPU6050; //- accelBias_MPU6050[2];

		// Calculate the gyro value into actual degrees per second
		DataStruct->MPU6050_Gyro_f32[0] = (float)gyroRaw_MPU6050[0]*gRes_MPU6050; //- gyroBias_MPU6050[0]; // get actual gyro value, this depends on scale being set
		DataStruct->MPU6050_Gyro_f32[1] = (float)gyroRaw_MPU6050[1]*gRes_MPU6050; //- gyroBias_MPU6050[1];
		DataStruct->MPU6050_Gyro_f32[2] = (float)gyroRaw_MPU6050[2]*gRes_MPU6050; //- gyroBias_MPU6050[2];
	}
}

// Accelerometer resolution scale calculator function
void getMPU6050Ares() {
  switch (Ascale_MPU6050)
  {
  // Possible accelerometer scales (and their register bit settings) are:
  // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          aRes_MPU6050 = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes_MPU6050 = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes_MPU6050 = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes_MPU6050 = 16.0/32768.0;
          break;
  }
}

// Gyro resolution scale calculator function
void getMPU6050Gres() {
  switch (Gscale_MPU6050)
  {
  // Possible gyro scales (and their register bit settings) are:
  // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
          gRes_MPU6050 = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes_MPU6050 = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes_MPU6050 = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes_MPU6050 = 2000.0/32768.0;
          break;
  }
}

void initMPU6050(I2C_HandleTypeDef *I2Cx){
	//pre def. vars
	uint8_t readData;
	uint8_t writeData;

	//Wake up device
	writeData = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, PWR_MGMT_1, 1, &writeData, 1, i2c_timeout);
	HAL_Delay(100);

	writeData = 0x01;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, PWR_MGMT_1, 1, &writeData, 1, i2c_timeout);
	HAL_Delay(100);

	writeData = 0x03;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, CONFIG, 1, &writeData, 1, i2c_timeout);
	HAL_Delay(100);

	writeData = 0; //0x07; 0x04
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, SMPLRT_DIV, 1, &writeData, 1, i2c_timeout);
	HAL_Delay(100);

	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDRESS, GYRO_CONFIG, 1, &readData, 1, i2c_timeout);
	readData = readData & ~0x03; // Clear Fchoice bits [1:0]
	readData = readData & ~0x18; // Clear GFS bits [4:3]
	readData = readData | Gscale_MPU6050 << 3; // Set full scale range for the gyro
	HAL_Delay(100);

	writeData = readData;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, GYRO_CONFIG, 1, &writeData, 1, i2c_timeout);
	HAL_Delay(100);

	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDRESS, ACCEL_CONFIG, 1, &readData, 1, i2c_timeout);
	readData = readData & ~0x18;  // Clear AFS bits [4:3]
	readData = readData | Ascale_MPU6050 << 3; // Set full scale range for the accelerometer

	writeData = readData;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, ACCEL_CONFIG, 1, &writeData, 1, i2c_timeout);
	HAL_Delay(100);
	//**
	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDRESS, ACCEL_CONFIG2, 1, &readData, 1, i2c_timeout);
	readData = readData & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	readData = readData | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

	writeData = readData;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, ACCEL_CONFIG2, 1, &writeData, 1, i2c_timeout);
	HAL_Delay(100);
}

//read raw Accelerometer values from registers
void readMPU6050AccelData(I2C_HandleTypeDef *I2Cx, int16_t * destination){
  uint8_t rawAccelData[6];  // x/y/z accel register data stored here
  HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDRESS, ACCEL_XOUT_H, 1, &rawAccelData[0], 6, i2c_timeout); // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawAccelData[0] << 8) | rawAccelData[1];  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawAccelData[2] << 8) | rawAccelData[3];
  destination[2] = ((int16_t)rawAccelData[4] << 8) | rawAccelData[5];
}

//read raw Gyro values from registers
void readMPU6050GyroData(I2C_HandleTypeDef *I2Cx, int16_t * destination){
  uint8_t rawGyroData[6];  // x/y/z gyro register data stored here
  HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDRESS, GYRO_XOUT_H, 1, &rawGyroData[0], 6, i2c_timeout);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawGyroData[0] << 8) | rawGyroData[1];  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawGyroData[2] << 8) | rawGyroData[3];
  destination[2] = ((int16_t)rawGyroData[4] << 8) | rawGyroData[5];
}


// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU6050(I2C_HandleTypeDef *I2Cx, MPU6050_t*DataStruct, float * dest1, float * dest2){
  //pre def. vars
  uint8_t writeData;

	uint8_t calibData[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

	// reset device
	writeData = 0x80;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, PWR_MGMT_1, 1, &writeData, 1, i2c_timeout);// Write a one to bit 7 reset bit; toggle reset device
	HAL_Delay(100);

	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready
	// else use the internal oscillator, bits 2:0 = 001
	writeData = 0x01;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, PWR_MGMT_1, 1, &writeData, 1, i2c_timeout);
	writeData = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, PWR_MGMT_2, 1, &writeData, 1, i2c_timeout);
	HAL_Delay(200);

	// Configure device for bias calculation
	writeData = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, INT_ENABLE, 1, &writeData, 1, i2c_timeout);// Disable all interrupts
	writeData = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, FIFO_EN, 1, &writeData, 1, i2c_timeout);// Disable FIFO
	writeData = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, PWR_MGMT_1, 1, &writeData, 1, i2c_timeout);// Turn on internal clock source
	writeData = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, I2C_MST_CTRL, 1, &writeData, 1, i2c_timeout);// Disable I2C master
	writeData = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, USER_CTRL, 1, &writeData, 1, i2c_timeout);// Disable FIFO and I2C master modes
	writeData = 0x0C;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, USER_CTRL, 1, &writeData, 1, i2c_timeout);// Reset FIFO and DMP
	HAL_Delay(15);

	// Configure MPU6050 gyro and accelerometer for bias calculation
	writeData = 0x01;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, CONFIG, 1, &writeData, 1, i2c_timeout);// Set low-pass filter to 188 Hz
	writeData = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, SMPLRT_DIV, 1, &writeData, 1, i2c_timeout);// Set sample rate to 1 kHz
	writeData = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, GYRO_CONFIG, 1, &writeData, 1, i2c_timeout);// Set gyro full-scale to 250 degrees per second, maximum sensitivity
	writeData = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, ACCEL_CONFIG, 1, &writeData, 1, i2c_timeout);// Set accelerometer full-scale to 2 g, maximum sensitivity

	uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	writeData = 0x40;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, USER_CTRL, 1, &writeData, 1, i2c_timeout);// Enable FIFO
	writeData = 0x78;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, FIFO_EN, 1, &writeData, 1, i2c_timeout);// Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	HAL_Delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	writeData = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, FIFO_EN, 1, &writeData, 1, i2c_timeout);// Disable gyro and accelerometer sensors for FIFO
	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDRESS, FIFO_COUNTH, 1, &calibData[0], 2, i2c_timeout);// read FIFO sample count
	fifo_count = ((uint16_t)calibData[0] << 8) | calibData[1];
	packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDRESS, FIFO_R_W, 1, &calibData[0], 12, i2c_timeout);

		//Form signed 16-bit integer for each sample in FIFO
		accel_temp[0] = (int16_t) (((int16_t)calibData[0] << 8) | calibData[1]  ) ;
		accel_temp[1] = (int16_t) (((int16_t)calibData[2] << 8) | calibData[3]  ) ;
		accel_temp[2] = (int16_t) (((int16_t)calibData[4] << 8) | calibData[5]  ) ;
		gyro_temp[0]  = (int16_t) (((int16_t)calibData[6] << 8) | calibData[7]  ) ;
		gyro_temp[1]  = (int16_t) (((int16_t)calibData[8] << 8) | calibData[9]  ) ;
		gyro_temp[2]  = (int16_t) (((int16_t)calibData[10] << 8) | calibData[11]) ;

		//Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[0] += (int32_t) accel_temp[0];
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];
	}

	//Normalize sums to get average count biases
	accel_bias[0] /= (int32_t) packet_count;
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0]  /= (int32_t) packet_count;
	gyro_bias[1]  /= (int32_t) packet_count;
	gyro_bias[2]  /= (int32_t) packet_count;

	//Remove gravity from the z-axis accelerometer bias calculation
	if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}
	else {accel_bias[2] += (int32_t) accelsensitivity;}

	//Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	calibData[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	calibData[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	calibData[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	calibData[3] = (-gyro_bias[1]/4)       & 0xFF;
	calibData[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	calibData[5] = (-gyro_bias[2]/4)       & 0xFF;

	//Push gyro biases to hardware registers
	writeData = calibData[0];
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, XG_OFFSET_H, 1, &writeData, 1, i2c_timeout);
	writeData = calibData[1];
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, XG_OFFSET_L, 1, &writeData, 1, i2c_timeout);
	writeData = calibData[2];
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, YG_OFFSET_H, 1, &writeData, 1, i2c_timeout);
	writeData = calibData[3];
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, YG_OFFSET_L, 1, &writeData, 1, i2c_timeout);
	writeData = calibData[4];
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, ZG_OFFSET_H, 1, &writeData, 1, i2c_timeout);
	writeData = calibData[5];
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, ZG_OFFSET_L, 1, &writeData, 1, i2c_timeout);

	//Output scaled gyro biases for display in the main program
	dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
	dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
	dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

	//Construct the accelerometer biases for push to the hardware accelerometer bias registers.
	int32_t accel_bias_reg[3] = {0, 0, 0}; //A place to hold the factory accelerometer trim biases
	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDRESS, XA_OFFSET_H, 1, &calibData[0], 2, i2c_timeout); //Read factory accelerometer trim values
	accel_bias_reg[0] = (int32_t) (((int16_t)calibData[0] << 8) | calibData[1]);
	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDRESS, YA_OFFSET_H, 1, &calibData[0], 2, i2c_timeout);
	accel_bias_reg[1] = (int32_t) (((int16_t)calibData[0] << 8) | calibData[1]);
	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDRESS, ZA_OFFSET_H, 1, &calibData[0], 2, i2c_timeout);
	accel_bias_reg[2] = (int32_t) (((int16_t)calibData[0] << 8) | calibData[1]);

	//Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint32_t mask = 1uL;
	//Define array to hold mask bit for each accelerometer bias axis
	uint8_t mask_bit[3] = {0, 0, 0};

	for(ii = 0; ii < 3; ii++) {
		//If temperature compensation bit is set, record that fact in mask_bit
		if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01;
	}

	//Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0]/8); //Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1]/8);
	accel_bias_reg[2] -= (accel_bias[2]/8);

	calibData[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	calibData[1] = (accel_bias_reg[0])      & 0xFF;
	calibData[1] = calibData[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	calibData[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	calibData[3] = (accel_bias_reg[1])      & 0xFF;
	calibData[3] = calibData[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	calibData[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	calibData[5] = (accel_bias_reg[2])      & 0xFF;
	calibData[5] = calibData[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

	//Push accelerometer biases to hardware registers
	writeData = calibData[0];
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, XA_OFFSET_H, 1, &writeData, 1, i2c_timeout);
	writeData = calibData[1];
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, XA_OFFSET_L, 1, &writeData, 1, i2c_timeout);
	writeData = calibData[2];
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, YA_OFFSET_H, 1, &writeData, 1, i2c_timeout);
	writeData = calibData[3];
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, YA_OFFSET_L, 1, &writeData, 1, i2c_timeout);
	writeData = calibData[4];
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, ZA_OFFSET_H, 1, &writeData, 1, i2c_timeout);
	writeData = calibData[5];
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, ZA_OFFSET_L, 1, &writeData, 1, i2c_timeout);

	//Output scaled gyro biases for display in the main program
	dest2[0] = (float) accel_bias[0]/(float) accelsensitivity;
	dest2[1] = (float) accel_bias[1]/(float) accelsensitivity;
	dest2[2] = (float) accel_bias[2]/(float) accelsensitivity;
	
	DataStruct->CALIBRATIN_OK_u8 = TRUE;
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU6050SelfTest(I2C_HandleTypeDef *I2Cx, float * destination) {
	uint8_t writeData;

	uint8_t rawTestData[6] = {0, 0, 0, 0, 0, 0};
	uint8_t SelfTest_MPU6050[6];
	int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
	float factoryTrim[6];
	uint8_t FS = 0;

	writeData = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, SMPLRT_DIV, 1, &writeData, 1, i2c_timeout);// Set gyro sample rate to 1 kHz
	writeData = 0x02;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, CONFIG, 1, &writeData, 1, i2c_timeout);// Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	writeData = FS<<3;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, GYRO_CONFIG, 1, &writeData, 1, i2c_timeout);// Set full scale range for the gyro to 250 dps
	writeData = 0x02;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, ACCEL_CONFIG2, 1, &writeData, 1, i2c_timeout);// Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	writeData = FS<<3;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, ACCEL_CONFIG, 1, &writeData, 1, i2c_timeout);// Set full scale range for the accelerometer to 2 g

	//get average current values of gyro and acclerometer
	for( int ii = 0; ii < 200; ii++) {

		HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDRESS, ACCEL_XOUT_H, 1, &rawTestData[0], 6, i2c_timeout);// Read the six raw data registers into data array
		aAvg[0] += (int16_t)(((int16_t)rawTestData[0] << 8) | rawTestData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		aAvg[1] += (int16_t)(((int16_t)rawTestData[2] << 8) | rawTestData[3]) ;
		aAvg[2] += (int16_t)(((int16_t)rawTestData[4] << 8) | rawTestData[5]) ;

		HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDRESS, GYRO_XOUT_H, 1, &rawTestData[0], 6, i2c_timeout);// Read the six raw data registers sequentially into data array
		gAvg[0] += (int16_t)(((int16_t)rawTestData[0] << 8) | rawTestData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		gAvg[1] += (int16_t)(((int16_t)rawTestData[2] << 8) | rawTestData[3]) ;
		gAvg[2] += (int16_t)(((int16_t)rawTestData[4] << 8) | rawTestData[5]) ;
	}

	//Get average of 200 values and store as average current readings
	for (int ii =0; ii < 3; ii++) {
		aAvg[ii] /= 200;
		gAvg[ii] /= 200;
	}

	//Configure the accelerometer for self-test
	writeData = 0xE0;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, ACCEL_CONFIG, 1, &writeData, 1, i2c_timeout);// Enable self test on all three axes and set accelerometer range to +/- 2 g
	writeData = 0xE0;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, GYRO_CONFIG, 1, &writeData, 1, i2c_timeout);// Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	HAL_Delay(25);  // Delay a while to let the device stabilize

	//get average self-test values of gyro and acclerometer
	for( int ii = 0; ii < 200; ii++) {

		HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDRESS, ACCEL_XOUT_H, 1, &rawTestData[0], 6, i2c_timeout);// Read the six raw data registers into data array
		aSTAvg[0] += (int16_t)(((int16_t)rawTestData[0] << 8) | rawTestData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		aSTAvg[1] += (int16_t)(((int16_t)rawTestData[2] << 8) | rawTestData[3]) ;
		aSTAvg[2] += (int16_t)(((int16_t)rawTestData[4] << 8) | rawTestData[5]) ;

		HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDRESS, GYRO_XOUT_H, 1, &rawTestData[0], 6, i2c_timeout);// Read the six raw data registers sequentially into data array
		gSTAvg[0] += (int16_t)(((int16_t)rawTestData[0] << 8) | rawTestData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		gSTAvg[1] += (int16_t)(((int16_t)rawTestData[2] << 8) | rawTestData[3]) ;
		gSTAvg[2] += (int16_t)(((int16_t)rawTestData[4] << 8) | rawTestData[5]) ;
	}

	//Get average of 200 values and store as average self-test readings
	for (int ii =0; ii < 3; ii++) {
		aSTAvg[ii] /= 200;
		gSTAvg[ii] /= 200;
	}

	//Configure the gyro and accelerometer for normal operation
	writeData = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, ACCEL_CONFIG, 1, &writeData, 1, i2c_timeout);
	writeData = 0x00;
	HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDRESS, GYRO_CONFIG, 1, &writeData, 1, i2c_timeout);
	HAL_Delay(25);  // Delay a while to let the device stabilize

	//Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDRESS, SELF_TEST_X_ACCEL, 1, &SelfTest_MPU6050[0], 1, i2c_timeout);// X-axis accel self-test results
	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDRESS, SELF_TEST_Y_ACCEL, 1, &SelfTest_MPU6050[1], 1, i2c_timeout);// Y-axis accel self-test results
	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDRESS, SELF_TEST_Z_ACCEL, 1, &SelfTest_MPU6050[2], 1, i2c_timeout);// Z-axis accel self-test results
	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDRESS, SELF_TEST_X_GYRO, 1, &SelfTest_MPU6050[3], 1, i2c_timeout);// X-axis gyro self-test results
	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDRESS, SELF_TEST_Y_GYRO, 1, &SelfTest_MPU6050[4], 1, i2c_timeout);// Y-axis gyro self-test results
	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDRESS, SELF_TEST_Z_GYRO, 1, &SelfTest_MPU6050[5], 1, i2c_timeout);// Z-axis gyro self-test results

	//Retrieve factory self-test value from self-test code reads
	factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)SelfTest_MPU6050[0] - 1.0) )); // FT[Xa] factory trim calculation
	factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)SelfTest_MPU6050[1] - 1.0) )); // FT[Ya] factory trim calculation
	factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)SelfTest_MPU6050[2] - 1.0) )); // FT[Za] factory trim calculation
	factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)SelfTest_MPU6050[3] - 1.0) )); // FT[Xg] factory trim calculation
	factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)SelfTest_MPU6050[4] - 1.0) )); // FT[Yg] factory trim calculation
	factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)SelfTest_MPU6050[5] - 1.0) )); // FT[Zg] factory trim calculation

	//Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	//To get percent, must multiply by 100
	for (int i = 0; i < 3; i++) {
		destination[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.;   // Report percent differences
		destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.; // Report percent differences
	}

   for (int i = 0; i < 3; i++) {
     destination[i+3]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.;   // Report percent differences
     destination[i+6] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.; // Report percent differences
   }
}
