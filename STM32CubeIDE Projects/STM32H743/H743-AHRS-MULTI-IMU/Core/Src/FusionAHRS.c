/*
 * FusionAHRS.c
 *
 *  Created on: Mar 20, 2022
 *      Author: Ibrahim Ozdemir
 *      GitHub: ibrahimcahit
 */

#include "FusionAHRS.h"

FusionVector3 gyroscopeSensitivity = {
	.axis.x = 1.0f,
	.axis.y = 1.0f,
	.axis.z = 1.0f,
};

FusionVector3 accelerometerSensitivity = {
	.axis.x = 1.0f,
	.axis.y = 1.0f,
	.axis.z = 1.0f,
};

FusionVector3 hardIronBias = {
	.axis.x = 0.0f,
	.axis.y = 0.0f,
	.axis.z = 0.0f,
};

//


void initFusionAHRS(FusionBias*fusionBias, FusionAhrs*fusionAhrs, FusionAHRS_t*DataStruct, float sample_S){

	DataStruct->samplePeriod = sample_S;
	
	// Initialise gyroscope bias correction algorithm
    FusionBiasInitialise(fusionBias, 0.5f, DataStruct->samplePeriod); // stationary threshold = 0.5 degrees per second

    // Initialise AHRS algorithm
    FusionAhrsInitialise(fusionAhrs, 0.5f); // gain = 0.5

    // Set optional magnetic field limits
    FusionAhrsSetMagneticField(fusionAhrs, 20.0f, 70.0f); // valid magnetic field range = 20 uT to 70 uT
}

void getFusionAHRS_9DoF(FusionBias*fusionBias, FusionAhrs*fusionAhrs, FusionAHRS_t*DataStruct, float accX, float accY, float accZ, float gyrX, float gyrY, float gyrZ, float magX, float magY, float magZ) {
	
	// Calibrate gyroscope
	FusionVector3 uncalibratedGyroscope = {
		.axis.x = gyrX, 
		.axis.y = gyrY, 
		.axis.z = gyrZ, 
	};
	FusionVector3 calibratedGyroscope = FusionCalibrationInertial(uncalibratedGyroscope, FUSION_ROTATION_MATRIX_IDENTITY, gyroscopeSensitivity, FUSION_VECTOR3_ZERO);

	// Calibrate accelerometer
	FusionVector3 uncalibratedAccelerometer = {
		.axis.x = accX, 
		.axis.y = accY, 
		.axis.z = accZ, 
	};
	FusionVector3 calibratedAccelerometer = FusionCalibrationInertial(uncalibratedAccelerometer, FUSION_ROTATION_MATRIX_IDENTITY, accelerometerSensitivity, FUSION_VECTOR3_ZERO);

	// Calibrate magnetometer
	FusionVector3 uncalibratedMagnetometer = {
		.axis.x = magX, 
		.axis.y = magY, 
		.axis.z = magZ, 
	};
	FusionVector3 calibratedMagnetometer = FusionCalibrationMagnetic(uncalibratedMagnetometer, FUSION_ROTATION_MATRIX_IDENTITY, hardIronBias);

	// Update gyroscope bias correction algorithm
	calibratedGyroscope = FusionBiasUpdate(fusionBias, calibratedGyroscope);

	// Update AHRS algorithm
	FusionAhrsUpdate(fusionAhrs, calibratedGyroscope, calibratedAccelerometer, calibratedMagnetometer, DataStruct->samplePeriod);

	// Get Euler angles
	FusionEulerAngles eulerAngles = FusionQuaternionToEulerAngles(FusionAhrsGetQuaternion(fusionAhrs));
	
	// Write Euler Angles into data structs
	DataStruct->YAW = eulerAngles.angle.yaw;
	DataStruct->PITCH = eulerAngles.angle.pitch;
	DataStruct->ROLL = eulerAngles.angle.roll;
}

void getFusionAHRS_6DoF(FusionBias*fusionBias, FusionAhrs*fusionAhrs, FusionAHRS_t*DataStruct, float accX, float accY, float accZ, float gyrX, float gyrY, float gyrZ) {
	
	// Calibrate gyroscope
	FusionVector3 uncalibratedGyroscope = {
		.axis.x = gyrX, /* replace this value with actual gyroscope x axis measurement in lsb */
		.axis.y = gyrY, /* replace this value with actual gyroscope y axis measurement in lsb */
		.axis.z = gyrZ, /* replace this value with actual gyroscope z axis measurement in lsb */
	};
	FusionVector3 calibratedGyroscope = FusionCalibrationInertial(uncalibratedGyroscope, FUSION_ROTATION_MATRIX_IDENTITY, gyroscopeSensitivity, FUSION_VECTOR3_ZERO);

	// Calibrate accelerometer
	FusionVector3 uncalibratedAccelerometer = {
		.axis.x = accX, /* replace this value with actual accelerometer x axis measurement in lsb */
		.axis.y = accY, /* replace this value with actual accelerometer y axis measurement in lsb */
		.axis.z = accZ, /* replace this value with actual accelerometer z axis measurement in lsb */
	};
	FusionVector3 calibratedAccelerometer = FusionCalibrationInertial(uncalibratedAccelerometer, FUSION_ROTATION_MATRIX_IDENTITY, accelerometerSensitivity, FUSION_VECTOR3_ZERO);

	// Update gyroscope bias correction algorithm
	calibratedGyroscope = FusionBiasUpdate(fusionBias, calibratedGyroscope);

	// Update AHRS algorithm
	FusionAhrsUpdateWithoutMagnetometer(fusionAhrs, calibratedGyroscope, calibratedAccelerometer, DataStruct->samplePeriod);

	// Get Euler angles
	FusionEulerAngles eulerAngles = FusionQuaternionToEulerAngles(FusionAhrsGetQuaternion(fusionAhrs));
	
	// Write Euler Angles into data structs
	DataStruct->YAW = eulerAngles.angle.yaw;
	DataStruct->PITCH = eulerAngles.angle.pitch;
	DataStruct->ROLL = eulerAngles.angle.roll;
}
