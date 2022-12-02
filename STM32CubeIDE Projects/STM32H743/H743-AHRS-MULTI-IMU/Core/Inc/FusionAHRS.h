/*
 * FusionAHRS.h
 *
 *  Created on: Mar 20, 2022
 *      Author: Ibrahim Ozdemir
 *      GitHub: ibrahimcahit
 */
 
 #include "..\Fusion\Fusion.h"
 
typedef struct
{
    float YAW;
    float PITCH;
    float ROLL;

    float samplePeriod;
} FusionAHRS_t;

void initFusionAHRS(FusionBias*fusionBias, FusionAhrs*fusionAhrs, FusionAHRS_t*DataStruct, float sample_S);
void getFusionAHRS_9DoF(FusionBias*fusionBias, FusionAhrs*fusionAhrs, FusionAHRS_t*DataStruct, float accX, float accY, float accZ, float gyrX, float gyrY, float gyrZ, float magX, float magY, float magZ);
void getFusionAHRS_6DoF(FusionBias*fusionBias, FusionAhrs*fusionAhrs, FusionAHRS_t*DataStruct, float accX, float accY, float accZ, float gyrX, float gyrY, float gyrZ);
