#ifndef INC_QUATERNION_H_
#define INC_QUATERNION_H_

#include "math.h"

typedef struct
{
	float GyroMeasError;
	float beta;
	float GyroMeasDrift;
	float zeta;

	float deltat; // integration interval for both filter schemes

	float pitch, yaw, roll;

	float q[4]; // = {1.0f, 0.0f, 0.0f, 0.0f};
} Quaternion_t;

void quaternionInit(Quaternion_t* DataStruct, float sampleTimeMicros);
void quaternionUpdate(Quaternion_t* DataStruct, float ax, float ay, float az, float gyrox, float gyroy, float gyroz);
void quaternionEulerUpdate(Quaternion_t* DataStruct);
void quaternionEulerUpdateHeading(Quaternion_t *DataStruct);
#endif /* INC_QUATERNION_H_ */
