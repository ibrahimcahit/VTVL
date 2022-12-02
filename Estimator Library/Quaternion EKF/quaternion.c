#include "quaternion.h"

float a12, a22, a31, a32, a33;

void quaternionInit(Quaternion_t *DataStruct, float sampleTimeMicros)
{
	DataStruct->GyroMeasError = M_PI * (40.0f / 180.0f);		// gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
	DataStruct->beta = sqrt(3.0f / 4.0f) * DataStruct->GyroMeasError;		// compute beta
	DataStruct->GyroMeasDrift = M_PI * (2.0f / 180.0f);		// gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
	DataStruct->zeta = sqrt(3.0f / 4.0f) * DataStruct->GyroMeasDrift;		// compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

	DataStruct->deltat = sampleTimeMicros / 1000000.0f;

	DataStruct->q[0] = 1.0f;
	DataStruct->q[1] = 0.0f;
	DataStruct->q[2] = 0.0f;
	DataStruct->q[3] = 0.0f;
}

void quaternionUpdate(Quaternion_t *DataStruct, float ax, float ay, float az, float gyrox, float gyroy, float gyroz)
{
    float q1 = DataStruct->q[0], q2 = DataStruct->q[1], q3 = DataStruct->q[2], q4 = DataStruct->q[3];         // short name local variable for readability
    float norm;                                               // vector norm
    float f1, f2, f3;                                         // objetive funcyion elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float qDot1, qDot2, qDot3, qDot4;
    float hatDot1, hatDot2, hatDot3, hatDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _halfq1 = 0.5f * q1;
    float _halfq2 = 0.5f * q2;
    float _halfq3 = 0.5f * q3;
    float _halfq4 = 0.5f * q4;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Compute the objective function and Jacobian
    f1 = _2q2 * q4 - _2q1 * q3 - ax;
    f2 = _2q1 * q2 + _2q3 * q4 - ay;
    f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
    J_11or24 = _2q3;
    J_12or23 = _2q4;
    J_13or22 = _2q1;
    J_14or21 = _2q2;
    J_32 = 2.0f * J_14or21;
    J_33 = 2.0f * J_11or24;

    // Compute the gradient (matrix multiplication)
    hatDot1 = J_14or21 * f2 - J_11or24 * f1;
    hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
    hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
    hatDot4 = J_14or21 * f1 + J_11or24 * f2;

    // Normalize the gradient
    norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
    hatDot1 /= norm;
    hatDot2 /= norm;
    hatDot3 /= norm;
    hatDot4 /= norm;

    // Compute the quaternion derivative
    qDot1 = -_halfq2 * gyrox - _halfq3 * gyroy - _halfq4 * gyroz;
    qDot2 =  _halfq1 * gyrox + _halfq3 * gyroz - _halfq4 * gyroy;
    qDot3 =  _halfq1 * gyroy - _halfq2 * gyroz + _halfq4 * gyrox;
    qDot4 =  _halfq1 * gyroz + _halfq2 * gyroy - _halfq3 * gyrox;

    // Compute then integrate estimated quaternion derivative
    q1 += (qDot1 -(DataStruct->beta * hatDot1)) * DataStruct->deltat;
    q2 += (qDot2 -(DataStruct->beta * hatDot2)) * DataStruct->deltat;
    q3 += (qDot3 -(DataStruct->beta * hatDot3)) * DataStruct->deltat;
    q4 += (qDot4 -(DataStruct->beta * hatDot4)) * DataStruct->deltat;

    // Normalize the quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    DataStruct->q[0] = q1 * norm;
    DataStruct->q[1] = q2 * norm;
    DataStruct->q[2] = q3 * norm;
    DataStruct->q[3] = q4 * norm;

    quaternionEulerUpdate(DataStruct);
}

void quaternionEulerUpdate(Quaternion_t *DataStruct)
{
	DataStruct->yaw   = atan2(2.0f * (DataStruct->q[1] * DataStruct->q[2] + DataStruct->q[0] * DataStruct->q[3]),
			DataStruct->q[0] * DataStruct->q[0] + DataStruct->q[1] * DataStruct->q[1] - DataStruct->q[2] * DataStruct->q[2] - DataStruct->q[3] * DataStruct->q[3]);
	DataStruct->pitch = -asin(2.0f * (DataStruct->q[1] * DataStruct->q[3] - DataStruct->q[0] * DataStruct->q[2]));
	DataStruct->roll  = atan2(2.0f * (DataStruct->q[0] * DataStruct->q[1] + DataStruct->q[2] * DataStruct->q[3]),
			DataStruct->q[0] * DataStruct->q[0] - DataStruct->q[1] * DataStruct->q[1] - DataStruct->q[2] * DataStruct->q[2] + DataStruct->q[3] * DataStruct->q[3]);
	DataStruct->pitch *= 180.0f / M_PI;
	DataStruct->yaw   *= 180.0f / M_PI;
	DataStruct->roll  *= 180.0f / M_PI;
}

void quaternionEulerUpdateHeading(Quaternion_t *DataStruct)
{
	// Convert quaternions to Euler angles
	a12 =   2.0f * (DataStruct->q[1] * DataStruct->q[2] + DataStruct->q[0] * DataStruct->q[3]);
	a22 =   DataStruct->q[0] * DataStruct->q[0] + DataStruct->q[1] * DataStruct->q[1] - DataStruct->q[2] * DataStruct->q[2] - DataStruct->q[3] * DataStruct->q[3];
	a31 =   2.0f * (DataStruct->q[0] * DataStruct->q[1] + DataStruct->q[2] * DataStruct->q[3]);
	a32 =   2.0f * (DataStruct->q[1] * DataStruct->q[3] - DataStruct->q[0] * DataStruct->q[2]);
	a33 =   DataStruct->q[0] * DataStruct->q[0] - DataStruct->q[1] * DataStruct->q[1] - DataStruct->q[2] * DataStruct->q[2] + DataStruct->q[3] * DataStruct->q[3];

	DataStruct->pitch = -asinf(a32);
	DataStruct->roll  = atan2f(a31, a33);
	DataStruct->yaw   = atan2f(a12, a22);
	DataStruct->pitch *= 180.0f / M_PI;
	DataStruct->yaw   *= 180.0f / M_PI;
//	yaw   += 5.53f; // Declination

	if(DataStruct->yaw < 0) DataStruct->yaw   += 360.0f; // Ensure yaw stays between 0 and 360
	DataStruct->roll  *= 180.0f / M_PI;
}
