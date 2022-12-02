#ifndef NOTCH_FILTER_H
#define NOTCH_FILTER_H

#include "math.h"

typedef struct
{
	float alpha;
	float beta;
	float x[3];
	float y[3];
} NotchFilter_t;

void NotchFilterInit(NotchFilter_t *filt, float centerFreHz, float notchWidth_Hz, float sampleTime_s);

float NotchFilter_Update(NotchFilter_t *filt, float data);

#endif

