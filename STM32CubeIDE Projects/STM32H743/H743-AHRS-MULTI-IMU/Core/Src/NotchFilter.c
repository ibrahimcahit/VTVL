#include "NotchFilter.h"

void NotchFilterInit(NotchFilter_t *filt, float centerFreHz, float notchWidth_Hz, float sampleTime_s)
{
	float w0_rps = 2.0f * M_PI * centerFreHz;
	float ww_rps = 2.0f * M_PI * notchWidth_Hz;
	
	float w0_pw_rps = (2.0f / sampleTime_s) * tanf(0.5f * w0_rps * sampleTime_s);
	
	filt->alpha = 4.0f + ((w0_pw_rps * w0_pw_rps) * (sampleTime_s * sampleTime_s));
	filt->beta = 2.0f + (ww_rps  * sampleTime_s);
	
	for (int n = 0; n<3; n++)
	{
		filt->x[n] = 0.0f;
		filt->y[n] = 0.0f;
	}
}

float NotchFilter_Update(NotchFilter_t *filt, float data)
{
	filt->x[2] = filt->x[1];
	filt->x[1] = filt->x[0];
	
	filt->y[2] = filt->y[1];
	filt->y[1] = filt->y[0];
	
	filt->x[0] = data;
	
	filt->y[0] = (filt->alpha * filt->x[0] + 2.0f * (filt->alpha - 8.0f) *  filt->x[1] + filt->alpha * filt->x[2]
			   - (2.0f * ((filt->alpha - 8.0f)) * filt->y[1] + (filt->alpha - filt->beta) *filt->y[2]))
			   / (filt->alpha + filt->beta);
			   
	return filt->y[0];
}
