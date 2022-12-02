#ifndef LPF_H
#define LPF_H

#include <stdint.h>
#include "math.h"

#define LPF_TYPE_BESSEL 1

typedef struct {
	float out;
	float buf[2];
	float coeffNum;
	float coeffDen[2];
} LPFTwoPole_t;

void LPFTwoPole_Init(LPFTwoPole_t *lpf, uint8_t type, float cutoffFrequency, float sampleTime);
float LPFTwoPole_Update(LPFTwoPole_t *lpf, float val);

#endif
