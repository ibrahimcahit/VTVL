# Digitial Low Pass and Notch Filter for 6Dof IMU sensors

## Keynotes
* 2nd order Two Pole Low Pass filter. Can be use with bot accelerometer and gyro
* 2nd order Notch Filter. Usefull for gyro when there are specifics vibrations present in system. Like motor blade vibrations
* For accelerometer, only Low Pass filter will be sufficent.
* For gyro, first Notch Filter and then Low Pass filter will be ideal, since Notch Filter is for specific frequencies so more raw data will result more accurate filtering

## How to use it? - Low Pass Filter

### 1) Import Filter

```
#include "LowPassFilter.h"
```

### 2) Create instance for each axis

```
LPFTwoPole_t LPF_accel_x, LPF_accel_y, LPF_accel_z;
```

### 3) Define filter coefficients

```
#define 	SAMPLE_FREQ_HZ 		1000.0f	// Data sample frequency in Hz
#define 	LPF_CTOFF_FREQ_HZ    256.0f	// LPF Cut-Off frequency
```

### 4) Get sample time in seconds

```
float sample_time_sec_f32 = 1.0f / SAMPLE_FREQ_HZ;
```

### 5) Create data array for filtered values

```
float accelLowPassFiltered_f32[3]
```

### 6) Init filter with predefined settings

```
LPFTwoPole_Init(&LPF_accel_x, LPF_TYPE_BESSEL, LPF_CTOFF_FREQ_HZ, sample_time_sec_f32);
LPFTwoPole_Init(&LPF_accel_y, LPF_TYPE_BESSEL, LPF_CTOFF_FREQ_HZ, sample_time_sec_f32);
LPFTwoPole_Init(&LPF_accel_z, LPF_TYPE_BESSEL, LPF_CTOFF_FREQ_HZ, sample_time_sec_f32);
```

###  7) In your while loop, call "LPFTwoPole_Update" to filter your measurments

```
accelLowPassFiltered_f32[0] = (LPFTwoPole_Update(&LPF_accel_x, <SENSOR_READING_X_AXIS>));
accelLowPassFiltered_f32[1] = (LPFTwoPole_Update(&LPF_accel_y, <SENSOR_READING_Y_AXIS>));
accelLowPassFiltered_f32[2] = (LPFTwoPole_Update(&LPF_accel_z, <SENSOR_READING_Z_AXIS>));
```

## How to use it? - Notch Filter

### 1) Import Filter

```
#include "NotchFilter.h"
```

### 2) Create instance for each axis

```
NotchFilter_t NF_gyro_x, NF_gyro_y, NF_gyro_z;
```

### 3) Define filter coefficients

```
#define 	SAMPLE_FREQ_HZ 		1000.0f	// Data sample frequency in Hz
#define 	NF_CENTER_FREQ_HZ	74.0f	// NF center frequency 
#define 	NF_NOTCH_WDTH_HZ	5.0f	// NF notch-width frequency 
```

### 4) Get sample time in seconds

```
float sample_time_sec_f32 = 1.0f / SAMPLE_FREQ_HZ;
```

### 5) Create data array for filtered values

```
float gyroLowPassFiltered_f32[3]
```

### 6) Init filter with predefined settings

```
NotchFilterInit(&NF_gyro_x, NF_CENTER_FREQ_HZ, NF_NOTCH_WDTH_HZ, sample_time_sec_f32);
NotchFilterInit(&NF_gyro_y, NF_CENTER_FREQ_HZ, NF_NOTCH_WDTH_HZ, sample_time_sec_f32);
NotchFilterInit(&NF_gyro_z, NF_CENTER_FREQ_HZ, NF_NOTCH_WDTH_HZ, sample_time_sec_f32);
```

### 7) In your while loop, call "LPFTwoPole_Update" to filter your measurments

```
gyroNotchFiltered_f32[0] = NotchFilter_Update(&NF_gyro_x, <SENSOR_READING_X_AXIS>);
gyroNotchFiltered_f32[1] = NotchFilter_Update(&NF_gyro_y, <SENSOR_READING_Y_AXIS>);
gyroNotchFiltered_f32[2] = NotchFilter_Update(&NF_gyro_z, <SENSOR_READING_Z_AXIS>);
```

## Disclaimer 

Although such filters are ideal for vehicles with propellers such as drones or vehicles subject to high accelerations such as rockets, serious testing is required for them to work efficiently. The coeficcient values given in these examples are purely for illustrative purposes, and the ideal coefficients must be found by performing the necessary tests for each system.