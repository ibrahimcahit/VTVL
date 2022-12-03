# MPU6050 vs BMI160 6DoF IMU comparision

## Test Bench

* STM32H743 MCU at 400 MHz
* Both IMU's Accelerometer scale/sensitivity at 8g
* Both IMU's Gyro scale/sensitivity at 2000 DPs
* Accelerometers measured in g, then put into Low Pass Filter, CutOff at 260 Hz
* Gyros measured in deg/s, first put into Notch Filter (Center Freq 74 Hz, Notch Width 5 Hz), then put into Low Pass Filter, CutOff at 260 Hz
* Sample frequency at 500 Hz

##  Measurments

### MPU6050 Euler Angles
![]()

### BMI160 Euler Angles
![]()

### Yaw
![]()

### Pitch
![]()

### Roll
![]()


