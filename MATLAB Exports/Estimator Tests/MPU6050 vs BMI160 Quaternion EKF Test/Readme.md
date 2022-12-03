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
![](https://raw.githubusercontent.com/ibrahimcahit/VTVL/main/MATLAB%20Exports/Estimator%20Tests/MPU6050%20vs%20BMI160%20Quaternion%20EKF%20Test/MPU6050%20Euler%20Angle%20Estimations%20Using%20Quaternions%2C%20at%20500Hz.png)

### BMI160 Euler Angles
![](https://raw.githubusercontent.com/ibrahimcahit/VTVL/main/MATLAB%20Exports/Estimator%20Tests/MPU6050%20vs%20BMI160%20Quaternion%20EKF%20Test/BMI160%20Euler%20Angle%20Estimations%20Using%20Quaternions%2C%20at%20500Hz.png)

### Yaw
![](https://raw.githubusercontent.com/ibrahimcahit/VTVL/main/MATLAB%20Exports/Estimator%20Tests/MPU6050%20vs%20BMI160%20Quaternion%20EKF%20Test/BMI160%20%26%20MPU6050%20Yaw%20Angles.png)

### Pitch
![](https://raw.githubusercontent.com/ibrahimcahit/VTVL/main/MATLAB%20Exports/Estimator%20Tests/MPU6050%20vs%20BMI160%20Quaternion%20EKF%20Test/BMI160%20%26%20MPU6050%20Pitch%20Angles.png)

### Roll
![](https://raw.githubusercontent.com/ibrahimcahit/VTVL/main/MATLAB%20Exports/Estimator%20Tests/MPU6050%20vs%20BMI160%20Quaternion%20EKF%20Test/BMI160%20%26%20MPU6050%20Roll%20Angles.png)


