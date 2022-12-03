# MPU6050 vs BMI160 6DoF IMU comparision

## Test Bench

* STM32H743 MCU at 400 MHz
* Both IMU's Accelerometer scale/sensitivity at 8g
* Both IMU's Gyro scale/sensitivity at 2000 DPs
* Accelerometers measured in g, then put into Low Pass Filter, CutOff at 260 Hz
* Gyros measured in deg/s, first put into Notch Filter (Center Freq 74 Hz, Notch Width 5 Hz), then put into Low Pass Filter, CutOff at 260 Hz

## Accelerometer Measurments

### MPU6050, 3 axis Raw vs Filtered
![](https://raw.githubusercontent.com/ibrahimcahit/VTVL/main/MATLAB%20Exports/Filter%20Tests/MPU6050%20vs%20BMI160/Accelerometer%20Raw%20vs%20LPF%20output/MPU6050%20500Hz%20Accelerometer%20Data%20-%20LPF%20at%20260%20Hz.png)

### BMI160, 3 axis Raw vs Filtered
![](https://raw.githubusercontent.com/ibrahimcahit/VTVL/main/MATLAB%20Exports/Filter%20Tests/MPU6050%20vs%20BMI160/Accelerometer%20Raw%20vs%20LPF%20output/BMI160%20500Hz%20Accelerometer%20Data%20-%20LPF%20at%20260%20Hz.png)

### X axis, Raw
![](https://raw.githubusercontent.com/ibrahimcahit/VTVL/main/MATLAB%20Exports/Filter%20Tests/MPU6050%20vs%20BMI160/Accelerometer%20Raw%20vs%20LPF%20output/BMI160%20%26%20MPU6050%20500Hz%20Accelerometer%20X%20Axis%20Data%20-%20RAW.png)

### X axis, Filtered
![](https://raw.githubusercontent.com/ibrahimcahit/VTVL/main/MATLAB%20Exports/Filter%20Tests/MPU6050%20vs%20BMI160/Accelerometer%20Raw%20vs%20LPF%20output/BMI160%20%26%20MPU6050%20500Hz%20Accelerometer%20X%20Axis%20Data%20-%20LPF%20at%20260%20Hz.png)

### Y axis, Raw
![](https://raw.githubusercontent.com/ibrahimcahit/VTVL/main/MATLAB%20Exports/Filter%20Tests/MPU6050%20vs%20BMI160/Accelerometer%20Raw%20vs%20LPF%20output/BMI160%20%26%20MPU6050%20500Hz%20Accelerometer%20Y%20Axis%20Data%20-%20RAW.png)

### Y axis, Filtered
![](https://raw.githubusercontent.com/ibrahimcahit/VTVL/main/MATLAB%20Exports/Filter%20Tests/MPU6050%20vs%20BMI160/Accelerometer%20Raw%20vs%20LPF%20output/BMI160%20%26%20MPU6050%20500Hz%20Accelerometer%20Y%20Axis%20Data%20-%20LPF%20at%20260%20Hz.png)

### Z axis, Raw
![](https://raw.githubusercontent.com/ibrahimcahit/VTVL/main/MATLAB%20Exports/Filter%20Tests/MPU6050%20vs%20BMI160/Accelerometer%20Raw%20vs%20LPF%20output/BMI160%20%26%20MPU6050%20500Hz%20Accelerometer%20Z%20Axis%20Data%20-%20RAW.png)

### Z axis, Filtered
![](https://raw.githubusercontent.com/ibrahimcahit/VTVL/main/MATLAB%20Exports/Filter%20Tests/MPU6050%20vs%20BMI160/Accelerometer%20Raw%20vs%20LPF%20output/BMI160%20%26%20MPU6050%20500Hz%20Accelerometer%20Z%20Axis%20Data%20-%20LPF%20at%20260%20Hz.png)

## Gyro Measurments

### MPU6050, 3 axis Raw vs Filtered
![](https://raw.githubusercontent.com/ibrahimcahit/VTVL/main/MATLAB%20Exports/Filter%20Tests/MPU6050%20vs%20BMI160/Gyro%20Raw%20vs%20NF%20and%20LPF%20output/MPU6050%20500Hz%20Gyro%20Data%20-%20LPF%20at%20265%20Hz.png)

### BMI160, 3 axis Raw vs Filtered
![](https://raw.githubusercontent.com/ibrahimcahit/VTVL/main/MATLAB%20Exports/Filter%20Tests/MPU6050%20vs%20BMI160/Gyro%20Raw%20vs%20NF%20and%20LPF%20output/BMI160%20500Hz%20Gyro%20Data%20-%20NF%20BW%20at%2074%20Hz%2C%20NW%20at%205%20Hz%2C%20LPF%20at%20256%20Hz.png)

### X axis, Raw
![](https://raw.githubusercontent.com/ibrahimcahit/VTVL/main/MATLAB%20Exports/Filter%20Tests/MPU6050%20vs%20BMI160/Gyro%20Raw%20vs%20NF%20and%20LPF%20output/BMI160%20%26%20MPU6050%20500Hz%20Gyro%20X%20Axis%20Data%20-%20RAW.png)

### X axis, Filtered
![](https://raw.githubusercontent.com/ibrahimcahit/VTVL/main/MATLAB%20Exports/Filter%20Tests/MPU6050%20vs%20BMI160/Gyro%20Raw%20vs%20NF%20and%20LPF%20output/BMI160%20%26%20MPU6050%20500Hz%20Gyro%20X%20Axis%20Data%20-%20NF%20BW%20at%2074%20Hz%2C%20NW%20at%205%20Hz%2C%20LPF%20at%20256%20Hz.png)

### Y axis, Raw
![](https://raw.githubusercontent.com/ibrahimcahit/VTVL/main/MATLAB%20Exports/Filter%20Tests/MPU6050%20vs%20BMI160/Gyro%20Raw%20vs%20NF%20and%20LPF%20output/BMI160%20%26%20MPU6050%20500Hz%20Gyro%20Y%20Axis%20Data%20-%20RAW.png)

### Y axis, Filtered
![](https://raw.githubusercontent.com/ibrahimcahit/VTVL/main/MATLAB%20Exports/Filter%20Tests/MPU6050%20vs%20BMI160/Gyro%20Raw%20vs%20NF%20and%20LPF%20output/BMI160%20%26%20MPU6050%20500Hz%20Gyro%20Y%20Axis%20Data%20-%20NF%20BW%20at%2074%20Hz%2C%20NW%20at%205%20Hz%2C%20LPF%20at%20256%20Hz.png)

### Z axis, Raw
![](https://raw.githubusercontent.com/ibrahimcahit/VTVL/main/MATLAB%20Exports/Filter%20Tests/MPU6050%20vs%20BMI160/Gyro%20Raw%20vs%20NF%20and%20LPF%20output/BMI160%20%26%20MPU6050%20500Hz%20Gyro%20Z%20Axis%20Data%20-%20RAW.png)

### Z axis, Filtered
![](https://raw.githubusercontent.com/ibrahimcahit/VTVL/main/MATLAB%20Exports/Filter%20Tests/MPU6050%20vs%20BMI160/Gyro%20Raw%20vs%20NF%20and%20LPF%20output/BMI160%20%26%20MPU6050%20500Hz%20Gyro%20Z%20Axis%20Data%20-%20NF%20BW%20at%2074%20Hz%2C%20NW%20at%205%20Hz%2C%20LPF%20at%20256%20Hz.png)
