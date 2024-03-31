# IMU PRO (From M5 Stack)

## Focus

1. BMI270: Read only acceleration data -> calculate inclination. Skip gyro
2. BMP280: Read atmosphere pressure and temperature
3. BMM150: Skip

## Usage

In sensor.h header file include 3 function
- read_imu() -> read both BMI270 and BMP280 after a period of time
- read_bmi270() -> only bmi270
- read_bmp280() -> only bmp280

## Note

This project is writing with ESP_IDF, target ESP-32
