
#ifndef MAIN_SENSOR_H_
#define MAIN_SENSOR_H_



//Function for global periodic sensor reading
void read_bmi270(void* pvParameter);
void read_bmp280(void* pvParameter);
void read_imu(void* pvParameter);

#endif /* MAIN_SENSOR_H_ */
