#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include "esp_system.h"
#include "unity.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bmi2.h"
#include "bmp280.h"
#include "sensor.h"


#define I2C_MASTER_SCL_IO 22      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

#define bmi2_I2C_ADDRESS         	0x68u /*!< I2C address of bmi270*/
#define bmp280_I2C_ADDRESS         	0x76u /*!< I2C address of bmp280*/

static const char* TAG = "imu";

static bmi2_handle_t bmi2 = NULL;
static bmp280_handle_t bmp280 = NULL;

static void i2c_bus_init(void)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    ESP_ERROR_CHECK(ret);
//    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C config returned error");

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    ESP_ERROR_CHECK(ret);
//    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C install returned error");

}

static void sensor_init(void)
{
    esp_err_t ret;

    i2c_bus_init();
    bmi2 = bmi2_create(I2C_MASTER_NUM, bmi2_I2C_ADDRESS);
    TEST_ASSERT_NOT_NULL_MESSAGE(bmi2, "bmi2 create returned NULL");

    ret = bmi2_begin(bmi2);
    ESP_ERROR_CHECK(ret);



    bmp280 = bmp280_create(I2C_MASTER_NUM, bmp280_I2C_ADDRESS);
    TEST_ASSERT_NOT_NULL_MESSAGE(bmp280, "bmi2 create returned NULL");

    ret = bmp280_begin(bmp280);
    ESP_ERROR_CHECK(ret);

}


/*
 * Modify the output:
 * 		BMI270 -> inclination, acce_avg
 * 		BMP280 -> temp, pressure
 * 		as you wish
 */
void read_imu(void* pvParameter)
{
	esp_err_t ret;
	sensor_init();

	//Variable for bmp280
	float temp, pressure;

	//Variable for bmi270
	bmi2_acce_value_t acce[3];
	int number_of_sample = 3;
	float acce_avg;
	float inclination;


	while(1)
	{
		//Process for bmp280
		ret = bmp280_read_float(bmp280, &temp, &pressure);
		ESP_ERROR_CHECK(ret);
//		printf("temp: %.2f press %.2f\n\n", temp, pressure);
		ESP_LOGI(TAG, "Temperature %.2f \t Pressure %.2f", temp, pressure);



		//Process for bmi270
		get_data_to_array:for(int i=0;i<number_of_sample;i++)
		{
			ret = bmi2_get_acce(bmi2, acce+i);
			ESP_ERROR_CHECK(ret);
			vTaskDelay(pdMS_TO_TICKS(5));				//the delay depend on sample rate, here 1600Hz -> minimum delay 0.625ms
		}
		if(averaging_acce(acce,number_of_sample,&acce_avg) == false) goto get_data_to_array;

//		printf("acce_x:%.5f, acce_y:%.5f, acce_z:%.5f\n", acce.acce_x, acce.acce_y, acce.acce_z);

		inclination = bmi2_get_inclination(acce_avg);
//		printf("DO nghieng: %.2f*\n\n", inclination);
		ESP_LOGI(TAG, "The incliation: %.2f degree", inclination);


		vTaskDelay(pdMS_TO_TICKS(10000));
	}


	bmi2_delete(bmi2);
	bmp280_delete(bmp280);

	ret = i2c_driver_delete(I2C_MASTER_NUM);
	ESP_ERROR_CHECK(ret);
	vTaskDelete(NULL);
}





//Read only bmp280, for testing purpose
void read_bmp280(void* pvParameter)
{
	esp_err_t ret;

	sensor_init();
	float temp, pressure;
	while(1)
	{
		ret = bmp280_read_float(bmp280, &temp, &pressure);
		ESP_ERROR_CHECK(ret);
		printf("temp: %.2f press %.2f\n\n", temp, pressure);


		vTaskDelay(pdMS_TO_TICKS(10000));
	}
	ret = i2c_driver_delete(I2C_MASTER_NUM);
	ESP_ERROR_CHECK(ret);
	bmi2_delete(bmi2);
	bmp280_delete(bmp280);
	vTaskDelete(NULL);
}


//Read only bmi270, for testing purpose
void read_bmi270(void* pvParameter)
{
	esp_err_t ret;
	bmi2_acce_value_t acce[5];
	float acce_avg;
	float inclination;
	int number_of_sample = 5;
	//init
	sensor_init();

	while(1)
	{
		get_data_to_array:for(int i=0;i<number_of_sample;i++)
		{
			ret = bmi2_get_acce(bmi2, acce+i);
//			assert(ESP_OK == ret);
			ESP_ERROR_CHECK(ret);
			vTaskDelay(pdMS_TO_TICKS(5));				//the delay depend on sample rate, here 1600Hz -> minimum delay 0.625ms
		}
		if(averaging_acce(acce,number_of_sample,&acce_avg) == false) goto get_data_to_array;

//		printf("acce_x:%.5f, acce_y:%.5f, acce_z:%.5f\n", acce.acce_x, acce.acce_y, acce.acce_z);

		inclination = bmi2_get_inclination(acce_avg);
		printf("DO nghieng: %.2f*\n\n", inclination);


		vTaskDelay(pdMS_TO_TICKS(10000));
	}

	bmi2_delete(bmi2);

	ret = i2c_driver_delete(I2C_MASTER_NUM);
//	TEST_ASSERT_EQUAL(ESP_OK, ret);
	ESP_ERROR_CHECK(ret);
	vTaskDelete(NULL);
}
