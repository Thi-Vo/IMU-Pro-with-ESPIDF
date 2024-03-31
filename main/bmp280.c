#include "bmp280.h"
#include <inttypes.h>
#include "stdint.h"
#include <esp_log.h>

#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "esp_system.h"
#include "driver/i2c.h"
#include <rom/ets_sys.h>
#include "esp_err.h"


static const char *TAG = "bmp280";

/**
 * BMP280 registers address
 */
#define BMP280_I2C_ADDRESS  	0x76 //!< I2C address
#define BMP280_CHIP_ID  		0x58 //!< BMP280 has chip-id 0x58
#define BMP280_REG_ID          	0xD0

#define BMP280_REG_CONFIG      	0xF5 /* bits: 7-5 t_sb; 4-2 filter; 0 spi3w_en */
#define BMP280_REG_CTRL        	0xF4 /* bits: 7-5 osrs_t; 4-2 osrs_p; 1-0 mode */
#define BMP280_REG_STATUS      	0xF3 /* bits: 3 measuring; 0 im_update */
#define BMP280_REG_CTRL_HUM    	0xF2 /* bits: 2-0 osrs_h; */
#define BMP280_REG_RESET       	0xE0
#define BMP280_REG_CALIBRATION       0x88	//!< Calibration offset, 24 bit long


bmp280_handle_t bmp280_create(i2c_port_t port, const uint16_t sensor_addr)
{
    bmp280_dev_t *sensor = (bmp280_dev_t *) calloc(1, sizeof(bmp280_dev_t));
    sensor->bus = port;
    sensor->dev_addr = sensor_addr << 1;
    bmp280_init_default_params((bmp280_handle_t)sensor);
//    ESP_LOGI(TAG, "BMP280 handle info:");
//    ESP_LOGI(TAG, "%d,%d,%d",sensor->bus, sensor->dev_addr, sensor->param.oversampling_temperature);
    return (bmp280_handle_t) sensor;
}


static esp_err_t bmp280_read(bmp280_handle_t sensor, const uint8_t reg_start_addr, uint8_t *const data_buf, const uint8_t data_len)
{
    bmp280_dev_t *sens = (bmp280_dev_t *) sensor;
    esp_err_t  ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    if(ret != ESP_OK) ESP_LOGE(TAG, "Start i2c failed at read func!!");
    ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_WRITE, true);
    if(ret != ESP_OK) ESP_LOGE(TAG, "Write byte i2c failed at read func!!");
    ret = i2c_master_write_byte(cmd, reg_start_addr, true);
    if(ret != ESP_OK) ESP_LOGE(TAG, "Write byte i2c failed at read func!!");
    ret = i2c_master_start(cmd);
    if(ret != ESP_OK) ESP_LOGE(TAG, "Start i2c failed at read func!!");
    ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_READ, true);
    if(ret != ESP_OK) ESP_LOGE(TAG, "Write byte i2c failed at read func!!");
    ret = i2c_master_read(cmd, data_buf, data_len, I2C_MASTER_LAST_NACK);
    if(ret != ESP_OK) ESP_LOGE(TAG, "Read byte i2c failed at read func!!");
    ret = i2c_master_stop(cmd);
    if(ret != ESP_OK) ESP_LOGE(TAG, "Stop i2c failed at read func!!");
    ret = i2c_master_cmd_begin(sens->bus, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t bmp280_write(bmp280_handle_t sensor, const uint8_t reg_start_addr, const uint8_t *const data_buf, const uint8_t data_len)
{
    bmp280_dev_t *sens = (bmp280_dev_t *) sensor;
    esp_err_t  ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    if(ret != ESP_OK) ESP_LOGE(TAG, "Start byte i2c failed at write func!!");
    ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_WRITE, true);
    if(ret != ESP_OK) ESP_LOGE(TAG, "Write byte i2c failed at write func!!");
    ret = i2c_master_write_byte(cmd, reg_start_addr, true);
    if(ret != ESP_OK) ESP_LOGE(TAG, "Write byte i2c failed at write func!!");
    ret = i2c_master_write(cmd, data_buf, data_len, true);
    if(ret != ESP_OK) ESP_LOGE(TAG, "Write i2c failed at write func!!");
    ret = i2c_master_stop(cmd);
    if(ret != ESP_OK) ESP_LOGE(TAG, "Stop i2c failed at write func!!");
    ret = i2c_master_cmd_begin(sens->bus, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}


static esp_err_t read_calibration_data(bmp280_handle_t sensor)
{
	bmp280_dev_t *sens = (bmp280_dev_t *) sensor;
	esp_err_t ret;
	uint8_t data[24];

	ret = bmp280_read(sensor, BMP280_REG_CALIBRATION, data, 24);
	if(ret != ESP_OK) ESP_LOGE(TAG, "Read failed at read calibration func!!");

	sens->dev.dig_T1 = (data[1] << 8) | data[0];
	sens->dev.dig_T2 = (data[3] << 8) | data[2];
	sens->dev.dig_T3 = (data[5] << 8) | data[4];
	sens->dev.dig_P1 = (data[7] << 8) | data[6];
	sens->dev.dig_P2 = (data[9] << 8) | data[8];
	sens->dev.dig_P3 = (data[11] << 8) | data[10];
	sens->dev.dig_P4 = (data[13] << 8) | data[12];
	sens->dev.dig_P5 = (data[15] << 8) | data[14];
	sens->dev.dig_P6 = (data[17] << 8) | data[16];
	sens->dev.dig_P7 = (data[19] << 8) | data[18];
	sens->dev.dig_P8 = (data[21] << 8) | data[20];
	sens->dev.dig_P9 = (data[23] << 8) | data[22];

	return ret;
}

esp_err_t bmp280_init_default_params(bmp280_handle_t sensor)
{
	bmp280_dev_t *sens = (bmp280_dev_t *) sensor;

    sens->param.mode = BMP280_MODE_NORMAL;
    sens->param.filter = BMP280_FILTER_OFF;
    sens->param.oversampling_pressure = BMP280_STANDARD;
    sens->param.oversampling_temperature = BMP280_STANDARD;
    sens->param.oversampling_humidity = BMP280_STANDARD;
    sens->param.standby = BMP280_STANDBY_250;

    return ESP_OK;
}


/*
 * Currently, dont need to use this func
 */
bool bmp280_is_measuring(bmp280_handle_t sensor)
{
	uint8_t reg_status_value, reg_control_value;
	esp_err_t ret;
	ret = bmp280_read(sensor, BMP280_REG_STATUS, &reg_status_value, 1);
//    if(ret != ESP_OK) ESP_LOGE(TAG, "Read failed at read calibration func!!");
	ESP_ERROR_CHECK_WITHOUT_ABORT(ret);

	ret = bmp280_read(sensor, BMP280_REG_CTRL, &reg_control_value, 1);
	ESP_ERROR_CHECK_WITHOUT_ABORT(ret);

	return ((reg_control_value & 0b11) == BMP280_MODE_FORCED) || (reg_status_value & (1 << 3));

}



static esp_err_t bmp280_check_device_id(bmp280_handle_t sensor)
{
	esp_err_t ret;
	uint8_t id;
	ret = bmp280_read(sensor, BMP280_REG_ID, &id, 1);
	if(ESP_OK != ret) ESP_LOGE(TAG, "Read failed at check device id func");
	if(id != BMP280_CHIP_ID) return ESP_ERR_INVALID_RESPONSE;

	return ESP_OK;
}

esp_err_t bmp280_begin(bmp280_handle_t sensor)
{
	bmp280_dev_t *sens = (bmp280_dev_t *) sensor;

	esp_err_t ret;

	ret = bmp280_check_device_id(sensor);
	if(ret != ESP_OK) {
		ESP_LOGE(TAG, "Invalid sensor id, expected bmp280 = 0x58");
		return ESP_ERR_INVALID_RESPONSE;
	}

	const uint8_t bmp280_reset_value = 0xB6;
	ret = bmp280_write(sensor, BMP280_REG_RESET, &bmp280_reset_value, 1);		//may be error, change value to variable
    if(ret != ESP_OK) ESP_LOGE(TAG, "Write failed at begin func!!");

	ets_delay_us(200000);

	uint8_t rst_status  = -1;
	wait_for_reset:
	ret = bmp280_read(sensor, BMP280_REG_STATUS, &rst_status, 1);
    if(ret != ESP_OK) ESP_LOGE(TAG, "Read failed at begin func!!");
	if((rst_status & 1) != 0) {
		printf("Status wrong, wait for it!!");
		goto wait_for_reset;
	}

	ets_delay_us(1000);

	ret = read_calibration_data(sensor);
    if(ret != ESP_OK) ESP_LOGE(TAG, "Read calibration data failed at begin func!!");


	uint8_t config = (sens->param.standby << 5) | (sens->param.filter << 2);
//	printf("Config value in begin func: %" PRIu8 "\n", config);
	ret = bmp280_write(sensor, BMP280_REG_CONFIG, &config, 1);
    if(ret != ESP_OK) ESP_LOGE(TAG, "Write failed at begin func!!");

	uint8_t ctrl = (sens->param.oversampling_temperature << 5) | (sens->param.oversampling_pressure << 2) | (sens->param.mode);
//	printf("Control value at begin func: %" PRIu8 "\n", ctrl);
	ret = bmp280_write(sensor, BMP280_REG_CTRL, &ctrl, 1);
    if(ret != ESP_OK) ESP_LOGE(TAG, "Write failed at begin func!!");

	ets_delay_us(2000);
	return ret;
}


void bmp280_delete(bmp280_handle_t sensor)
{
    bmp280_dev_t *sens = (bmp280_dev_t *) sensor;
    free(sens);
}

/**
 * Compensation algorithm is taken from BMP280 datasheet.
 *
 * Return value is in degrees Celsius.
 */
static int32_t compensate_temperature(bmp280_handle_t sensor, int32_t adc_temp, int32_t *fine_temp)
{
	bmp280_dev_t *sens = (bmp280_dev_t *) sensor;
    int32_t var1, var2;

    var1 = ((((adc_temp >> 3) - ((int32_t)sens->dev.dig_T1 << 1))) * (int32_t)sens->dev.dig_T2) >> 11;
    var2 = (((((adc_temp >> 4) - (int32_t)sens->dev.dig_T1) * ((adc_temp >> 4) - (int32_t)sens->dev.dig_T1)) >> 12) * (int32_t)sens->dev.dig_T3) >> 14;
//    ESP_LOGI(TAG, "Compensate temp value: var1-> %" PRId32 " , var2-> %" PRId32 "!", var1, var2);
    *fine_temp = var1 + var2;
    return (*fine_temp * 5 + 128) >> 8;
}

/**
 * Compensation algorithm is taken from BMP280 datasheet.
 *
 * Return value is in Pa, 24 integer bits and 8 fractional bits.
 */
static uint32_t compensate_pressure(bmp280_handle_t sensor, int32_t adc_press, int32_t fine_temp)
{
	bmp280_dev_t *sens = (bmp280_dev_t *) sensor;
    int64_t var1, var2, p;

    var1 = (int64_t)fine_temp - 128000;
    var2 = var1 * var1 * (int64_t)sens->dev.dig_P6;
    var2 = var2 + ((var1 * (int64_t)sens->dev.dig_P5) << 17);
    var2 = var2 + (((int64_t)sens->dev.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)sens->dev.dig_P3) >> 8) + ((var1 * (int64_t)sens->dev.dig_P2) << 12);
    var1 = (((int64_t)1 << 47) + var1) * ((int64_t)sens->dev.dig_P1) >> 33;

    if (var1 == 0)
    {
        return 0;  // avoid exception caused by division by zero
    }

    p = 1048576 - adc_press;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)sens->dev.dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)sens->dev.dig_P8 * p) >> 19;

    p = ((p + var1 + var2) >> 8) + ((int64_t)sens->dev.dig_P7 << 4);
    return p;
}


esp_err_t bmp280_read_fixed(bmp280_handle_t sensor, int32_t *temperature, uint32_t *pressure)
{

    int32_t adc_pressure;
    int32_t adc_temp;
    uint8_t data[6];

    // Need to read in one sequence to ensure they match.
    esp_err_t ret;
    ret = bmp280_read(sensor, 0xf7, data, 6);
    if(ret != ESP_OK) ESP_LOGE(TAG, "Read failed at read fixed func!!");

    adc_pressure = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    adc_temp = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
//    ESP_LOGI(TAG, "Pressure is: %" PRIu32 "\n", adc_pressure);
//    ESP_LOGI(TAG, "ADC Temp is: %" PRIu32 "\n", adc_temp);

    int32_t fine_temp;
    *temperature = compensate_temperature(sensor, adc_temp, &fine_temp);
    *pressure = compensate_pressure(sensor, adc_pressure, fine_temp);
//    ESP_LOGI(TAG, "Temperature after read compensate: %" PRId32 " ", *temperature);
    return ret;
}

esp_err_t bmp280_read_float(bmp280_handle_t sensor, float *temperature, float *pressure)
{
    int32_t fixed_temperature;
    uint32_t fixed_pressure;

    esp_err_t ret;
    ret = bmp280_read_fixed(sensor, &fixed_temperature, &fixed_pressure);
    if(ret != ESP_OK) ESP_LOGE(TAG, "Read fixed failed at read float func!!");

    *temperature = (float)fixed_temperature / 100;
    *pressure = (float)fixed_pressure / 256;

    return ret;
}

