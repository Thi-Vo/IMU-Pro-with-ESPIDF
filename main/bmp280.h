#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c.h>


typedef void *bmp280_handle_t;
/**
 * Mode of BMP280 module operation.
 */
typedef enum {
    BMP280_MODE_SLEEP = 0,  //!< Sleep mode
    BMP280_MODE_FORCED = 1, //!< Measurement is initiated by user
    BMP280_MODE_NORMAL = 3  //!< Continues measurement
} BMP280_Mode;

typedef enum {
    BMP280_FILTER_OFF = 0,
    BMP280_FILTER_2 = 1,
    BMP280_FILTER_4 = 2,
    BMP280_FILTER_8 = 3,
    BMP280_FILTER_16 = 4
} BMP280_Filter;

/**
 * Pressure oversampling settings
 */
typedef enum {
    BMP280_SKIPPED = 0,          //!< no measurement
    BMP280_ULTRA_LOW_POWER = 1,  //!< oversampling x1
    BMP280_LOW_POWER = 2,        //!< oversampling x2
    BMP280_STANDARD = 3,         //!< oversampling x4
    BMP280_HIGH_RES = 4,         //!< oversampling x8
    BMP280_ULTRA_HIGH_RES = 5    //!< oversampling x16
} BMP280_Oversampling;

/**
 * Stand by time between measurements in normal mode
 */
typedef enum {
    BMP280_STANDBY_05 = 0,      //!< stand by time 0.5ms
    BMP280_STANDBY_62 = 1,      //!< stand by time 62.5ms
    BMP280_STANDBY_125 = 2,     //!< stand by time 125ms
    BMP280_STANDBY_250 = 3,     //!< stand by time 250ms
    BMP280_STANDBY_500 = 4,     //!< stand by time 500ms
    BMP280_STANDBY_1000 = 5,    //!< stand by time 1s
    BMP280_STANDBY_2000 = 6,    //!< stand by time 2s BMP280, 10ms BME280
    BMP280_STANDBY_4000 = 7,    //!< stand by time 4s BMP280, 20ms BME280
} BMP280_StandbyTime;

/**
 * Configuration parameters for BMP280 module.
 * Use function ::bmp280_init_default_params() to use default configuration.
 */
typedef struct {
    BMP280_Mode mode;
    BMP280_Filter filter;
    BMP280_Oversampling oversampling_pressure;
    BMP280_Oversampling oversampling_temperature;
    BMP280_Oversampling oversampling_humidity;
    BMP280_StandbyTime standby;
} bmp280_params_t;

/**
 * Device descriptor
 */
typedef struct {
    uint16_t  dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t  dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

} bmp280_t;

typedef struct {
    i2c_port_t bus;
    uint16_t dev_addr;
    bmp280_t dev;
    bmp280_params_t param;
} bmp280_dev_t;




/*
 * Create instance bmp280 and initialize its parameter
 * Include: i2c port
 * 			sensor address 0x76
 * 			default parameter
 */

bmp280_handle_t bmp280_create(i2c_port_t port, const uint16_t sensor_addr);

/**
 * @brief Initialize default parameters
 *
 * Default configuration:
 *
 *  - mode: NORMAL
 *  - filter: OFF
 *  - oversampling: x4
 *  - standby time: 250ms
 *
 * @param[out] params Default parameters
 * @return `ESP_OK` on success
 */
esp_err_t bmp280_init_default_params(bmp280_handle_t sensor);

/*
 * Start bmp280 activities
 * Include: Check device id
 * 			reset sensor
 * 			read calibration data
 * 			load config value
 */
esp_err_t bmp280_begin(bmp280_handle_t sensor);


/**
 * @brief Read raw compensated temperature and pressure data
 *
 * Temperature in degrees Celsius times 100.
 *
 * Pressure in Pascals in fixed point 24 bit integer 8 bit fraction format.
 *
 * Humidity is optional and only read for the BME280, in percent relative
 * humidity as a fixed point 22 bit integer and 10 bit fraction format.
 *
 * @param dev Device descriptor
 * @param[out] temperature Temperature, deg.C * 100
 * @param[out] pressure Pressure
 * @param[out] humidity Humidity, optional
 * @return `ESP_OK` on success
 */
esp_err_t bmp280_read_fixed(bmp280_handle_t sensor, int32_t *temperature, uint32_t *pressure);

/**
 * @brief Read compensated temperature and pressure data
 *
 * Humidity is optional and only read for the BME280.
 *
 * @param dev Device descriptor
 * @param[out] temperature Temperature, deg.C
 * @param[out] pressure Pressure, Pascal
 * @param[out] humidity Relative humidity, percents (optional)
 * @return `ESP_OK` on success
 */
esp_err_t bmp280_read_float(bmp280_handle_t sensor, float *temperature, float *pressure);


/*
 * Delete bmp280 object
 */
void bmp280_delete(bmp280_handle_t sensor);

#ifdef __cplusplus
}
#endif

