#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "esp_system.h"
#include "unity.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensor.h"


void app_main(void)
{
//	xTaskCreate(read_bmi270, "bmi270", 10240, NULL, 10, NULL);
//	xTaskCreate(read_bmp280, "bmp280", 10240, NULL, 10, NULL);
	xTaskCreate(read_imu, "imu", 20480, NULL, 10, NULL);

//	ESP_LOGE(TAG, "error");
//	ESP_LOGW(TAG, "warning");
//	ESP_LOGI(TAG, "Info");
}
