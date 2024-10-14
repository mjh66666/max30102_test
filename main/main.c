/*
 * @Author: mojionghao
 * @Date: 2024-08-02 11:13:32
 * @LastEditors: mojionghao
 * @LastEditTime: 2024-08-13 13:59:26
 * @FilePath: \max30102_test\main\main.c
 * @Description:
 */
#include <stdio.h>
#include "driver/gpio.h"
#include "myi2c.h"
#include "max30102.h"
#include "blood.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

max30102_handle_t max30102;
float temp, spo2, heart;
static const char *TAG = "main";
void max30102_task(void *p)
{
    while (1)
    {
        ESP_ERROR_CHECK(max30102_read_temp(max30102, &temp));
        ESP_LOGI(TAG, "temp:%f", temp);
        temp = 0.0;
        blood_Loop(max30102, &heart, &spo2);
        ESP_LOGI(TAG, "SPO2:%.2f,HEART:%.2f", spo2, heart);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());

    max30102 = max30102_create(0, MAX30102_Device_address, GPIO_NUM_18);
    ESP_ERROR_CHECK(max30102_config(max30102));

    xTaskCreate(max30102_task, "max30102", 4096, NULL, 6, NULL);
}
