#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_spi_flash.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "wifi_helper.h"
#include "mqtt_helper.h"

// WiFi credentials.
#define WIFI_SSID "Milkrun"
#define WIFI_PASS "55382636751623425906"

// MQTT details
static const char* mqtt_server = "mqtt://raspberrypi.fritz.box";

static const char* ota_url = "http://debian.fritz.box:8032/esp32/RGBM.bin";

static const char *TAG = "RGBM";

#define B_GPIO GPIO_NUM_2
#define R_GPIO GPIO_NUM_8
#define G_GPIO GPIO_NUM_9
#define M_GPIO GPIO_NUM_20

#define B_CHANNEL LEDC_CHANNEL_0
#define R_CHANNEL LEDC_CHANNEL_1
#define G_CHANNEL LEDC_CHANNEL_2
#define M_CHANNEL LEDC_CHANNEL_3


#define LEDC_DUTY_LOW     (0)
#define LEDC_DUTY_HIGH    (2047) // Max for 11 bits
#define LEDC_FADE_TIME    (1000)

static void ota_task(void * pvParameter) {
    ESP_LOGI(TAG, "Starting OTA update...");

    esp_http_client_config_t config = {};
    config.url = ota_url;

    esp_https_ota_config_t ota_config = {};
    ota_config.http_config = &config;

    ESP_LOGI(TAG, "Attempting to download update from %s", config.url);
    esp_err_t ret = esp_https_ota(&ota_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "OTA Succeed, Rebooting...");
        esp_restart();
    } else {
        ESP_LOGE(TAG, "Firmware Upgrades Failed");
    }
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void subscribeTopics() {
    subscribeDevTopic("$update");
    subscribeTopic("/ledstrip1/#");
}

static void handleMessage(const char* topic1, const char* topic2, const char* topic3, const char* data) {
    if(
        strcmp(topic1, "$update") == 0 && 
        topic2 == NULL && 
        topic3 == NULL
    ) {
        xTaskCreate(&ota_task, "ota_task", 8192, NULL, 5, NULL);
    }
}

static bool handleAnyMessage(const char* topic, const char* data) {

    if (strcmp(topic,"/ledstrip1/value/fade") == 0) {
        const char color = data[0];
        ledc_channel_t channel = R_CHANNEL;
        if(color == 'g') {
            channel = G_CHANNEL;
        }
        if(color == 'b') {
            channel = B_CHANNEL;
        }
        if(color == 'm') {
            channel = M_CHANNEL;
        }

        int newvalue = atoi(data + 1);
        if(newvalue < LEDC_DUTY_LOW || newvalue > LEDC_DUTY_HIGH ) {
            return true;
        }
        printf("LEDC fade to duty = %d\n", newvalue);
        ESP_ERROR_CHECK(ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, channel, newvalue, LEDC_FADE_TIME));
        ESP_ERROR_CHECK(ledc_fade_start(LEDC_LOW_SPEED_MODE, channel, LEDC_FADE_NO_WAIT));
        return true;
    }

    if (strcmp(topic,"/ledstrip1/value/set") == 0) {
        const char color = data[0];
        ledc_channel_t channel = R_CHANNEL;
        if(color == 'g') {
            channel = G_CHANNEL;
        }
        if(color == 'b') {
            channel = B_CHANNEL;
        }
        if(color == 'm') {
            channel = M_CHANNEL;
        }

        int newvalue = atoi(data + 1);
        if(newvalue < LEDC_DUTY_LOW || newvalue > LEDC_DUTY_HIGH ) {
            return true;
        }
        printf("LEDC set duty = %d\n", newvalue);
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, newvalue));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
        return true;
    }

    if (strcmp(topic,"/ledstrip1/update") == 0) {
        xTaskCreate(&ota_task, "ota_task", 8192, NULL, 5, NULL);
        return true;
    }
    return false;
}

extern "C" void app_main() {
    // The LED pin should be on by default
    gpio_set_direction(R_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(R_GPIO, 0);
    gpio_set_direction(G_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(G_GPIO, 0);
    gpio_set_direction(B_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(B_GPIO, 0);
    gpio_set_direction(M_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(M_GPIO, 0);
    

    // Configure timer with 11 bits resolution, and 39khz (39062.5 is max for 11 bit)
    // We use high speed mode
    ledc_timer_config_t ledc_timer = {};
    ledc_timer.duty_resolution = LEDC_TIMER_11_BIT;
    ledc_timer.freq_hz = 39000;
    ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_timer.timer_num = LEDC_TIMER_0;
    ledc_timer.clk_cfg = LEDC_USE_APB_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {};
    ledc_channel.gpio_num   = R_GPIO;
    ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel.channel    = R_CHANNEL;
    ledc_channel.timer_sel  = LEDC_TIMER_0;
    ledc_channel.duty       = LEDC_DUTY_LOW;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_channel.gpio_num   = G_GPIO;
    ledc_channel.channel    = G_CHANNEL;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_channel.gpio_num   = B_GPIO;
    ledc_channel.channel    = B_CHANNEL;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_channel.gpio_num   = M_GPIO;
    ledc_channel.channel    = M_CHANNEL;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    // Initialize fade service.
    ESP_ERROR_CHECK(ledc_fade_func_install(0));

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);


    // Initialize WiFi
    wifiStart();

    ESP_LOGI(TAG, "Waiting for wifi");
    wifiWait();

    ESP_ERROR_CHECK(mqttStart("RGBM", subscribeTopics, handleMessage, handleAnyMessage));

    ESP_LOGI(TAG, "Waiting for MQTT");

    mqttWait();

    vTaskDelete(NULL);
}
