#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "MCP2515.h"
#include "esp_log.h"

static const char *TAG = "ECU";

// CAN configuration
#define CAN_SPEED CAN_500KBPS
#define CAN_TX_PIN 5
#define CAN_RX_PIN 4
#define MCP2515_CS_PIN 15  // SPI CS pin for MCP2515

// MQTT configuration
#define MQTT_BROKER_URL "mqtt://broker.emq.io"
#define TOPIC_BATTERY "/topic/battery"
#define TOPIC_COLLISION "/topic/collision"
#define TOPIC_TEMPERATURE "/topic/temperature"
#define TOPIC_GPS "/topic/gps"
#define TOPIC_SPEED "/topic/speed"

// CAN structure and initialization
MCP2515 mcp2515(&SPI, MCP2515_CS_PIN);  // Initialize MCP2515 with SPI

// CAN frame structure to store received message
struct can_frame frame;

// Semaphore for synchronization between tasks
SemaphoreHandle_t ecu_mutex;
SemaphoreHandle_t mqtt_mutex;

// MQTT client handle
esp_mqtt_client_handle_t mqtt_client;

// Function to initialize CAN
void can_init() {
    mcp2515.reset();
    mcp2515.setBitrate(CAN_SPEED);
    mcp2515.setNormalMode();
}

// Function to read CAN data and process it
void receive_data() {
    if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
        switch (frame.can_id) {
            case 0x51:  // Battery Monitoring
                float bat_perc;
                memcpy(&bat_perc, frame.data, sizeof(bat_perc));
                ESP_LOGI(TAG, "Battery percentage: %.2f", bat_perc);
                // Publish to MQTT
                char payload[50];
                snprintf(payload, sizeof(payload), "%.2f", bat_perc);
                xSemaphoreTake(mqtt_mutex, portMAX_DELAY);
                esp_mqtt_client_publish(mqtt_client, TOPIC_BATTERY, payload, 0, 1, 0);
                xSemaphoreGive(mqtt_mutex);
                break;

            case 0x52:  // Collision Sensor
                int collision_distance = (frame.data[0] << 8) | frame.data[1];
                ESP_LOGI(TAG, "Collision Distance: %d cm", collision_distance);
                // Publish to MQTT
                snprintf(payload, sizeof(payload), "%d", collision_distance);
                xSemaphoreTake(mqtt_mutex, portMAX_DELAY);
                esp_mqtt_client_publish(mqtt_client, TOPIC_COLLISION, payload, 0, 1, 0);
                xSemaphoreGive(mqtt_mutex);
                break;

            case 0x53:  // Engine Temperature Sensor
                float temp;
                memcpy(&temp, frame.data, sizeof(temp));
                ESP_LOGI(TAG, "Engine Temperature: %.2f°C", temp);
                // Publish to MQTT
                snprintf(payload, sizeof(payload), "%.2f", temp);
                xSemaphoreTake(mqtt_mutex, portMAX_DELAY);
                esp_mqtt_client_publish(mqtt_client, TOPIC_TEMPERATURE, payload, 0, 1, 0);
                xSemaphoreGive(mqtt_mutex);
                break;

            case 0x54:  // GPS Sensor
                float lat, lon;
                memcpy(&lat, frame.data, sizeof(lat));
                memcpy(&lon, frame.data + sizeof(lat), sizeof(lon));
                ESP_LOGI(TAG, "GPS - Latitude: %.6f, Longitude: %.6f", lat, lon);
                // Publish to MQTT
                snprintf(payload, sizeof(payload), "Lat: %.6f, Lon: %.6f", lat, lon);
                xSemaphoreTake(mqtt_mutex, portMAX_DELAY);
                esp_mqtt_client_publish(mqtt_client, TOPIC_GPS, payload, 0, 1, 0);
                xSemaphoreGive(mqtt_mutex);
                break;

            case 0x55:  // Speed Sensor
                int speed = (frame.data[0] << 8) | frame.data[1];
                ESP_LOGI(TAG, "Speed: %d km/h", speed);
                // Publish to MQTT
                snprintf(payload, sizeof(payload), "%d", speed);
                xSemaphoreTake(mqtt_mutex, portMAX_DELAY);
                esp_mqtt_client_publish(mqtt_client, TOPIC_SPEED, payload, 0, 1, 0);
                xSemaphoreGive(mqtt_mutex);
                break;

            default:
                ESP_LOGI(TAG, "Unknown CAN ID: 0x%X", frame.can_id);
                break;
        }
    }
}

// Task to receive CAN data and process it
void can_task(void *pvParameters) {
    while (1) {
        if (xSemaphoreTake(ecu_mutex, portMAX_DELAY) == pdTRUE) {
            receive_data();
            xSemaphoreGive(ecu_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // Delay 100 ms between reads
    }
}

// MQTT Event Handler
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED");
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event_id);
        break;
    }
}

// Initialize MQTT client and start the connection
static void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = MQTT_BROKER_URL,
    };
    
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

void app_main(void) {
    ESP_LOGI(TAG, "[APP] Startup..");

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());

    // Initialize CAN
    can_init();

    // Initialize GPIO and create mutexes
    ecu_mutex = xSemaphoreCreateMutex();
    mqtt_mutex = xSemaphoreCreateMutex();
    if (ecu_mutex == NULL || mqtt_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }

    // Start CAN data reception task
    xTaskCreate(can_task, "can_task", 2048, NULL, 5, NULL);

    // Initialize and start MQTT
    mqtt_app_start();
}
