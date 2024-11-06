#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "MCP2515.h"
#include "esp_log.h"

// CAN configuration
#define CAN_SPEED CAN_500KBPS
#define CAN_TX_PIN 5
#define CAN_RX_PIN 4
#define MCP2515_CS_PIN 15  // SPI CS pin for MCP2515

SemaphoreHandle_t ecu_mutex;
MCP2515 mcp2515(&SPI, MCP2515_CS_PIN);  // Initialize MCP2515 with SPI

// Structure to receive CAN messages
struct can_frame frame;

void can_init() {
    mcp2515.reset();
    mcp2515.setBitrate(CAN_SPEED);
    mcp2515.setNormalMode();
}

// Read CAN message and process
void receive_data() {
    if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
        // Process the CAN message based on its ID
        switch (frame.can_id) {
            case 0x51:  //Battery monitoring
                float bat_perc;
                memcpy(&bat_perc, frame.data, sizeof(bat_perc));
                ESP_LOGI("ECU", "Battery percentage: %.2f", bat_perc);
                break;

            case 0x52:  // Collision Sensor
                int collision_distance = (frame.data[0] << 8) | frame.data[1];
                ESP_LOGI("ECU", "Collision Distance: %d cm", collision_distance);
                break;

            case 0x53:  // Engine Temperature Sensor
                float temp;
                memcpy(&temp, frame.data, sizeof(temp));
                ESP_LOGI("ECU", "Engine Temperature: %.2f°C", temp);
                break;

            case 0x54:  // GPS Sensor
                float lat, lon;
                memcpy(&lat, frame.data, sizeof(lat));
                memcpy(&lon, frame.data + sizeof(lat), sizeof(lon));
                ESP_LOGI("ECU", "GPS - Latitude: %.6f, Longitude: %.6f", lat, lon);
                break;

            case 0x55:  // Speed Sensor
                int speed = (frame.data[0] << 8) | frame.data[1];
                ESP_LOGI("ECU", "Speed: %d km/h", speed);
                break;
                
            case 0x56:  // Engine Temperature Sensor
                float pressure;
                memcpy(&pressure, frame.data, sizeof(pressure));
                ESP_LOGI("ECU", "Tire Pressure: %.2f", pressure);
                break;
            
            default:
                ESP_LOGI("ECU", "Unknown CAN ID: 0x%X", frame.can_id);
                break;
        }
    }
}

void ecu_task(void *param) {
    while (1) {
        if (xSemaphoreTake(ecu_mutex, portMAX_DELAY) == pdTRUE) {
            receive_data();
            xSemaphoreGive(ecu_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // Delay 100 ms
    }
}

void app_main() {
    ESP_LOGI("ECU", "Starting ECU...");
    can_init();

    ecu_mutex = xSemaphoreCreateMutex();
    if (ecu_mutex == NULL) {
        ESP_LOGE("ECU", "Failed to create mutex!");
        return;
    }

    xTaskCreate(ecu_task, "ecu_task", 2048, NULL, 5, NULL);
}
