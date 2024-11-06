#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "mcp2515.h"
#include "esp_log.h"

// Define the trigger and echo GPIO pins
#define TRIG_PIN 13
#define ECHO_PIN 12

// CAN configuration
#define CAN_SPEED CAN_500KBPS
#define CAN_TX_PIN 5
#define CAN_RX_PIN 4
#define MCP2515_CS_PIN 15  // SPI CS pin for MCP2515

SemaphoreHandle_t sensor_mutex;
MCP2515 mcp2515(&SPI, MCP2515_CS_PIN);  // Initialize MCP2515 with SPI

// Structure to send CAN messages
struct can_frame frame;

// Setup the GPIO for the ultrasonic sensor
void gpio_init() {
    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
}

// Send a pulse to trigger the ultrasonic sensor
void trigger_pulse() {
    gpio_set_level(TRIG_PIN, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);  // Trigger pulse duration (10ms)
    gpio_set_level(TRIG_PIN, 0);
}

// Measure distance using ultrasonic sensor
int measure_distance() {
    trigger_pulse();

    // Wait for the echo to start
    uint32_t start_time = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 0) {
        if (esp_timer_get_time() - start_time > 1000000) {
            return -1;  // Timeout if no pulse received
        }
    }
    start_time = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 1) {
        if (esp_timer_get_time() - start_time > 1000000) {
            return -1;  // Timeout if echo signal is not received
        }
    }
    uint32_t duration = esp_timer_get_time() - start_time;
    // Calculate distance based on the speed of sound (343m/s) and convert to cm
    return (duration * 0.0343) / 2;  // In cm
}

// CAN initialization
void can_init() {
    mcp2515.reset();
    mcp2515.setBitrate(CAN_SPEED);
    mcp2515.setNormalMode();
}

// Send CAN message with collision distance
void send_data_to_ecu(int distance) {
    frame.can_id = 0x52;  // Example CAN ID for collision sensor
    frame.can_dlc = 2;  // 2 bytes for distance (in cm)
    frame.data[0] = (distance >> 8) & 0xFF;
    frame.data[1] = distance & 0xFF;

    mcp2515.sendMessage(&frame);  // Send the frame over CAN
}

void sensor_task(void *param) {
    while (1) {
        if (xSemaphoreTake(sensor_mutex, portMAX_DELAY) == pdTRUE) {
            int distance = measure_distance();
            if (distance >= 0) {
                ESP_LOGI("Collision Sensor", "Distance: %d cm", distance);
                send_data_to_ecu(distance);
            } else {
                ESP_LOGW("Collision Sensor", "No valid distance measured");
            }
            xSemaphoreGive(sensor_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));  // Delay 1 second
    }
}

void app_main() {
    ESP_LOGI("Collision Sensor", "Starting Collision Sensor Node...");
    gpio_init();
    can_init();
    
    sensor_mutex = xSemaphoreCreateMutex();
    if (sensor_mutex == NULL) {
        ESP_LOGE("Collision Sensor", "Failed to create mutex!");
        return;
    }

    xTaskCreate(sensor_task, "sensor_collision_task", 2048, NULL, 5, NULL);
}
