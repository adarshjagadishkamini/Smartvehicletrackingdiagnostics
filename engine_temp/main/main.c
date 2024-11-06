#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "mcp2515.h"
#include "esp_log.h"

// I2C configuration for temperature sensor (assume TMP102)
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define TMP102_ADDR 0x48  // I2C address of the TMP102 temperature sensor
#define I2C_FREQ_HZ 100000

// CAN configuration
#define CAN_SPEED CAN_500KBPS
#define CAN_TX_PIN 5
#define CAN_RX_PIN 4
#define MCP2515_CS_PIN 15  // SPI CS pin for MCP2515

SemaphoreHandle_t sensor_mutex;
MCP2515 mcp2515(&SPI, MCP2515_CS_PIN);  // Initialize MCP2515 with SPI

// Structure to send CAN messages
struct can_frame frame;

// I2C initialization for temperature sensor
void i2c_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
}

// Read temperature from TMP102
float read_temperature() {
    uint8_t data[2];
    i2c_master_write_to_device(I2C_MASTER_NUM, TMP102_ADDR, NULL, 0, 1000 / portTICK_PERIOD_MS);
    i2c_master_read_from_device(I2C_MASTER_NUM, TMP102_ADDR, data, 2, 1000 / portTICK_PERIOD_MS);
    int16_t raw_temp = (data[0] << 8) | data[1];
    return raw_temp * 0.0625;  // Convert to Celsius
}

// CAN initialization
void can_init() {
    mcp2515.reset();
    mcp2515.setBitrate(CAN_SPEED);
    mcp2515.setNormalMode();
}

// Send CAN message with temperature data
void send_data_to_ecu(float temperature) {
    frame.can_id = 0x53;  // Example CAN ID for engine temperature sensor
    frame.can_dlc = 4;  // 4 bytes for temperature (float value)
    memcpy(frame.data, &temperature, sizeof(temperature));

    mcp2515.sendMessage(&frame);  // Send the frame over CAN
}

void sensor_task(void *param) {
    while (1) {
        if (xSemaphoreTake(sensor_mutex, portMAX_DELAY) == pdTRUE) {
            float temperature = read_temperature();
            ESP_LOGI("Temperature Sensor", "Engine Temperature: %.2f°C", temperature);
            send_data_to_ecu(temperature);
            xSemaphoreGive(sensor_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));  
    }
}

void app_main() {
    ESP_LOGI("Temperature Sensor", "Starting Engine Temperature Sensor...");
    i2c_init();
    can_init();

    sensor_mutex = xSemaphoreCreateMutex();
    if (sensor_mutex == NULL) {
        ESP_LOGE("Temperature Sensor", "Failed to create mutex!");
        return;
    }

    xTaskCreate(sensor_task, "sensor_temp_task", 2048, NULL, 5, NULL);
}
