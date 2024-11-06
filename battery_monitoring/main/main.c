#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "mcp2515.h"  
#include "esp_log.h"

// I2C configuration for INA219
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define INA219_ADDR 0x40  // I2C address for INA219 sensor

// CAN configuration
#define CAN_SPEED CAN_500KBPS
#define CAN_TX_PIN 5
#define CAN_RX_PIN 4
#define MCP2515_CS_PIN 15  // SPI CS pin for MCP2515

SemaphoreHandle_t sensor_mutex;
MCP2515 mcp2515(&SPI, MCP2515_CS_PIN);  // Initialize MCP2515 with SPI

// Structure to send CAN messages
struct can_frame frame;

// I2C Initialization
void i2c_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
}

// Read battery voltage using INA219
int read_battery_voltage() {
    uint8_t data[2];
    uint8_t cmd = 0x02; // Command to read voltage
    i2c_master_write_to_device(I2C_MASTER_NUM, INA219_ADDR, &cmd, 1, 1000 / portTICK_PERIOD_MS);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    i2c_master_read_from_device(I2C_MASTER_NUM, INA219_ADDR, data, 2, 1000 / portTICK_PERIOD_MS);
    
    // Combine two bytes and calculate voltage (example conversion)
    int voltage_raw = (data[0] << 8) | data[1];
    return voltage_raw * 0.002;  // Example: scale to mV
}

// CAN initialization
void can_init() {
    mcp2515.reset();
    mcp2515.setBitrate(CAN_SPEED);
    mcp2515.setNormalMode();  // Set to Normal Mode for sending/receiving
}

// Send CAN message
void send_data_to_ecu(int voltage) {
    frame.can_id = 0x123;  // Example CAN ID for battery sensor
    frame.can_dlc = 2;  // Data length (2 bytes for voltage)
    frame.data[0] = (voltage >> 8) & 0xFF;
    frame.data[1] = voltage & 0xFF;

    mcp2515.sendMessage(&frame);  // Send the frame over CAN
}

void sensor_task(void *param) {
    while (1) {
        if (xSemaphoreTake(sensor_mutex, portMAX_DELAY) == pdTRUE) {
            int voltage = read_battery_voltage();
            ESP_LOGI("Battery Sensor", "Battery Voltage: %d mV", voltage);
            send_data_to_ecu(voltage);
            xSemaphoreGive(sensor_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));  // Delay 1 second
    }
}

void app_main() {
    ESP_LOGI("Battery Sensor", "Starting Battery Level Sensor Node...");
    i2c_init();
    can_init();
    
    sensor_mutex = xSemaphoreCreateMutex();
    if (sensor_mutex == NULL) {
        ESP_LOGE("Battery Sensor", "Failed to create mutex!");
        return;
    }

    xTaskCreate(sensor_task, "sensor_battery_task", 2048, NULL, 5, NULL);
}
