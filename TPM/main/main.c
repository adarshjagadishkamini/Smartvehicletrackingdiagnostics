#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "mcp2515.h"
#include "esp_log.h"

// I2C configuration for TPMS
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define TPMS_SENSOR_ADDR 0x76  // Example I2C address for TPMS

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

// Read tire pressure from TPMS sensor
int read_tire_pressure() {
    uint8_t cmd = 0x10;  // Command to read tire pressure
    uint8_t data[2];

    // Read pressure data
    i2c_master_write_to_device(I2C_MASTER_NUM, TPMS_SENSOR_ADDR, &cmd, 1, 1000 / portTICK_PERIOD_MS);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    i2c_master_read_from_device(I2C_MASTER_NUM, TPMS_SENSOR_ADDR, data, 2, 1000 / portTICK_PERIOD_MS);

    int pressure_raw = (data[0] << 8) | data[1];
    return pressure_raw / 10;  // Example conversion to pressure in kPa
}

// CAN initialization
void can_init() {
    mcp2515.reset();
    mcp2515.setBitrate(CAN_SPEED);
    mcp2515.setNormalMode();
}

// Send CAN message
void send_data_to_ecu(int pressure) {
    frame.can_id = 0x124;  // Example CAN ID for tire pressure sensor
    frame.can_dlc = 2;  // 2 bytes for pressure
    frame.data[0] = (pressure >> 8) & 0xFF;
    frame.data[1] = pressure & 0xFF;

    mcp2515.sendMessage(&frame);  // Send the frame over CAN
}

void sensor_task(void *param) {
    while (1) {
        if (xSemaphoreTake(sensor_mutex, portMAX_DELAY) == pdTRUE) {
            int tire_pressure = read_tire_pressure();
            ESP_LOGI("Tire Pressure", "Tire Pressure: %d kPa", tire_pressure);
            send_data_to_ecu(tire_pressure);
            xSemaphoreGive(sensor_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));  // Delay 1 second
    }
}

void app_main() {
    ESP_LOGI("Tire Pressure", "Starting Tire Pressure Sensor Node...");
    i2c_init();
    can_init();
    
    sensor_mutex = xSemaphoreCreateMutex();
    if (sensor_mutex == NULL) {
        ESP_LOGE("Tire Pressure", "Failed to create mutex!");
        return;
    }

    xTaskCreate(sensor_task, "sensor_tire_pressure_task", 2048, NULL, 5, NULL);
}
