#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "driver/uart.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "driver/spi_master.h"
#include "mcp2515.h"

// UART configuration
#define UART_NUM UART_NUM_1
#define GPS_TX_PIN (17)
#define GPS_RX_PIN (16)
#define UART_BUF_SIZE 1024

// CAN configuration
#define CAN_SPEED CAN_500KBPS
#define CAN_TX_PIN 5
#define CAN_RX_PIN 4
#define MCP2515_CS_PIN 15  // SPI CS pin for MCP2515

// Log tag
static const char *TAG = "GPS_CAN_Parser";

// GPS NMEA sentence types
#define GPGGA "GPGGA"
#define GPRMC "GPRMC"

// GPS data structure
typedef struct {
    float latitude;
    float longitude;
    float altitude;
    char time[11]; // HH:MM:SS format
} gps_data_t;

// CAN message frame structure
struct can_frame frame;

// MCP2515 CAN controller instance
MCP2515 mcp2515(&SPI, MCP2515_CS_PIN);  // Initialize MCP2515 with SPI

// Function to parse GPGGA sentence
void parse_gpgga(const char *sentence, gps_data_t *data) {
    char *token;
    token = strtok((char *)sentence, ",");
    
    // Skip sentence identifier (GPGGA)
    token = strtok(NULL, ",");  // Time (HHMMSS.SS)
    if (token) {
        strncpy(data->time, token, 10);
        data->time[10] = '\0';
    }
    token = strtok(NULL, ",");  // Latitude
    if (token) {
        data->latitude = atof(token);
    }
    token = strtok(NULL, ",");  // North/South Indicator
    if (token && token[0] == 'S') {
        data->latitude = -data->latitude;
    }
    token = strtok(NULL, ",");  // Longitude
    if (token) {
        data->longitude = atof(token);
    }
    token = strtok(NULL, ",");  // East/West Indicator
    if (token && token[0] == 'W') {
        data->longitude = -data->longitude;
    }
    token = strtok(NULL, ",");  // Altitude (m)
    if (token) {
        data->altitude = atof(token);
    }
}

// Function to parse GPRMC sentence
void parse_gprmc(const char *sentence, gps_data_t *data) {
    char *token;
    token = strtok((char *)sentence, ",");
    
    // Skip sentence identifier (GPRMC)
    token = strtok(NULL, ",");  // Time (HHMMSS.SS)
    token = strtok(NULL, ",");  // Status
    token = strtok(NULL, ",");  // Latitude
    if (token) {
        data->latitude = atof(token);
    }
    token = strtok(NULL, ",");  // North/South Indicator
    if (token && token[0] == 'S') {
        data->latitude = -data->latitude;
    }
    token = strtok(NULL, ",");  // Longitude
    if (token) {
        data->longitude = atof(token);
    }
    token = strtok(NULL, ",");  // East/West Indicator
    if (token && token[0] == 'W') {
        data->longitude = -data->longitude;
    }
}

// Function to initialize CAN
void can_init() {
    mcp2515.reset();
    mcp2515.setBitrate(CAN_SPEED);
    mcp2515.setNormalMode();
}

// Function to send GPS data over CAN
void send_data_to_ecu(const gps_data_t *data) {
    // Prepare CAN frame with GPS data (latitude, longitude, altitude)
    frame.can_id = 0x126;  // CAN ID for GPS data
    frame.can_dlc = 8;  // 8 bytes: 4 for latitude, 4 for longitude (float data)
    memcpy(frame.data, &data->latitude, sizeof(float));  // Latitude
    memcpy(frame.data + 4, &data->longitude, sizeof(float));  // Longitude
    mcp2515.sendMessage(&frame);  // Send the frame over CAN
    
    ESP_LOGI(TAG, "Sent GPS Data: Latitude %.6f, Longitude %.6f", data->latitude, data->longitude);
}

// Function to read and parse GPS data
void gps_task(void *pvParameters) {
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_BITS_8,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // Install UART driver
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE, 0, 0, NULL, 0));

    uint8_t *data = (uint8_t *)malloc(UART_BUF_SIZE);
    gps_data_t gps_data = {0};

    while (1) {
        int length = uart_read_bytes(UART_NUM, data, UART_BUF_SIZE, 100 / portTICK_RATE_MS);
        if (length > 0) {
            data[length] = '\0'; // Null-terminate the received data
            ESP_LOGI(TAG, "Received: %s", data);

            // Look for GPGGA or GPRMC sentence
            if (strstr((char *)data, GPGGA) != NULL) {
                parse_gpgga((char *)data, &gps_data);
                ESP_LOGI(TAG, "Time: %s, Latitude: %.6f, Longitude: %.6f, Altitude: %.2f meters",
                         gps_data.time, gps_data.latitude, gps_data.longitude, gps_data.altitude);
                send_data_to_ecu(&gps_data);  // Send parsed GPS data over CAN
            }
            else if (strstr((char *)data, GPRMC) != NULL) {
                parse_gprmc((char *)data, &gps_data);
                ESP_LOGI(TAG, "Latitude: %.6f, Longitude: %.6f", gps_data.latitude, gps_data.longitude);
                send_data_to_ecu(&gps_data);  // Send parsed GPS data over CAN
            }
        }
    }
}

void app_main() {
    // Initialize CAN
    can_init();

    // Create the GPS task
    xTaskCreate(gps_task, "gps_task", 4096, NULL, 10, NULL);
}
