#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "mcp2515.h"

// Constants
#define WHEEL_DIAMETER   0.7    // Diameter of the wheel in meters (e.g., 70 cm wheel)
#define PULSES_PER_REV   20     // Number of pulses per wheel revolution (e.g., from a Hall effect sensor)
#define SECONDS_IN_MINUTE 60
#define HALL_SENSOR_PIN   GPIO_NUM_18  // Hall Effect sensor input pin

// CAN configuration
#define CAN_SPEED CAN_500KBPS
#define CAN_TX_PIN 5
#define CAN_RX_PIN 4
#define MCP2515_CS_PIN 15  // SPI CS pin for MCP2515

// Global variables
uint32_t rpm = 0;               // RPM (Revolutions per minute)
uint32_t pulse_count = 0;       // Count of pulses detected in a fixed time window
float vehicle_speed = 0.0;      // Speed in meters per second (m/s)
float distance_traveled = 0.0;  // Distance traveled in meters
uint32_t last_time = 0;         // Last time (in seconds) for calculating RPM
uint32_t current_time = 0;      // Current time (in seconds)

SemaphoreHandle_t sensor_mutex;  // Mutex to ensure safe access to shared resources
MCP2515 mcp2515(&SPI, MCP2515_CS_PIN);  // Initialize MCP2515 with SPI

// Structure to send CAN messages
struct can_frame frame;


// Function to handle pulse detection from the Hall Effect sensor (ISR or interrupt handler)
void IRAM_ATTR pulse_detected_isr() {
    pulse_count++;  // Increment pulse count on each interrupt
}

// Function to calculate RPM from pulse count over a fixed time window
void calculate_rpm() {
    uint32_t current_time = esp_timer_get_time();  // Get current time in seconds

    // Calculate pulses in the last time window
    uint32_t pulses_in_window = pulse_count;

    // Calculate RPM: (pulses_in_window / time_window) * 60
    rpm = (pulses_in_window * SECONDS_IN_MINUTE) / (current_time - last_time);

    // Reset pulse count after RPM calculation
    pulse_count = 0;

    last_time = current_time;  // Update last time for the next window
}

// Function to calculate speed based on RPM and wheel circumference
void calculate_speed() {
    // Calculate wheel circumference: C = π * diameter
    float wheel_circumference = M_PI * WHEEL_DIAMETER;  // Circumference in meters

    // Calculate speed: Speed = (Circumference * RPM) / 60 (to get speed in meters per second)
    vehicle_speed = (wheel_circumference * rpm) / SECONDS_IN_MINUTE;

    // Print speed (in meters per second)
    printf("Vehicle Speed: %.2f m/s\n", vehicle_speed);
}

// Function to calculate distance traveled
void calculate_distance() {
    // Calculate distance = speed * time (assuming time is in seconds)
    distance_traveled += vehicle_speed;
    printf("Distance Traveled: %.2f meters\n", distance_traveled);
}

// Setup GPIO for the Hall Effect sensor
void hall_sensor_init() {
    gpio_pad_select_gpio(HALL_SENSOR_PIN);
    gpio_set_direction(HALL_SENSOR_PIN, GPIO_MODE_INPUT);
    gpio_set_intr_type(HALL_SENSOR_PIN, GPIO_INTR_POSEDGE);  // Interrupt on rising edge
    gpio_intr_enable(HALL_SENSOR_PIN);
    esp_intr_alloc(ETS_GPIO_INTR_SOURCE, ESP_INTR_FLAG_LEVEL1, pulse_detected_isr, NULL, NULL);  // Attach ISR to handle interrupts
}

// CAN initialization
void can_init() {
    mcp2515.reset();
    mcp2515.setBitrate(CAN_SPEED);  // Set the desired CAN speed
    mcp2515.setNormalMode();        // Set MCP2515 to normal mode
    ESP_LOGI("CAN", "CAN Initialized");
}

// Send CAN message with speed data
void send_data_to_ecu(int speed) {
    frame.can_id = 0x55;  
    frame.can_dlc = 2;     // Data length (2 bytes for speed)
    frame.data[0] = (speed >> 8) & 0xFF;  // High byte of speed
    frame.data[1] = speed & 0xFF;         // Low byte of speed

    // Send the CAN frame with the speed data
    mcp2515.sendMessage(&frame);  // Send the message over the CAN bus
    ESP_LOGI("CAN", "Sent Speed: %d km/h", speed);
}

// Speed Sensor Task
void speed_sensor_task(void *pvParameters) {
    hall_sensor_init();  // Initialize the Hall Effect sensor

    // Main loop
    while (1) {
        // Simulate time passing (or use a timer to track actual time)
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Wait for 1 second
        calculate_rpm();  // Calculate RPM based on pulse count over 1 second
        calculate_speed();  // Calculate vehicle speed in m/s
        calculate_distance();  // Calculate distance traveled in meters
    }
}

// CAN Communication Task
void can_task(void *pvParameters) {
    while (1) {
        if (xSemaphoreTake(sensor_mutex, portMAX_DELAY) == pdTRUE) {
            int speed_kmh = (int)(vehicle_speed * 3.6);  // Convert speed to km/h
            ESP_LOGI("Speed Sensor", "Sending Speed: %d km/h", speed_kmh);
            send_data_to_ecu(speed_kmh);  // Send the speed data via CAN
            xSemaphoreGive(sensor_mutex);  // Release the mutex after use
        }
        vTaskDelay(pdMS_TO_TICKS(1000));  // Delay for 1 second before reading again
    }
}

void app_main() {
    ESP_LOGI("Speed Sensor", "Starting Speed Sensor...");

    // Initialize CAN
    can_init();

    // Create a mutex for sensor data protection
    sensor_mutex = xSemaphoreCreateMutex();
    if (sensor_mutex == NULL) {
        ESP_LOGE("Speed Sensor", "Failed to create mutex!");
        return;
    }

    // Create tasks for pulse detection, RPM calculation, and sending data to ECU
    xTaskCreate(speed_sensor_task, "speed_sensor_task", 2048, NULL, 10, NULL);
    xTaskCreate(can_task, "can_task", 2048, NULL, 5, NULL);
}
