**Smart Vehicle Tracking and Diagnostics System**

*Overview*

This project aims to design a Smart Vehicle Tracking and Diagnostics System that integrates real-time vehicle diagnostics, telemetry data monitoring, and remote accessibility. The system leverages a variety of sensors, controllers, and communication protocols to provide comprehensive insights into vehicle health and performance.

*Key Components*
Sensors:

* Battery Level Sensor: Monitors vehicle battery status.
* Tire Pressure Sensor (TPMS): Checks tire pressure levels.
* Collision Sensors: Detects collisions or nearby obstacles.
* GPS Module: Tracks the real-time location of the vehicle.
* Engine Temperature Sensor: Monitors engine temperature.
* Speed Sensor: Tracks vehicle speed to ensure compliance with speed limits.
* Brake Monitoring: Tracks brake status and functionality.
* ECU (Electronic Control Unit):

Primary controller based on ESP32 for data collection, processing, and communication. Interfaces with various sensors using CAN, MQTT, and Wi-Fi protocols.

*Communication Protocols:*

* CAN Bus: For in-vehicle communication between controllers and the ECU.
* MQTT: For remote data transmission to a cloud or mobile application.
* BLE: Enables mobile app connectivity for easy configuration.
* Wi-Fi: Supports remote monitoring and data transmission to cloud servers.

*Software Platform:*

FreeRTOS: Manages real-time tasks, prioritizing critical tasks like engine temperature monitoring and speed violation alerts.
