# REW&MS: River Early Warning & Monitoring System

<img width="512" height="374" alt="image" src="https://github.com/user-attachments/assets/aa00ee3f-04f3-4efe-81b7-2f60550d1c6f" />



**REW&MS** (River Early Warning & Monitoring System) is a drone-based, fully autonomous environmental monitoring platform designed to assess river health and predict flood risks in real-time. The system integrates an ARM-based Pixhawk Flight Control Unit (FCU) with an ARM-based STM32F030R8 microcontroller powering a comprehensive sensor pod. The architecture ensures efficient data collection, communication, and storage for reliable, long-term analysis.

---

## Features

- **Autonomous Drone Operation:**
  - Powered by Pixhawk FCU (ARM-based)
  - Supports Auto RTL (Return to Launch) & waypoint navigation
  - Compatible with PX4 and Ardupilot firmware

- **Comprehensive Sensor Pod:**
  - Water turbidity, detection, and gas sensors (MQ series) interfaced via 10-bit ADC channels
  - BMP280 sensor for barometric pressure and temperature via I2C
  - STM32F030R8 handles all sensor data acquisition & processing

- **Reliable Communication & Logging:**
  - HC-12 (433 MHz) & HC-05 (Bluetooth) modules for UART-based wireless transmission
  - Arduino microcontroller logs data to SD card for redundancy

---

## System Components

### 1. Flight Control Unit (FCU)

| Component | Specification |
|----------|--------------|
| **Pixhawk FCU** | ARM-based architecture |
|              | Auto RTL & waypoint navigation support |
|              | Compatible with PX4 or Ardupilot firmware |

---

### 2. Sensor Pod

| Sensor                 | Interface              | Function                                          |
|----------------------|------------------------|---------------------------------------------------|
| Water Turbidity Sensor | Analog (10-bit ADC)    | Measures water clarity                             |
| Water Detection Sensor | Analog (10-bit ADC)    | Detects presence of water                         |
| MQ-2 Gas Sensor        | Analog (10-bit ADC)    | Detects flammable gases & smoke                   |
| MQ-7 Gas Sensor        | Analog (10-bit ADC)    | Detects carbon monoxide                           |
| MQ-9 Gas Sensor        | Analog (10-bit ADC)    | Detects carbon monoxide & combustible gases       |
| BMP280                 | I2C                    | Measures barometric pressure & temperature        |

---

### 3. Communication & Data Logging

| Module                | Interface | Functionality                                          |
|----------------------|----------|-------------------------------------------------------|
| HC-12                 | UART     | Wireless communication at 433 MHz                      |
| HC-05                 | UART     | Bluetooth communication                               |
| Arduino + SD Card     | UART     | Logs data serially from STM32 for redundancy           |

---

### Microcontroller Specifications (STM32F030R8)

| Feature          | Specification                   |
|------------------|----------------------------------|
| Core             | ARM Cortex-M0                   |
| Flash Memory     | 64 KB                           |
| RAM              | 8 KB                            |
| ADC              | 12 channels, 10-bit resolution  |
| UART             | 1 (shared with comm modules + Arduino) |
| I2C              | 1 (used for BMP280)             |

---

## Quick Start Guide

### Pre-Flight Preparation

1. **Mission Planning:**
   - Define waypoints and upload mission to Pixhawk FCU using QGroundControl or Mission Planner.

2. **System Initialization:**
   - Power up the sensor pod, ensure sensor and communication module functionality.

---

### Flight Execution

1. **Autonomous Operation:**
   - Launch drone; follows waypoint mission autonomously while collecting environmental data.

2. **Data Acquisition:**
   - STM32 samples data from all connected sensors through ADC & I2C channels.

3. **Data Transmission:**
   - Data transmitted in real-time over HC-12/HC-05.
   - Data simultaneously logged by Arduino to SD card (black box functionality).

---

### Post-Flight Procedure

1. **Data Retrieval:**
   - Retrieve SD card from Arduino logger post-landing.

2. **Data Analysis:**
   - Analyze river health metrics and potential flood risks using the recorded data.

---

## Applications

- **Flood Prediction & Early Warning:**
  - Real-time data for proactive response to potential floods.

- **Water Quality Monitoring:**
  - Assess turbidity, gas concentrations, barometric data to evaluate river health.

- **Environmental Research:**
  - Enables large-scale ecological and hydrological data collection.

- **Infrastructure Planning:**
  - Supports water management and civil engineering projects.

---

## Images

![WhatsApp Image 2025-03-23 at 3 21 33 PM (2)](https://github.com/user-attachments/assets/2b13958d-abf3-4156-a200-42c2cca565a3)
![WhatsApp Image 2025-03-23 at 3 21 31 PM](https://github.com/user-attachments/assets/40d34247-2e08-49d3-bf15-8fee57c04a97)
![WhatsApp Image 2025-03-23 at 3 21 33 PM](https://github.com/user-attachments/assets/2b80c17b-aed8-442d-8e4e-126b76aa4f86)
![WhatsApp Image 2025-03-23 at 3 21 33 PM (1)](https://github.com/user-attachments/assets/8408b56d-690c-4a64-8eed-c8c4302ffb99)
![WhatsApp Image 2025-03-23 at 3 21 34 PM](https://github.com/user-attachments/assets/c23837fe-4320-40f0-a8e6-c27c81bf6774)
![WhatsApp Image 2025-03-23 at 3 21 37 PM](https://github.com/user-attachments/assets/a9028501-8111-46d6-a37e-22dad1bc2e1b)
![WhatsApp Image 2025-03-23 at 3 21 39 PM](https://github.com/user-attachments/assets/8ea2778a-70dc-4b05-be1c-24ad7ad949ff)


This project is licensed under the MIT License. See `LICENSE` file for details.

---

## Contact

For inquiries or collaborations, feel free to open an issue or reach out to the maintainers!
