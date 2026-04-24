# ESP32-C6 Stepper Motor Controller (A4988)

This project drives a stepper motor using an A4988 driver and the ESP32-C6's RMT peripheral. It generates precise pulse trains on hardware to ensure smooth movement without CPU overhead.

## Wiring Diagram

| ESP32-C6 Pin | A4988 Pin | Description |
| :--- | :--- | :--- |
| **GPIO 4** | **STEP** | Step pulse input |
| **GPIO 5** | **DIR** | Direction control |
| **GND** | **GND** | Logic ground |
| **External VCC** | **VMOT** | Motor power (e.g. 12V) |
| **GND** | **GND (Motor)** | Motor power ground |

*Note: GPIO 20/21 were avoided as they are the default UART0 console pins.*

## Features
- **RMT v2 Driver**: Uses the latest ESP-IDF RMT driver for pulse generation.
- **Hardware Precision**: Pulse timing is handled by the RMT hardware.
- **No Acceleration**: Constant speed movement as requested.
- **Full Step**: Configured for standard full-step mode.

## Build and Flash

1.  **Set up ESP-IDF**: Ensure you have ESP-IDF v5.0 or later installed and sourced.
2.  **Set Target**:
    ```bash
    idf.py set-target esp32c6
    ```
3.  **Build**:
    ```bash
    idf.py build
    ```
4.  **Flash**:
    ```bash
    idf.py flash monitor
    ```

## Usage
The code will move the motor 200 steps forward, wait 1 second, move 200 steps backward, and repeat.
