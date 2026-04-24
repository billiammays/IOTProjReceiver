# Differential Pitch-Yaw Stepper Controller

ESP32-C6 firmware controlling two NEMA17-34 stepper motors (A4988 drivers) in a differential pitch-yaw configuration, with cloud control via ESP RainMaker.

## Hardware

| Signal | GPIO |
|--------|------|
| STEP 1 (RMT) | 23 |
| DIR 1 | 15 |
| STEP 2 (esp_timer) | 20 |
| DIR 2 | 21 |

Both motors are driven simultaneously. Motor 1 uses the RMT peripheral; motor 2 uses an esp_timer ISR (the ESP32-C6 has one RMT TX channel).

**Differential kinematics:**
```
m1 = pitch + yaw
m2 = pitch - yaw   (DIR2 inverted for opposing physical mount)
```

## Cloud Control

Wi-Fi is provisioned over BLE on first boot using the [ESP RainMaker](https://rainmaker.espressif.com) mobile app. After connecting, write an integer to the `mode` parameter to command motion:

| Value | Action |
|-------|--------|
| 0 | Stop |
| 1 | Yaw left |
| 2 | Yaw right |
| 3 | Pitch up |
| 4 | Pitch down |

Motion is continuous until a new mode is received. Mode changes take effect within one step cycle (~16 ms).

## Build and Flash

```bash
idf.py set-target esp32c6
idf.py build
idf.py flash monitor
```

On first flash, use `idf.py erase-flash` beforehand to clear provisioning state.

## Documentation

Doxygen documentation is configured in `Doxyfile`. Generate with:

```bash
doxygen Doxyfile
```

Output: `html/index.html` and `latex/refman.tex`.
