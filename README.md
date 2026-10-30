## 👥 Project Team

- **Ayush Agnihotri** — Hardware and Backend
- **Shaury Sharma** — software and Frontend https://www.linkedin.com/in/shaury-sharma-056642446/

### Repository
This repository contains the HelioCore project firmware, dashboard, and documentation.# HelioCore SatHealth — ESP32 Environmental, Fire/Gas Warning & MPU6500 Dashboard

## 1. Project overview

HelioCore SatHealth is a local web dashboard hosted by an ESP32. The firmware reads environmental, proximity, flame, gas/smoke, and inertial measurements and exposes them to a browser as JSON. The dashboard presents live sensor values, warning states, event logs, and a CSS-based 3D attitude/orientation visualization for the MPU6500.

The project is designed to connect to an existing Wi-Fi network in **station mode**. It does not intentionally start an ESP32 access-point/hotspot. The dashboard HTML/CSS/JavaScript is embedded in `index_html.h` and served directly from flash using `PROGMEM`.

> **Source-grounded note:** This README describes the supplied `ayushagnihotri_fixed.ino` and `index_html.h`. Hardware behavior and compilation should still be verified on the actual board. Thresholds and sensor module output polarity may need calibration for the specific modules used.

## 2. Main files

| File | Purpose |
|---|---|
| `ayushagnihotri.ino` | ESP32 firmware, sensor polling, MPU6500 setup/calibration, warning logic, Wi-Fi, and HTTP endpoints. |
| `index_html.h` | Embedded dashboard document (HTML, CSS, JavaScript and background image data). Defines `INDEX_HTML` in program memory. |
| `README.md` | Project overview, wiring, setup, endpoints, timing, and troubleshooting. |

Keep `ayushagnihotri.ino` and `index_html.h` in the **same Arduino sketch folder**. The header name must match the include exactly: `#include "index_html.h"`.

## 3. Hardware and pin mapping

The pin assignments below are the ones defined in the supplied firmware.

| Module / signal | ESP32 pin | Firmware configuration | Notes |
|---|---:|---|---|
| DHT22 / AM2302 data | GPIO 26 | `DHT_PIN 26`, `DHT_TYPE DHT22` | Reads temperature and relative humidity. |
| HC-SR04 trigger | GPIO 5 | `TRIG_PIN 5` | Sends the ultrasonic trigger pulse. |
| HC-SR04 echo | GPIO 18 | `ECHO_PIN 18` | Receives echo pulse; check voltage compatibility before connecting. |
| Flame detector digital OUT | GPIO 33 | `FLAME_PIN 33` | Active-low is configured by default. Set `FLAME_ACTIVE_LOW` to `0` for an active-high module. |
| MQ-2 analog output (AO) | GPIO 34 | `MQ2_PIN 34` | Reads 12-bit ADC raw value. GPIO34 is input-only and is on ADC1. |
| MPU6500 SDA | GPIO 21 | `SDA_PIN 21` | I²C data. |
| MPU6500 SCL | GPIO 22 | `SCL_PIN 22` | I²C clock; configured for 400 kHz. |
| Warning LED | GPIO 14 | `LED_WARNING 14` | One shared LED output: major warning = steady ON; minor warning = blinking; no warning = OFF. Use a suitable series resistor with a bare LED. |

### Wiring and power cautions

- Connect all module grounds to ESP32 GND so signals share a common reference.
- Power the MPU6500 module according to its breakout-board specification; 3.3 V logic is the safe default for ESP32 I²C signals.
- **Do not connect a 5 V HC-SR04 ECHO signal directly to an ESP32 GPIO.** Use a suitable voltage divider or level shifter.
- **Do not feed a 5 V MQ-2 analog output directly into GPIO34.** Use a voltage divider/conditioning circuit so the ESP32 input remains within its allowed voltage range. The firmware comment explicitly calls this out.
- MQ-2 modules require a warm-up/stabilization period for useful measurements. The current code sets `MQ2_STARTUP_WARMUP_MS` to `0`, so the dashboard's `mq2Ready` flag does not enforce a real warm-up delay. Calibrate thresholds only after the physical sensor has stabilized.
- Confirm the exact pinout of each breakout board before powering it. Do not assume all flame or IMU modules have identical pin labels.

## 4. Software requirements

### Firmware environment

- Arduino IDE or a compatible Arduino build environment.
- ESP32 board support package installed and the correct ESP32 board/serial port selected.
- Libraries used by the sketch:
  - `WiFi.h` (ESP32 core)
  - `WiFiClient.h` (ESP32 core)
  - `WebServer.h` (ESP32 core)
  - `ESPmDNS.h` (ESP32 core)
  - `Wire.h` (I²C; ESP32 core)
  - `DHT.h` (Adafruit DHT sensor library, plus its required dependency if requested by Library Manager)
- No external web CDN is required by the dashboard; its visual layout and 3D model are bundled into the HTML.

### Network configuration

In the firmware, locate the Wi-Fi configuration near the top:

```cpp
const char* wifiSSID = "YOUR_WIFI_NAME";
const char* wifiPassword = "YOUR_WIFI_PASSWORD";
```

Replace these placeholders with the local Wi-Fi network credentials before uploading. Do not publish real credentials in a public repository or screenshot. The ESP32 uses `WIFI_STA`, tries to connect for up to 15 seconds during setup, and retries a lost connection in the background. No AP/hotspot is started by this code.

When connected, the Serial Monitor prints the device IP address and attempts to advertise `http://heliocore.local` using mDNS. If mDNS does not resolve on your device/network, open the IP printed in Serial Monitor instead.

## 5. Sensor sampling and responsiveness

The firmware separates fast sensors from the slower DHT22 cycle to avoid delaying every reading until the temperature/humidity sensor is ready.

| Subsystem | Configured timing / method | Details |
|---|---|---|
| DHT22 | 2,000 ms | `READ_INTERVAL`; DHT22 should not be polled faster than about every 2 seconds. |
| HC-SR04 | 150 ms | `DISTANCE_SAMPLE_INTERVAL_MS`; `pulseIn()` uses a 15,000 µs timeout. A timeout produces an invalid (`NaN`) distance rather than a fabricated value. |
| MPU6500 | 10 ms target | `MPU_SAMPLE_INTERVAL_MS`, approximately 100 Hz. I²C runs at 400 kHz. |
| MQ-2 | 100 ms | `MQ2_SAMPLE_INTERVAL_MS`; each sample averages 8 analog reads with a 250 µs pause between reads. |
| Flame input | 50 ms | `FLAME_SAMPLE_INTERVAL`; two matching samples are used for basic confirmation/debounce. |
| Browser dashboard | 250 ms | `POLL_MS` in the dashboard JavaScript; HTTP request timeout is 1,500 ms. The UI can therefore refresh faster than the DHT22 produces new measurements. |
| Stale data | 3 × DHT interval | `STALE_MS` is 6,000 ms. Missing, invalid, or stale numeric values are returned as JSON `null`. |

These are configured targets, not guarantees of end-to-end latency. Browser scheduling, Wi-Fi quality, server handling, sensor response time, and blocking hardware calls can affect observed timing.

## 6. MPU6500 details

- The firmware probes I²C addresses `0x68` and `0x69` and reads the `WHO_AM_I` register (`0x75`) for diagnostics.
- It configures the accelerometer range to ±2 g and gyro range to ±250 degrees/second.
- Raw acceleration is converted to m/s² using 16,384 LSB/g and standard gravity `9.80665 m/s²`.
- Raw gyro data is converted using 131 LSB/(degree/second), subtracts calibrated gyro bias, then converts angular velocity to rad/s for the dashboard.
- The firmware samples 6-axis data and calculates roll and pitch using a complementary filter combining gyro integration and accelerometer-derived angles. Yaw is integrated from the Z-axis gyro and is subject to drift over time.
- Startup gyro calibration uses 500 attempted samples spaced about 4,000 µs apart. Keep the IMU stationary during calibration. The firmware accepts calibration when at least 400 valid samples are available.
- The dashboard includes a calibration action that sends a POST request to `/calibrate`. Keep the board still while it recalibrates.
- If initialization or a read fails, diagnostic messages are printed to Serial Monitor. The code retries MPU initialization approximately every 5 seconds when the device is unavailable.

## 7. Warning logic and thresholds

### Proximity and gas/fire warning priority

The firmware determines warning level as follows:

1. **Major:** flame sensor detects a flame **or** a valid distance is at or below **15 cm**.
2. **Minor:** no major warning is active, and either the MQ-2 reading is at/above its warning threshold or a valid distance is at or below **50 cm**.
3. **Normal:** neither major nor minor condition is active.

Major warnings take priority over minor warnings. The firmware drives GPIO14 accordingly:

- Major warning: LED is continuously ON.
- Minor warning: LED toggles at a 500 ms interval.
- No warning: LED is OFF.

The dashboard also shows alert banners and records warning transitions in an in-session event log. The dashboard event log is held in browser memory and is limited to the most recent 100 events; it is not persistent storage on the ESP32.

### MQ-2 calibration values

| Setting | Current value | Meaning |
|---|---:|---|
| `MQ2_WARNING_THRESHOLD` | 2300 | Raw 12-bit ADC reading at or above which the warning activates. |
| `MQ2_CLEAR_THRESHOLD` | 2200 | Reading must fall below this to clear the warning (hysteresis helps reduce flicker). |
| `MQ2_SAMPLE_INTERVAL_MS` | 100 ms | Sampling interval. |
| `MQ2_STARTUP_WARMUP_MS` | 0 ms | Software warm-up flag delay; this does not mean the physical MQ-2 is warmed up. |

ADC values are raw relative readings, not a calibrated gas concentration in ppm. Tune thresholds using the actual module, supply voltage, divider, baseline environment, and sensor warm-up. MQ-2 response depends on the target gas, module circuitry, temperature, humidity, and sensor age. Do not treat this prototype as a certified life-safety gas detector.

### Flame detector polarity

`FLAME_ACTIVE_LOW` is set to `1` by default, which means LOW is interpreted as flame detected. If the particular module asserts HIGH during detection, change the macro to `0`, recompile, and retest both flame-present and flame-absent states safely.

## 8. HTTP API

The ESP32 serves these endpoints on port 80:

| Method | Path | Purpose |
|---|---|---|
| `GET` | `/` | Serves the embedded dashboard from `INDEX_HTML` in program memory. |
| `GET` | `/data` | Returns live telemetry as JSON with `Cache-Control: no-store`. |
| `POST` | `/calibrate` | Requests MPU6500 gyro bias calibration. Returns HTTP 503 if the MPU is unavailable, HTTP 409 if calibration is already in progress, or HTTP 202 when queued. |
| `GET` | `/health` | Returns a small health response identifying the device as `HelioCore-CubeWays`. |

### `/data` JSON fields

The response includes the following fields (numeric values may be `null` when unavailable or stale):

- `temperature`, `humidity`: DHT22 readings.
- `distance`: ultrasonic distance in centimetres.
- `speedCms`: estimated closing/incoming speed in cm/s based on change between distance samples; this is a simple estimate and can be zero when the distance is not decreasing meaningfully.
- `mq2Raw`, `mq2Warning`, `mq2Ready`: raw MQ-2 ADC reading and warning/readiness flags.
- `fireDetected`: flame detector state.
- `accelX`, `accelY`, `accelZ`: acceleration in m/s².
- `gyroX`, `gyroY`, `gyroZ`: angular velocity in rad/s.
- `mpuTemperature`: temperature value read from the MPU6500's internal temperature register.
- `roll`, `pitch`, `yaw`: orientation angles in degrees relative to the stored zero reference, wrapped to the usual signed angle range.
- `gyroCalibrated`, `calibrationBusy`, `calibrationFailed`, `mpuOk`: calibration and IMU status flags.
- `sampleAgeMs`: age of the most recent `readSensors()` completion, or `-1` if not yet set.
- `uptimeMs`: milliseconds since ESP32 boot.

## 9. Dashboard capabilities

The bundled dashboard includes:

- Live sensor cards and trend charts.
- MPU6500 six-axis values and a CSS 3D attitude/spacecraft visualization.
- Roll, pitch, and yaw orientation readouts and a gyro calibration control.
- Prominent onboard fire hazard and MQ-2 warning banners.
- Major/minor warning state presentation and event logs.
- Live and demo display modes; demo mode uses simulated values in the browser, while live mode fetches `/data` from the ESP32.
- Responsive layout for desktop and smaller screens.

The 3D spacecraft is a dashboard visualization, not a physical spacecraft model measurement. Its displayed orientation is derived from the IMU values supplied by the firmware.

## 10. Installation and first run

1. Create/open the Arduino sketch folder named `ayushagnihotri` (or use the folder matching the `.ino` filename).
2. Place `ayushagnihotri.ino` and `index_html.h` together in that folder.
3. Install the ESP32 board package and the DHT library.
4. Set `wifiSSID` and `wifiPassword` to your local network credentials.
5. Connect the hardware according to the pin table and verify safe signal voltage levels.
6. In Arduino IDE, select the correct ESP32 board and serial port.
7. Compile/Verify the sketch. Resolve any missing library errors before upload.
8. Upload the firmware and open Serial Monitor at **115200 baud**.
9. Watch for I²C scan results, MPU6500 initialization/calibration messages, Wi-Fi connection, and the dashboard IP.
10. Open `http://<ESP32-IP>/` from a device on the same Wi-Fi network. You can also try `http://heliocore.local/` if mDNS works on the network.
11. Test each sensor independently, then test warning thresholds in a controlled and safe environment.

## 11. Troubleshooting

### `unterminated #ifndef` or `index_html.h` compilation error

- Confirm `index_html.h` contains the entire file, not a partial copy.
- The file should begin with `#ifndef INDEX_HTML_H` and end with `#endif`.
- Keep the final HTML string terminator (`</script></body></html>";`) before `#endif`.
- Do not paste a filesystem path such as `/Users/.../Downloads/index_html.h` into the `.ino` source.

### Dashboard does not open

- Confirm ESP32 and browser device are on the same network.
- Read the IP address from Serial Monitor; try that IP directly.
- Check Wi-Fi SSID/password and router client isolation/firewall settings.
- The code does not create its own hotspot; it needs an existing Wi-Fi network.

### MPU6500 not detected or gyro values are null

- Check 3V3, GND, SDA→GPIO21, and SCL→GPIO22.
- Check that the module is compatible with the supply and logic levels.
- Inspect Serial Monitor for I²C ACKs at `0x68` or `0x69` and `WHO_AM_I` output.
- Keep the board stationary during calibration. Yaw drift is normal for gyro integration without a magnetometer/reference correction.

### Distance reading is missing or unstable

- Confirm TRIG→GPIO5 and ECHO→GPIO18.
- Ensure ECHO voltage is safely reduced to ESP32 logic level.
- Check common ground and sensor supply.
- Out-of-range echoes time out after 15 ms and are treated as invalid, so the dashboard may show no current distance.

### MQ-2 warning is always on or never activates

- Confirm the module's AO is connected through suitable voltage conditioning to GPIO34.
- Observe raw `mq2Raw` values after sensor stabilization.
- Adjust the 2300 warning and 2200 clear thresholds based on observed readings; avoid setting the clear threshold above the warning threshold.
- Remember that the current code's startup warm-up flag is set to zero and is not a real sensor warm-up routine.

### Flame state appears inverted

- Check the module's digital output behavior.
- Set `FLAME_ACTIVE_LOW` to `1` if detection pulls OUT LOW, or `0` if detection drives OUT HIGH.
- Verify both clear and detected states after uploading.

### Slow or stale display

- The browser polls every 250 ms, but DHT22 readings update only every 2 seconds by design.
- Distance is sampled about every 150 ms and MQ-2 about every 100 ms in the firmware loop; actual browser display latency also depends on Wi-Fi and request handling.
- Inspect Serial Monitor for sensor errors and use `/health` and `/data` to distinguish a network problem from a sensor problem.

## 12. Known limitations

- This is a prototype monitoring dashboard, not a certified fire, gas, or safety system.
- MQ-2 values are raw ADC counts and are not calibrated ppm measurements.
- Gyroscope-integrated yaw drifts over time; no magnetometer fusion is implemented here.
- The browser event log is session-only and is not written to a database or persistent file.
- The firmware includes a simple distance-derived speed estimate; it is not a full object-tracking algorithm.
- Sensor thresholds, polarity, voltage conditioning, and calibration must be validated against the exact hardware.
- The current firmware source contains local Wi-Fi configuration values. Replace them with your own credentials and avoid sharing real credentials publicly.

## 13. Suggested future improvements

- Add a configurable, documented MQ-2 stabilization period and baseline calibration routine.
- Persist event logs to storage or a backend if historical records are required.
- Add sensor self-test/status reporting for DHT22, HC-SR04, flame input, and MQ-2.
- Consider complementary/madgwick-style sensor fusion and a heading reference if stable yaw is required.
- Move Wi-Fi credentials to a separate untracked configuration file or provisioning workflow before publishing the project.
- Validate end-to-end update latency with timestamps on sensor sampling, JSON response, and browser rendering.

---

**Project:** HelioCore SatHealth  
**Device identity returned by `/health`:** `HelioCore-CubeWays`  
**Dashboard host name (mDNS, when available):** `heliocore.local`  
**Serial Monitor baud rate:** `115200`
