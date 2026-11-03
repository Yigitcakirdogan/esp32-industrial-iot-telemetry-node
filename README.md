# ESP32 Fault-Tolerant Industrial IoT Telemetry Node (`esp32-industrial-iot-telemetry-node`)

[![CI - Build & Unit Tests](https://github.com/Yigitcakirdogan/esp32-industrial-iot-telemetry-node/actions/workflows/ci.yml/badge.svg)](https://github.com/Yigitcakirdogan/esp32-industrial-iot-telemetry-node/actions/workflows/ci.yml)
![Language](https://img.shields.io/badge/Language-C11%20(ESP--IDF%20Compatible)-00599C?style=flat-square&logo=c)
![Target](https://img.shields.io/badge/SoC-ESP32%20%2F%20ESP32--S3-E7352C?style=flat-square&logo=espressif)
![DSP](https://img.shields.io/badge/DSP-Kalman%20%26%20Median--5%20Filter-blueviolet?style=flat-square)
![License](https://img.shields.io/badge/License-MIT-green?style=flat-square)

An industrial-grade **ESP32 telemetry node architecture** engineered for harsh factory and remote field environments where **ADC sensor noise**, **intermittent Wi-Fi/cellular connectivity**, and **strict battery/thermal budgets** must be handled deterministically.

---

## Architecture & Deterministic Finite State Machine (FSM)

```mermaid
stateDiagram-v2
    [*] --> STATE_BOOT_INIT
    STATE_BOOT_INIT --> STATE_SENSOR_ACQUIRE : Peripherals & NVS Ready
    STATE_SENSOR_ACQUIRE --> STATE_WIFI_CONNECT : Median-5 + 1D Kalman Filtered
    STATE_WIFI_CONNECT --> STATE_MQTT_TLS_PUBLISH : IP Acquired
    STATE_WIFI_CONNECT --> STATE_OFFLINE_SPOOL : Timeout -> Backoff
    STATE_MQTT_TLS_PUBLISH --> STATE_DRAIN_OFFLINE_SPOOL : Publish ACK + Spool > 0
    STATE_MQTT_TLS_PUBLISH --> STATE_DEEP_SLEEP : Publish ACK (Spool Empty)
    STATE_MQTT_TLS_PUBLISH --> STATE_OFFLINE_SPOOL : Broker Drop / TLS Error
    STATE_DRAIN_OFFLINE_SPOOL --> STATE_DEEP_SLEEP : Spool Flushed
    STATE_OFFLINE_SPOOL --> STATE_DEEP_SLEEP : Packet Saved to Flash Ring (CRC32)
    STATE_DEEP_SLEEP --> STATE_SENSOR_ACQUIRE : RTC Timer Wakeup
```

---

## Core Engineering Modules

1. **Two-Stage Industrial Sensor Conditioning (`sensor_filter.c`):**
   - **Stage 1 — Median-of-5 Impulse Filter:** Rejects single-sample electromagnetic interference (EMI) spikes caused by nearby VFD motor drives or relay switching on `4-20 mA` / `PT100` ADC lines without smearing step edges.
   - **Stage 2 — 1D Discrete Kalman Filter:** Tracks true physical state ($x_k$) while adapting Kalman gain ($K_k$) based on process variance ($Q$) and measurement noise covariance ($R$).
2. **Store-and-Forward Offline Resilience (`telemetry_fsm.c`):**
   - When Wi-Fi or the MQTT/TLS broker is unreachable, telemetry frames are serialized into a compact **24-byte binary frame protected by IEEE 802.3 CRC-32** and pushed into a non-volatile circular spool.
   - Implements **bounded exponential backoff (`1s -> 2s -> 4s ... -> 60s max`)** to prevent battery drain during prolonged outages.
3. **Zero-Malloc Binary & JSON Codec:**
   - Generates both compact binary frames (for LoRaWAN / NVS flash storage) and deterministic JSON payloads (for AWS IoT Core / Azure IoT Hub MQTT topics) using fixed static buffers.

---

## Power Budget Analysis (ESP32-S3 @ 60-Second Reporting Interval)

| Operating Phase | Duration | Average Current | Charge Consumed |
| :--- | :---: | :---: | :---: |
| **Sensor Excitation & 64x ADC Oversampling** | `15 ms` | `28 mA` | `0.42 mAs` |
| **Wi-Fi Fast-Connect (Cached BSSID/Channel) + TLS Publish** | `320 ms` | `115 mA` | `36.80 mAs` |
| **RTC Deep Sleep (`ULP` off, RTC RAM retained)** | `59.665 s` | `14 µA` | `0.83 mAs` |
| **Effective Average Current** | **60.0 s Cycle** | **~0.63 mA** | **>6 Months on 3000 mAh LiFePO4** |

---

## Build & Run Unit Tests

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

---

## License

Released under the [MIT License](LICENSE).
