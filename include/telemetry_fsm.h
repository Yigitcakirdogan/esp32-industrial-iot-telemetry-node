/**
 * @file    telemetry_fsm.h
 * @author  Yigit Cakirdogan
 * @brief   Deterministic IoT Telemetry Finite State Machine, exponential backoff,
 *          offline circular flash spooling, and CRC32 packet integrity.
 */

#ifndef TELEMETRY_FSM_H
#define TELEMETRY_FSM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define TELEMETRY_OFFLINE_SPOOL_CAPACITY 16U

typedef enum {
    TELEM_STATE_BOOT_INIT = 0,
    TELEM_STATE_SENSOR_ACQUIRE,
    TELEM_STATE_WIFI_CONNECT,
    TELEM_STATE_MQTT_TLS_PUBLISH,
    TELEM_STATE_DRAIN_OFFLINE_SPOOL,
    TELEM_STATE_OFFLINE_SPOOL,
    TELEM_STATE_DEEP_SLEEP
} telemetry_state_t;

/**
 * @brief Compact 24-byte binary telemetry record protected by IEEE 802.3 CRC32.
 */
typedef struct {
    uint32_t sequence_id;
    uint32_t timestamp_sec;
    int16_t  temperature_centi_c;   /**< Temperature in 0.01 C (e.g. 2450 = 24.50 C) */
    uint16_t pressure_mbar;         /**< Pressure in mbar                            */
    uint16_t vibration_rms_mg;      /**< Vibration RMS in milli-g                    */
    uint16_t battery_mv;            /**< Supply voltage in mV                        */
    uint32_t status_flags;
    uint32_t crc32;                 /**< IEEE 802.3 CRC-32 over preceding 20 bytes   */
} telemetry_packet_t;

typedef struct {
    telemetry_state_t  state;
    uint32_t           sequence_counter;
    uint32_t           backoff_seconds;
    uint32_t           published_count;
    uint32_t           spooled_count;
    telemetry_packet_t active_packet;
    telemetry_packet_t spool[TELEMETRY_OFFLINE_SPOOL_CAPACITY];
    size_t             spool_head;
    size_t             spool_tail;
    size_t             spool_count;
    bool               wifi_connected;
    bool               mqtt_broker_ack;
} telemetry_node_t;

/**
 * @brief Compute IEEE 802.3 CRC-32 checksum.
 */
uint32_t telemetry_crc32_compute(const uint8_t *data, size_t length);

/**
 * @brief Seal a telemetry packet by computing and storing its CRC-32 field.
 */
void telemetry_packet_seal(telemetry_packet_t *pkt);

/**
 * @brief Verify the CRC-32 integrity of a stored telemetry packet.
 */
bool telemetry_packet_verify(const telemetry_packet_t *pkt);

/**
 * @brief Serialize a verified telemetry packet into a static JSON string buffer.
 */
int telemetry_packet_to_json(const telemetry_packet_t *pkt, char *out_buf, size_t buf_size);

/**
 * @brief Initialize the telemetry node FSM and clear the offline spool.
 */
void telemetry_node_init(telemetry_node_t *node);

/**
 * @brief Execute one deterministic step of the telemetry node state machine.
 */
telemetry_state_t telemetry_node_step(telemetry_node_t *node);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_FSM_H */
