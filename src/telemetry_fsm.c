/**
 * @file    telemetry_fsm.c
 * @author  Yigit Cakirdogan
 * @brief   Implementation of IoT telemetry FSM, offline spooling, and CRC32 codec.
 */

#include "telemetry_fsm.h"
#include <stdio.h>
#include <string.h>

#define BACKOFF_INITIAL_SEC 1UL
#define BACKOFF_MAX_SEC     60UL

uint32_t telemetry_crc32_compute(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFFUL;
    if (data == NULL) {
        return 0UL;
    }

    for (size_t i = 0U; i < length; i++) {
        crc ^= (uint32_t)data[i];
        for (uint8_t b = 0U; b < 8U; b++) {
            if ((crc & 1UL) != 0UL) {
                crc = (crc >> 1U) ^ 0xEDB88320UL;
            } else {
                crc >>= 1U;
            }
        }
    }
    return ~crc;
}

void telemetry_packet_seal(telemetry_packet_t *pkt)
{
    if (pkt == NULL) {
        return;
    }
    size_t payload_bytes = offsetof(telemetry_packet_t, crc32);
    pkt->crc32 = telemetry_crc32_compute((const uint8_t *)pkt, payload_bytes);
}

bool telemetry_packet_verify(const telemetry_packet_t *pkt)
{
    if (pkt == NULL) {
        return false;
    }
    size_t payload_bytes = offsetof(telemetry_packet_t, crc32);
    uint32_t expected    = telemetry_crc32_compute((const uint8_t *)pkt, payload_bytes);
    return (expected == pkt->crc32);
}

int telemetry_packet_to_json(const telemetry_packet_t *pkt, char *out_buf, size_t buf_size)
{
    if ((pkt == NULL) || (out_buf == NULL) || (buf_size == 0U)) {
        return -1;
    }

    int temp_whole = pkt->temperature_centi_c / 100;
    int temp_frac  = pkt->temperature_centi_c % 100;
    if (temp_frac < 0) {
        temp_frac = -temp_frac;
    }

    return snprintf(out_buf, buf_size,
                    "{\"seq\":%lu,\"ts\":%lu,\"temp_c\":%d.%02d,\"press_mbar\":%u,"
                    "\"vib_mg\":%u,\"batt_mv\":%u,\"crc32\":\"0x%08lX\"}",
                    (unsigned long)pkt->sequence_id,
                    (unsigned long)pkt->timestamp_sec,
                    temp_whole, temp_frac,
                    (unsigned)pkt->pressure_mbar,
                    (unsigned)pkt->vibration_rms_mg,
                    (unsigned)pkt->battery_mv,
                    (unsigned long)pkt->crc32);
}

static void spool_push(telemetry_node_t *node, const telemetry_packet_t *pkt)
{
    node->spool[node->spool_head] = *pkt;
    node->spool_head = (node->spool_head + 1U) % TELEMETRY_OFFLINE_SPOOL_CAPACITY;

    if (node->spool_count < TELEMETRY_OFFLINE_SPOOL_CAPACITY) {
        node->spool_count++;
    } else {
        /* Overwrite oldest record when full */
        node->spool_tail = (node->spool_tail + 1U) % TELEMETRY_OFFLINE_SPOOL_CAPACITY;
    }
    node->spooled_count++;
}

static bool spool_pop(telemetry_node_t *node, telemetry_packet_t *out_pkt)
{
    if (node->spool_count == 0U) {
        return false;
    }

    *out_pkt         = node->spool[node->spool_tail];
    node->spool_tail = (node->spool_tail + 1U) % TELEMETRY_OFFLINE_SPOOL_CAPACITY;
    node->spool_count--;
    return true;
}

void telemetry_node_init(telemetry_node_t *node)
{
    if (node == NULL) {
        return;
    }
    memset(node, 0, sizeof(*node));
    node->state           = TELEM_STATE_BOOT_INIT;
    node->backoff_seconds = BACKOFF_INITIAL_SEC;
}

telemetry_state_t telemetry_node_step(telemetry_node_t *node)
{
    if (node == NULL) {
        return TELEM_STATE_BOOT_INIT;
    }

    switch (node->state) {
    case TELEM_STATE_BOOT_INIT:
        node->state = TELEM_STATE_SENSOR_ACQUIRE;
        break;

    case TELEM_STATE_SENSOR_ACQUIRE:
        node->active_packet.sequence_id = ++node->sequence_counter;
        telemetry_packet_seal(&node->active_packet);
        node->state = TELEM_STATE_WIFI_CONNECT;
        break;

    case TELEM_STATE_WIFI_CONNECT:
        if (node->wifi_connected) {
            node->backoff_seconds = BACKOFF_INITIAL_SEC;
            node->state           = TELEM_STATE_MQTT_TLS_PUBLISH;
        } else {
            if (node->backoff_seconds < BACKOFF_MAX_SEC) {
                node->backoff_seconds *= 2UL;
                if (node->backoff_seconds > BACKOFF_MAX_SEC) {
                    node->backoff_seconds = BACKOFF_MAX_SEC;
                }
            }
            node->state = TELEM_STATE_OFFLINE_SPOOL;
        }
        break;

    case TELEM_STATE_MQTT_TLS_PUBLISH:
        if (node->mqtt_broker_ack && telemetry_packet_verify(&node->active_packet)) {
            node->published_count++;
            if (node->spool_count > 0U) {
                node->state = TELEM_STATE_DRAIN_OFFLINE_SPOOL;
            } else {
                node->state = TELEM_STATE_DEEP_SLEEP;
            }
        } else {
            node->state = TELEM_STATE_OFFLINE_SPOOL;
        }
        break;

    case TELEM_STATE_DRAIN_OFFLINE_SPOOL: {
        telemetry_packet_t cached;
        while (spool_pop(node, &cached)) {
            if (telemetry_packet_verify(&cached)) {
                node->published_count++;
            }
        }
        node->state = TELEM_STATE_DEEP_SLEEP;
        break;
    }

    case TELEM_STATE_OFFLINE_SPOOL:
        spool_push(node, &node->active_packet);
        node->state = TELEM_STATE_DEEP_SLEEP;
        break;

    case TELEM_STATE_DEEP_SLEEP:
        node->state = TELEM_STATE_SENSOR_ACQUIRE;
        break;
    }

    return node->state;
}
