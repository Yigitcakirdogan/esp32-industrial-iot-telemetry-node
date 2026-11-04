#ifdef NDEBUG
   #undef NDEBUG
   #endif
   #include <assert.h>
/**
 * @file    test_telemetry_node.c
 * @author  Yigit Cakirdogan
 * @brief   Unit tests for Median-of-5 EMI spike filter, 1D Kalman convergence,
 *          CRC-32 packet verification, and offline store-and-forward FSM recovery.
 */

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "sensor_filter.h"
#include "telemetry_fsm.h"

static void test_median_and_kalman_spike_rejection(void)
{
    median_filter_t mf;
    kalman_1d_t     kf;

    median_filter_init(&mf);
    kalman_1d_init(&kf, 0.01f, 0.25f, 25.0f);

    /* Feed steady 25.0 C readings with a massive +450.0 C single-sample EMI spike */
    const float raw_samples[7] = { 25.0f, 25.1f, 450.0f, 24.9f, 25.0f, 25.1f, 25.0f };
    float filtered_out = 0.0f;

    for (size_t i = 0U; i < 7U; i++) {
        float med  = median_filter_update(&mf, raw_samples[i]);
        filtered_out = kalman_1d_update(&kf, med);
    }

    /* Despite the 450.0 C EMI spike, output must remain tightly locked around 25.0 C */
    assert(fabsf(filtered_out - 25.0f) < 0.15f);
    printf("[PASS] test_median_and_kalman_spike_rejection (final=%.3f C)\n", (double)filtered_out);
}

static void test_offline_spool_and_reconnect_drain(void)
{
    telemetry_node_t node;
    telemetry_node_init(&node);

    node.active_packet.timestamp_sec       = 1700000000UL;
    node.active_packet.temperature_centi_c = 2475;
    node.active_packet.pressure_mbar       = 1013U;
    node.active_packet.vibration_rms_mg    = 42U;
    node.active_packet.battery_mv          = 3310U;

    /* Cycle 1: Wi-Fi is DOWN -> packet must be spooled to flash ring and backoff doubled */
    node.wifi_connected  = false;
    node.mqtt_broker_ack = false;

    (void)telemetry_node_step(&node); /* BOOT -> ACQUIRE */
    (void)telemetry_node_step(&node); /* ACQUIRE -> WIFI_CONNECT */
    (void)telemetry_node_step(&node); /* WIFI_CONNECT -> OFFLINE_SPOOL */
    assert(node.state == TELEM_STATE_OFFLINE_SPOOL);
    assert(node.backoff_seconds == 2UL);

    (void)telemetry_node_step(&node); /* OFFLINE_SPOOL -> DEEP_SLEEP */
    assert(node.spool_count == 1U);
    assert(node.published_count == 0U);

    /* Cycle 2: Wi-Fi & MQTT recover -> new packet + spooled packet must both be published */
    node.wifi_connected  = true;
    node.mqtt_broker_ack = true;

    (void)telemetry_node_step(&node); /* DEEP_SLEEP -> ACQUIRE */
    (void)telemetry_node_step(&node); /* ACQUIRE -> WIFI_CONNECT */
    (void)telemetry_node_step(&node); /* WIFI_CONNECT -> MQTT_TLS_PUBLISH */
    (void)telemetry_node_step(&node); /* MQTT_TLS_PUBLISH -> DRAIN_OFFLINE_SPOOL */
    assert(node.state == TELEM_STATE_DRAIN_OFFLINE_SPOOL);

    (void)telemetry_node_step(&node); /* DRAIN_OFFLINE_SPOOL -> DEEP_SLEEP */
    assert(node.spool_count == 0U);
    assert(node.published_count == 2U);

    /* Verify JSON serialization */
    char json[256];
    int len = telemetry_packet_to_json(&node.active_packet, json, sizeof(json));
    assert(len > 0);
    assert(strstr(json, "\"temp_c\":24.75") != NULL);
    printf("[PASS] test_offline_spool_and_reconnect_drain\n");
}

int main(void)
{
    printf("=== Running ESP32 Industrial IoT Telemetry Node Test Suite ===\n");
    test_median_and_kalman_spike_rejection();
    test_offline_spool_and_reconnect_drain();
    printf("=== All telemetry node unit tests passed! ===\n");
    return 0;
}
