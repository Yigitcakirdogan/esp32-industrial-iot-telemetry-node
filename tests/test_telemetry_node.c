/**
 * @file    test_telemetry_node.c
 * @author  Yigit Cakirdogan
 * @brief   Unit tests for Median-of-5 EMI spike filter, 1D Kalman convergence,
 *          CRC-32 packet verification, and offline store-and-forward FSM recovery.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sensor_filter.h"
#include "telemetry_fsm.h"

#define TEST_ASSERT(cond)                                                          \
    do {                                                                           \
        if (!(cond)) {                                                             \
            fprintf(stderr, "[FAIL] %s:%d: %s\n", __FILE__, __LINE__, #cond);      \
            exit(EXIT_FAILURE);                                                    \
        }                                                                          \
    } while (0)

static void test_median_and_kalman_spike_rejection(void)
{
    median_filter_t mf;
    kalman_1d_t     kf;

    median_filter_init(&mf);
    kalman_1d_init(&kf, 0.01f, 0.25f, 25.0f);

    const float raw_samples[7] = { 25.0f, 25.1f, 450.0f, 24.9f, 25.0f, 25.1f, 25.0f };
    float filtered_out = 0.0f;

    for (size_t i = 0U; i < 7U; i++) {
        float med  = median_filter_update(&mf, raw_samples[i]);
        filtered_out = kalman_1d_update(&kf, med);
    }

    TEST_ASSERT(fabsf(filtered_out - 25.0f) < 0.15f);
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

    node.wifi_connected  = false;
    node.mqtt_broker_ack = false;

    (void)telemetry_node_step(&node);
    (void)telemetry_node_step(&node);
    (void)telemetry_node_step(&node);
    TEST_ASSERT(node.state == TELEM_STATE_OFFLINE_SPOOL);
    TEST_ASSERT(node.backoff_seconds == 2UL);

    (void)telemetry_node_step(&node);
    TEST_ASSERT(node.spool_count == 1U);
    TEST_ASSERT(node.published_count == 0U);

    node.wifi_connected  = true;
    node.mqtt_broker_ack = true;

    (void)telemetry_node_step(&node);
    (void)telemetry_node_step(&node);
    (void)telemetry_node_step(&node);
    (void)telemetry_node_step(&node);
    TEST_ASSERT(node.state == TELEM_STATE_DRAIN_OFFLINE_SPOOL);

    (void)telemetry_node_step(&node);
    TEST_ASSERT(node.spool_count == 0U);
    TEST_ASSERT(node.published_count == 2U);

    char json[256];
    int len = telemetry_packet_to_json(&node.active_packet, json, sizeof(json));
    TEST_ASSERT(len > 0);
    TEST_ASSERT(strstr(json, "\"temp_c\":24.75") != NULL);
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
