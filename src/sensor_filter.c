/**
 * @file    sensor_filter.c
 * @author  Yigit Cakirdogan
 * @brief   Implementation of Median-of-5 impulse rejection and 1D Kalman Filter.
 */

#include "sensor_filter.h"
#include <string.h>

void kalman_1d_init(kalman_1d_t *kf, float q_process_var, float r_measure_var, float initial_estimate)
{
    if (kf == NULL) {
        return;
    }

    kf->q_process_var = (q_process_var > 0.0f) ? q_process_var : 1e-4f;
    kf->r_measure_var = (r_measure_var > 0.0f) ? r_measure_var : 1e-2f;
    kf->x_estimate    = initial_estimate;
    kf->p_error_cov   = 1.0f;
    kf->kalman_gain   = 0.0f;
    kf->initialized   = true;
}

float kalman_1d_update(kalman_1d_t *kf, float measurement)
{
    if (kf == NULL) {
        return measurement;
    }

    if (!kf->initialized) {
        kalman_1d_init(kf, 0.01f, 0.25f, measurement);
    }

    /* 1. Time Update (Prediction) */
    float p_pred = kf->p_error_cov + kf->q_process_var;

    /* 2. Measurement Update (Correction) */
    float k         = p_pred / (p_pred + kf->r_measure_var);
    kf->x_estimate  = kf->x_estimate + k * (measurement - kf->x_estimate);
    kf->p_error_cov = (1.0f - k) * p_pred;
    kf->kalman_gain = k;

    return kf->x_estimate;
}

void median_filter_init(median_filter_t *mf)
{
    if (mf == NULL) {
        return;
    }
    memset(mf, 0, sizeof(*mf));
}

float median_filter_update(median_filter_t *mf, float raw_sample)
{
    if (mf == NULL) {
        return raw_sample;
    }

    mf->history[mf->index] = raw_sample;
    mf->index = (uint8_t)((mf->index + 1U) % MEDIAN_WINDOW_SIZE);
    if (mf->count < MEDIAN_WINDOW_SIZE) {
        mf->count++;
    }

    /* Copy active window and insertion-sort (<= 5 elements, zero recursion) */
    float sorted[MEDIAN_WINDOW_SIZE];
    for (uint8_t i = 0U; i < mf->count; i++) {
        sorted[i] = mf->history[i];
    }

    for (uint8_t i = 1U; i < mf->count; i++) {
        float key = sorted[i];
        int   j   = (int)i - 1;
        while ((j >= 0) && (sorted[j] > key)) {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = key;
    }

    return sorted[mf->count / 2U];
}
