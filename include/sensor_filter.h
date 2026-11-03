/**
 * @file    sensor_filter.h
 * @author  Yigit Cakirdogan
 * @brief   Two-stage industrial sensor conditioning: Median-of-5 spike rejection
 *          followed by a 1D Discrete Kalman Filter.
 */

#ifndef SENSOR_FILTER_H
#define SENSOR_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MEDIAN_WINDOW_SIZE 5U

typedef struct {
    float   q_process_var;   /**< Process noise variance (Q)           */
    float   r_measure_var;   /**< Measurement noise variance (R)       */
    float   x_estimate;      /**< Current a-posteriori state estimate  */
    float   p_error_cov;     /**< Current estimate error covariance    */
    float   kalman_gain;     /**< Last computed Kalman gain (K)        */
    bool    initialized;
} kalman_1d_t;

typedef struct {
    float   history[MEDIAN_WINDOW_SIZE];
    uint8_t index;
    uint8_t count;
} median_filter_t;

/**
 * @brief Initialize a 1D Kalman filter.
 */
void kalman_1d_init(kalman_1d_t *kf, float q_process_var, float r_measure_var, float initial_estimate);

/**
 * @brief Update the 1D Kalman filter with a new measurement.
 */
float kalman_1d_update(kalman_1d_t *kf, float measurement);

/**
 * @brief Initialize a Median-of-5 impulse spike rejection filter.
 */
void median_filter_init(median_filter_t *mf);

/**
 * @brief Push a raw sample into the Median-of-5 window and return the median value.
 */
float median_filter_update(median_filter_t *mf, float raw_sample);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_FILTER_H */
