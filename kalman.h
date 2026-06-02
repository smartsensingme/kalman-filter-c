/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef KALMAN_H_
#define KALMAN_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Configuration struct for Kalman 2D Filter */
typedef struct {
  float q_theta; /* Process noise covariance for position/angle */
  float q_omega; /* Process noise covariance for velocity */
  float r;       /* Measurement noise covariance */
} kalman_2d_config_t;

/* Default config macro for Kalman 2D Filter (pre-tuned for AS5600) */
#define KALMAN_2D_CONFIG_DEFAULT() { \
  .q_theta = 0.001f, \
  .q_omega = 50.0f, \
  .r = 0.018f, \
}

/* Kalman 2D Filter State Structure */
struct kalman_2d {
  float x[2];    /* State vector: [0] = position/angle, [1] = velocity */
  float P[2][2]; /* Estimate error covariance matrix */
  float Q_theta; /* Process noise covariance for position/angle */
  float Q_omega; /* Process noise covariance for velocity */
  float R;       /* Measurement noise covariance */
};

/**
 * @brief Initialize the 2D Kalman Filter
 * 
 * @param k           Pointer to the 2D Kalman filter state structure.
 * @param initial_val Initial value for position/angle state.
 * @param cfg         Pointer to the configuration structure. If NULL, default values are applied.
 */
void kalman_2d_init(struct kalman_2d *k, float initial_val, const kalman_2d_config_t *cfg);

/**
 * @brief Update the general-purpose linear 2D Kalman Filter estimation
 * 
 * @param k              Pointer to the 2D Kalman filter state structure.
 * @param measured_value Measured position/value.
 * @param dt             Time step since last update.
 */
void kalman_2d_update(struct kalman_2d *k, float measured_value, float dt);

/* Configuration struct for Kalman 3D Filter */
typedef struct {
  float q_theta; /* Process noise covariance for position/angle */
  float q_omega; /* Process noise covariance for velocity */
  float q_alpha; /* Process noise covariance for acceleration */
  float r;       /* Measurement noise covariance */
} kalman_3d_config_t;

/* Default config macro for Kalman 3D Filter */
#define KALMAN_3D_CONFIG_DEFAULT() { \
  .q_theta = 0.001f, \
  .q_omega = 10.0f, \
  .q_alpha = 100.0f, \
  .r = 0.018f, \
}

/* Kalman 3D Filter State Structure */
struct kalman_3d {
  float x[3];        /* State vector: [0]=position/angle, [1]=velocity, [2]=acceleration */
  float P[3][3];     /* Estimate error covariance matrix */
  float Q_theta;     /* Process noise covariance for position/angle */
  float Q_omega;     /* Process noise covariance for velocity */
  float Q_alpha;     /* Process noise covariance for acceleration */
  float R;           /* Measurement noise covariance */
};

/**
 * @brief Initialize the 3D Kalman Filter
 * 
 * @param k           Pointer to the 3D Kalman filter state structure.
 * @param initial_val Initial value for position/angle state.
 * @param cfg         Pointer to the configuration structure. If NULL, default values are applied.
 */
void kalman_3d_init(struct kalman_3d *k, float initial_val, const kalman_3d_config_t *cfg);

/**
 * @brief Update the general-purpose linear 3D Kalman Filter estimation
 * 
 * @param k              Pointer to the 3D Kalman filter state structure.
 * @param measured_value Measured position/value.
 * @param dt             Time step since last update.
 */
void kalman_3d_update(struct kalman_3d *k, float measured_value, float dt);

#ifdef __cplusplus
}
#endif

#endif /* KALMAN_H_ */
