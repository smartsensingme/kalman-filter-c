/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include "kalman.h"

void kalman_2d_init(struct kalman_2d *k, float initial_val, const kalman_2d_config_t *cfg) {
  k->x[0] = initial_val;
  k->x[1] = 0.0f; /* Start stationary */

  k->P[0][0] = 1.0f;
  k->P[0][1] = 0.0f;
  k->P[1][0] = 0.0f;
  k->P[1][1] = 1.0f;

  if (cfg != NULL) {
    k->Q_theta = cfg->q_theta;
    k->Q_omega = cfg->q_omega;
    k->R = cfg->r;
  } else {
    /* Apply default values pre-tuned for AS5600 */
    k->Q_theta = 0.001f;
    k->Q_omega = 50.0f;
    k->R = 0.018f;
  }
}

void kalman_2d_update(struct kalman_2d *k, float measured_value, float dt) {
  /* --- 1. PREDICT STEP --- */
  /* x_pred = F * x */
  float x_pred_val = k->x[0] + k->x[1] * dt;
  float x_pred_vel = k->x[1];

  /* P_pred = F * P * F^T + Q */
  float P_pred_00 = k->P[0][0] +
                    dt * (k->P[1][0] + k->P[0][1] + dt * k->P[1][1]) +
                    k->Q_theta;
  float P_pred_01 = k->P[0][1] + dt * k->P[1][1];
  float P_pred_10 = k->P[1][0] + dt * k->P[1][1];
  float P_pred_11 = k->P[1][1] + k->Q_omega;

  /* --- 2. UPDATE / CORRECT STEP --- */
  /* Innovation (measurement error) */
  float y = measured_value - x_pred_val;

  /* Innovation Covariance: S = H * P_pred * H^T + R */
  float S = P_pred_00 + k->R;

  /* Kalman Gain: K = P_pred * H^T * S^-1 */
  float K_0 = P_pred_00 / S;
  float K_1 = P_pred_10 / S;

  /* Correct State: x = x_pred + K * y */
  k->x[0] = x_pred_val + K_0 * y;
  k->x[1] = x_pred_vel + K_1 * y;

  /* Correct Covariance: P = (I - K * H) * P_pred */
  k->P[0][0] = (1.0f - K_0) * P_pred_00;
  k->P[0][1] = (1.0f - K_0) * P_pred_01;
  k->P[1][0] = P_pred_10 - K_1 * P_pred_00;
  k->P[1][1] = P_pred_11 - K_1 * P_pred_01;
}

void kalman_3d_init(struct kalman_3d *k, float initial_val, const kalman_3d_config_t *cfg) {
  k->x[0] = initial_val;
  k->x[1] = 0.0f; /* Start stationary */
  k->x[2] = 0.0f; /* Start with zero acceleration */

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      k->P[i][j] = (i == j) ? 1.0f : 0.0f;
    }
  }

  if (cfg != NULL) {
    k->Q_theta = cfg->q_theta;
    k->Q_omega = cfg->q_omega;
    k->Q_alpha = cfg->q_alpha;
    k->R = cfg->r;
  } else {
    /* Apply default values */
    k->Q_theta = 0.001f;
    k->Q_omega = 10.0f;
    k->Q_alpha = 100.0f;
    k->R = 0.018f;
  }
}

void kalman_3d_update(struct kalman_3d *k, float measured_value, float dt) {
  /* --- 1. PREDICT STEP --- */
  float h = 0.5f * dt * dt;

  /* x_pred = F * x */
  float x_pred_val = k->x[0] + k->x[1] * dt + k->x[2] * h;
  float x_pred_vel = k->x[1] + k->x[2] * dt;
  float x_pred_acc = k->x[2];

  /* P_pred = F * P * F^T + Q */
  /* Row lines of F * P */
  float a0 = k->P[0][0] + dt * k->P[1][0] + h * k->P[2][0];
  float a1 = k->P[0][1] + dt * k->P[1][1] + h * k->P[2][1];
  float a2 = k->P[0][2] + dt * k->P[1][2] + h * k->P[2][2];

  float a4 = k->P[1][1] + dt * k->P[2][1];
  float a5 = k->P[1][2] + dt * k->P[2][2];

  /* (F * P) * F^T + Q components */
  float P_pred_00 = a0 + dt * a1 + h * a2 + k->Q_theta;
  float P_pred_01 = a1 + dt * a2;
  float P_pred_02 = a2;

  float P_pred_10 = P_pred_01;
  float P_pred_11 = a4 + dt * a5 + k->Q_omega;
  float P_pred_12 = a5;

  float P_pred_20 = P_pred_02;
  float P_pred_22 = k->P[2][2] + k->Q_alpha;

  /* --- 2. UPDATE / CORRECT STEP --- */
  /* Innovation (measurement error) */
  float y = measured_value - x_pred_val;

  /* Innovation Covariance: S = H * P_pred * H^T + R */
  float S = P_pred_00 + k->R;

  /* Kalman Gain: K = P_pred * H^T * S^-1 */
  float K_0 = P_pred_00 / S;
  float K_1 = P_pred_10 / S;
  float K_2 = P_pred_20 / S;

  /* Correct State: x = x_pred + K * y */
  k->x[0] = x_pred_val + K_0 * y;
  k->x[1] = x_pred_vel + K_1 * y;
  k->x[2] = x_pred_acc + K_2 * y;

  /* Correct Covariance: P = (I - K * H) * P_pred */
  k->P[0][0] = (1.0f - K_0) * P_pred_00;
  k->P[0][1] = (1.0f - K_0) * P_pred_01;
  k->P[0][2] = (1.0f - K_0) * P_pred_02;

  k->P[1][0] = k->P[0][1];
  k->P[1][1] = P_pred_11 - K_1 * P_pred_01;
  k->P[1][2] = P_pred_12 - K_1 * P_pred_02;

  k->P[2][0] = k->P[0][2];
  k->P[2][1] = k->P[1][2];
  k->P[2][2] = P_pred_22 - K_2 * P_pred_02;
}
