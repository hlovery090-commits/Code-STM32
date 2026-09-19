#include "ann_foc_full.h"
#include "ann_foc_full_weights.h"
#include <math.h>

static uint8_t g_last_prediction_valid = 1;

static float tansig(float x)
{
    // numerically stable tansig: 2/(1+exp(-2x)) - 1
    if (x > 20.0f)  return 1.0f;
    if (x < -20.0f) return -1.0f;
    return (2.0f / (1.0f + expf(-2.0f * x))) - 1.0f;
}

uint8_t ANN_FULL_IsLastPredictionValid(void)
{
    return g_last_prediction_valid;
}

float ANN_FULL_Predict(float i_d_meas, float i_q_meas, float i_q_ref, float encoder_speed)
{
    float x_raw[ANNFULL_N_INPUTS] = { i_d_meas, i_q_meas, i_q_ref, encoder_speed };
    float x_norm[ANNFULL_N_INPUTS];
    float h1[ANNFULL_N_HIDDEN1];
    float h2[ANNFULL_N_HIDDEN2];
    float y_norm;
    float y_out;
    int i, j;

    // ---- Input normalization (mapminmax, range [-1,1]) ----
    for (i = 0; i < ANNFULL_N_INPUTS; i++) {
        x_norm[i] = (x_raw[i] - ANNFULL_X_OFFSET[i]) * ANNFULL_X_GAIN[i] - 1.0f;
    }

    // ---- Hidden layer 1 ----
    for (i = 0; i < ANNFULL_N_HIDDEN1; i++) {
        float acc = ANNFULL_B1[i];
        for (j = 0; j < ANNFULL_N_INPUTS; j++) {
            acc += ANNFULL_IW1[i * ANNFULL_N_INPUTS + j] * x_norm[j];
        }
        h1[i] = tansig(acc);
    }

    // ---- Hidden layer 2 ----
    for (i = 0; i < ANNFULL_N_HIDDEN2; i++) {
        float acc = ANNFULL_B2[i];
        for (j = 0; j < ANNFULL_N_HIDDEN1; j++) {
            acc += ANNFULL_LW2[i * ANNFULL_N_HIDDEN1 + j] * h1[j];
        }
        h2[i] = tansig(acc);
    }

    // ---- Output layer (purelin) ----
    {
        float acc = ANNFULL_B3[0];
        for (j = 0; j < ANNFULL_N_HIDDEN2; j++) {
            acc += ANNFULL_LW3[j] * h2[j];
        }
        y_norm = acc;
    }

    // ---- Output denormalization ----
    y_out = (y_norm + 1.0f) / ANNFULL_Y_GAIN + ANNFULL_Y_OFFSET;

    if (!isfinite(y_out)) {
        g_last_prediction_valid = 0;
        return 0.0f; // caller must fall back to PI
    }

    g_last_prediction_valid = 1;
    return y_out;
}
