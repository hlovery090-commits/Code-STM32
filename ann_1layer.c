#include "ann_1layer.h"
#include "ann_1layer_weights.h"
#include <math.h>

static uint8_t g_last_prediction_valid = 1;

static float tansig(float x)
{
    if (x > 20.0f)  return 1.0f;
    if (x < -20.0f) return -1.0f;
    return (2.0f / (1.0f + expf(-2.0f * x))) - 1.0f;
}

uint8_t ANN_1L_IsLastPredictionValid(void)
{
    return g_last_prediction_valid;
}

/* Single-hidden-layer forward pass:
 *   h[i]   = tansig( sum_j( IW1[i][j] * x_norm[j] ) + B1[i] )
 *   y_norm = sum_i( LW2[i] * h[i] ) + B2                     (purelin) */
float ANN_1L_Predict(float i_d_meas, float i_q_meas, float i_q_ref, float encoder_speed)
{
    float x_raw[ANN1L_N_INPUTS] = { i_d_meas, i_q_meas, i_q_ref, encoder_speed };
    float x_norm[ANN1L_N_INPUTS];
    float h[ANN1L_N_HIDDEN];
    float y_norm;
    float y_out;
    int i, j;

    /* ---- Input normalization (mapminmax, range [-1,1]) ---- */
    for (i = 0; i < ANN1L_N_INPUTS; i++) {
        x_norm[i] = (x_raw[i] - ANN1L_X_OFFSET[i]) * ANN1L_X_GAIN[i] - 1.0f;
    }

    /* ---- Hidden layer (single, 24 neurons) ---- */
    for (i = 0; i < ANN1L_N_HIDDEN; i++) {
        float acc = ANN1L_B1[i];
        for (j = 0; j < ANN1L_N_INPUTS; j++) {
            acc += ANN1L_IW1[i * ANN1L_N_INPUTS + j] * x_norm[j];
        }
        h[i] = tansig(acc);
    }

    /* ---- Output layer (purelin, no activation) ---- */
    {
        float acc = ANN1L_B2;
        for (i = 0; i < ANN1L_N_HIDDEN; i++) {
            acc += ANN1L_LW2[i] * h[i];
        }
        y_norm = acc;
    }

    /* ---- Output denormalization ---- */
    y_out = (y_norm + 1.0f) / ANN1L_Y_GAIN + ANN1L_Y_OFFSET;

    if (!isfinite(y_out)) {
        g_last_prediction_valid = 0;
        return 0.0f;
    }

    g_last_prediction_valid = 1;
    return y_out;
}
