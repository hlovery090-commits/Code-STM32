#ifndef ANN_1LAYER_H
#define ANN_1LAYER_H

#include <stdint.h>

/* Single-hidden-layer MLP: 4 -> [24 tansig] -> 1 purelin.
   Separate name from ANN_FULL_Predict() so both architectures can coexist
   in the same build (e.g. for shadow comparison or quick A/B toggling). */
float ANN_1L_Predict(float i_d_meas, float i_q_meas, float i_q_ref, float encoder_speed);
uint8_t ANN_1L_IsLastPredictionValid(void);

#endif // ANN_1LAYER_H
