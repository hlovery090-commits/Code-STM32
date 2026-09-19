#ifndef ANN_FOC_FULL_H
#define ANN_FOC_FULL_H

#include <stdint.h>

// Predicts V_Q directly, replacing the PI current controller for Iq.
// Inputs must be in the SAME physical units used during training
// (Amps for currents, raw MCSDK units for ENCODER_SPEED).
float ANN_FULL_Predict(float i_d_meas, float i_q_meas, float i_q_ref, float encoder_speed);

// Returns 1 if the last prediction was valid (finite), 0 if NaN/Inf was caught.
uint8_t ANN_FULL_IsLastPredictionValid(void);

#endif // ANN_FOC_FULL_H
