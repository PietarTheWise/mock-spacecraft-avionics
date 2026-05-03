#include "app/fusion/fusion.h"
#include <stddef.h>

#define FUSION_DEFAULT_DT_S (0.01f)
#define FUSION_MAX_DT_S (0.2f)
#define FUSION_REF_TEMP_C (25.0f)

/* TODO(WIP-CAL): Replace hardcoded coefficients with calibrated values. */
static const float kTempBiasSlopeDpsPerDegC[3] = {0.020f, -0.015f, 0.010f};
static const float kTempBiasAtRefDps[3] = {0.000f, 0.000f, 0.000f};

static float Fusion_ComputeDtSeconds(uint32_t now_ms, uint32_t prev_ms) {
  float dt_s = FUSION_DEFAULT_DT_S;
  if (now_ms > prev_ms) {
    dt_s = (float)(now_ms - prev_ms) / 1000.0f;
    if (dt_s > FUSION_MAX_DT_S) {
      dt_s = FUSION_DEFAULT_DT_S;
    }
  }
  return dt_s;
}

static void Fusion_ComputeGyroBias(float temperature_c, float bias_out[3]) {
  float delta_temp_c = temperature_c - FUSION_REF_TEMP_C;
  uint32_t axis = 0U;

  for (axis = 0U; axis < 3U; axis++) {
    bias_out[axis] =
        kTempBiasAtRefDps[axis] + (kTempBiasSlopeDpsPerDegC[axis] * delta_temp_c);
  }
}

bool Fusion_Init(FusionState *state) {
  uint32_t axis = 0U;

  if (state == NULL) {
    return false;
  }

  state->timestamp_ms = 0U;
  state->roll_deg = 0.0f;
  state->pitch_deg = 0.0f;
  state->yaw_deg = 0.0f;
  state->temperature_c = FUSION_REF_TEMP_C;
  state->is_placeholder = true;

  for (axis = 0U; axis < 3U; axis++) {
    state->gyro_bias_dps[axis] = kTempBiasAtRefDps[axis];
  }

  return true;
}

bool Fusion_Update(const ImuSample *imu, const Bmp280Sample *bmp280,
                   FusionState *state) {
  float bias_dps[3] = {0.0f, 0.0f, 0.0f};
  float corrected_gyro_dps[3] = {0.0f, 0.0f, 0.0f};
  float temperature_c = FUSION_REF_TEMP_C;
  float dt_s = FUSION_DEFAULT_DT_S;

  if ((imu == NULL) || (bmp280 == NULL) || (state == NULL)) {
    return false;
  }

  temperature_c = bmp280->temperature_c;
  dt_s = Fusion_ComputeDtSeconds(imu->timestamp_ms, state->timestamp_ms);
  Fusion_ComputeGyroBias(temperature_c, bias_dps);

  corrected_gyro_dps[0] = imu->gyro_dps[0] - bias_dps[0];
  corrected_gyro_dps[1] = imu->gyro_dps[1] - bias_dps[1];
  corrected_gyro_dps[2] = imu->gyro_dps[2] - bias_dps[2];

  /*
   * WIP placeholder fusion:
   * Integrate temperature-corrected gyro only. This is intentionally simple
   * until real sensor calibration and full fusion are added.
   */
  state->roll_deg += corrected_gyro_dps[0] * dt_s;
  state->pitch_deg += corrected_gyro_dps[1] * dt_s;
  state->yaw_deg += corrected_gyro_dps[2] * dt_s;

  state->gyro_bias_dps[0] = bias_dps[0];
  state->gyro_bias_dps[1] = bias_dps[1];
  state->gyro_bias_dps[2] = bias_dps[2];
  state->temperature_c = temperature_c;
  state->timestamp_ms = imu->timestamp_ms;
  state->is_placeholder = true;

  return true;
}
