#ifndef APP_FUSION_FUSION_H
#define APP_FUSION_FUSION_H

#include "app/common/app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

bool Fusion_Init(FusionState *state);
bool Fusion_Update(const ImuSample *imu, const Bmp280Sample *bmp280,
                   FusionState *state);

#ifdef __cplusplus
}
#endif

#endif /* APP_FUSION_FUSION_H */
