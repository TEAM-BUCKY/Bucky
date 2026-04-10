#ifndef BUCKY_SELFLOCALIZATIONEKF_H
#define BUCKY_SELFLOCALIZATIONEKF_H

#include <cstdint>

#include "field/DigitalField.h"

struct SelfLocState {
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;
    float vx = 0.0f;
    float vy = 0.0f;
    float omega = 0.0f;
    float P_xy = 0.0f;
};

struct SonarUpdateResult {
    bool used_for_localization = false;
    bool anomaly_detected = false;
    float obstacle_x = 0.0f;
    float obstacle_y = 0.0f;
};

struct SelfLocalizationFilter {
    float x[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float P[6][6] = {};
};

void selfloc_reset(SelfLocalizationFilter* filter, float x = 0.0f, float y = 0.0f, float theta = 0.0f);
void selfloc_predict(SelfLocalizationFilter* filter, float dt, float vxBody, float vyBody, float omega);
void selfloc_update_compass(SelfLocalizationFilter* filter, float thetaMeasuredRad, float totalPwmDuty = 0.0f);
SonarUpdateResult selfloc_update_sonar(SelfLocalizationFilter* filter, uint8_t sensorIdx, float distanceM);
float selfloc_expected_wall_distance(const SelfLocalizationFilter* filter, float beamAngleRad);
SelfLocState selfloc_get_state(const SelfLocalizationFilter* filter);

#endif // BUCKY_SELFLOCALIZATIONEKF_H

