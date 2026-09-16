
#pragma once
#include <Arduino.h>
#include "Globals.h"

struct WandPose {
    float roll;
    float pitch;
    float yaw;
};

class PoseEstimator {
public:
    void update(const IMUData &imu, float dt);
    const WandPose &getPose() const;
    void setYaw(float yaw);

private:
    WandPose pose{};
    float attitudeTrust(float diff);
    bool initialized = false;

    // Number of degrees (2-8) between trusting and not trusting 
    const float DIFF_FULL_TRUST = 2.0f * PI / 180.0f;
    const float DIFF_ZERO_TRUST = 8.0f * PI / 180.0f;
};