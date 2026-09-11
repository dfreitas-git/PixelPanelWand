
#pragma once
#include <Arduino.h>

struct WandPose {
    float roll;
    float pitch;
    float yaw;
};

class PoseEstimator {
public:
    void update(const IMUData &imu, float dt);

    const WandPose &getPose() const;

private:
    WandPose pose{};
    bool initialized = false;
};