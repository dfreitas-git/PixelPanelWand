
#pragma once

#include <Arduino.h>
#include "Globals.h"
#include "WandIMU.h"
#include "PoseEstimator.h"

struct WandMotion {
    // Acceleration
    float ax;
    float ay;
    float az;
    
    // Velocity
    float vx;
    float vy;
    float vz;
};

class MotionEstimator {
public:
    void update(const IMUData &imu, const WandPose &pose, float dt);

    const WandMotion &getMotion() const {
        return motion;
    }

private:
    float gainX = 1.0f;
    float gainY = 1.0f;
    float gainZ = 1.0f;
    float damping = 0.9f;

    WandMotion motion{};
};