
#pragma once

#include <Arduino.h>
#include "Globals.h"
#include "WandProtocol.h"
#include "GestureRecognizer.h"


struct WandState {
    float roll;
    float pitch;
    float yaw;

    float ax;
    float ay;
    float az;

    float vx;
    float vy;
    float vz;

    WandMode mode;
    GestureEvent gesture;
};

class Wand {
public:
    void updateContinuousState(const IMUData& imu, const WandPose& pose, const WandMotion& motion, float dt);

    const WandState &getState() const {
        return(state);
    }

private:
    WandState state{};
    WandMode mode{};
    GestureRecognizer gestureRecognizer;
};