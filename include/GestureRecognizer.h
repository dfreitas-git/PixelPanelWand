
#pragma once

#include <Arduino.h>
#include "Globals.h"
#include "WandProtocol.h"
#include "WandIMU.h"
#include "PoseEstimator.h"
#include "MotionEstimator.h"


enum class ThrustState : uint8_t {
    Idle,
    Tracking,
    Cooldown
};

class GestureRecognizer {
public:
    void update(const IMUData& imu,
                const WandPose& pose,
                const WandMotion& motion,
                float dt);

    bool eventAvailable() const;
    const GestureEvent &getEvent() const {
        return(event);
    }

private:
    GestureEvent event;

    ThrustState thrustState = ThrustState::Idle;

    float thrustPeakVelocity = 0.0f;
    int8_t thrustDirection = 0;   // +1 forward, -1 backward
    uint32_t thrustStartMs = 0;
    uint32_t thrustCooldownStartMs = 0;
    const float REARM_ACCEL = 0.5f;

    void detectThrust(const WandMotion& motion);
    void detectSwipe();
    void detectCircle();

    void emitGesture(WandGesture gesture);
};