
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
enum class SwipeState : uint8_t {
    Idle,
    Candidate,
    Direction,
    Cooldown
};

enum class ActiveGesture {
    None,
    Thrust,
    Swipe
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
    SwipeState swipeState = SwipeState::Idle;
    ActiveGesture activeGesture = ActiveGesture::None;

    const float REARM_ACCEL = 0.5f;
    const float THRUST_START_ACCEL       = 3.0f;
    const float THRUST_MIN_PEAK_VELOCITY = 0.15f;
    const uint32_t THRUST_MAX_THRUST_MS  = 500;
    const uint32_t THRUST_COOLDOWN_MS    = 300;

    const float SWIPE_START_ACCEL       = 1.5f;
    const float SWIPE_MIN_PEAK_VELOCITY = 0.15f;
    const uint32_t SWIPE_ACCUMULATE_MS  = 120;
    const uint32_t SWIPE_COOLDOWN_MS    = 300;

    float thrustPeakVelocity = 0.0f;
    int8_t thrustDirection = 0;   // +1 forward, -1 backward
    uint32_t thrustStartMs = 0;
    uint32_t thrustCooldownStartMs = 0;

    float swipePeakVelocity = 0.0f;
    uint32_t swipeStartMs = 0;
    uint32_t swipeDtStartMs = 0;
    uint32_t swipeCooldownStartMs = 0;
    float swipeRightDist = 0.0f;
    float swipeUpDist = 0.0f;
    float swipeDirection = 0.0f;
    float sumRight = 0.0f;
    float sumUp = 0.0f;

    void detectThrust(const WandMotion& motion);
    void detectSwipe(const WandMotion& motion, float dt);
    void detectCircle();

    void emitGesture(WandGesture gesture);
};