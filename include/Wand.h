
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
    WandMode getMode() const { return mode; }

    void toggleMode() {
        mode = (mode == WandMode::Scene)
             ? WandMode::Panel
             : WandMode::Scene;
    }

    void updateContinuousState(const IMUData& imu, const WandPose& pose, const WandMotion& motion, float dt);
    const bool getFlashingState() const {
        return flashing;
    }
    void updateLED();
    void setFlashingLED(const uint8_t numberOfFlashes, const uint16_t durationMs, const uint8_t R , const uint8_t G, uint8_t B); 
    void setLED(const uint8_t R , const uint8_t G, uint8_t B); 

    const WandState &getState() const {
        return(state);
    }


private:
    WandState state{};
    WandMode mode = WandMode::Scene;
    GestureRecognizer gestureRecognizer;
    bool flashing = false;
    bool ledOn = false;
    uint8_t ledFlashCount = 0;
    uint8_t flashR = 0;
    uint8_t flashG = 0;
    uint8_t flashB = 0;
    uint16_t flashDurationMs = 250;
    uint32_t nextFlash;

};