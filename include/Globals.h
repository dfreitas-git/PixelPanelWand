
#pragma once

#define DEBUG_PRINT 1   // 0 - no debug print, 1 - one level of print, 2 - more detail, 3 even more detail

constexpr float G = 9.80665f;

extern float accelMag;
extern float accelRollTrust;
extern float accelPitchTrust;
extern float rollDiffTrust;
extern float pitchDiffTrust;
extern float predictedPitch;
extern float accelPitch;

// RGB LED pins
const byte led_red = 1;   // GPIO pin for the LED
const byte led_green = 2; // GPIO pin for the LED
const byte led_blue = 3;  // GPIO pin for the LED

// pwm channels
constexpr uint8_t RED_CHANNEL = 0; 
constexpr uint8_t GREEN_CHANNEL = 1;  
constexpr uint8_t BLUE_CHANNEL = 2;

// mode button
constexpr uint8_t BUTTON_PIN = 4;

constexpr uint32_t CALIBRATION_HOLD_MS = 2500;  // time button must be held to enter calibration sequence
constexpr uint32_t RADIO_INTERVAL_US = 20000;   // 50 Hz
constexpr uint32_t IMU_READ_INTERVAL_MS = 10;   // 100 Hz