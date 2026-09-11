
/*  PixelPanelWand:  ESP32 code written in the Arduino PlatformIO IDE.

    PixelPanelWand is the code that controls the hand-held wand that is used to control
    the PixelPanel.  It has a 9-axis IMU (MPU9250) to measure the user's movements.  We 
    send ESP-NOW packets to the PixelPanel to control the scenes and PixelPanel settings.

    The wand operates in two modes:  Pose-mode and gesture-mode.  In pose-mode, information
    about the location and attitude of the wand is transmitted to control effects in the 
    currently displayed scene.  In gesture-mode the wand transmitts decoded gesture-commands
    (swipe-up/down/left/right, thrust-in/out, circle-cw/ccw, etc.).  Thes commands are used
    my the PixelPanel to execute control options (change scene, change brightness, change 
    diffuser screen distance, etc.

    The wand has a single momentary-contact button used to switch between modes.  Also, 
    on boot, if the button is held vor ~2.5 seconds we will enter a calibration routine 
    where the wand is oriented around all six directions (+/-X, +/-Y, +/-Z) and lying still
    to calibrate the accelerometer and gyro (and eventually the magnetometer).  The offset
    bias and scale factors are stored in non-volatile RAM using ESP Preferences.

    I am collaborating with chatGPT in designing and coding this project.

    dlf 9/9/2026
*/

#include <Arduino.h>
#include <Wire.h>
#include "WandIMU.h"
#include "PoseEstimator.h"

// Class Instances 
WandIMU wandIMU(Wire, 0x68);
PoseEstimator poseEstimator;

// Prototypes
bool calibrationRequestedAtBoot();

constexpr uint8_t BUTTON_PIN = 25;
constexpr uint32_t CALIBRATION_HOLD_MS = 2500;
uint32_t lastUpdateUs = 0;
uint32_t lastPrintMs = 0;

void setup()
{
    Serial.begin(115200);
    delay(500);

    pinMode(BUTTON_PIN, INPUT_PULLUP);

    Serial.println();
    Serial.println("PixelPanel Wand");

    bool doCalibration = calibrationRequestedAtBoot();

    if (!wandIMU.begin()) {
        Serial.println("Fatal IMU error.");
        while (true) {
            delay(1000);
        }
    }

    if (doCalibration) {
        wandIMU.runCalibration(BUTTON_PIN);
    }
}


void loop()
{
    IMUData imu;

    if (wandIMU.read(imu)) {
        uint32_t nowUs = micros();

        if (lastUpdateUs != 0) {
            float dt = (nowUs - lastUpdateUs) * 1.0e-6f;
            poseEstimator.update(imu, dt);
        }
        lastUpdateUs = nowUs;

        if (millis() - lastPrintMs >= 100) {
            lastPrintMs = millis();

            const WandPose &pose = poseEstimator.getPose();

            Serial.print("Roll: ");
            Serial.print(pose.roll * 180.0f / PI, 3);

            Serial.print("  Pitch: ");
            Serial.print(pose.pitch * 180.0f / PI, 3);

            Serial.print("  Yaw: ");
            Serial.println(pose.yaw * 180.0f / PI, 3);
        }
    }
    delay(10);
}

bool calibrationRequestedAtBoot()
{
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // No button press at startup.
    if (digitalRead(BUTTON_PIN) == HIGH) {
        return false;
    }

    uint32_t start = millis();

    // If the button is held down for CALIBRATION_HOLD_MS during boot, then enter the calibration routine
    while (digitalRead(BUTTON_PIN) == LOW) {

        if (millis() - start >= CALIBRATION_HOLD_MS) {
            return true;
        }

        delay(10);
    }

    return false;
}