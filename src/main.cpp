
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

    Here are the wand coordinate definitions used in the code:

    WAND COORDINATE / SIGN CONVENTION

    Translation:
      +X = forward / toward wand tip
      -X = backward / toward handle

      +Y = right
      -Y = left

      +Z = down
      -Z = up

    Rotation:
      +Roll  = clockwise/right
      -Roll  = counterclockwise/left

      +Pitch = front/tip up
      -Pitch = front/tip down

      +Yaw   = clockwise viewed from above
      -Yaw   = counterclockwise viewed from above

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
#include "Globals.h"
#include <WiFi.h>
#include "WandIMU.h"
#include "PoseEstimator.h"
#include "MotionEstimator.h"
#include "WandRadio.h"
#include "GestureRecognizer.h"
#include "Wand.h"
#include "WandProtocol.h"

// Class Instances 
Wand wand;
WandMode currentMode = WandMode::Scene;
WandIMU wandIMU(Wire, 0x68);
PoseEstimator poseEstimator;
MotionEstimator motionEstimator;
GestureRecognizer gestureRecognizer;
WandRadio wandRadio;

// Prototypes
bool calibrationRequestedAtBoot();
static bool modeButtonPressed();

// For ESP-NOW
uint32_t lastRadioUs = 0;
uint32_t packetSequence = 0;

uint32_t lastUpdateUs = 0;
uint32_t lastImuReadMs = 0;
uint32_t lastPrintMs = 0;
bool poseUpdated = false;

// Used for synchronizaton between Wand and Panel
uint32_t sessionId;

void setup()
{
    Serial.begin(115200);

    // Wait for USB serial connection (min 2 seconds, max 5) so serial prints show up...
    while ((!Serial || millis() < 2000) && millis() < 5000) {
        delay(10);
    }

    Serial.println();
    Serial.println(F("PixelPanel Wand"));

    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // Set up the wand tip RGB LED 
    ledcSetup(RED_CHANNEL, 4000, 8);       // Set channel to 4 kHz, 8-bit resolution
    ledcSetup(GREEN_CHANNEL, 4000, 8);     
    ledcSetup(BLUE_CHANNEL, 4000, 8);       
    ledcAttachPin(led_red, RED_CHANNEL); 
    ledcAttachPin(led_green, GREEN_CHANNEL); 
    ledcAttachPin(led_blue, BLUE_CHANNEL); 

    // Set LED red until calibration is done
    wand.setLED(128,0,0);

    // Set up ESP-NOW
    if (!wandRadio.begin(PANEL_MAC)) {
        Serial.println(F("Fatal ESP-NOW initialization error."));
        while (true) {
            delay(1000);
        }
    }

    bool doAccelCalibration = calibrationRequestedAtBoot();

    if (!wandIMU.begin()) {
        Serial.println(F("Fatal IMU error."));
        while (true) {
            delay(1000);
        }
    }

    if (doAccelCalibration) {
        wandIMU.runAccelCalibration(BUTTON_PIN);
    }

    // Always run the gyro calibration on boot-up
    wandIMU.gyroCalStart = millis();

    // Create a sessionID that will be monitored by the PixePanel to be sure both
    // are running from the same start (i.e. that one didn't get re-booted)
    sessionId = esp_random();
}



void loop() {
    IMUData imu;

    // First calibrate the gyro before doing the regular loop operations
    if (!wandIMU.gyroCalDone) {
        wandIMU.runGyroCalibration();
        delay(10);
        return;
    }

    // Toggle the current mode (scene for displaying animations, panel for setting system parameters)
    if (modeButtonPressed()) {
        wand.toggleMode();
        if(DEBUG_PRINT == 1) {
            Serial.print("Toggle wand mode to: ");
            if(wand.getMode() == WandMode::Panel) {
                Serial.println("Panel");
            } else {
                Serial.println("Scene");
            }
        }
        // Temporary code to flash the LED if the button is pushed (just for testing- dlf)
        //wand.setFlashingLED(3,250,200,0,128);  // Flash Purple
    }

    if(wand.getFlashingState()) {
        wand.updateLED();
    } else {
        // Idle with dim blue for Scene, Red if we are doing panel commands
        if(wand.getMode() == WandMode::Scene) {
            wand.setLED(0,0,35);
        } else {
            wand.setLED(35,0,0);
        }
    }

    if (millis() - lastImuReadMs >= IMU_READ_INTERVAL_MS) {
        WandPacket wandPacket{};
        if (wandIMU.read(imu)) {
            uint32_t nowUs = micros();
            if (lastUpdateUs != 0) {
                float dt = (nowUs - lastUpdateUs) * 1.0e-6f;

                poseEstimator.update(imu, dt);
                poseUpdated = true;

                const WandPose &pose = poseEstimator.getPose();
                motionEstimator.update(imu,pose,dt);

                const WandMotion &motion = motionEstimator.getMotion();
                gestureRecognizer.update(imu,pose,motion,dt);

                static uint32_t lastLEDSequence = 0;
                const GestureEvent& event = gestureRecognizer.getEvent();
                
                // If the sequence number has changed it means we got a new gesture event
                if (event.eventSequence != 0 &&
                    event.eventSequence != lastLEDSequence) {
                
                    lastLEDSequence = event.eventSequence;
                
                    switch (event.gesture) {
                        case WandGesture::ThrustIn:
                            wand.setFlashingLED(1,250,0,128,0);  // Flash green once
                            break;
                
                        case WandGesture::ThrustOut:
                            wand.setFlashingLED(2,250,0,128,0);  // Flash green twice
                            break;
                
                        default:
                            break;
                    }
                }

                wandPacket.gesture = event.gesture;
                wandPacket.eventSequence = event.eventSequence;
                wandPacket.eventTimestampMs = event.eventTimestampMs;
                wandPacket.swipeDirection = event.swipeDirection;

            }
            lastUpdateUs = nowUs;
    
    
            // Send ESP-NOW packet at its own time interval
            if (nowUs - lastRadioUs >= RADIO_INTERVAL_US) {
            
                lastRadioUs += RADIO_INTERVAL_US;
            
                const WandPose &pose = poseEstimator.getPose();
                const WandMotion &motion = motionEstimator.getMotion();
            
                wandPacket.magic       = WAND_PROTOCOL_MAGIC;
                wandPacket.version     = WAND_PROTOCOL_VERSION;
                wandPacket.sessionId   =  sessionId;
                wandPacket.packetSequence    = packetSequence++;
                wandPacket.packetTimestampMs = millis();
                wandPacket.mode = currentMode;

                // Get the pose parameters
                wandPacket.roll  = pose.roll;
                wandPacket.pitch = pose.pitch;
                wandPacket.yaw   = pose.yaw;
            
                wandPacket.gx = imu.gx;
                wandPacket.gy = imu.gy;
                wandPacket.gz = imu.gz;
            
                wandPacket.vx = motion.vx;
                wandPacket.vy = motion.vy;
                wandPacket.vz = motion.vz;

                if (!wandRadio.send(wandPacket)) {
                    //dlf Serial.println(F("ESP-NOW send failed."));
                }
            
            }
    
            
            // Debug printing - once a second so we don't fill the serial monitor too quickly
            // or full 50hz if pushing the button (not debounced)
            if ((digitalRead(BUTTON_PIN) == 0 && DEBUG_PRINT == 1) || (DEBUG_PRINT == 2 && poseUpdated && millis() - lastPrintMs >= 1000)) {
                lastPrintMs = millis();
    
                const WandPose &pose = poseEstimator.getPose();
                const WandMotion &motion = motionEstimator.getMotion();

                Serial.print(F("Roll: "));
                Serial.print(pose.roll * 180.0f / PI, 3);
    
                Serial.print(F("  Pitch: "));
                Serial.print(pose.pitch * 180.0f / PI, 3);
    
                Serial.print(F("  Yaw: "));
                Serial.println(pose.yaw * 180.0f / PI, 3);
    
                Serial.print(F("MotionY: "));
                Serial.print(motion.ay, 3);
    
                Serial.print(F("  MotionX: "));
                Serial.print(motion.ax, 3);
    
                Serial.print(F("  MotionZ: "));
                Serial.println(motion.az, 3);
    
                Serial.print(F("aRight: "));
                Serial.print(motion.aRight, 3);
    
                Serial.print(F(" aUp: "));
                Serial.println(motion.aUp, 3);
    
                Serial.print(F("VelocityY: "));
                Serial.print(motion.vy, 3);
    
                Serial.print(F("  VelocityX: "));
                Serial.print(motion.vx, 3);
    
                Serial.print(F("  VelocityZ: "));
                Serial.println(motion.vz, 3);
                
                Serial.print(F("GyroY: "));
                Serial.print(imu.gy, 3);
    
                Serial.print(F(" GyroX: "));
                Serial.print(imu.gx, 3);
    
                Serial.print(F(" GyroZ: "));
                Serial.println(imu.gz, 3);
    
                
                Serial.print(F("accelPitch: "));
                Serial.println(accelPitch);
                Serial.print(F("predictedPitch: "));
                Serial.println(predictedPitch);
                Serial.print(F("rollDiffTrust: "));
                Serial.println(rollDiffTrust);
                Serial.print(F("pitchDiffTrust: "));
                Serial.println(pitchDiffTrust);
                Serial.print(F("accelRollTrust: "));
                Serial.println(accelRollTrust);
                Serial.print(F("accelPitchTrust: "));
                Serial.println(accelPitchTrust);
                
                Serial.println();
                
            }
        }
        lastImuReadMs=millis();
    }
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

// Returns true once for each debounced button press
static bool modeButtonPressed() {
    static bool lastReading = HIGH;
    static bool stableState = HIGH;
    static uint32_t lastChangeMs = 0;

    constexpr uint32_t DEBOUNCE_MS = 20;

    bool reading = digitalRead(BUTTON_PIN);
    uint32_t now = millis();

    if (reading != lastReading)
        lastChangeMs = now;

    lastReading = reading;

    if ((now - lastChangeMs >= DEBOUNCE_MS) &&
        (reading != stableState)) {

        stableState = reading;
        return stableState == LOW;
    }

    return false;
}