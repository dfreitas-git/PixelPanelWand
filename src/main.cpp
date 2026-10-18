
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
#include "WandIMU.h"
#include "PoseEstimator.h"
#include "MotionEstimator.h"
#include "WandRadio.h"
#include "GestureRecognizer.h"
#include "Wand.h"
#include "WandProtocol.h"

// Class Instances 
Wand wand;
WandIMU wandIMU(Wire, 0x68);
PoseEstimator poseEstimator;
MotionEstimator motionEstimator;
GestureRecognizer gestureRecognizer;
WandRadio wandRadio;

// Prototypes
bool calibrationRequestedAtBoot();

// For ESP-NOW
uint32_t lastRadioUs = 0;
uint32_t packetSequence = 0;

uint32_t lastUpdateUs = 0;
uint32_t lastImuReadMs = 0;
uint32_t lastPrintMs = 0;
bool poseUpdated = false;

void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("PixelPanel Wand");

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
        Serial.println("Fatal ESP-NOW initialization error.");
        while (true) {
            delay(1000);
        }
    }

    bool doAccelCalibration = calibrationRequestedAtBoot();

    if (!wandIMU.begin()) {
        Serial.println("Fatal IMU error.");
        while (true) {
            delay(1000);
        }
    }

    if (doAccelCalibration) {
        wandIMU.runAccelCalibration(BUTTON_PIN);
    }

    // Always run the gyro calibration on boot-up
    wandIMU.gyroCalStart = millis();

    wand.setLED(0,0,35);
}



void loop() {
    IMUData imu;

    // If we are not already flashing the LED, check the control button.
    if(!wand.getFlashingState() && digitalRead(BUTTON_PIN) == 0) {
        // Temporary code to flash the LED if the button is pushed (just for testing- dlf)
        wand.setFlashingLED(3,250,200,0,128);  // Flash Purple
    }
    if(wand.getFlashingState()) {
        wand.updateLED();
    } else {
        // Idle with dim blue 
        wand.setLED(0,0,35);
    }

    // First calibrate the gyro before doing the regular loop operations
    if (!wandIMU.gyroCalDone) {
        wandIMU.runGyroCalibration();
        delay(10);
        return;
    }

    if (millis() - lastImuReadMs >= IMU_READ_INTERVAL_MS) {
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
            }
            lastUpdateUs = nowUs;
    
    
            // Send ESP-NOW packet at its own time interval
            if (nowUs - lastRadioUs >= RADIO_INTERVAL_US) {
            
                lastRadioUs += RADIO_INTERVAL_US;
            
                const WandPose &pose = poseEstimator.getPose();
                const WandMotion &motion = motionEstimator.getMotion();
            
                WandPacket wandPacket{};
            
                wandPacket.magic       = WAND_PROTOCOL_MAGIC;
                wandPacket.version     = WAND_PROTOCOL_VERSION;
            
                wandPacket.sequence    = packetSequence++;
                wandPacket.timestampMs = millis();
            
                wandPacket.mode        = WandMode::Scene;

                if(gestureRecognizer.eventAvailable()) {
                    const GestureEvent &event = gestureRecognizer.getEvent();
                    wandPacket.gesture = event.gesture;
                    wandPacket.sequence = event.sequence;

                    if(event.gesture == WandGesture::ThrustIn) {
                        wand.setFlashingLED(1,250,0,128,0);
                    }
                    if(event.gesture == WandGesture::ThrustOut) {
                        wand.setFlashingLED(2,250,0,128,0);
                    }
                }
            
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
                    //dlf Serial.println("ESP-NOW send failed.");
                }
            }
    
            
            // Debug printing - once a second so we don't fill the serial monitor too quickly
            if (poseUpdated && millis() - lastPrintMs >= 1000) {
                lastPrintMs = millis();
    
                const WandPose &pose = poseEstimator.getPose();
                const WandMotion &motion = motionEstimator.getMotion();

                if(gestureRecognizer.eventAvailable()) {
                    const GestureEvent &event = gestureRecognizer.getEvent();
                    if(event.gesture == WandGesture::ThrustIn){
                        Serial.println("Gesture: ThrustIn");
                    } else if(event.gesture == WandGesture::ThrustOut){
                        Serial.println("Gesture: ThrustOut");
                    }
                    Serial.print("Sequence: "); Serial.println(event.sequence);
                    Serial.print("timestampMs: "); Serial.println(event.timestampMs);
                    gestureRecognizer.clearEvent();
                }
    
                
                Serial.print("Roll: ");
                Serial.print(pose.roll * 180.0f / PI, 3);
    
                Serial.print("  Pitch: ");
                Serial.print(pose.pitch * 180.0f / PI, 3);
    
                Serial.print("  Yaw: ");
                Serial.println(pose.yaw * 180.0f / PI, 3);
    
                Serial.print("MotionY: ");
                Serial.print(motion.ay, 3);
    
                Serial.print("  MotionX: ");
                Serial.print(motion.ax, 3);
    
                Serial.print("  MotionZ: ");
                Serial.println(motion.az, 3);
    
               // Serial.print("accelMag: ");
               // Serial.print(accelMag,3);
               // Serial.print("  accelTrust: ");
               // Serial.println(accelTrust,3);
    
                Serial.print("VelocityY: ");
                Serial.print(motion.vy, 3);
    
                Serial.print("  VelocityX: ");
                Serial.print(motion.vx, 3);
    
                Serial.print("  VelocityZ: ");
                Serial.println(motion.vz, 3);
                
                Serial.print("accelPitch: ");
                Serial.println(accelPitch);
                Serial.print("predictedPitch: ");
                Serial.println(predictedPitch);
                Serial.print("rollDiffTrust: ");
                Serial.println(rollDiffTrust);
                Serial.print("pitchDiffTrust: ");
                Serial.println(pitchDiffTrust);
                Serial.print("accelRollTrust: ");
                Serial.println(accelRollTrust);
                Serial.print("accelPitchTrust: ");
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
