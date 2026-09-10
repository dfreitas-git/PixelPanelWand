
#include <Arduino.h>
#include <Wire.h>

#include "WandIMU.h"


constexpr uint8_t BUTTON_PIN = 25;
constexpr uint32_t CALIBRATION_HOLD_MS = 2500;

WandIMU wandIMU(Wire, 0x68);


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

    Serial.println("Normal operation.");
}


void loop()
{
    IMUData imu;

    if (wandIMU.read(imu)) {

        Serial.print("Accel: ");
        Serial.print(imu.ax, 3);
        Serial.print('\t');
        Serial.print(imu.ay, 3);
        Serial.print('\t');
        Serial.println(imu.az, 3);

        Serial.print("Gyro: ");
        Serial.print(imu.gx, 3);
        Serial.print('\t');
        Serial.print(imu.gy, 3);
        Serial.print('\t');
        Serial.println(imu.gz, 3);

        Serial.println();
    }

    delay(100);
}