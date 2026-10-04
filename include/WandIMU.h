
#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include "Globals.h"
#include "MPU9250.h"

// moved to Globals.h
//static constexpr float G = 9.80665f;

struct AccelCalibration {
    float biasX  = 0.0f;
    float biasY  = 0.0f;
    float biasZ  = 0.0f;

    float scaleX = 1.0f;
    float scaleY = 1.0f;
    float scaleZ = 1.0f;

    bool valid = false;
};

struct GyroCalibration {
    float biasX  = 0.0f;
    float biasY  = 0.0f;
    float biasZ  = 0.0f;

    bool valid = false;
};

struct SessionGyroCalibration {
    float biasX  = 0.0f;
    float biasY  = 0.0f;
    float biasZ  = 0.0f;
};

struct IMUData {
    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;

    float mx;
    float my;
    float mz;

    float temperature;
};


class WandIMU {
public:

    WandIMU(TwoWire &wire, uint8_t address);

    bool begin();
    bool read(IMUData &data);
    void runAccelCalibration(uint8_t buttonPin);
    void runGyroCalibration();
    bool loadCalibration();
    bool saveCalibration();

    bool gyroCalDone = false;
    bool gyroCalFirstCycle = true;
    uint32_t gyroCalStart = 0;


private:
    MPU9250 imu;
    Preferences preferences;

    AccelCalibration accelCal;
    GyroCalibration gyroCal;
    SessionGyroCalibration sessionGyroCal;


    static constexpr uint16_t CAL_ACCEL_SAMPLES = 200;
    static constexpr uint16_t CAL_GYRO_SAMPLES = 750;
    static constexpr uint16_t CAL_SAMPLE_DELAY_MS = 10;
    static constexpr uint32_t CAL_VERSION = 2;
    static constexpr uint32_t SESSION_GYRO_CAL_TIME_mS = 2000;   

    void waitForButtonPress(uint8_t buttonPin);
    void waitForButtonRelease(uint8_t buttonPin);

    void captureAccelAverage(float &x, float &y, float &z);
    void captureGyroAverage(float &x, float &y, float &z);

    bool computeAccelCalibration(
        float xPos, float xNeg,
        float yPos, float yNeg,
        float zPos, float zNeg
    );
    bool computeGyroCalibration(
        float gyX, float gyY, float gyZ
    );

    bool validateAccelCalibration() const;
    bool validateGyroCalibration() const;
    void printCalibration() const;

    float calGyroX = 0.0f;
    float calGyroY = 0.0f;
    float calGyroZ = 0.0f;
    int gyroCalCycles = 0;
    float MAX_CAL_GYRO_RATE = 0.05f;  // rad/sec
};