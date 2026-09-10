
#include "WandIMU.h"


WandIMU::WandIMU(TwoWire &wire, uint8_t address)
    : imu(wire, address)
{
}


bool WandIMU::begin()
{
    int status = imu.begin();

    if (status < 0) {
        Serial.print("MPU9250 initialization failed. Status = ");
        Serial.println(status);
        return false;
    }

    if (loadCalibration()) {
        Serial.println("IMU calibration loaded.");
        printCalibration();
    }
    else {
        Serial.println("No valid IMU calibration found.");
    }

    return true;
}


bool WandIMU::read(IMUData &data)
{
    if (imu.readSensor() < 0) {
        return false;
    }

    // Read raw library values.
    float rawAx = imu.getAccelX_mss();
    float rawAy = imu.getAccelY_mss();
    float rawAz = imu.getAccelZ_mss();
    /*
    Serial.print("=>");
    Serial.print(rawAx,3);
    Serial.print("\t");
    Serial.print(rawAy,3);
    Serial.print("\t");
    Serial.println(rawAz,3);
    */

    // Apply accererometer calibration
    data.ax = (rawAx - accelCal.biasX) * accelCal.scaleX;
    data.ay = (rawAy - accelCal.biasY) * accelCal.scaleY;
    data.az = (rawAz - accelCal.biasZ) * accelCal.scaleZ;

    // Apply gyro zero-rate bias correction.
    data.gx = imu.getGyroX_rads() - gyroCal.biasX;
    data.gy = imu.getGyroY_rads() - gyroCal.biasY;
    data.gz = imu.getGyroZ_rads() - gyroCal.biasZ;

    // Magnetometer are currently uncalibrated.
    data.mx = imu.getMagX_uT();
    data.my = imu.getMagY_uT();
    data.mz = imu.getMagZ_uT();

    data.temperature = imu.getTemperature_C();

    return true;
}


void WandIMU::captureAccelAverage(float &x, float &y, float &z)
{
    double sx = 0.0;
    double sy = 0.0;
    double sz = 0.0;

    // Important:
    // use the MPU9250 library values directly here, not our corrected
    // read() values, because we are determining the calibration itself.
    for (uint16_t i = 0; i < CAL_ACCEL_SAMPLES; i++) {

        imu.readSensor();

        sx += imu.getAccelX_mss();
        sy += imu.getAccelY_mss();
        sz += imu.getAccelZ_mss();

        delay(CAL_SAMPLE_DELAY_MS);
    }

    x = sx / CAL_ACCEL_SAMPLES;
    y = sy / CAL_ACCEL_SAMPLES;
    z = sz / CAL_ACCEL_SAMPLES;
}

void WandIMU::captureGyroAverage(float &x, float &y, float &z)
{
    double sx = 0.0;
    double sy = 0.0;
    double sz = 0.0;

    // Important:
    // use the MPU9250 library values directly here, not our corrected
    // read() values, because we are determining the calibration itself.
    for (uint16_t i = 0; i < CAL_GYRO_SAMPLES; i++) {

        imu.readSensor();

        sx += imu.getGyroX_rads();
        sy += imu.getGyroY_rads();
        sz += imu.getGyroZ_rads();

        delay(CAL_SAMPLE_DELAY_MS);
    }

    x = sx / CAL_GYRO_SAMPLES;
    y = sy / CAL_GYRO_SAMPLES;
    z = sz / CAL_GYRO_SAMPLES;
}

void WandIMU::waitForButtonRelease(uint8_t buttonPin)
{
    while (digitalRead(buttonPin) == LOW) {
        delay(10);
    }

    // Basic debounce.
    delay(50);
}


void WandIMU::waitForButtonPress(uint8_t buttonPin)
{
    while (digitalRead(buttonPin) == HIGH) {
        delay(10);
    }

    // Basic debounce.
    delay(50);

    // Wait until released so the same press cannot advance twice.
    waitForButtonRelease(buttonPin);
}


void WandIMU::runCalibration(uint8_t buttonPin)
{
    float xPos, xNeg;
    float yPos, yNeg;
    float zPos, zNeg;
    float gyX, gyY, gyZ;

    float dummy1, dummy2;

    // Save copies or original to restore in case calibration fails
    AccelCalibration oldAccelCal = accelCal;
    GyroCalibration oldGyroCal = gyroCal;

    Serial.println();
    Serial.println("================================");
    Serial.println("ACCELEROMETER CALIBRATION");
    Serial.println("================================");
    Serial.println();
    Serial.println("Keep the wand motionless for each measurement.");
    Serial.println("Press the button after placing it in the requested orientation.");
    Serial.println();

    // Ensure the boot/calibration-entry button press has been released.
    waitForButtonRelease(buttonPin);


    // ------------------------------------------------------------
    // Sensor X axis
    // Wand bottom edge down => approximately +X gravity reading
    // ------------------------------------------------------------

    Serial.println("1/6: Place BOTTOM EDGE of wand DOWN.");
    Serial.println("Press button when motionless.");
    waitForButtonPress(buttonPin);

    captureAccelAverage(xPos, dummy1, dummy2);

    Serial.print("X+ = ");
    Serial.println(xPos, 6);


    Serial.println();
    Serial.println("2/6: Place TOP EDGE of wand DOWN.");
    Serial.println("Press button when motionless.");
    waitForButtonPress(buttonPin);

    captureAccelAverage(xNeg, dummy1, dummy2);

    Serial.print("X- = ");
    Serial.println(xNeg, 6);


    // ------------------------------------------------------------
    // Sensor Y axis
    // Left edge down => approximately +Y gravity reading
    // ------------------------------------------------------------

    Serial.println();
    Serial.println("3/6: Place LEFT EDGE of wand DOWN.");
    Serial.println("Press button when motionless.");
    waitForButtonPress(buttonPin);

    captureAccelAverage(dummy1, yPos, dummy2);

    Serial.print("Y+ = ");
    Serial.println(yPos, 6);


    Serial.println();
    Serial.println("4/6: Place RIGHT EDGE of wand DOWN.");
    Serial.println("Press button when motionless.");
    waitForButtonPress(buttonPin);

    captureAccelAverage(dummy1, yNeg, dummy2);

    Serial.print("Y- = ");
    Serial.println(yNeg, 6);


    // ------------------------------------------------------------
    // Sensor Z axis
    //
    // From our measurements:
    // face DOWN gives positive Z
    // face UP gives negative Z
    // ------------------------------------------------------------

    Serial.println();
    Serial.println("5/6: Place FACE of wand DOWN.");
    Serial.println("Press button when motionless.");
    waitForButtonPress(buttonPin);

    captureAccelAverage(dummy1, dummy2, zPos);

    Serial.print("Z+ = ");
    Serial.println(zPos, 6);


    Serial.println();
    Serial.println("6/6: Place FACE of wand UP.");
    Serial.println("Press button when motionless.");
    waitForButtonPress(buttonPin);

    captureAccelAverage(dummy1, dummy2, zNeg);

    Serial.print("Z- = ");
    Serial.println(zNeg, 6);

    Serial.println();

    Serial.println("GYROSCOPE CALIBRATION");
    Serial.println("Place wand motionless on a stable surface.");
    Serial.println("Press button when ready.");

    waitForButtonPress(buttonPin);

    Serial.println("Hold still...");
    delay(500);
    captureGyroAverage(gyX, gyY, gyZ);

    Serial.print("gyX = ");
    Serial.print(gyX, 6);
    Serial.print("  gyY = ");
    Serial.print(gyY, 6);
    Serial.print("  gyZ = ");
    Serial.println(gyZ, 6);

    Serial.println();
    Serial.println("Computing calibration...");


    if (!computeAccelCalibration(
            xPos, xNeg,
            yPos, yNeg,
            zPos, zNeg)) {

        accelCal = oldAccelCal;
        gyroCal = oldGyroCal;

        Serial.println("ACCELEROMETER CALIBRATION FAILED.");
        Serial.println("Old calibration restored.");
        return;
    }

    if (!computeGyroCalibration(gyX, gyY, gyZ)) {

        accelCal = oldAccelCal;
        gyroCal = oldGyroCal;

        Serial.println("GYRO CALIBRATION FAILED.");
        Serial.println("Old calibration restored.");
        return;
    }

    printCalibration();

    if (saveCalibration()) {
        Serial.println("Calibration saved to NVS.");
    }
    else {
        Serial.println("ERROR: calibration could not be saved.");
    }

    Serial.println("Calibration complete.");
}


bool WandIMU::computeAccelCalibration(
    float xPos, float xNeg,
    float yPos, float yNeg,
    float zPos, float zNeg)
{
    AccelCalibration newCal;

    newCal.biasX = (xPos + xNeg) * 0.5f;
    newCal.biasY = (yPos + yNeg) * 0.5f;
    newCal.biasZ = (zPos + zNeg) * 0.5f;

    float xHalfSpan = (xPos - xNeg) * 0.5f;
    float yHalfSpan = (yPos - yNeg) * 0.5f;
    float zHalfSpan = (zPos - zNeg) * 0.5f;

    // Protect against bad or reversed measurements.
    if (xHalfSpan <= 0.0f ||
        yHalfSpan <= 0.0f ||
        zHalfSpan <= 0.0f) {

        return false;
    }

    newCal.scaleX = G / xHalfSpan;
    newCal.scaleY = G / yHalfSpan;
    newCal.scaleZ = G / zHalfSpan;

    newCal.valid = true;


    // Don't replace the current calibration unless the new one passes
    // sanity checks.
    AccelCalibration oldCal = accelCal;
    accelCal = newCal;

    if (!validateAccelCalibration()) {
        accelCal = oldCal;
        return false;
    }

    return true;
}

bool WandIMU::computeGyroCalibration(
    float gyX, float gyY, float gyZ)
{
    GyroCalibration newCal;

    newCal.biasX = gyX;
    newCal.biasY = gyY;
    newCal.biasZ = gyZ;

    newCal.valid = true;


    // Don't replace the current calibration unless the new one passes
    // sanity checks.
    GyroCalibration oldCal = gyroCal;
    gyroCal = newCal;

    if (!validateGyroCalibration()) {
        gyroCal = oldCal;
        return false;
    }

    return true;
}


bool WandIMU::validateAccelCalibration() const
{
    if (!accelCal.valid) {
        return false;
    }

    // Fairly generous limits for now. Once we collect more real MPU9250
    // calibration data, these can be tightened.
    if (accelCal.scaleX < 0.80f || accelCal.scaleX > 1.20f) {
        return false;
    }

    if (accelCal.scaleY < 0.80f || accelCal.scaleY > 1.20f) {
        return false;
    }

    if (accelCal.scaleZ < 0.80f || accelCal.scaleZ > 1.20f) {
        return false;
    }

    return true;
}

bool WandIMU::validateGyroCalibration() const
{
    if (!gyroCal.valid) {
        return false;
    }

    // Fairly generous limits for now. Once we collect more real MPU9250
    // calibration data, these can be tightened.
    if (fabsf(gyroCal.biasX) > 0.10f) {
        return false;
    }
    if (fabsf(gyroCal.biasY) > 0.10f) {
        return false;
    }
    if (fabsf(gyroCal.biasZ) > 0.10f) {
        return false;
    }

    return true;
}


bool WandIMU::saveCalibration()
{
    if (!accelCal.valid) {
        return false;
    }
    if (!gyroCal.valid) {
        return false;
    }

    if (!preferences.begin("wandimu", false)) {
        return false;
    }

    bool ok = true;

    ok &= (preferences.putUInt("version", CAL_VERSION) != 0);

    ok &= (preferences.putFloat("axb", accelCal.biasX) != 0);
    ok &= (preferences.putFloat("ayb", accelCal.biasY) != 0);
    ok &= (preferences.putFloat("azb", accelCal.biasZ) != 0);

    ok &= (preferences.putFloat("axs", accelCal.scaleX) != 0);
    ok &= (preferences.putFloat("ays", accelCal.scaleY) != 0);
    ok &= (preferences.putFloat("azs", accelCal.scaleZ) != 0);

    ok &= (preferences.putFloat("gxb", gyroCal.biasX) != 0);
    ok &= (preferences.putFloat("gyb", gyroCal.biasY) != 0);
    ok &= (preferences.putFloat("gzb", gyroCal.biasZ) != 0);

    preferences.end();

    return ok;
}


bool WandIMU::loadCalibration()
{
    if (!preferences.begin("wandimu", true)) {
        return false;
    }

    uint32_t version = preferences.getUInt("version", 0);

    if (version != CAL_VERSION) {
        preferences.end();
        return false;
    }

    AccelCalibration loadedAccl;
    GyroCalibration loadedGyro;

    loadedAccl.biasX = preferences.getFloat("axb", NAN);
    loadedAccl.biasY = preferences.getFloat("ayb", NAN);
    loadedAccl.biasZ = preferences.getFloat("azb", NAN);

    loadedGyro.biasX = preferences.getFloat("gxb", NAN);
    loadedGyro.biasY = preferences.getFloat("gyb", NAN);
    loadedGyro.biasZ = preferences.getFloat("gzb", NAN);

    loadedAccl.scaleX = preferences.getFloat("axs", NAN);
    loadedAccl.scaleY = preferences.getFloat("ays", NAN);
    loadedAccl.scaleZ = preferences.getFloat("azs", NAN);

    preferences.end();

    if (!isfinite(loadedAccl.biasX) ||
        !isfinite(loadedAccl.biasY) ||
        !isfinite(loadedAccl.biasZ) ||
        !isfinite(loadedGyro.biasX) ||
        !isfinite(loadedGyro.biasY) ||
        !isfinite(loadedGyro.biasZ) ||
        !isfinite(loadedAccl.scaleX) ||
        !isfinite(loadedAccl.scaleY) ||
        !isfinite(loadedAccl.scaleZ)) {

        return false;
    }

    loadedAccl.valid = true;
    loadedGyro.valid = true;

    AccelCalibration oldCal = accelCal;
    accelCal = loadedAccl;

    GyroCalibration oldGyroCal = gyroCal;
    gyroCal = loadedGyro;

    if (!validateAccelCalibration() || !validateGyroCalibration()) {
        accelCal = oldCal;
        gyroCal = oldGyroCal;
        return false;
    }

    return true;
}


void WandIMU::printCalibration() const
{
    Serial.println();
    Serial.println("Accelerometer calibration:");

    Serial.print("Bias:  ");
    Serial.print(accelCal.biasX, 6);
    Serial.print("\t");
    Serial.print(accelCal.biasY, 6);
    Serial.print("\t");
    Serial.println(accelCal.biasZ, 6);

    Serial.print("Scale: ");
    Serial.print(accelCal.scaleX, 6);
    Serial.print("\t");
    Serial.print(accelCal.scaleY, 6);
    Serial.print("\t");
    Serial.println(accelCal.scaleZ, 6);
    Serial.println();

    Serial.println("Gyroscope calibration:");
    Serial.print("Bias:  ");
    Serial.print(gyroCal.biasX, 6);
    Serial.print("\t");
    Serial.print(gyroCal.biasY, 6);
    Serial.print("\t");
    Serial.println(gyroCal.biasZ, 6);

    Serial.println();
}