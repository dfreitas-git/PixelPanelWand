
#include "WandIMU.h"
#include "PoseEstimator.h"

void PoseEstimator::update(const IMUData &imu, float dt)
{
    float accelRoll = atan2f(
        -imu.ay,
        -imu.az
    );

    float accelPitch = atan2f(
        imu.ax,
        sqrtf(imu.ay * imu.ay + imu.az * imu.az)
    );

    if (!initialized) {
        pose.roll  = accelRoll;
        pose.pitch = accelPitch;
        pose.yaw   = 0.0f;

        initialized = true;
        return;
    }

    // Blending the result of the gyro and accelerometer.  Mostly use the gyro, then add a small
    // percent of the accel to eliminate gyro drift.
    constexpr float tau = 0.5f;    // seconds
    float alpha = tau / (tau + dt);

    pose.roll = alpha * (pose.roll + imu.gx * dt) + (1.0f - alpha) * accelRoll;
    pose.pitch = alpha * (pose.pitch + imu.gy * dt) + (1.0f - alpha) * accelPitch;
    pose.yaw += imu.gz * dt;
}

const WandPose &PoseEstimator::getPose() const
{
    return pose;
}