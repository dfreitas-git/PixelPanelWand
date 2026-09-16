
#include "WandIMU.h"
#include "PoseEstimator.h"

float PoseEstimator::attitudeTrust(float diff)
{
    if (diff <= DIFF_FULL_TRUST)
        return 1.0f;

    if (diff >= DIFF_ZERO_TRUST)
        return 0.0f;

    return 1.0f -
        (diff - DIFF_FULL_TRUST) /
        (DIFF_ZERO_TRUST - DIFF_FULL_TRUST);
}

void PoseEstimator::update(const IMUData &imu, float dt)
{
    float accelRoll = atan2f(
        -imu.ay,
        -imu.az
    );

    accelPitch = atan2f(
        imu.ax,
        sqrtf(imu.ay * imu.ay + imu.az * imu.az)
    );

    // We compute this to use later in deciding whether or not to trust the accel.  If we have a large
    // non gravity component, then we can't trust the result as much since motion acceleration is present.
    accelMag = sqrtf( imu.ax * imu.ax + imu.ay * imu.ay + imu.az * imu.az);

    if (!initialized) {
        pose.roll  = accelRoll;
        pose.pitch = accelPitch;
        pose.yaw   = 0.0f;

        initialized = true;
        return;
    }

    float error = fabsf(accelMag - G);

    float predictedRoll  = pose.roll  + imu.gx * dt;
    predictedPitch = pose.pitch + imu.gy * dt;

    // Assign our trust level
    float magnitudeTrust;

    if (error < 0.25f) {
        magnitudeTrust = 1.0f;
    }
    else if (error > 1.5f) {
        magnitudeTrust = 0.0f;
    }
    else {
        magnitudeTrust = 1.0f - (error - 0.25f) / (1.5f - 0.25f);
    }

    rollDiffTrust = attitudeTrust(fabsf(predictedRoll - accelRoll));
    pitchDiffTrust = attitudeTrust(fabsf(predictedPitch - accelPitch));

    accelRollTrust  = fmin(magnitudeTrust, rollDiffTrust);
    accelPitchTrust = fmin(magnitudeTrust, pitchDiffTrust);

    // Blending the result of the gyro and accelerometer depending on which we trust the most
    // for the current accelerometer/gyro results.
    // Gyro predicts where we moved,  accelerometer gently corrects that prediction (only when trustworthy)
    constexpr float tau = 0.5f;
    float baseAlpha = tau / (tau + dt);
    float rollAlpha = 1.0f - (1.0f - baseAlpha) * accelRollTrust;
    float pitchAlpha = 1.0f - (1.0f - baseAlpha) * accelPitchTrust;

    pose.roll = rollAlpha * predictedRoll + (1.0f - rollAlpha) * accelRoll;
    pose.pitch = pitchAlpha * predictedPitch + (1.0f - pitchAlpha) * accelPitch;
    pose.yaw += imu.gz * dt;

}

const WandPose &PoseEstimator::getPose() const
{
    return pose;
}

void PoseEstimator::setYaw(float yaw)
{
    pose.yaw = yaw;
}
