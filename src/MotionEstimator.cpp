
#include "WandIMU.h"
#include "MotionEstimator.h"
#include "PoseEstimator.h"


void MotionEstimator::update(const IMUData &imu, const WandPose &pose,float dt)
{

    float gravityX = G * sinf(pose.pitch);
    float gravityY = -G * sinf(pose.roll) * cosf(pose.pitch);
    float gravityZ = -G * cosf(pose.roll) * cosf(pose.pitch);

    motion.ax = imu.ax - gravityX;
    motion.ay = imu.ay - gravityY;
    motion.az = imu.az - gravityZ;

    // deadband for small accelerations so we don't constantly move location
    if (fabsf(motion.ax) < 0.1f){
        motion.ax = 0.0f;
    }
    if (fabsf(motion.ay) < 0.1f){
        motion.ay = 0.0f;
    }
    if (fabsf(motion.az) < 0.1f){
        motion.az = 0.0f;
    }
    motion.vx += motion.ax * gainX * dt;
    motion.vy += motion.ay * gainY * dt;
    motion.vz += motion.az * gainZ * dt;

    // Make the damping a function of the sampling time
    float velocityTau = 0.15f;  // seconds
    float damping = expf(-dt / velocityTau);

    motion.vx *= damping;
    motion.vy *= damping;
    motion.vz *= damping;

}