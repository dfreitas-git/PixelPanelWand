
#include "GestureRecognizer.h"

void GestureRecognizer::update( const IMUData& imu, const WandPose& pose, const WandMotion& motion, float dt) {
    detectThrust(motion);

    // Later:
    // detectSwipe(...);
    // detectCircle(...);
}

void GestureRecognizer::clearEvent() {
    event.gesture = WandGesture::None;
}

bool GestureRecognizer::eventAvailable() const
{
    return event.gesture != WandGesture::None;
}


void GestureRecognizer::detectThrust(const WandMotion& motion)
{
    constexpr float START_ACCEL       = 1.5f;
    constexpr float MIN_PEAK_VELOCITY = 0.15f;
    constexpr uint32_t MAX_THRUST_MS  = 500;
    constexpr uint32_t COOLDOWN_MS    = 300;

    uint32_t now = millis();

    switch (thrustState) {

        case ThrustState::Idle:

            if (fabsf(motion.ax) > START_ACCEL) {

                thrustDirection = (motion.ax > 0.0f) ? 1 : -1;

                thrustState = ThrustState::Tracking;
                thrustStartMs = now;
                thrustPeakVelocity = 0.0f;
            }
            break;


        case ThrustState::Tracking: {

            // Express velocity in the detected direction.
            float directedVelocity = motion.vx * thrustDirection;
            float directedAccel = motion.ax * thrustDirection;

            if (directedVelocity > thrustPeakVelocity) {
                thrustPeakVelocity = directedVelocity;
            }
            Serial.print("ThrustState: Tracking "); Serial.print(" directedVelocity: "); Serial.println(directedVelocity);

            // Acceleration has reversed: braking phase.
            if (directedAccel < 0.0f) {

                if (thrustPeakVelocity > MIN_PEAK_VELOCITY) {

                    WandGesture gesture =
                        (thrustDirection > 0)
                        ? WandGesture::ThrustIn
                        : WandGesture::ThrustOut;

                    emitGesture(gesture);
                }

                thrustState = ThrustState::Cooldown;
                thrustCooldownStartMs = now;
            }

            else if (now - thrustStartMs > MAX_THRUST_MS) {
                thrustState = ThrustState::Cooldown;
                thrustCooldownStartMs = now;
            }

            break;
        }

        case ThrustState::Cooldown:

            // We waif for some time plus acceleration must drop low enough before we allow rearming
            if ((now - thrustCooldownStartMs > COOLDOWN_MS) &&
                (fabsf(motion.ax) < REARM_ACCEL)) {

                thrustState = ThrustState::Idle;
            }
            break;
    }
}

void GestureRecognizer::detectSwipe(){
}

void GestureRecognizer::detectCircle(){
}

void GestureRecognizer::emitGesture(WandGesture gesture){
    static uint32_t sequence = 0;

    event.gesture = gesture;
    event.sequence = ++sequence;
    event.timestampMs = millis();
}
