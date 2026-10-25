
#include "GestureRecognizer.h"
#include "Wand.h"

void GestureRecognizer::update( const IMUData& imu, const WandPose& pose, const WandMotion& motion, float dt) {

    float axialAccel = fabsf(motion.ax);
    float lateralAccel = hypotf(motion.aRight, motion.aUp);
    switch (activeGesture) {

        case ActiveGesture::None:

            // Thrust has priority
            if (axialAccel > THRUST_START_ACCEL) {
                activeGesture = ActiveGesture::Thrust;
                detectThrust(motion);
            }
            else if (lateralAccel > SWIPE_START_ACCEL) {
                activeGesture = ActiveGesture::Swipe;
                detectSwipe(motion,dt);
            }
            break;

        case ActiveGesture::Thrust:

            detectThrust(motion);

            if (thrustState == ThrustState::Idle)
                activeGesture = ActiveGesture::None;
            break;

        case ActiveGesture::Swipe:

            detectSwipe(motion,dt);

            if (swipeState == SwipeState::Idle)
                activeGesture = ActiveGesture::None;
            break;
    }

    // Later:
    // detectCircle(...);
}

bool GestureRecognizer::eventAvailable() const
{
    return event.gesture != WandGesture::None;
}


void GestureRecognizer::detectThrust(const WandMotion& motion)
{

    uint32_t now = millis();

    // See if we have a thrust movement
    switch (thrustState) {

        case ThrustState::Idle:

            if (fabsf(motion.ax) > THRUST_START_ACCEL) {

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
            //Serial.print(F("ThrustState: Tracking ")); Serial.print(F(" directedVelocity: ")); Serial.println(directedVelocity);

            // Acceleration has reversed: braking phase.
            if (directedAccel < 0.0f) {

                if (thrustPeakVelocity > THRUST_MIN_PEAK_VELOCITY) {

                    WandGesture gesture =
                        (thrustDirection > 0)
                        ? WandGesture::ThrustIn
                        : WandGesture::ThrustOut;

                    emitGesture(gesture);
                }

                thrustState = ThrustState::Cooldown;
                thrustCooldownStartMs = now;

            } else if (now - thrustStartMs > THRUST_MAX_THRUST_MS) {
                thrustState = ThrustState::Cooldown;
                thrustCooldownStartMs = now;
            }
          }
          break;

        case ThrustState::Cooldown:

            // We wait for some time plus acceleration must drop low enough before we allow rearming
            if ((now - thrustCooldownStartMs > THRUST_COOLDOWN_MS) && (fabsf(motion.ax) < REARM_ACCEL)) {
                thrustState = ThrustState::Idle;
            }
            break;
    }
}

void GestureRecognizer::detectSwipe(const WandMotion& motion, float dt)
{
    constexpr float SWIPE_START_ACCEL       = 1.5f;
    constexpr float SWIPE_MIN_PEAK_VELOCITY = 0.15f;
    constexpr uint32_t SWIPE_ACCUMULATE_MS  = 120;
    constexpr uint32_t SWIPE_COOLDOWN_MS    = 300;

    uint32_t now = millis();

    // See if we have a swipe movement
    switch (swipeState) {

        case SwipeState::Idle:

            if (hypotf(motion.aRight, motion.aUp) > SWIPE_START_ACCEL) {
                swipeState = SwipeState::Candidate;
                swipeStartMs = now;
                swipeDtStartMs = now;
                swipePeakVelocity = 0.0f;
                sumRight = motion.aRight * dt * 1000.0f;  // dt is in uS so convert to mS
                sumUp = motion.aUp * dt * 1000.0f;
            }
            break;


        case SwipeState::Candidate: 

            if (now - swipeStartMs > SWIPE_ACCUMULATE_MS) {
                swipeState = SwipeState::Direction;
            } else {
                sumRight += motion.aRight * (now - swipeDtStartMs);
                sumUp += motion.aUp * (now - swipeDtStartMs);
                swipeDtStartMs = millis();
            }
            break;

        case SwipeState::Direction: 

            swipeDirection = atan2(sumUp, sumRight);
            Serial.print("Swipe direction in degrees: ");
            Serial.println(swipeDirection * 180 / PI);

            swipeState = SwipeState::Cooldown;
            swipeCooldownStartMs = now;
            break;

        case SwipeState::Cooldown:
            // We wait for some time plus acceleration must drop low enough before we allow rearming
            float lateralAccel = hypotf(motion.aRight, motion.aUp);

            if ((now - swipeCooldownStartMs > SWIPE_COOLDOWN_MS) && lateralAccel < REARM_ACCEL) {
                swipeState = SwipeState::Idle;
            }
            break;
    }
}

void GestureRecognizer::detectCircle(){
}

void GestureRecognizer::emitGesture(WandGesture gesture){
    static uint32_t eventSequence = 0;

    event.gesture = gesture;
    event.eventSequence = ++eventSequence;
    event.eventTimestampMs = millis();

}