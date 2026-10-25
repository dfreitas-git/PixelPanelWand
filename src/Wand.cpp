
#include "Wand.h"


void Wand::updateContinuousState(const IMUData& imu, const WandPose& pose, const WandMotion& motion, float dt) {
}

void Wand::updateLED(){
    uint8_t R;
    uint8_t G;
    uint8_t B;
    if(flashing) {
        if(millis() > nextFlash) {
            if(ledOn) {
                ledOn=false;
                setLED(0,0,0);
                ledFlashCount--;
                if(ledFlashCount <= 0) {
                    flashing=false;
                }
            } else {
                ledOn=true;
                R= flashR;
                G= flashG;
                B= flashB;
            }
            nextFlash = millis() + flashDurationMs;
            setLED(R,G,B);
        }
    }
}

void Wand::setLED(const uint8_t R , const uint8_t G, uint8_t B){ 
    
    // Common anode so dutyCycle 0 is full bright, 255 is off
    ledcWrite(RED_CHANNEL, 255 - R);    
    ledcWrite(GREEN_CHANNEL, 255 - G);
    ledcWrite(BLUE_CHANNEL, 255 - B);  
}

void Wand::setFlashingLED( const uint8_t numberOfFlashes, const uint16_t durationMs, const uint8_t R , const uint8_t G, uint8_t B){ 
    
    // Common anode so dutyCycle 0 is full bright, 255 is off
    flashR =  R;
    flashG =  G;
    flashB =  B;
    flashDurationMs = durationMs;
    ledFlashCount = numberOfFlashes;
    flashing = true;
    ledOn = true;
    setLED(flashR, flashG, flashB);
    nextFlash = millis() + durationMs;

}