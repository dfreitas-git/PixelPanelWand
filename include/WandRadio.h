
#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include "WandProtocol.h"

class WandRadio {
public:
    bool begin(const uint8_t panelMac[6]);
    bool send(const WandPacket &wandPacket);
    void printLocalMac() const;

private:
    uint8_t peerMac[6];

    bool initialized = false;
    static void onDataSent( const uint8_t *macAddr, esp_now_send_status_t status);
};