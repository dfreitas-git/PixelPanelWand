
#include "WandRadio.h"


bool WandRadio::begin(const uint8_t panelMac[6])
{
    memcpy(peerMac, panelMac, 6);

    // ESP-NOW can operate through the station interface.
    WiFi.mode(WIFI_STA);

    Serial.print("Wand WiFi MAC: ");
    Serial.println(WiFi.macAddress());

    // Initialize ESP-NOW.
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW initialization failed.");
        return false;
    }

    // Optional, but very useful while bringing the link up.
    if (esp_now_register_send_cb(onDataSent) != ESP_OK) {
        Serial.println("ESP-NOW send callback registration failed.");
        return false;
    }

    // Describe the PixelPanel as our peer.
    esp_now_peer_info_t peerInfo{};
    
    memcpy(peerInfo.peer_addr, peerMac, 6);

    // Channel 0 means use the current WiFi channel.
    peerInfo.channel = 0;

    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add PixelPanel ESP-NOW peer.");
        return false;
    }

    initialized = true;

    Serial.println("ESP-NOW initialized.");

    return true;
}


bool WandRadio::send(const WandPacket &wandPacket)
{
    if (!initialized) {
        return false;
    }

    esp_err_t result = esp_now_send(
        peerMac,
        reinterpret_cast<const uint8_t *>(&wandPacket),
        sizeof(wandPacket)
    );

    return result == ESP_OK;
}


void WandRadio::printLocalMac() const
{
    Serial.print("Local MAC: ");
    Serial.println(WiFi.macAddress());
}


void WandRadio::onDataSent( const uint8_t *macAddr, esp_now_send_status_t status)
{
    // Don't normally print successful transmissions here.
    // At 50 Hz that would make Serial output rather unpleasant.

    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.println("ESP-NOW delivery failed.");
    }
}