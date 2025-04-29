#pragma once

#include <vector>
#include <Arduino.h>
#include <LilyGoLib.h>

// Structure to represent a detected signal
struct SignalSource {
    String source;        // SSID or BLE name
    String uuid;         // mac or ble address
    float strength;       // RSSI
    String type;          // "WiFi" or "BLE"
    unsigned long detectedAt;  // millis() timestamp
    float angle;          // Angle in degrees (0-360)
    double latitude;
    double longitude;
    String manufacturer;  // Manufacturer detected (from MAC if available)
    String deviceType;    // "Body Cam" or "Taser" or other
    String extra;         // Extra data from BLE or WiFi
};

// Exposed Functions
void radar_setup(LilyGoLib* watch);
void radar_loop(LilyGoLib* watch);

// Exposed for settings
void open_settings();

// Internal helpers (optional if you want)
void detectWiFi();
void detectBLE();
void gps_update();
void draw_radar();

// Compass functions
void compass_setup();
float read_compass_heading();

// Heading management
void update_current_heading();