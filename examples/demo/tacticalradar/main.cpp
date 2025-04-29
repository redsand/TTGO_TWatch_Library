#include <LilyGoLib.h>
#include <WiFi.h>
#include <NimBLEDevice.h>
#include "LV_Helper.h"
#include "radar.h"
#include "settings.h"


void handle_serial_command(String cmd) {
    cmd.trim();
    cmd.toLowerCase();

    if (cmd == "settings") {
        //open_settings();
    } 
    else if (cmd == "log") {
        File f = FFat.open("/detections.log", FILE_READ);
        if (!f) {
            Serial.println("[ERROR] Cannot open log file.");
            return;
        }
        Serial.println("=== Log Start ===");
        while (f.available()) {
            Serial.write(f.read());
        }
        Serial.println("\n=== Log End ===");
        f.close();
    } 
    else if (cmd == "unknown") {
        File f = FFat.open("/unknown_devices.log", FILE_READ);
        if (!f) {
            Serial.println("[ERROR] Cannot open log file.");
            return;
        }
        Serial.println("=== Log Start ===");
        while (f.available()) {
            Serial.write(f.read());
        }
        Serial.println("\n=== Log End ===");
        f.close();
    } 
    else if (cmd == "clear") {
        FFat.remove("/detections.log");
        FFat.remove("/unknown_devices.log");
        Serial.println("[OK] Log file cleared.");
    } 
    else if (cmd == "restart") {
        Serial.println("[INFO] Restarting...");
        delay(500);
        ESP.restart();
    }
    else if (cmd == "help") {
        Serial.println("[Commands Available]");
        Serial.println(" unknown   - Print unknown devices");
        Serial.println(" log       - Print detection log");
        Serial.println(" clear     - Clear detection log");
        Serial.println(" restart   - Restart device");
        Serial.println(" help      - Show commands");
    }
    else {
        Serial.println("[Unknown Command] Type 'help' for list.");
    }
}


void setup() {

    pinMode(BUTTON_PIN, INPUT_PULLUP); 
    
    Serial.begin(115200);
    Serial.println("Booting Watch...");

    watch.begin(NULL); 
    beginLvglHelper(false);

    pinMode(BOARD_TOUCH_INT, INPUT_PULLUP); 

    WiFi.mode(WIFI_OFF);    // Save battery
    NimBLEDevice::init(""); // BLE initialized

    if (!FFat.begin()) {
        Serial.println("[ERROR] FFat init failed, formatting...");
        if (FFat.format()) {
            FFat.begin();
            Serial.println("[OK] FFat formatted and ready.");
        } else {
            Serial.println("[CRITICAL] Failed to format FFat. Device may reboot.");
            delay(2000);
            ESP.restart();
        }
    }

    radar_setup(&watch);

    Serial.println("[READY] Type 'help' in serial terminal for commands.");
}

void loop() {
    // Serial.println("Radar loop.\n");
    radar_loop(&watch);

    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        handle_serial_command(cmd);
    }

}