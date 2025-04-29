#include "radar.h"
#include "settings.h"
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <TinyGPSPlus.h>
#include <QMC5883LCompass.h>
#include <FFat.h>

static LilyGoLib* _watch;
//static std::vector<SignalSource> signals;
static unsigned long last_scan_time = 0;
static int detectionThreshold = -70;

static SignalSource lastDetections[50];
static int detectionIndex = 0;
static int detectionCtr = 0;

static lv_obj_t* radar_screen = nullptr;
static lv_obj_t* battery_label = nullptr;
static lv_obj_t* battery_icon = nullptr;

double currentLat = 0.0;
double currentLon = 0.0;
double lastLat = 0.0;
double lastLon = 0.0;
float currentHeading = 0.0;
bool firstFix = true;
unsigned long lastTouchTime = 0;
bool screenOn = true;

#define SCREEN_IDLE_TIMEOUT 30000  // 30 seconds

#ifndef GPSSerial
#define GPSSerial Serial1
#endif

TinyGPSPlus gps;

// ---- Compass ----
bool compass_available = false;
QMC5883LCompass compass;


unsigned long lastGpsDataTime = 0;
bool gpsConnected = false;
unsigned long gpsLastResetAttempt = 0;
const unsigned long GPS_RESET_TIMEOUT = 15000; // 15 seconds without data = reset
const unsigned long GPS_RESET_COOLDOWN = 5000; // Wait 5s after reset

void gps_diagnostics();

static int sweep_angle = 0;

void draw_sweep() {
    int cx = 105;
    int cy = 105;
    int radius = 100;

    // Calculate end point based on sweep_angle
    float angle_rad = radians(sweep_angle);
    int x = cx + radius * cos(angle_rad);
    int y = cy + radius * sin(angle_rad);

    static lv_point_t line_points[2];
    line_points[0].x = cx;
    line_points[0].y = cy;
    line_points[1].x = x;
    line_points[1].y = y;

    lv_obj_t* line = lv_line_create(radar_screen);
    lv_line_set_points(line, line_points, 2);
    lv_obj_set_style_line_color(line, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);
    lv_obj_set_style_line_width(line, 2, LV_PART_MAIN);
    lv_obj_clear_flag(line, LV_OBJ_FLAG_CLICKABLE);
}


void save_detection(const SignalSource& sig) {
    // Save into circular buffer
    lastDetections[detectionIndex] = sig;
    detectionIndex = (detectionIndex + 1) % 50;
    detectionCtr++;
    if(detectionCtr >= 50) detectionCtr = 50;
    // Append to FFat
    File f = FFat.open("/detections.log", FILE_APPEND);
    if (f) {
        f.printf("%s,%s,%s,%s,%s,%d,%.6f,%.6f,%s\n", 
                 sig.type.c_str(), sig.source.c_str(), sig.uuid.c_str(), sig.manufacturer.c_str(), sig.deviceType.c_str(), 
                 (int)sig.strength, sig.latitude, sig.longitude, sig.extra.c_str());
        f.close();
    }
}

void add_or_update_detection(SignalSource sig) {
    bool found = false;

    // Check existing in `signals`
    //for (auto& existing : signals) {
    int i;
    for(i=0; i < detectionCtr; i+=1) {
        SignalSource existing = lastDetections[i];
        if (existing.source == sig.source && existing.type == sig.type && existing.uuid == sig.uuid) {
            // Update existing signal with latest info
            existing.strength = sig.strength;
            existing.latitude = sig.latitude;
            existing.longitude = sig.longitude;
            existing.angle = sig.angle;
            existing.detectedAt = millis();
            found = true;
            break;
        }
    }

    if (!found) {
        // New device detected
        Serial.println("New device detected: " + sig.source);
        _watch->vibrate(50);
        save_detection(sig);
        //signals.push_back(sig);
    }
}

bool is_new_detection(const SignalSource& candidate) {
    for (int i = 0; i < detectionCtr; i++) {
        if (lastDetections[i].source == candidate.source) {
            return false; // Already detected
        }
    }
    return true; // New device
}

void compass_setup() {
    Serial.println("Initializing Compass...");
    compass.init();
    //compass.setReset();
    compass.setCalibration(-10, 10, -10, 10, -10, 10);
    compass_available = false; // not available
    
}

float read_compass_heading() {
    if (!compass_available) return 0.0;
    compass.read();
    float heading =  (float ) compass.getAzimuth();
    Serial.println("Compass Heading: " + String(heading));
    return heading;
}

bool gps_has_fix() {
    return gps.location.isValid() && gps.satellites.isValid() && gps.satellites.value() >= 4;
}

void gps_reset() {
    Serial.println("Resetting GPS serial...");

    GPSSerial.end();
    delay(500);

    // Restart GPSSerial (double-check correct pins and baud!)
    GPSSerial.begin(38400, SERIAL_8N1, SHIELD_GPS_RX, SHIELD_GPS_TX);
    delay(500);

    Serial.println("GPS serial restarted.");
}

// ---- GPS ----

void gps_update() {
    /*
    bool gotNewData = false;

    while (GPSSerial.available()) {
        char c = GPSSerial.read();
        gps.encode(c);
        gotNewData = true;
    }
    */

    if (gps.location.isValid()) {
        lastGpsDataTime = millis(); // We received something
        gpsConnected = true;

        if (gps.speed.isValid() && gps.speed.kmph() > 1.0) { 
            if (gps.course.isValid()) {
                currentHeading = gps.course.deg();
                Serial.printf("GPS Moving Heading: %.2f degrees\n", currentHeading);
            }
        } else if (gps.location.isValid()) {
            double lat = gps.location.lat();
            double lon = gps.location.lng();

            Serial.printf("GPS Location: %.6f, %.6f\n", lat, lon);

            if (!firstFix) {
                double dLon = radians(lon - lastLon);
                double lat1 = radians(lastLat);
                double lat2 = radians(lat);

                float y = sin(dLon) * cos(lat2);
                float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);

                float heading = atan2(y, x) * 180.0 / PI;
                if (heading < 0) heading += 360.0;
                currentHeading = heading;

                Serial.printf("Calculated Static Heading: %.2f degrees\n", currentHeading);
            }

            lastLat = currentLat = lat;
            lastLon = currentLon = lon;
            firstFix = false;
        } else {
            Serial.println("GPS Location invalid.");
        }
    }

    // --- Auto Recovery if no data for too long ---
    if ((millis() - lastGpsDataTime > GPS_RESET_TIMEOUT) && (millis() - gpsLastResetAttempt > GPS_RESET_COOLDOWN)) {
        Serial.println("[WARN] GPS appears stalled, resetting GPSSerial...");
        gps_reset();
        gpsLastResetAttempt = millis();
    }
}
/*
void gps_update() {
    if (gps.speed.isValid() && gps.speed.kmph() > 1.0) { // Moving at least 1 km/h
        if (gps.course.isValid()) {
            currentHeading = gps.course.deg();
            Serial.printf("GPS Heading: %.2f degrees\n", currentHeading);
        }
    } else {
     
      if (gps.location.isValid()) {
        Serial.println("GPS Location: " + String(gps.location.lat(), 6) + ", " + String(gps.location.lng(), 6));
        double lat = gps.location.lat();
        double lon = gps.location.lng();

        if (!firstFix) {
            double dLon = radians(lon - lastLon);
            double lat1 = radians(lastLat);
            double lat2 = radians(lat);

            float y = sin(dLon) * cos(lat2);
            float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);

            float heading = atan2(y, x) * 180.0 / PI;
            if (heading < 0) heading += 360.0;
            currentHeading = heading;

            
        }

        lastLat = currentLat = lat;
        lastLon = currentLon = lon;
        firstFix = false;
      } else Serial.println("GPS Location invalid.");
    } 
}*/

void update_current_heading() {
    if (gps.location.isValid()) {
        Serial.println("GPS Location valid.");
        gps_update();  // Heading already calculated
    } else if (compass_available) {
        currentHeading = read_compass_heading();
    } else { 
        Serial.println("No GPS or Compass available.");
        // gps_diagnostics();
    }

    if(currentHeading > 0)
        Serial.println("Current Heading: " + String(currentHeading));
    
}

// ---- MAC vendor mapping ----
String lookupManufacturer(const String& mac) {
    if (mac.startsWith("00:16:53") || mac.startsWith("70:88:6B") || mac.startsWith("00:25:DF") || mac.startsWith("00:58:28") || mac.startsWith("00:C0:D4")) return "Axon";         // Axon/Taser
    if (mac.startsWith("B8:27:EB")) return "Reveal";
    if (mac.startsWith("00:1D:43")) return "Motorola";
    return "";
}

String classifyDevice(String name) {
    name.toLowerCase();
    if (name.startsWith("ydxj_") || name.indexOf("axon") != -1 || name.indexOf("ds-mcw405") != -1 || name.indexOf("vb400") != -1 || name.indexOf("zednx") != -1) return "Body Cam";
    if (name.indexOf("taser") != -1 || name.indexOf("x2") != -1 || name.indexOf("x26") != -1) return "Taser";
    return "Unknown";
}

// ---- Detection ----
void detectWiFi() {
    int n = WiFi.scanNetworks();
    for (int i = 0; i < n; ++i) {
        String ssid = WiFi.SSID(i);
        String bssid = WiFi.BSSIDstr();
        int32_t rssi = WiFi.RSSI(i);
        ssid.toLowerCase();
        String mfg = lookupManufacturer(bssid);
        if ( ssid.startsWith("bl4ck") || ssid.startsWith("ydxj_") || ssid.startsWith("zednx") || ssid.startsWith("ds-mcw405") || ssid.indexOf("axon") != -1 || ssid.indexOf("vb400") != -1 || ssid.indexOf("taser") != -1 || mfg.length() != 0)
        {

            SignalSource sig;
            sig.source = ssid;
            sig.strength = rssi;
            sig.type = "WiFi";
            sig.detectedAt = millis();
            if(currentHeading > 0)
                sig.angle = currentHeading;
            else
                sig.angle = random(0, 360); // Default to 0 if heading is not available
            sig.latitude = currentLat;
            sig.longitude = currentLon;
            sig.manufacturer = mfg;
            sig.deviceType = classifyDevice(ssid);
            sig.uuid = bssid;
            sig.extra = String(WiFi.channel(i));
            sig.extra += "|" + String(WiFi.encryptionType(i)) + "|" + String(WiFi.RSSI(i)); 

            Serial.println("[WiFi] " + ssid + " (" + sig.manufacturer + ") [" + sig.uuid + "] detected");

            add_or_update_detection(sig);
        }
    }
}

class RadarBLEScan : public NimBLEAdvertisedDeviceCallbacks {
    
    void onResult(NimBLEAdvertisedDevice* advertisedDevice) override {
        String name = advertisedDevice->getName().c_str();
        String address = advertisedDevice->getAddress().toString().c_str();
        int rssi = advertisedDevice->getRSSI();
        name.toLowerCase();
        String mfg = lookupManufacturer(address);

        if ( name.startsWith("ydxj") || name.indexOf("zednx") != -1 || name.indexOf("axon") != -1 || name.indexOf("vb400") != -1 || name.indexOf("taser") != -1 || mfg.length() != 0)
        {
            SignalSource sig;
            sig.source = name;
            sig.strength = rssi;
            sig.type = "BLE";
            sig.uuid = advertisedDevice->getAddress().toString().c_str();
            sig.detectedAt = millis();
            // sig.angle = currentHeading;
            if(currentHeading > 0)
                sig.angle = currentHeading;
            else
                sig.angle = random(0, 360); // Default to 0 if heading is not available
            sig.latitude = currentLat;
            sig.longitude = currentLon;
            sig.manufacturer = mfg;
            sig.deviceType = classifyDevice(name);
            sig.extra = advertisedDevice->getManufacturerData().c_str();
            Serial.println("[BLE] " + name + " (" + sig.manufacturer + ") [" + sig.uuid + "] detected");

            add_or_update_detection(sig);
        } else if (advertisedDevice->haveManufacturerData()) {
            std::string mfgData = advertisedDevice->getManufacturerData();
            if (mfgData.length() >= 2) {
                uint16_t companyID = ((uint8_t)mfgData[1] << 8) | (uint8_t)mfgData[0];
                bool detected = false;
                String vendor = "Unknown";
                String deviceType = "Unknown";
                switch (companyID) {
                    case 0x2E03: // Axon Enterprise
                        vendor = "Axon";
                        deviceType = "Body Camera / Taser";
                        detected = true;
                        break;
                    case 0x03DE: // Reveal Media
                        vendor = "Reveal Media";
                        deviceType = "Body Camera";
                        detected = true;
                        break;
                    case 0x000A: // Motorola
                        vendor = "Motorola";
                        deviceType = "Body Camera";
                        detected = true;
                        break;
                    // Add more company IDs here!
                }

                if(detected) {
                    SignalSource sig;
                    sig.source = name;
                    sig.uuid = advertisedDevice->getAddress().toString().c_str();
                    sig.strength = rssi;
                    sig.extra = advertisedDevice->getManufacturerData().c_str();
                    sig.type = "BLE";
                    sig.detectedAt = millis();
                    // sig.angle = currentHeading;
                    if(currentHeading > 0)
                        sig.angle = currentHeading;
                    else
                        sig.angle = random(0, 360); // Default to 0 if heading is not available
                    sig.latitude = currentLat;
                    sig.longitude = currentLon;
                    sig.manufacturer = mfg;
                    sig.deviceType = classifyDevice(name);
                    
                    Serial.println("[BLE] " + name + " (" + sig.manufacturer + ") [" + sig.uuid + "] detected");

                    add_or_update_detection(sig);
                    // Save detection to FFat    
                }
            }
        }    
    }


};

void detectBLE() {
    NimBLEScan* pScan = NimBLEDevice::getScan();
    pScan->setAdvertisedDeviceCallbacks(new RadarBLEScan(), false);
    pScan->setActiveScan(true);
    pScan->start(5, false);
}

// ---- Radar UI ----
void draw_radar() {

    if (radar_screen == nullptr) {
        radar_screen = lv_obj_create(lv_scr_act());
        lv_obj_set_size(radar_screen, 240, 240);
        lv_obj_center(radar_screen);
        lv_obj_set_style_bg_color(radar_screen, lv_color_black(), LV_PART_MAIN);
    }
    lv_obj_clean(radar_screen);

    int cx = 120;
    int cy = 120;
    int radius = 100;

    // Battery status
    uint16_t voltage = _watch->getBattVoltage();
    int batteryPercent = map(voltage, 3300, 4200, 0, 100);
    batteryPercent = constrain(batteryPercent, 0, 100);

    battery_label = lv_label_create(radar_screen);
    lv_label_set_text_fmt(battery_label, "%d%%", batteryPercent);
    lv_obj_align(battery_label, LV_ALIGN_TOP_RIGHT, -1, 0);

    //Serial.println("Drawing radar circles...");
    // Radar circles
    for (int r = radius; r > 0; r -= 33) {
        lv_obj_t* circle = lv_arc_create(radar_screen);
        lv_obj_set_size(circle, r * 2, r * 2);
        lv_obj_center(circle);
        lv_arc_set_bg_angles(circle, 0, 360);
        lv_obj_clear_flag(circle, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_style_arc_color(circle, lv_palette_main(LV_PALETTE_GREY), LV_PART_MAIN);
        lv_obj_set_style_arc_width(circle, 1, LV_PART_MAIN);
        lv_obj_set_style_bg_color(circle, lv_color_black(), LV_PART_MAIN);
    }

    // Serial.println("Drawing signals...");
    // Draw detected signals
    int i;
    for(i=0; i < detectionCtr; i+=1) {
    //for (auto& sig : signals) {
        SignalSource sig = lastDetections[i];
        int strength_radius = map(sig.strength, -100, -30, radius, 0);
        strength_radius = constrain(strength_radius, 0, radius);
        float angle_rad = radians(sig.angle);
        int x = cx + strength_radius * cos(angle_rad);
        int y = cy + strength_radius * sin(angle_rad);

        lv_obj_t* dot = lv_obj_create(radar_screen);
        lv_obj_set_size(dot, 8, 8);
        lv_obj_align(dot, LV_ALIGN_CENTER, x - cx, y - cy);
        lv_obj_set_style_radius(dot, 4, LV_PART_MAIN);
        //lv_obj_set_style_bg_color(dot, lv_color_black(), LV_PART_MAIN);
        uint8_t normalized_strength = constrain(map(sig.strength, -100, -30, 255, 0), 0, 255);
        //Serial.printf("%s -> %d (%d) norm: %d", sig.source, sig.strength, strength_radius, normalized_strength);
        lv_color_t color;
        if (normalized_strength > 170) {
            color = lv_palette_main(LV_PALETTE_GREEN);
        } else if (normalized_strength > 85) {
            color = lv_palette_main(LV_PALETTE_YELLOW);
        } else {
            color = lv_palette_main(LV_PALETTE_RED);
            //_watch->vibrate(50);  // Vibrate for high signal strength
        }
        lv_obj_set_style_bg_color(dot, color, LV_PART_MAIN);
        String label_text;
        if(sig.source.length() > 0) label_text +=  sig.source;
        else label_text += "?";
        if(sig.manufacturer.length() > 0) label_text += " (" + sig.manufacturer + ")";
        if(sig.deviceType.length() > 0) label_text += " [" + sig.deviceType + "]";


        lv_obj_t* label = lv_label_create(radar_screen);
        lv_label_set_text(label, label_text.c_str());
        lv_obj_align_to(label, dot, LV_ALIGN_OUT_RIGHT_MID, 2, 0);  // Align text to right of dot
        lv_obj_set_style_text_color(label, lv_color_white(), LV_PART_MAIN);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_12, LV_PART_MAIN);
    }

        // === GPS Icon ===
    lv_obj_t* gps_icon = lv_label_create(radar_screen);

    if (gps_has_fix()) {
        lv_label_set_text(gps_icon, LV_SYMBOL_WIFI);  // You can choose other symbols!
        lv_obj_set_style_text_color(gps_icon, lv_palette_main(LV_PALETTE_GREEN), LV_PART_MAIN);
    } else {
        lv_label_set_text(gps_icon, LV_SYMBOL_CLOSE);  // Cross mark if no GPS fix
        lv_obj_set_style_text_color(gps_icon, lv_palette_main(LV_PALETTE_RED), LV_PART_MAIN);
    }

    // Position near battery icon
    //lv_obj_set_pos(gps_icon, 1, 5);  // Adjust X/Y position if needed
    lv_obj_align(gps_icon, LV_ALIGN_TOP_LEFT, 1, 0);


}

void gps_diagnostics() {
    Serial.println("=== GPS Diagnostic Check ===");

    // 1. Is serial open?
    Serial.println("- Sending dummy config...");
    GPSSerial.println(""); // Send dummy

    delay(500); // Give time for GPS to respond

    // 2. Check if anything is coming from GPS
    int availableBytes = GPSSerial.available();
    Serial.print("- Bytes available from GPS: ");
    Serial.println(availableBytes);

    if (availableBytes > 0) {
        Serial.println("- Reading incoming GPS data:");
        while (GPSSerial.available()) {
            char c = GPSSerial.read();
            Serial.write(c);
        }
    } else {
        Serial.println("- No data. Possible causes:");
        Serial.println("  * GPS module not powered?");
        Serial.println("  * Wrong TX/RX wiring?");
        Serial.println("  * Wrong GPS baud rate? (Default usually 9600 or 38400)");
        Serial.println("  * GPS still cold-starting?");
    }

    // 3. Check voltage (optional if you have battery monitor)
    uint16_t voltage = _watch->getBattVoltage();
    Serial.print("- Battery voltage: ");
    Serial.print(voltage);
    Serial.println(" mV");

    Serial.println("=== End GPS Diagnostic ===\n");
}

void radar_setup(LilyGoLib* watch) {
    _watch = watch;
    _watch->enableBLDO1();
    GPSSerial.begin(38400, SERIAL_8N1, SHIELD_GPS_RX, SHIELD_GPS_TX);
    
    compass_setup();
}

void radar_loop(LilyGoLib* watch) {
    _watch = watch;

    
    if (GPSSerial.available() > 0) {
        gps.encode(GPSSerial.read());
    }
   
    if (last_scan_time == 0 || millis() - last_scan_time > 8000) {
        
        //signals.clear();
        
        //Serial.println("Updating heading.\n");
        update_current_heading();
        
        //Serial.println("Radar BLE.");
        detectBLE();

        if ( !screenOn && ( digitalRead(16) == LOW || digitalRead(0) == LOW)) {
            _watch->setBrightness(80);
            lastTouchTime = millis();
            screenOn = true;
        }
        //Serial.println("Detect wifi.");
        detectWiFi();
        
        if ( !screenOn && ( digitalRead(16) == LOW || digitalRead(0) == LOW)) {
            _watch->setBrightness(80);
            lastTouchTime = millis();
            screenOn = true;
        }

        draw_radar();
      
        //Serial.println("Completed.");
        last_scan_time = millis();
    }


    if(millis() - lastTouchTime % 1000) {
        //Serial.println("Draw radar.");            
        // Then inside radar_loop() after drawing radar:
        sweep_angle = (sweep_angle + 5) % 360; // Adjust speed here
        draw_sweep(); // Draw the radar sweep line
    }

    lv_timer_handler();

    if (millis() - lastTouchTime > SCREEN_IDLE_TIMEOUT && screenOn) {  
        //Serial.println("Turning off brightness\n");
        _watch->setBrightness(0);
        screenOn = false;
    }
    
    if ( digitalRead(16) == LOW || digitalRead(0) == LOW) {
        lastTouchTime = millis();
        /*if (!screenOn)*/ {
            _watch->setBrightness(80);
            lastTouchTime = millis();
            screenOn = true;
        }
    }

}
