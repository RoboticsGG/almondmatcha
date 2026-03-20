/**
 * @file gnss_addon_arduinojson.ino
 * @brief Professional telemetry output using ArduinoJson for Spresense Add-on
 */

/** 
    here is an example of json string output */
    /** {"timestamp":"2023-04-01T12:34:56",
"usec":123456,"fix_mode":3,"num_sat":8,"lat":37.7749,
"lon":-122.4194,"alt":10.5,"speed_ms":5.2,"heading":45.0,
"pdop":1.2,"hdop":1.0,"vdop":1.5} */

#include <GNSS.h>
#include <ArduinoJson.h> // Install via Arduino Library Manager

static SpGnssAddon Gnss;

void setup() {
  Serial.begin(115200);

  // Initialize Add-on hardware
  if (Gnss.begin() != 0) {
    Serial.println("{\"error\":\"GNSS Add-on initialization failed\"}");
    while (1);
  }

  // Enable all constellations to maximize Dual-Band (L1/L5) performance
  Gnss.select(GPS);
  Gnss.select(GLONASS);
  Gnss.select(GALILEO);
  Gnss.select(BEIDOU);
  Gnss.select(QZ_L1CA);
  Gnss.select(QZ_L1S);

  if (Gnss.start(COLD_START) != 0) {
    Serial.println("{\"error\":\"GNSS start failed\"}");
    while (1);
  }
}

void loop() {
  if (Gnss.waitUpdate(-1)) {
    SpNavData nav;
    Gnss.getNavData(&nav);

    // Create a JSON document (ArduinoJson 7 syntax)
    JsonDocument doc;

    // Time & Status
    char timeStr[32];
    snprintf(timeStr, sizeof(timeStr), "%04d-%02d-%02dT%02d:%02d:%02d",
             nav.time.year, nav.time.month, nav.time.day,
             nav.time.hour, nav.time.minute, nav.time.sec);
    
    doc["timestamp"] = timeStr;
    doc["usec"] = nav.time.usec;
    doc["fix_mode"] = nav.posFixMode; // 1:No Fix, 2:2D, 3:3D
    doc["num_sat"] = nav.numSatellites;

    // Position, Motion, and Precision
    if (nav.posDataExist) {
      doc["lat"] = nav.latitude;
      doc["lon"] = nav.longitude;
      doc["alt"] = nav.altitude;
      doc["speed_ms"] = nav.velocity;   // Speed over ground
      doc["heading"] = nav.direction;   // Course over ground in degrees
      doc["pdop"] = nav.pdop;           // Position Dilution of Precision
      doc["hdop"] = nav.hdop;           // Horizontal Dilution of Precision
      doc["vdop"] = nav.vdop;           // Vertical Dilution of Precision
    } else {
      // Explicitly set nulls for a consistent schema
      doc["lat"] = JsonVariant();
      doc["lon"] = JsonVariant();
      doc["alt"] = JsonVariant();
      doc["speed_ms"] = JsonVariant();
      doc["heading"] = JsonVariant();
      doc["pdop"] = JsonVariant();
      doc["hdop"] = JsonVariant();
      doc["vdop"] = JsonVariant();
    }

    // Serialize and send
    serializeJson(doc, Serial);
    Serial.println();
  }
}