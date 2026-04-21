/**
 * @file gnss_addon_arduinojson_legacy_compat.ino
 * @brief GNSS Telemetry Output (JSON) for Spresense
 * (100% Backwards compatible keys + Extended Telemetry & Dual Blinking LEDs)
 */

#include <GNSS.h>

// Disable PROGMEM to fix Sony Spresense compilation error
#define ARDUINOJSON_ENABLE_PROGMEM 0
#include <ArduinoJson.h> 

static SpGnssAddon Gnss;
static bool streamLedState = false; 
static bool fixLedState = false; 

void setup() {
  Serial.begin(115200);

  // Initialize LEDs
  pinMode(LED0, OUTPUT); // LED0: Serial Streaming Heartbeat
  pinMode(LED1, OUTPUT); // LED1: GNSS Fix Status
  digitalWrite(LED0, LOW);
  digitalWrite(LED1, LOW);

  // EXACT OLD CODE STARTUP MSG
  Serial.println("{\"status\":\"GNSS USB0 Output Ready\"}");

  if (Gnss.begin() != 0) {
    Serial.println("{\"error\":\"GNSS initialization failed\"}"); // EXACT OLD CODE ERROR MSG
    while (1);
  }

  // Enable all constellations for maximum accuracy
  Gnss.select(GPS);
  Gnss.select(GLONASS);
  Gnss.select(GALILEO);
  Gnss.select(BEIDOU);
  Gnss.select(QZ_L1CA);
  Gnss.select(QZ_L1S);

  if (Gnss.start(COLD_START) != 0) {
    Serial.println("{\"error\":\"GNSS start failed\"}"); // EXACT OLD CODE ERROR MSG
    while (1);
  }

  // EXACT OLD CODE SETUP OK MSG
  Serial.println("{\"status\":\"GNSS setup OK\"}");
}

void loop() {
  if (Gnss.waitUpdate(-1)) {
    SpNavData nav;
    Gnss.getNavData(&nav);

    JsonDocument doc;

    // Format time strictly as "YYYY-MM-DD HH:MM:SS" (Matches old code)
    char timeStr[32];
    snprintf(timeStr, sizeof(timeStr), "%04d-%02d-%02d %02d:%02d:%02d",
             nav.time.year, nav.time.month, nav.time.day,
             nav.time.hour, nav.time.minute, nav.time.sec);
    
    // --- DUAL BLINKING LED LOGIC ---
    // 1. Streaming Heartbeat (LED0): Toggles every time a valid loop completes
    streamLedState = !streamLedState;
    digitalWrite(LED0, streamLedState ? HIGH : LOW);

    // 2. Fix Status (LED1): Toggles only if a fix is present, otherwise forced OFF
    bool hasFix = (nav.posFixMode != FixInvalid);
    if (hasFix) {
      fixLedState = !fixLedState;
      digitalWrite(LED1, fixLedState ? HIGH : LOW);
    } else {
      fixLedState = false;
      digitalWrite(LED1, LOW);
    }
    // -------------------------------

    // ==========================================
    // 1. EXACT OLD CODE KEYS (Guaranteed compatible)
    // ==========================================
    doc["time"] = timeStr;          
    doc["numSatellites"] = nav.numSatellites; 
    doc["fix"] = hasFix; 

    if (nav.posDataExist) {
      doc["latitude"] = nav.latitude;  
      doc["longitude"] = nav.longitude; 
      doc["altitude"] = nav.altitude;  
    } else {
      doc["latitude"] = nullptr;
      doc["longitude"] = nullptr;
      doc["altitude"] = nullptr;
    }

    // ==========================================
    // 2. NEW EXTENDED KEYS (Safe additions)
    // ==========================================
    doc["usec"] = nav.time.usec;
    if (nav.posDataExist) {
      doc["speed_ms"] = nav.velocity;   
      doc["heading"] = nav.direction;   
      doc["pdop"] = nav.pdop;           
      doc["hdop"] = nav.hdop;           
      doc["vdop"] = nav.vdop;           
    } else {
      doc["speed_ms"] = nullptr;
      doc["heading"] = nullptr;
      doc["pdop"] = nullptr;
      doc["hdop"] = nullptr;
      doc["vdop"] = nullptr;
    }

    // Serialize and send the complete JSON string
    serializeJson(doc, Serial);
    Serial.println();
    
  } else {
    // EXACT OLD CODE FALLBACK MSG
    Serial.println("{\"error\":\"GNSS data not updated\"}");
  }

  // EXACT OLD CODE DELAY (1Hz loop cycle)
  delay(1000); 
}
