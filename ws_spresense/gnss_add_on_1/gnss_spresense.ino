/**
 * This produces same string output format as gnss_v2.ino, but using the Spresense GNSS Add-on Board (CXD5602PWBGNSS1W).
 * The add-on board supports more simultaneous satellite systems (GPS, GLONASS, Galileo, BeiDou, QZSS) and can provide better accuracy in challenging environments.
 * @file gnss_addon_usb0.ino
 * @brief GNSS Example - Adjusted for Sony Spresense GNSS Add-on Board
 * * Target Hardware: Sony Spresense + CXD5602PWBGNSS1W Add-on
 */

#include <GNSS.h>

#define STRING_BUFFER_SIZE 128
#define RESTART_CYCLE (60 * 5)

// CHANGE: Use SpGnssAddon instead of SpGnss
static SpGnssAddon Gnss; 

/* Satellite system selection for the Add-on Board (Supports L1/L5) */
enum ParamSat {
  eSatGps,
  eSatGpsGlonass,
  eSatGpsBeidou,      // Recommended for SE Asia
  eSatGpsGalileo,
  eSatGpsQz1c,        // QZSS (Japan/Asia coverage)
  eSatFullCombo       // GPS + GLONASS + Galileo + BeiDou + QZSS
};

// Set this to eSatFullCombo for maximum accuracy in Thailand
static enum ParamSat satType = eSatFullCombo; 

void setup() {
  Serial.begin(115200);
  Serial.println("{\"status\":\"GNSS Add-on USB0 Output Ready\"}");

  /* Initialize GNSS Add-on Board */
  if (Gnss.begin() != 0) {
    Serial.println("{\"error\":\"GNSS Add-on initialization failed. Check mounting.\"}");
    while (1); 
  }

  /* Select GNSS systems - The Add-on board (CXD5610) handles more simultaneous systems */
  switch (satType) {
    case eSatGps:
      Gnss.select(GPS);
      break;
    case eSatGpsGlonass:
      Gnss.select(GPS);
      Gnss.select(GLONASS);
      break;
    case eSatGpsBeidou:
      Gnss.select(GPS);
      Gnss.select(BEIDOU);
      break;
    case eSatGpsGalileo:
      Gnss.select(GPS);
      Gnss.select(GALILEO);
      break;
    case eSatGpsQz1c:
      Gnss.select(GPS);
      Gnss.select(QZ_L1CA);
      break;
    case eSatFullCombo:
    default:
      Gnss.select(GPS);
      Gnss.select(GLONASS);
      Gnss.select(GALILEO);
      Gnss.select(BEIDOU);
      Gnss.select(QZ_L1CA);
      break;
  }

  /* Start positioning */
  if (Gnss.start(COLD_START) != 0) {
    Serial.println("{\"error\":\"GNSS start failed\"}");
    while (1);
  }

  Serial.println("{\"status\":\"GNSS Add-on setup OK\"}");
}

static void send_data_usb(SpNavData *pNavData) {
  Serial.print("{");

  /* Timestamp */
  Serial.print("\"time\":\"");
  Serial.printf("%04d-%02d-%02d %02d:%02d:%02d\",",
                   pNavData->time.year, pNavData->time.month, pNavData->time.day,
                   pNavData->time.hour, pNavData->time.minute, pNavData->time.sec);

  /* Satellite count */
  Serial.print("\"numSatellites\":");
  Serial.print(pNavData->numSatellites);
  Serial.print(",");

  /* Fix status */
  Serial.print("\"fix\":");
  Serial.print((pNavData->posFixMode != FixInvalid) ? "true" : "false");
  Serial.print(",");

  /* Position data */
  if (pNavData->posDataExist == 0) {
    Serial.print("\"latitude\":null,\"longitude\":null,\"altitude\":null");
  } else {
    Serial.print("\"latitude\":");
    Serial.print(pNavData->latitude, 6);
    Serial.print(",");
    Serial.print("\"longitude\":");
    Serial.print(pNavData->longitude, 6);
    Serial.print(",");
    Serial.print("\"altitude\":");
    Serial.print(pNavData->altitude, 2);
  }

  Serial.println("}");
}

void loop() {
  // Add-on board still uses waitUpdate and getNavData the same way
  if (Gnss.waitUpdate(-1)) {
    SpNavData NavData;
    Gnss.getNavData(&NavData);
    send_data_usb(&NavData);
  } else {
    Serial.println("{\"error\":\"GNSS data not updated\"}");
  }

  // Removed extra delay to allow waitUpdate to control the timing
}