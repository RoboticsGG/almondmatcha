/**
 * @file gnss_usb0.ino
 * @author Sony Semiconductor Solutions Corporation (Modified)
 * @brief GNSS Example - Send Data via USB0 in JSON format
 *
 * @details This sketch initializes the Spresense GNSS module and transmits GNSS
 *          data through USB0 as key-value pairs (JSON-like format). Data includes:
 *          - Timestamp
 *          - Latitude & Longitude
 *          - Satellite count
 *          - Fix status
 *          If GNSS data is unavailable, an error message is sent.
 *
 *          Simply upload the sketch and open a serial monitor on USB0.
 *          The output will be in JSON-like format.
 */

#include <GNSS.h>

#define STRING_BUFFER_SIZE 128  /**< Buffer size for formatted strings */
#define RESTART_CYCLE (60 * 5)  /**< Restart GNSS every 5 minutes */

static SpGnss Gnss;  /**< Spresense GNSS object */

/* Define satellite system (Modify based on region) */
enum ParamSat {
  eSatGps,            /**< GPS                     World wide coverage  */
  eSatGlonass,        /**< GLONASS                 World wide coverage  */
  eSatGpsSbas,        /**< GPS+SBAS                North America        */
  eSatGpsGlonass,     /**< GPS+Glonass             World wide coverage  */
  eSatGpsBeidou,      /**< GPS+BeiDou              World wide coverage  */
  eSatGpsGalileo,     /**< GPS+Galileo             World wide coverage  */
  eSatGpsQz1c,        /**< GPS+QZSS_L1CA           East Asia & Oceania  */
  eSatGpsGlonassQz1c, /**< GPS+Glonass+QZSS_L1CA   East Asia & Oceania  */
  eSatGpsBeidouQz1c,  /**< GPS+BeiDou+QZSS_L1CA    East Asia & Oceania  */
  eSatGpsGalileoQz1c, /**< GPS+Galileo+QZSS_L1CA   East Asia & Oceania  */
  eSatGpsQz1cQz1S,    /**< GPS+QZSS_L1CA+QZSS_L1S  Japan                */
};

static enum ParamSat satType = eSatGps;  /**< Selected satellite system */

/**
 * @brief Initialize GNSS and USB0 communication.
 */
void setup() {
  Serial.begin(115200);   // Standard Serial (Optional)

  Serial.println("{\"status\":\"GNSS USB0 Output Ready\"}");

  /* Activate GNSS */
  if (Gnss.begin() != 0) {
    Serial.println("{\"error\":\"GNSS initialization failed\"}");
    while (1); // Stop execution on error
  }

  /* Select GNSS system */
  switch (satType) {
    case eSatGps:
      Gnss.select(GPS);
      break;
    case eSatGpsSbas:
      Gnss.select(GPS);
      Gnss.select(SBAS);
      break;
    case eSatGlonass:
      Gnss.select(GLONASS);
      Gnss.deselect(GPS);
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
    case eSatGpsQz1cQz1S:
      Gnss.select(GPS);
      Gnss.select(QZ_L1CA);
      Gnss.select(QZ_L1S);
      break;
    case eSatGpsBeidouQz1c:
      Gnss.select(GPS);
      Gnss.select(BEIDOU);
      Gnss.select(QZ_L1CA);
      break;
    case eSatGpsGalileoQz1c:
      Gnss.select(GPS);
      Gnss.select(GALILEO);
      Gnss.select(QZ_L1CA);
      break;
    case eSatGpsGlonassQz1c:
    default:
      Gnss.select(GPS);
      Gnss.select(GLONASS);
      Gnss.select(QZ_L1CA);
      break;
  }

  /* Start GNSS positioning */
  if (Gnss.start(COLD_START) != 0) {
    Serial.println("{\"error\":\"GNSS start failed\"}");
    while (1); // Stop execution on error
  }

  Serial.println("{\"status\":\"GNSS setup OK\"}");
}

/**
 * @brief Send GNSS data via USB0 in JSON format.
 */
static void send_data_usb(SpNavData *pNavData) {
  Serial.print("{");

  /* Print timestamp */
  Serial.print("\"time\":\"");
  Serial.printf("%04d-%02d-%02d %02d:%02d:%02d\",",
                   pNavData->time.year, pNavData->time.month, pNavData->time.day,
                   pNavData->time.hour, pNavData->time.minute, pNavData->time.sec);

  /* Print satellite count */
  Serial.print("\"numSatellites\":");
  Serial.print(pNavData->numSatellites);
  Serial.print(",");

  /* Print position fix status */
  Serial.print("\"fix\":");
  Serial.print((pNavData->posFixMode != FixInvalid) ? "true" : "false");
  Serial.print(",");

  /* Print latitude & longitude */
  if (pNavData->posDataExist == 0) {
    Serial.print("\"latitude\":null,\"longitude\":null");
  } else {
    Serial.print("\"latitude\":");
    Serial.print(pNavData->latitude, 6);
    Serial.print(",");
    Serial.print("\"longitude\":");
    Serial.print(pNavData->longitude, 6);
  }

  Serial.println("}");
}

/**
 * @brief Main loop - Continuously send GNSS data.
 */
void loop() {
  if (Gnss.waitUpdate(-1)) {
    SpNavData NavData;
    Gnss.getNavData(&NavData);
    send_data_usb(&NavData);  // Send GNSS data via USB0
  } else {
    Serial.println("{\"error\":\"GNSS data not updated\"}");
  }

  delay(1000); // Send data every second
}
