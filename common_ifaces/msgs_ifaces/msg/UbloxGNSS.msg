# Ublox GNSS Message (SimpleRTK2b)
# Parsed from NMEA strings

string date                  # Date in YYYYMMDD format
string time                  # UTC time in HHMMSS format
float64 latitude             # Latitude in decimal degrees
float64 longitude            # Longitude in decimal degrees
float64 altitude             # Altitude in meters above sea level
int32 satellites_tracked     # Number of satellites being tracked
string fix_quality           # Fix quality: Auto, DGPS, Float, RTK Fixed
float32 snr                  # Signal-to-Noise Ratio (dB)
float64 speed                # Speed in m/s
float32 centimeter_error     # Position error in centimeters
