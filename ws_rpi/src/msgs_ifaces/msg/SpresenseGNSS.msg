# Spresense GNSS (Global Navigation Satellite System) Data
# Contains GPS/GNSS positioning and timing information from Sony Spresense board

string date              # Date string (YYYY-MM-DD)
string time              # Time string (HH:MM:SS)
int32 num_satellites     # Number of satellites in view
bool fix                 # GNSS fix status (true=fix acquired, false=no fix)
float64 latitude         # Latitude in decimal degrees (-90 to 90)
float64 longitude        # Longitude in decimal degrees (-180 to 180)
float64 altitude         # Altitude above mean sea level (meters)