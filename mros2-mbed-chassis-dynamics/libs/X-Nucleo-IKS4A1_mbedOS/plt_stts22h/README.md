# PLT_STTS22H
This is an mbedOS library for driving the STTS22H temperature sensor
## Description

The STTS22H is an ultralow-power, high-accuracy digital temperature sensor renowned for its performance across the entire operating temperature range.

This sensor integrates a bandgap temperature sensor, an A/D converter, signal processing logic, and an I²C/SMBus 3.0 interface into a single ASIC.

Housed in a small 2 x 2 x 0.50 mm 6-lead UDFN package with an exposed pad down configuration, the STTS22H ensures better temperature matching with its surrounding environment.

Factory calibrated, the STTS22H requires no additional calibration efforts on the customer's end. Furthermore, all units undergo 100% testing on a production setup that is NIST traceable and verified with equipment calibrated in accordance with the IATF 16949:2016 standard.

### Key Features:
- Integrated high-accuracy temperature sensor
- Factory calibrated with NIST traceability
- One-shot mode for power saving
- Electrical Specifications:
  - Supply voltage: 1.5 to 3.6 V
  - I²C, SMBus 3.0 with ALERT (ARA) support
  - Programmable thresholds with interrupt pin
  - Supports up to 1 MHz serial clock
  - Up to 4 I²C/SMBus slave addresses
  - Ultralow current: 1.75 µA in one-shot mode
- Sensing Specifications:
  - Operating temperature: -40 °C to +125 °C
  - Temperature accuracy (max.): ± 0.5 °C (-10 °C to +60 °C)
  - 16-bit temperature data output
- Package Specifications:
  - UDFN 2.0 x 2.0 x 0.50 mm, 6 leads with exposed pad down
  - ECOPACK and RoHS compliant
---
[Official documentation](https://www.st.com/en/mems-and-sensors/stts22h.html)

---
## Examples

[xNucleo-IKS4A1_LPS22DF_mbedOS_000-GetTemperature_GetPressure](https://github.com/Perlatecnica/xNucleo-IKS4A1_LPS22DF_mbedOS_000-GetTemperature_GetPressure)
