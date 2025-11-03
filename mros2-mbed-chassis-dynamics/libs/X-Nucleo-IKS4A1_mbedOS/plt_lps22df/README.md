# PLT_LPS22DF

This is an mbedOS library for driving the LPS22DF pressure and temperature sensor

## Description

The LPS22DF is an ultracompact, piezoresistive, absolute pressure sensor designed to function as a digital output barometer. Compared to its predecessor, the LPS22DF boasts lower power consumption and achieves lower pressure noise.

This device consists of a sensing element and an IC interface, facilitating communication over I²C, MIPI I3CSM, or SPI interfaces between the sensing element and the application. Additionally, it supports a wide Vdd IO range for the digital interfaces. The sensing element, responsible for detecting absolute pressure, features a suspended membrane manufactured using a specialized process developed by ST.

The LPS22DF is housed in a full-mold, holed LGA package (HLGA), which ensures operation over a temperature range spanning from -40 °C to +85 °C. The holed package design enables external pressure to reach the sensing element.

### Key Features:
- Absolute pressure range: 260 to 1260 hPa
- Low current consumption: down to 1.7 μA
- Absolute pressure accuracy: 0.2 hPa
- Robustness to soldering stress: 0.15 hPa
- Low pressure sensor noise: 0.34 Pa
- High-performance TCO (Temperature Coefficient Offset): 0.45 Pa/°C
- Embedded temperature compensation
- 24-bit pressure data output
- Output Data Rate (ODR) from 1 Hz to 200 Hz
- SPI, I²C, or MIPI I3CSM interface
- Supports 1.08 V digital interface
- Embedded FIFO (First In, First Out)
- Interrupt functions: data-ready, FIFO flags, pressure thresholds
- Supply voltage: 1.7 to 3.6 V
- High shock survivability: 22,000 g
- Small and thin package
- ECOPACK lead-free compliant

##Official Documentation
---
[Official documentation](https://www.st.com/en/mems-and-sensors/lps22df.html)
