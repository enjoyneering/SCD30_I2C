[![license-badge][]][license] ![version] [![stars][]][stargazers] ![hit-count] [![github-issues][]][issues]

# SCD30_I2C
This is Arduino library for Sensirion SCD30 sensor controlled via I²C bus

- NDIR CO2 optical sensor with dual-channel detection technology
- power supply voltage +3.3v..+5.5v
- maximum average current 19mA (idle state & processing), peak current 75mA (processing) **(1)**
- energy consumption 120mJ per measurement
- integrated SHT31 temperature & humidity sensor to compensate sensor self heating
- external FRC (forced re-calibration) procedure or continuous ASC (automatic self-calibration) algorithm **(NOTE)**
- CO2 working temperature range 0C..+50°C at 5%..95%, if out wait for 2 hours at 25°C **(2)**
- CO2 measurement range 0ppm..40000ppm (I²C, UART/Modbus), 0..5000ppm (PWM)
- CO2 accuracy ±(30ppm + 3% of measured value) in range 400ppm..10000ppm **(2)(3)**
- CO2 repeatability ±10ppm in range 400ppm..10000ppm
- CO2 temperature stability ±2.5ppm/°C in range 0°C..+50°C **(2)**
- CO2 measurement interval 2sec..1800sec, recomended 5sec..60sec **(1)**
- humidity measurement range 0%..100%
- humidity accuracy ±2%
- temperature measurement range -40C..+70C
- temperature accuracy ±0.3°C
- interfaces:
  - UART/Modbus **(4)**
  - I²C **(5)**
  - PWM
 - lifetime 15years **(6)**

Supports all sensors features:
 - start/stop continuous measurement with with optional ambient pressure compensation
 - set/get measurement interval
 - get data ready status
 - read measurement (CO2, temperature, humidity) **(7)**
 - set/get ASC (automatic self-calibration) algorithm
 - set/get FRC (forced re-calibration) value
 - set/get temperature offset
 - set/get altitude compensation
 - read FW version
 - soft reset
 - bonus, get error code of the last I²C command

**NOTE:**
- Major difference between ASC & FRC is the source of the reference value. FRC requires reference gas & ASC generates a reference value autonomously from historical sensor data. Sensor accuracy is the same after FRC or ASC as long as conditions & assumptions for ASC are fulfilled.
- The theory behind ASC calibration is that if a building's ventilation system is designed correctly, at some point each day the building is empty and CO2 levels should return to 400ppm for at least 1 hour. ASC works correctly if the sensor is in continuous measurement mode and regularly sees fresh air for 1 hour every day, so that ASC can constantly recalibrate. Successfully calculated parameters are stored in non-volatile memory along with previously found ASC parameters to create a calibration curve. Powering down the sensor will delay new reference value. The sensor is smart enough to discount periodic elevated readings that occur if a room is occupied for 24 hours a day over a few days. However, CO2 concentrations greater than 400ppm during the week will eventually cause the sensor to drift when the ASC is activated. In this case apply FRC from time to time to reduce sensor deviations & drift. The Sensirion does not specify when FRC must be done, but other sources mention an interval of 6 months to 1 year.
- The FRC value is applied immediately and can be executed multiple times. Start continuous measurement mode and set 2sec measurement interval. Put sensor in environment where CO2 concentration level is equal to target concentration, after 2 minutes set FRC with target value. The most accurate method is to expose sensor to a known gas, typically 100% nitrogen (0ppm CO2), but it is expense. If maximum accuracy is less important than cost, sensor can be calibrated in fresh air at 400ppm CO2 (outdoor air is usually between 350ppm..480ppm in normal geographic locations).
- The temperature offset is designed to compensate for thermal heating from other electrical components. Run sensor in the desired enclosure at wanted sampling interval for 10 minutes for thermal stabilization. Measure ambient temperature outside of the enclosure with a separate calibrated temperature sensor. Calculate and apply offset.

**(1)** Longer measurement interval reduces power consumption but makes the sensor slower and unable to track rapidly changing CO2 concentrations<br>
**(2)** CO2 levels and accuracy are not guaranteed at negative temperatures<br>
**(3)** Any air flow around sensor generate pressure drops, back pressure and dynamic fluctuations leading to increased sensor noise and reduced accuracy<br>
**(3)** Optical cell should never be in contact with any objects. SCD30 are delicate optical systems and mechanical stresses on the optical cavity could change physical properties & measurement accuracy, in cases of mechanical stress run calibration to restore accuracyy<br>
**(4)** UART/Modbus require pullup between "SEL" pin and VDD during power-up, do not exceed +4.0v<br>
**(5)** I²C requires "SEL" pin floating or connected to GND, speed 50KHz..100KHz, clock stretching 30ms..150ms and internal pull-up 45kOhm to +3.0v **(not 5v tolerant, level converter required)**<br>
**(6)** Exposing sensor to direct sunlight can cause temperature fluctuations and rapid aging of SCD30<br>
**(7)** Library returns **55555** for CO2, T & RH if a communication error occurs or CRC doesn't match<br>

[license-badge]: https://img.shields.io/badge/License-GPLv3-blue.svg
[license]:       https://choosealicense.com/licenses/gpl-3.0/
[version]:       https://img.shields.io/badge/Version-1.0.4-green.svg
[stars]:         https://img.shields.io/github/stars/enjoyneering/SCD30_I2C.svg
[stargazers]:    https://github.com/enjoyneering/SCD30_I2C/stargazers
[hit-count]:     https://hits.seeyoufarm.com/api/count/incr/badge.svg?url=https%3A%2F%2Fgithub.com%2Fenjoyneering%2FSCD30_I2C&count_bg=%2379C83D&title_bg=%23555555&icon=&icon_color=%23E7E7E7&title=hits&edge_flat=false
[github-issues]: https://img.shields.io/github/issues/enjoyneering/SCD30_I2C.svg
[issues]:        https://github.com/enjoyneering/SCD30_I2C/issues/
