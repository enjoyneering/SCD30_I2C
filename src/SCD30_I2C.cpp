/***************************************************************************************************/
/*
   This is an Arduino library for a NDIR CO2 optical Sensirion SCD30 sensor controlled via I2C bus

   written by : enjoyneering
   sourse code: https://github.com/enjoyneering/

   Sensirion SCD30 spec:
   - NDIR CO2 optical sensor with dual-channel detection technology
   - power supply voltage +3.3v..+5.5v
   - maximum average current 19mA (idle state & processing), peak current 75mA (processing) (1)
   - energy consumption 120mJ per measurement
   - integrated SHT31 temperature & humidity sensor to compensate sensor self heating
   - external FRC (forced re-calibration) procedure or continuous ASC (automatic self-calibration) algorithm
   - CO2 working temperature range 0C..+50C at 5%..95%, if out wait for 2 hours at 25C (2)
   - CO2 measurement range 0ppm..40000ppm (I2C, UART/Modbus), 0..5000ppm (PWM)
   - CO2 accuracy +-(30ppm + 3% of measured value) in range 400ppm..10000ppm (2)(3)
   - CO2 repeatability +-10ppm in range 400ppm..10000ppm
   - CO2 temperature stability +-2.5ppm/C in range 0C..+50C (2)
   - CO2 measurement interval 2sec..1800sec, recomended 5sec..60sec (1)
   - humidity measurement range 0%..100%
   - humidity accuracy +-2%
   - temperature measurement range -40C..+70C
   - temperature accuracy +-0.3C
   - interfaces:
     - UART-Modbus (4)
     - I2C (5)
     - PWM
   - lifetime 15years (6)

   (1) longer measurement interval reduces power consumption but makes the sensor slower and unable
       to track rapidly changing CO2 concentrations
   (2) CO2 levels and accuracy are not guaranteed at negative temperatures
   (3) any air flow around sensor generate pressure drops, back pressure and dynamic fluctuations
       leading to increased sensor noise and reduced accuracy
   (3) optical cell should never be in contact with any objects, SCD30 are delicate optical systems
       and mechanical stresses on the optical cavity could change physical properties & measurement
       accuracy, in cases of mechanical stress run calibration to restore accuracy
   (4) UART-Modbus require pullup between SEL pin and VDD during power-up, do not exceed +4.0v
   (5) I2C requires SEL pin floating or connected to GND, address 0x61, speed 50KHz..100KHz,
       clock stretching 30ms..150ms, internal pull-up 45kOhm to +3.0v (not 5v tolerant, level converter required)
   (6) exposing sensor to direct sunlight can cause temperature fluctuations and rapid aging of SCD30


   This device uses I2C bus to communicate, specials pins are required to interface
   Board                                     SDA              SCL              Level
   Uno, Mini, Pro, ATmega168, ATmega328..... A4               A5               5v
   Mega2560................................. 20               21               5v
   Due, SAM3X8E............................. 20               21               3.3v
   Leonardo, Micro, ATmega32U4.............. 2                3                5v
   Digistump, Trinket, Gemma, ATtiny85...... PB0/D0           PB2/D2           3.3v/5v
   Blue Pill*, STM32F103xxxx boards*........ PB7/PB9          PB6/PB8          3.3v/5v
   ESP8266 ESP-01**......................... GPIO0            GPIO2            3.3v/5v
   NodeMCU 1.0**, WeMos D1 Mini**........... GPIO4/D2         GPIO5/D1         3.3v/5v
   ESP32***................................. GPIO21/D21       GPIO22/D22       3.3v
                                             GPIO16/D16       GPIO17/D17       3.3v
                                            *hardware I2C Wire mapped to Wire1 in stm32duino
                                             see https://github.com/stm32duino/wiki/wiki/API#I2C
                                           **most boards has 10K..12K pullup-up resistor
                                             on GPIO0/D3, GPIO2/D4/LED & pullup-down on
                                             GPIO15/D8 for flash & boot
                                          ***hardware I2C Wire mapped to TwoWire(0) aka GPIO21/GPIO22 in Arduino ESP32

   Supported frameworks:
   Arduino Core - https://github.com/arduino/Arduino/tree/master/hardware
   ATtiny  Core - https://github.com/SpenceKonde/ATTinyCore
   ESP8266 Core - https://github.com/esp8266/Arduino
   ESP32   Core - https://github.com/espressif/arduino-esp32
   STM32   Core - https://github.com/stm32duino/Arduino_Core_STM32

   GNU GPL license, all text above must be included in any redistribution,
   see link for details - https://www.gnu.org/licenses/licenses.html
*/
/***************************************************************************************************/

#include "SCD30_I2C.h"


/**************************************************************************/
/*
    Constructor
*/
/**************************************************************************/
SCD30_I2C::SCD30_I2C()
{
  _cmdStatus = SCD30_I2C_NO_ERROR;
}


/**************************************************************************/
/*
    begin()

    Initialize I2C bus & look for sensor

    NOTE:
    - call this function before doing anything else!!!
    - speed in Hz, stretch in usec

    - during start-up sensor automatically loads all calibrations & saved
      settings (ASC or FRC, temperature offset, etc)
    - sensor I2C speed 50KHz..100KHz, recomended 50KHz or smaller, clock
      stretching period 30msec, however due to internal ASC calibration
      processes a maximal clock stretching of 150msec may occur once per day

    - SCD30 proper command sequence:
      - set sempling interval if needed
      - set FRC procedure or continuous ASC algorithm if needed
      - set temperature offset if needed
      - start measurement

    - returned value by "Wire.endTransmission()":
      - 0, success
      - 1, data too long to fit in transmit data buffer
      - 2, received NACK on transmit of address
      - 3, received NACK on transmit of data
      - 4, other error
*/
/**************************************************************************/
#if defined (__AVR__)
bool SCD30_I2C::begin(uint32_t speed, uint32_t stretch)
{
  Wire.begin();

  Wire.setClock(speed);                                    //experimental! AVR I2C bus speed 31kHz..400kHz, default 100000Hz

  #if !defined (__AVR_ATtiny85__)                          //for backwards compatibility with ATtiny Core
  Wire.setWireTimeout(stretch, false);                     //experimental! default 25000usec, true=Wire hardware will be automatically reset to default on timeout
  #endif

#elif defined (ESP8266)
bool SCD30_I2C::begin(uint8_t sda, uint8_t scl, uint32_t speed, uint32_t stretch)
{
  Wire.begin(sda, scl);

  Wire.setClock(speed);                                    //experimental! ESP8266 I2C bus speed 1kHz..400kHz, default 100000Hz

  Wire.setClockStretchLimit(stretch);                      //experimental! default 150000usec

#elif defined (ESP32)
bool SCD30_I2C::begin(int32_t sda, int32_t scl, uint32_t speed, uint32_t stretch) //"int32_t" for Master SDA & SCL, "uint8_t" for Slave SDA & SCL
{
  if (Wire.begin(sda, scl, speed) != true) {return false;} //experimental! ESP32 I2C bus speed ???kHz..400kHz, default 100000Hz

  Wire.setTimeout(stretch / 1000);                         //experimental! default 50msec

#elif defined (ARDUINO_ARCH_STM32)
bool SCD30_I2C::begin(uint32_t sda, uint32_t scl, uint32_t speed) //"uint32_t" for pins only, "uint8_t" calls wrong "setSCL(PinName scl)"
{
  Wire.begin(sda, scl);

  Wire.setClock(speed);                                    //experimental! STM32 I2C bus speed ???kHz..400kHz, default 100000Hz

#else
bool SCD30_I2C::begin()
{
  Wire.begin();
#endif

  delay(SCD30_I2C_POWER_ON_DELAY);           //wait for sensor to boot

  Wire.beginTransmission(SCD30_I2C_ADDRESS);

  _cmdStatus = Wire.endTransmission(true);   //true=send stop after transmission

  return (_cmdStatus == 0);                  //true=success, false=I2C error
}


/**************************************************************************/
/*
    startContinuousMeasurement()

    Start continuous measurement with optional ambient pressure compensation

    NOTE:
    - continuous measurement mode is stored in non-volatile memory &
      activated automatically after power on
    - measurement data which is not read from sensor will be overwritten
    - for setting a new ambient pressure when continuous measurement is
      running the whole command has to be written to sensor

    - CO2 measurement based on the NDIR principle depends on altitude
      & can be compensated by altitude or ambient pressure
    - ambient pressure compensation range 700mBar...1400mBar or set
      0mBar to disable ambient pressure compensation
    - setting ambient pressure will overwrite previous settings of
      altitude compensation, see "setAltitude()" for details
    - standard/normal atmosphere pressure 1013.25mBar
*/
/**************************************************************************/
bool SCD30_I2C::startContinuousMeasurement(uint16_t ambientPressure)
{
  if (ambientPressure != 0) {ambientPressure = constrain(ambientPressure, 700, 1400);} //ambient pressure range 700mBar...1400mBar, 0=ambient pressure compensation off

  return _writeRegister(SCD30_I2C_START_CONT_MSRMNT_REG, ambientPressure);
}


/**************************************************************************/
/*
    stopContinuousMeasurement()

    Stop continuous measurement

    NOTE:
    - see "startContinuousMeasurement()" for details
*/
/**************************************************************************/
bool SCD30_I2C::stopContinuousMeasurement()
{
  return _writeRegister(SCD30_I2C_STOP_CONT_MSRMNT_REG);
}


/**************************************************************************/
/*
    setMeasurementInterval()

    Set measurement interval for continuous measurement mode

    NOTE:
    - measurement interval range 2sec..1800sec (by default 2sec), saved in
      non-volatile memory & activated automatically after power on
    - sampling interval do NOT cycle sensor power on/off
    - modifying sampling interval reduces CO2 signal accuracy, causes
      out-of-spec measurements & CO2 signal recalibration is recomended
    - Sensirion recommends sampling interval 5sec..60sec to reduce power
      consumption while keeping response time low 
    - there is an inherent trade-off between low power consumption & fast
      response time, e.g. when operated at a sampling interval of 15sec
      SCD30 average current consumption is 6.5mA at a response time of 72sec,
      for small sampling intervals 2sec..5sec the response time is < 40sec &
      for high sampling intervals > 60sec the response time is several minutes
    - rule of thumb, response time is 4..5 times bigger than the sampling
      interval, see "Nyquist rate" for details
    - sensor reloads all calibration data before each measurement (ASC or 
      FRC, temperature offset etc)
*/
/**************************************************************************/
bool SCD30_I2C::setMeasurementInterval(uint16_t interval)
{
  interval = constrain(interval, 2, 1800);                             //measurement interval range 2sec..1800sec, see NOTE

  return _writeRegister(SCD30_I2C_CONT_MSRMNT_INTERVAL_REG, interval);
}


/**************************************************************************/
/*
    getMeasurementInterval()

    Get measurement interval for continuous measurement mode

    NOTE:
    - see "setMeasurementInterval()" for details
*/
/**************************************************************************/
uint16_t SCD30_I2C::getMeasurementInterval()
{
  return _readRegister(SCD30_I2C_CONT_MSRMNT_INTERVAL_REG); //in sec
}


/**************************************************************************/
/*
    getCommandStatus()

    Get error code of the last I2C command

    NOTE:
    - returned value by "Wire.endTransmission()":
      - 0x00, success
      - 0x01, data too long to fit in transmit data buffer
      - 0x02, received NACK on transmit of address
      - 0x03, received NACK on transmit of data
      - 0x04, other error

    - extra values:
      - 0x05, measurement not ready (measurement interval too long)
      - 0x06, received CRC8 not match computed CRC8
*/
/**************************************************************************/
uint8_t SCD30_I2C::getCommandStatus()
{
  return _cmdStatus;
}


/**************************************************************************/
/*
    getMeasurementStatus()

    Get data ready status to check if the measurement can be read from
    sensor buffer

    NOTE:
    - 0x0001, measurement is available for reading from the sensor
    - 0x0000, as soon as measurement has been read

    - measurement data which is not read from sensor will be overwritten
*/
/**************************************************************************/
bool SCD30_I2C::getMeasurementStatus()
{
  return (_readRegister(SCD30_I2C_GET_MSRMNT_READY_REG) == 0x0001); //0x0001=measurement is available for reading from the sensor
}


/**************************************************************************/
/*
    getMeasurement()

    Read measurement data from sensor buffer

    NOTE:
    - sensor buffer structure:
      0    1    2    3    4    5    6  7  8    9  10 11   12 13 14   15 16 17
      CO2, CO2, CRC, CO2, CO2, CRC, T, T, CRC, T, T, CRC, H, H, CRC, H, H, CRC
      ------------co2-------------  --------temp--------  --------humd--------

    - measurement data can be aborted after any data byte by sending a NACK
      followed by a stop condition
    - measurement data which is not read from sensor will be overwritten
    - it is recommended to use data ready status byte before readout of the
      measurement values

    - CO2 measurement range 0ppm..40000ppm +-(30ppm + 3% of measured value)
      - response time (t63) 20sec at measurement interval range 2sec
    - SHT31 humidity measurement range 0%..100% +-2%
      - response time (t63) 8sec
    - SHT31 temperature measurement range -40C..+70C +-0.3C
      - response time (t63) 10sec

    - see "_writeRegister(uint16_t reg)" for details
    - see "time()" in .../cores/esp8266/time.cpp for example
      - how to use:
        float co2, temp, humd;
        getMeasurement(&co2, &temp, &humd);
*/
/**************************************************************************/
void SCD30_I2C::getMeasurement(float *co2, float *temp, float *humd)
{
  /* check if new measurement ready */
  if ((getMeasurementStatus() != true) && (_cmdStatus == SCD30_I2C_NO_ERROR)) //measurement not ready && ready status was read without errors
  {
    _cmdStatus = SCD30_I2C_READY_ERROR;                                       //measurement not ready (measurement interval too long)

    return;                                                                   //"break" just exits "loop"/"if-else" & "return" terminates entire function
  }

  /* request measurement data */
  if (_writeRegister(SCD30_I2C_READ_MSRMNT_REG) != true)                      //I2C error, no reason to continue
  {
    *co2  = SCD30_I2C_ERROR_VALUE;
    *temp = SCD30_I2C_ERROR_VALUE;
    *humd = SCD30_I2C_ERROR_VALUE;

    return;
  }

  delay(SCD30_I2C_RW_DELAY);                                                //delay between writing & reading part

  /* read measurement data */
  Wire.requestFrom((uint8_t)SCD30_I2C_ADDRESS, (uint8_t)18, (uint8_t)true); //read 18-bytes from slave to "wire.h" rxBuffer, true=send stop after transmission

  if (Wire.available() != 18)                                               //check for 18-bytes in "wire.h" rxBuffer, see NOTE
  {
    _cmdStatus = SCD30_I2C_DATA_SIZE_ERROR;                                 //received data smaller/bigger than expected

    *co2  = SCD30_I2C_ERROR_VALUE;
    *temp = SCD30_I2C_ERROR_VALUE;
    *humd = SCD30_I2C_ERROR_VALUE;

    return;
  }

  /* convert 18-bytes to co2/temp/humd */
  *co2  = _readMeasurement();                                               //read 1-st 6-bytes from "wire.h" rxBuffer
  *temp = _readMeasurement();                                               //read 2-nd 6-bytes from "wire.h" rxBuffer
  *humd = _readMeasurement();                                               //read 3-rd 6-bytes from "wire.h" rxBuffer
}


/**************************************************************************/
/*
    setAutoCalibration()

    Activate/deactivate ASC (automatic self-calibration) algorithm

    NOTE:
    - ASC (automatic self-calibration) is used in buildings to measure
      occupancy or indoor air quality (IAQ) levels, where removing units
      from the wall to bench calibrate is expensive, required trained
      staff etc. The theory behind ASC calibration for use in IAQ is that
      if a building's ventilation system is designed correctly, at some
      point each day the building is empty and CO2 levels should return
      to 400ppm for at least 1 hour

    - ASC values applied automatically after power on
    - new calculated ASC parameter always overrides the corrections
      from external FRC (forced recalibration) procedure & vice versa,
      see "setManualCalibration()" for details

    - sensor self-calibrates approximately every 1..3 weeks, if ASC is
      activated & the environment meets the requirements for successful
      operation of the ASC. Successfully calculated parameters are
      stored in non-volatile memory along with previously found ASC
      parameters to create a calibration curve. Powering down the sensor
      will delay new reference value

    - ASC assumes that the lowest CO2 concentration the sensor is exposed
      to corresponds to 400ppm. Generating a reference value that does not
      match 400ppm results in an erroneous calibration update & reduces
      sensor accuracy. To prevent erroneous self-calibration, ASC uses an
      internal self-consistency check to discount periodic elevated
      readings that occur if a room is occupied for 24 hours a day over a
      few days. Once the sensor has collected 14 days worth of low CO2
      concentration periods, it performs a statistical analysis to see if
      there has been any small changes in the background levels readings
      that could be attributable to sensor drift. If the analysis concludes
      there is drift, a small correction factor is made to the sensor
      calibration to adjust for this change. This automatic calibration
      requires that at least three of the last 14 days have space CO2
      levels that reach 400ppm for an hour or more. However, CO2
      concentrations greater than 400ppm during the 14 days will eventually
      cause the sensor to drift when the ASC is activated. In this case
      apply FRC (forced re-calibration) rom time to time to reduce sensor
      deviations & drift. The Sensirion does not specify when FRC must be
      done, but other sources mention an interval of 6 months to 1 year

    - when ASC activated for the first time a period of at least 7 days
      needed for the algorithm to find its initial set of ASC parameters.
      During this first period sensor must be exposed to fresh air for at 
      least 1 hour each day. Also, during this period, the sensor must not
      be disconnected from the power source, otherwise procedure for
      searching for calibration parameters will be aborted & restarted

    - major difference between ASC & FRC is the source of the reference
      value. FRC requires reference gas & ASC generates a reference value
      autonomously from historical sensor data. Sensor accuracy is the
      same after FRC or ASC as long as conditions & assumptions for
      ASC are fulfilled
*/
/**************************************************************************/
bool SCD30_I2C::setAutoCalibration(bool value)
{
 return _writeRegister(SCD30_I2C_ASC_REG, (uint16_t)value); //0x0001=enable continuous ASC, 0x0000=disable continuous ASC
}


/**************************************************************************/
/*
    getAutoCalibration()

    Return ASC(automatic self-calibration) algorithm status, on/off 

    NOTE:
    - see "setAutoCalibration()" for details
*/
/**************************************************************************/
bool SCD30_I2C::getAutoCalibration()
{
  return _readRegister(SCD30_I2C_ASC_REG); //0x0001=ASC on, 0x0000=ASC off       
}


/**************************************************************************/
/*
    setManualCalibration()

    Calibrate CO2 concentration manualy, in ppm

    NOTE:
    - FRC (forced re-calibration) is recommended for calibration during
      production after assembly. The FRC value is applied immediately &
      can be executed multiple times at arbitrary intervals. Before
      applying FRC, sensor needs to be operated for 2 minutes in
      continuous mode and recommended measurement interval 2sec
    - during FRC user has to feed a reference value to sensor, the most
      accurate method is to expose sensor to a known gas, typically
      100% nitrogen (0ppm CO2), but it is expense. If maximum accuracy
      is less important than cost, sensor can be calibrated in fresh air
      at 400ppm CO2 (outdoor air is usually between 350ppm..480ppm in
      normal geographic locations).

    - FRC reference value range 400ppm..2000ppm
    - FRC value is saved in a non-volatile memory & will persist until
      it is overwritten either by a new reference value via FRC or by
      a new reference value generated by ASC, see "setAutoCalibration()"
      for details

    Recomended manual calibration procedure:
    - set continuous mode, than put the sensor in environment where CO2
      concentration level is equal to target concentration, e.g. by
      placing it close to an open window or outside, avoiding direct sun
      light & strong air flow, fresh air temperature should be within
      0С..+50С at 5%..95%
    - confirm the stability of ambient conditions
    - after 2 minutes, apply FRC with reference value 400ppm (fresh air)
      via the digital interface
    - confirm to end user that calibration procedure is completed
*/
/**************************************************************************/
bool SCD30_I2C::setManualCalibration(uint16_t concentration)
{
  concentration = constrain(concentration, 400, 2000);     //FRC reference value range 400ppm..2000ppm, see NOTE

  return _writeRegister(SCD30_I2C_FRC_REG, concentration);
}


/**************************************************************************/
/*
    getManualCalibration()

    Return FRC (forced re-calibration) value, in ppm

    NOTE:
    - see "setManualCalibration()" for details
*/
/**************************************************************************/
uint16_t SCD30_I2C::getManualCalibration()
{
  return _readRegister(SCD30_I2C_FRC_REG); //value range 400ppm..2000ppm   
}


/**************************************************************************/
/*
    setTemperatureOffset()

    Set temperature offset to compensate for thermal heating from other
    electrical components

    NOTE:
    - offset 0C..+70C in step 0.01C e.g. 0.00C, 0.01C, 0.02C,.., 70.00C
    - Sensirion SCD30 compensate self heating from internal electrical
      components (e.g. IR light source) with SHT31 temperature & humidity
      sensor, but if the sensor is encapsulated with other components, it
      may not show the correct ambient temperature
    - actual ambient temperature should be measured with a separate
      calibrated temperature sensor that is not in contact with any heat
      source. The CO2 sensor must be operated in the desired enclosure at
      the wanted sampling interval. If new sampling interval is set, user
      must wait approximately 10 minutes for thermal stabilization
    - temperature offset value is stored in non-volatile memory & activated
      automatically after power on

    - CO2 sensor working temperature range 0C..+50C at 5%..95%
    - SHT31 temperature measurement range -40C..+70C +-0.3C
*/
/**************************************************************************/
bool SCD30_I2C::setTemperatureOffset(const float offsetTemperature)
{
  int16_t offset = offsetTemperature * 100;                          //float to int16=C x 100, see NOTE

  offset = offset + _readRegister(SCD30_I2C_TEMPERATURE_OFFSET_REG); //offset=newOffset + oldOffset, see "Low Power Mode for SCD30" p.2

  if (_cmdStatus != SCD30_I2C_NO_ERROR) {return false;}              //I2C error on last command, no reason to continue

  offset = constrain(offset, 0, 7000);                               //offset 0C..+70C, see NOTE

  return _writeRegister(SCD30_I2C_TEMPERATURE_OFFSET_REG, offset);
}


/**************************************************************************/
/*
    getTemperatureOffset()

    Get temperature offset

    NOTE:
    - see "setTemperatureOffset()" for details
*/
/**************************************************************************/
float SCD30_I2C::getTemperatureOffset()
{
  float offset = _readRegister(SCD30_I2C_TEMPERATURE_OFFSET_REG);

  if (_cmdStatus != SCD30_I2C_NO_ERROR) {return offset;}          //I2C error on last command, no reason to continue

  return (offset / 100);                                          //int16 to float, see NOTE
}


/**************************************************************************/
/*
    setAltitude()

    Set altitude compensation, in meters

    NOTE:
    - CO2 measurements based on the NDIR principle influenced by altitude
      & can be compensated by applying current altitude
    - altitude value is stored in non-volatile memory & activated
      automatically after power on (if ambient pressure set to zero)
    - altitude compensation range -3100m...+3300m, calculated based on
      ambient pressure compensation range 700mBar...1400mBar at 0C

    - to activate ambient pressure compensation call
      "startContinuousMeasurement()" with pressure equal to 0mBar
*/
/**************************************************************************/
bool SCD30_I2C::setAltitude(int16_t altitude)
{
  altitude = constrain(altitude, -3100, 3300);             //altitude compensation range -3100m...+3300m

  return _writeRegister(SCD30_I2C_ALTITUDE_REG, altitude); //in meters
}


/**************************************************************************/
/*
    getAltitude()

    Set altitude compensation, in meters

    NOTE:
    - see "setAltitude()" for details
*/
/**************************************************************************/
int16_t SCD30_I2C::getAltitude()
{
  return _readRegister(SCD30_I2C_ALTITUDE_REG); //in meters
}



/**************************************************************************/
/*
    readFirmwareVersion()

    Read FW version
*/
/**************************************************************************/
String SCD30_I2C::readFirmwareVersion()
{
  uint16_t fwValue = _readRegister(SCD30_I2C_GET_FW_REG);     //major+minor version

  if (fwValue == SCD30_I2C_ERROR_VALUE) {return F("55.555");} //I2C error on last command, no reason to continue

  String fwStr((char *)0);                                    //string for FW, prevent allocating 1-byte by passing null to String constructor
         fwStr.reserve(8);                                    //set string size to minimize heap memory fragmentation

  fwStr  = highByte(fwValue);                                 //major version
  fwStr += F(".");
  fwStr += lowByte(fwValue);                                  //minor version

  return fwStr;                                               //e.g. "3.66"
}


/**************************************************************************/
/*
    reset()

    Reset sensor

    NOTE:
    - sensor is able to receive reset command at any time, regardless
      of its internal state
    - soft reset puts the sensor in the same state as after power-up,
      automatically reloads all calibrations & saved settings (ASC or
      FRC, temperature offset, etc)
*/
/**************************************************************************/
bool SCD30_I2C::reset()
{
  if (_writeRegister(SCD30_I2C_SOFT_RESET_REG) != true) {return false;} //false=I2C error, no reason to continue

  delay(SCD30_I2C_POWER_ON_DELAY);                                      //wait for sensor to reinitialize

  return true;                                                          //true=success, no I2C error
}




/**********************************private*********************************/
/**************************************************************************/
/*
    _writeRegister()

    Write command to sensor register

    NOTE:
    - registers/commands are 16-bit with an optional 16-bit parameter/data
    - any 16-bit parameter/data sent from or received by sensor protected
      by a CRC
    - sensor does not support repeated start condition
    - clock stretching period in write-read frames 30msec, however due to
      internal calibration processes a maximal clock stretching of 150msec
      may occur once per day

    - returned value by "Wire.endTransmission()":
      - 0 success
      - 1 data too long to fit in transmit data buffer
      - 2 received NACK on transmit of address
      - 3 received NACK on transmit of data
      - 4 other error
*/
/**************************************************************************/
bool SCD30_I2C::_writeRegister(const uint16_t reg)
{
  Wire.beginTransmission(SCD30_I2C_ADDRESS);

  Wire.write(highByte(reg));                 //write register MSB-byte to "wire.h" txBuffer
  Wire.write(lowByte(reg));                  //write register LSB-byte to "wire.h" txBuffer

  _cmdStatus = Wire.endTransmission(true);   //write data from "wire.h" txBuffer to slave, true=send stop after transmission

  return (_cmdStatus == 0);                  //true=success, false=I2C error
}


/**************************************************************************/
/*
    _writeRegister()

    Write data to sensor register

    NOTE:
    - see "_writeRegister(uint16_t reg)" for details
*/
/**************************************************************************/
bool SCD30_I2C::_writeRegister(const uint16_t reg, const uint16_t value)
{
  Wire.beginTransmission(SCD30_I2C_ADDRESS);

  Wire.write(highByte(reg));               //write register MSB-byte to "wire.h" txBuffer
  Wire.write(lowByte(reg));                //write register LSB-byte to "wire.h" txBuffer
  Wire.write(highByte(value));             //write argument LSB-byte to "wire.h" txBuffer
  Wire.write(lowByte(value));              //write argument LSB-byte to "wire.h" txBuffer
  Wire.write(_calculateCRC8(value));       //write argument CRC-byte to "wire.h" txBuffer

  _cmdStatus = Wire.endTransmission(true); //write data from "wire.h" txBuffer to slave, true=send stop after transmission

  return (_cmdStatus == 0);                //true=success, false=I2C error
}


/**************************************************************************/
/*
    _readRegister()

    Read data from sensor register

    NOTE:
    - see "_writeRegister(uint16_t reg)" for details
    - see "getMeasurement()" for details
    - see "time()" in .../cores/esp8266/time.cpp
*/
/**************************************************************************/
uint16_t SCD30_I2C::_readRegister(const uint16_t reg)
{
  if (_writeRegister(reg) != true) {return SCD30_I2C_ERROR_VALUE;}         //I2C error, no reason to continue

  delay(SCD30_I2C_RW_DELAY);                                               //delay between writing & reading part

  Wire.requestFrom((uint8_t)SCD30_I2C_ADDRESS, (uint8_t)3, (uint8_t)true); //read 3-bytes from slave to "wire.h" rxBuffer, true=send stop after transmission

  if (Wire.available() != 3)                                               //check for 3-bytes in "wire.h" rxBuffer
  {
    _cmdStatus = SCD30_I2C_DATA_SIZE_ERROR;                                //received data smaller/bigger than expected

    return SCD30_I2C_ERROR_VALUE;
  }

  uint16_t regValue = Wire.read();                                         //read data MSB-byte from "wire.h" rxBuffer

  regValue = (regValue << 8) | Wire.read();                                //read data LSB-byte from "wire.h" rxBuffer

  if (Wire.read() != _calculateCRC8(regValue))                             //read data CRC-byte & check with computed CRC8
  {
    _cmdStatus = SCD30_I2C_CRC8_ERROR;                                     //CRC error, received CRC8 not match computed CRC8

    return SCD30_I2C_ERROR_VALUE;
  }

  return regValue;                                    
}


/**************************************************************************/
/*
    _readMeasurement()

    Auxiliary function for "getMeasurement()" to read 6-bytes of
    measurement data from "wire.h" rxBuffer

    NOTE:
    - no "Wire.available()" size check for "wire.h" rxBuffer, use with
      caution

    - "wire.h" rxBuffer structure:
      6    5    4    3    2    1     6  5  4    3  2  1     6  5  4    3  2  1
      CO2, CO2, CRC, CO2, CO2, CRC,  T, T, CRC, T, T, CRC,  H, H, CRC, H, H, CRC
      ------------co2-------------   --------temp--------   --------humd--------

    - see "_writeRegister(uint16_t reg)" for details
*/
/**************************************************************************/
float SCD30_I2C::_readMeasurement()
{
  uint32_t regBigEnd = Wire.read();                   //read measurement data 6-th byte from "wire.h" rxBuffer

  regBigEnd = (regBigEnd << 8) | Wire.read();         //read measurement data 5-th byte from "wire.h" rxBuffer

  if (Wire.read() == _calculateCRC8(regBigEnd))       //read measurement CRC  4-th byte from "wire.h" rxBuffer
  {
    uint16_t regLittleEnd = Wire.read();              //read measurement data 3-rd byte from "wire.h" rxBuffer

    regLittleEnd = (regLittleEnd << 8) | Wire.read(); //read measurement data 2-nd byte from "wire.h" rxBuffer

    if (Wire.read() == _calculateCRC8(regLittleEnd))  //read measurement CRC  1-st byte from "wire.h" rxBuffer
    {
      regBigEnd = ((regBigEnd << 16) | regLittleEnd); //regBigEnd + regLittleEnd, "return *((float*)&regBigEnd);" gives compile warning 

      float fValue;

      memcpy(&fValue, &regBigEnd, sizeof(fValue));    //convert uint32_t value to float

      return fValue;
    }
    else                                              //received CRC 1-st byte not match computed "little end" CRC for 3-rd & 2-nd byte
    {
      _cmdStatus = SCD30_I2C_CRC8_ERROR;              //CRC error, received CRC8 not match computed CRC8

      return SCD30_I2C_ERROR_VALUE;
    }
  }
  else                                                //received CRC 4-th byte not match computed "big end" CRC for 6-th & 5-th byte
  {
    Wire.read();                                      //remove useless measurement data 3-rd byte from "wire.h" rxBuffer
    Wire.read();                                      //remove useless measurement data 2-nd byte from "wire.h" rxBuffer
    Wire.read();                                      //remove useless measurement CRC  1-st byte from "wire.h" rxBuffer

    _cmdStatus = SCD30_I2C_CRC8_ERROR;                //CRC error, received CRC8 not match computed CRC8

    return SCD30_I2C_ERROR_VALUE;
  }
}


/**************************************************************************/
/*
    _calculateCRC8()

    Calculate CRC8 checksum

    NOTE:
    - see "_writeRegister(uint16_t reg)" for details
    - polynomial x^8+x^5+x^4+1 = 0x31
    - initialization value 0xFF
*/
/**************************************************************************/
uint8_t SCD30_I2C::_calculateCRC8(const uint16_t value)
{
  uint8_t data[2] = {highByte(value), lowByte(value)};    //MSB-byte, LSB-byte 
  uint8_t crc     = 0xFF;                                 //CRC initialization value

  for (uint8_t x = 0; x < 2; x++)                         //0=start array index, 2=array size
  {
    crc ^= data[x];                                       //XOR crc with next input byte

    for (uint8_t i = 0; i < 8; i++)                       //8=8-bits
    {
      if   ((crc & 0x80) != 0) {crc = (crc << 1) ^ 0x31;}
      else                     {crc =  crc << 1;}
    }
  }

  return crc;
}
