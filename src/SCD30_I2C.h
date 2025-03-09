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
   (4) UART/Modbus require pullup between SEL pin and VDD during power-up, do not exceed +4.0v
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
   ESP32-S3................................. GPIO8            GPIO9            3.3V
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

#ifndef _SCD30_I2C_h
#define _SCD30_I2C_h


#include <Arduino.h>
#include <Wire.h>

#if defined (ARDUINO_ARCH_AVR)
#include <avr/pgmspace.h>          //for Arduino AVR PROGMEM support
#elif defined (ARDUINO_ARCH_ESP8266) || defined (ARDUINO_ARCH_ESP32)
#include <pgmspace.h>              //for Arduino ESP8266 PROGMEM support
#elif defined (ARDUINO_ARCH_STM32)
#include <avr/pgmspace.h>          //for Arduino STM32 PROGMEM support
#endif



/* address */
#define SCD30_I2C_ADDRESS                   0x61     //I2C address

/* commands */
#define SCD30_I2C_START_CONT_MSRMNT_REG     0x0010   //start continuous measurement with optional ambient pressure compensation register
#define SCD30_I2C_STOP_CONT_MSRMNT_REG      0x0104   //stop continuous measurement register
#define SCD30_I2C_CONT_MSRMNT_INTERVAL_REG  0x4600   //set-get measurement interval
#define SCD30_I2C_GET_MSRMNT_READY_REG      0x0202   //get data ready status register
#define SCD30_I2C_READ_MSRMNT_REG           0x0300   //read measurement register
#define SCD30_I2C_ASC_REG                   0x5306   //on/off ASC (automatic self-calibration) algorithm register
#define SCD30_I2C_FRC_REG                   0x5204   //FRC (forced re-calibration) register
#define SCD30_I2C_TEMPERATURE_OFFSET_REG    0x5403   //temperature offset register
#define SCD30_I2C_ALTITUDE_REG              0x5102   //altitude compensation register
#define SCD30_I2C_GET_FW_REG                0xD100   //read firmware version register
#define SCD30_I2C_SOFT_RESET_REG            0xD304   //soft reset register

/* I2C bus communication errors */
#define SCD30_I2C_NO_ERROR                  0x00     //success, no errors
#define SCD30_I2C_DATA_SIZE_ERROR           0x01     //received data smaller/bigger than expected
#define SCD30_I2C_ADDR_ACK_ERROR            0x02     //sensor didn't return ACK to address request (not connected, power issue, broken, long wires (reduce speed), bus locked by slave (increase stretch limit))
#define SCD30_I2C_DATA_ACK_ERROR            0x03     //sensor didn't return ACK to data request (not connected, power issue, broken, long wires (reduce speed), bus locked by slave (increase stretch limit))
#define SCD30_I2C_OTHER_ERROR               0x04     //other errors
#define SCD30_I2C_READY_ERROR               0x05     //measurement not ready (measurement interval too long)
#define SCD30_I2C_CRC8_ERROR                0x06     //received CRC8 not match computed CRC8

/* sensor delays etc */
#define SCD30_I2C_POWER_ON_DELAY            2        //wait for sensor to initialize after power-on, in msec
#define SCD30_I2C_SPEED                     100000   //sensor I2C speed 50KHz..100KHz (recomended 50KHz or smaller), in Hz
#define SCD30_I2C_ACK_STRETCH               30000    //sensor I2C read/write stretch time 30msec, in usec
#define SCD30_I2C_ACK_ASC_STRETCH           150000   //sensor I2C stretch time 150msec due to internal ASC calibration which may occur once per day, in usec
#define SCD30_I2C_RW_DELAY                  3        //delay between writing & reading part of the command should be 3ms, in msec

/* misc */
#define SCD30_I2C_ERROR_VALUE               0x0D903  //return 55555 for CO2/T/H, if any I2C bus communication errors is occurred



class SCD30_I2C
{
  public:
   SCD30_I2C();

  #if defined (ARDUINO_ARCH_AVR)
   bool     begin(uint32_t speed = SCD30_I2C_SPEED, uint32_t stretch = SCD30_I2C_ACK_ASC_STRETCH);
  #elif defined (ARDUINO_ARCH_ESP8266)
   bool     begin(uint8_t sda = SDA, uint8_t scl = SCL, uint32_t speed = SCD30_I2C_SPEED, uint32_t stretch = SCD30_I2C_ACK_ASC_STRETCH);
  #elif defined (ARDUINO_ARCH_ESP32)
   bool     begin(int32_t sda = SDA, int32_t scl = SCL, uint32_t speed = SCD30_I2C_SPEED, uint32_t stretch = SCD30_I2C_ACK_ASC_STRETCH);
  #elif defined (ARDUINO_ARCH_STM32)
   bool     begin(uint32_t sda = SDA, uint32_t scl = SCL, uint32_t speed = SCD30_I2C_SPEED);
  #else
   bool     begin();
  #endif

   bool     startContinuousMeasurement(uint16_t ambientPressure = 1013);
   bool     stopContinuousMeasurement();

   bool     setMeasurementInterval(uint16_t interval = 5);
   uint16_t getMeasurementInterval();

   uint8_t  getCommandStatus();
   bool     getMeasurementStatus();
   void     getMeasurement(float *co2, float *temp, float *humd);

   bool     setAutoCalibration(bool value);
   bool     getAutoCalibration();

   bool     setManualCalibration(uint16_t concentration);
   uint16_t getManualCalibration();

   bool     setTemperatureOffset(const float offsetTemperature);
   float    getTemperatureOffset();

   bool     setAltitude(int16_t altitude);
   int16_t  getAltitude();

   String   readFirmwareVersion();
   bool     reset();

  private:
   uint8_t  _cmdStatus;

   bool     _writeRegister(const uint16_t reg);
   bool     _writeRegister(const uint16_t reg, const uint16_t value);
   uint16_t _readRegister(const uint16_t reg);
   float    _readMeasurement();
   uint8_t  _calculateCRC8(const uint16_t value);
};

#endif
