/***************************************************************************************************/
/*
   This is an Arduino example for a Sensirion SCD30 sensor controlled via I2C bus

   written by : enjoyneering
   sourse code: https://github.com/enjoyneering/

   Sensirion SCD30 spec:
   - NDIR CO2 optical sensor with dual-channel detection technology
   - power supply voltage +3.3v..+5.5v
   - maximum average current 19mA (idle state & processing), peak current 75mA (processing)
   - energy consumption 120mJ per measurement
   - CO2 working temperature range 0C..+50C at 5%..95%, if out wait for 2 hours at 25C
   - CO2 measurement range 0ppm..40000ppm
   - CO2 accuracy +-(30ppm + 3% of measured value) in range 400ppm..10000ppm
   - humidity measurement range 0%..100%
   - humidity accuracy +-2%
   - temperature measurement range -40C..+70C
   - temperature accuracy +-0.3C
   - I2C not 5v tolerant, level converter required & requires SEL pin floating or connected to GND
   - lifetime 15years


   This device uses I2C bus to communicate, specials pins are required to interface
   Board                                   SDA                   SCL                   Level
   Uno, Mini, Pro, ATmega168, ATmega328... A4                    A5                    5v
   Mega2560............................... 20                    21                    5v
   Due, SAM3X8E........................... 20                    21                    3.3v
   Leonardo, Micro, ATmega32U4............ 2                     3                     5v
   Digistump, Trinket, ATtiny85........... PB0                   PB2                   5v
   Blue Pill*, STM32F103xxxx boards*...... PB9/PB7               PB8/PB6               3.3v/5v
   ESP8266 ESP-01**....................... GPIO0                 GPIO2                 3.3v/5v
   NodeMCU 1.0**, WeMos D1 Mini**......... GPIO4/D2              GPIO5/D1              3.3v/5v
   ESP32.................................. GPIO21/D21            GPIO22/D22            3.3v
                                          *hardware I2C Wire mapped to Wire1 in stm32duino
                                           see https://github.com/stm32duino/wiki/wiki/API#i2c
                                         **boards has 10-12kOhm pullup-up resistor on GPIO2/D4/LED,
                                           GPIO0/D3 & pullup-down on GPIO15/D8 for flash & boot

   Supported frameworks:
   AVR     Core       -  https://github.com/arduino/ArduinoCore-avr
   ATtiny  Core       -  https://github.com/SpenceKonde/ATTinyCore
   ESP32   Core       -  https://github.com/espressif/arduino-esp32
   ESP8266 Core       -  https://github.com/esp8266/Arduino
   STM32   Core       -  https://github.com/stm32duino/Arduino_Core_STM32


   GNU GPL license, all text above must be included in any redistribution,
   see link for details  - https://www.gnu.org/licenses/licenses.html
*/
/***************************************************************************************************/

#include <Wire.h>
#include <SCD30_I2C.h>


SCD30_I2C sensorCO2;


float co2;  //in ppm
float temp; //in C
float humd; //in RH%



/************************************************************************************/
/*
    setup()

    Main setup

    NOTE:
    - SCD30 proper command sequence:
      - set sempling interval, if needed
      - set FRC procedure or continuous ASC algorithm, if needed
      - set temperature offset, if needed
      - start measurement
*/
/************************************************************************************/
void setup()
{
  /* start serial */ 
  Serial.begin(115200);
  Serial.println();

  /* start I2C bus & check for Sensirion SCD30 */
  while (sensorCO2.begin() != true)                                            //for ESP8266 ESP-01 use sensorCO2.begin(0, 2)
  {
    Serial.println(F("Sensirion SCD30 not detected"));                         //make sure Sensirion SCD30 "SEL" pin is floating or connected to GND
    printStatus();                                                             //get error code of the last I2C command, in our case last command is "sensorCO2.begin()"
    delay(5000);
  }
  Serial.println(F("Sensirion SCD30 detected"));

  /* print fw version */
  Serial.print(F("fw: "));
  Serial.println(sensorCO2.readFirmwareVersion());

  /* start continuous measurement with ambient pressure compensation */
  sensorCO2.startContinuousMeasurement(1013);                                  //range 700mBar...1400mBar & 0mBar to disable ambient pressure compensation

  /* reset */
  if (sensorCO2.getCommandStatus() != SCD30_I2C_NO_ERROR) {sensorCO2.reset();} //reset as last chance to make it work
}


/************************************************************************************/
/*
    loop()

    Main loop

    NOTE:
    - outdoor air is usually between 350ppm..480ppm in normal geographic locations
    - library returns 99999 for CO2, T & RH if a communication error occurs or CRC
      doesn't match
*/
/************************************************************************************/
void loop()
{
  sensorCO2.getMeasurement(&co2, &temp, &humd);           //get measurement & update co2/temp/humd variables, see NOTE

  Serial.println(F("measurement:"));

  if (sensorCO2.getCommandStatus() == SCD30_I2C_NO_ERROR) //get error code of the last I2C command, in our case last command is "sensorCO2.getMeasurement()"
  {
    Serial.print(F(" -co2: "));
    Serial.print(co2);
    Serial.println(F("ppm"));

    Serial.print(F(" -temp: "));
    Serial.print(temp);
    Serial.println(F("C"));

    Serial.print(F(" -humd: "));
    Serial.print(humd);
    Serial.println(F("%"));
  }
  else
  {
    printStatus();
  }

  delay(10000);
}


/************************************************************************************/
/*
    printStatus()

    Get error code of the last I2C command

    NOTE:
    - returned values by "getCommandStatus()":
      - 0x00, success
      - 0x01, data too long to fit in transmit data buffer
      - 0x02, received NACK on transmit of address
      - 0x03, received NACK on transmit of data
      - 0x04, other error

      - 0x05, measurement not ready (measurement interval too long)
      - 0x06, received CRC8 not match computed CRC8
*/
/************************************************************************************/
void printStatus()
{
  Serial.print(F(" -error: "));

  switch (sensorCO2.getCommandStatus())
  {
    case SCD30_I2C_NO_ERROR:
      Serial.println(F("normal operation"));                            //success
      break;

    case SCD30_I2C_DATA_SIZE_ERROR:
      Serial.println(F("received data smaller/bigger than expected"));  //not connected, power issue, broken, long wires (reduce speed), bus locked by slave (increase stretch limit)
      break;

    case SCD30_I2C_ADDR_ACK_ERROR:
      Serial.println(F("sensor didn't return ACK to address request")); //not connected, power issue, broken, long wires (reduce speed), bus locked by slave (increase stretch limit)
      break;

    case SCD30_I2C_DATA_ACK_ERROR:
      Serial.println(F("sensor didn't return ACK to data request"));    //not connected, power issue, broken, long wires (reduce speed), bus locked by slave (increase stretch limit)
      break;

    case SCD30_I2C_OTHER_ERROR:
      Serial.println(F("communication error"));
      break;

    case SCD30_I2C_READY_ERROR:
      Serial.println(F("measurement not ready"));                       //sensor measurement interval too long
      break;

    case SCD30_I2C_CRC8_ERROR:
      Serial.println(F("CRC8 mismatch"));                               //received CRC8 not match computed CRC8
      break;

    default:
      Serial.println(F("uknown"));
      break;
  }
}
