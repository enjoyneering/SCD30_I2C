/***************************************************************************************************/
/*
   This is an Arduino example for a Sensirion SCD30 sensor controlled via I2C bus

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
     - UART/Modbus (4)
     - I2C (5)
     - PWM
   - lifetime 15years (6)

   (1) longer measurement interval reduces power consumption but makes the sensor slower and unable
       to track rapidly changing CO2 concentrations
   (2) CO2 levels and accuracy are not guaranteed at negative temperatures
   (3) any air flow around sensor generate pressure drops, back pressure and dynamic fluctuations
       leading to increased sensor noise and reduced accuracy
   (3) optical cell should never be in contact with an object, SCD30 are delicate optical systems
       and mechanical stresses on the optical cavity could change physical properties & measurement
       accuracy, in cases of mechanical stress run calibration to restore accuracy
   (4) UART/Modbus require pullup between SEL pin and VDD during power-up, do not exceed +4.0v
   (5) I2C requires SEL pin floating or connected to GND, address 0x61, speed 50KHz..100KHz,
       clock stretching 30ms..150ms, internal pull-up 45kOhm to +3.0v (not 5v tolerant, level converter required)
   (6) exposing sensor to direct sunlight can cause temperature fluctuations and rapid aging of SCD30


   This device uses I2C bus to communicate, specials pins are required to interface
   Board                                   SDA                   SCL                   Level
   ESP8266 ESP-01**....................... GPIO0                 GPIO2                 3.3v/5v
   NodeMCU 1.0**, WeMos D1 Mini**......... GPIO4/D2              GPIO5/D1              3.3v/5v
                                         **boards has 10-12kOhm pullup-up resistor on GPIO2/D4/LED,
                                           GPIO0/D3 & pullup-down on GPIO15/D8 for flash & boot


   GNU GPL license, all text above must be included in any redistribution,
   see link for details  - https://www.gnu.org/licenses/licenses.html
*/
/***************************************************************************************************/

#include <ESP8266WiFi.h>
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
  
    - apart from the hardware FIFO (128-bytes for TX & RX), Serial.h has an
      additional customizable 256-byte RX buffer. For transmit-only operation,
      the 256-byte RX buffer can be switched off to save RAM by passing mode
      "SERIAL_TX_ONLY" to "Serial.begin()"
    - to chage RX buffer size use "Serial.setRxBufferSize(size)", should be called
      before "Serial.begin()", default 256-byte
*/
/************************************************************************************/
void setup()
{
  /* disable WiFi */
  WiFi.persistent(false);                              //disable saving WiFi settings in SPI flash
  WiFi.mode(WIFI_OFF);                                 //disable radio (STA & AP off), WiFi.forceSleepBegin() will not work without it 
  WiFi.forceSleepBegin(0);                             //put modem to sleep, 0=forever or usec
  WiFi.persistent(true);                               //enable saving WiFi settings in SPI flash

  /* start serial */ 
  Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);    //see NOTE
  Serial.println();

  /* start I2C bus & check for Sensirion SCD30 */
  while (sensorCO2.begin(4, 5, 100000) != true)        //4=SDA, 5=SCL, 100000=100KHz I2C speed, for ESP8266 ESP-01 use sensorCO2.begin(0, 2)
  {
    Serial.println(F("Sensirion SCD30 not detected")); //make sure Sensirion SCD30 "SEL" pin is floating or connected to GND
    printStatus();                                     //get error code of the last I2C command, in our case last command is "sensorCO2.begin()"
    delay(5000);
  }
  Serial.println(F("Sensirion SCD30 detected"));

  /* print measurement interval */
     //longer measurement interval reduces power consumption but makes sensor slower and unable to track rapidly changing CO2 concentrations
  Serial.print(F("interval: "));
  Serial.print(sensorCO2.getMeasurementInterval());
  Serial.println(F("sec"));

  /* print ASC (automatic self-calibration) algorithm status */
     //ASC self-calibrates approximately every 1..3 weeks, if sensor regularly sees fresh air for 1 hour every day
     //if CO2 is more than 400ppm throughout the day, apply FRC (forced re-calibration) from time to time to reduce sensor deviations & drift
  Serial.print(F("ASC: "));
  Serial.println(sensorCO2.getAutoCalibration() ? "on" : "off");


  /* print FRC (forced re-calibration) procedure compensation value */
     //ASC overrides FRC value, if ASC is enabled for at least 1 week and conditions & assumptions for ASC are fulfilled (sensor regularly sees fresh air (400ppm) for 1 hour every day)
  Serial.print(F("FRC: "));
  Serial.print(sensorCO2.getManualCalibration());
  Serial.println(F("ppm"));

  /* print temperature offset compensation value */
     //CO2 level depends on ambient temperature, humidity & pressure (altitude)
     //to compensate incorrect ambient temperature
  Serial.print(F("offset: "));
  Serial.print(sensorCO2.getTemperatureOffset());
  Serial.println(F("C"));

  /* print altitude compensation value */
     //CO2 level depends on ambient temperature, humidity & pressure (altitude)
     //new ambient pressure value overrides previous altitude compensation value
  Serial.print(F("altitude: "));
  Serial.print(sensorCO2.getAltitude());
  Serial.println(F("m"));

  /* print fw version */
  Serial.print(F("fw: "));
  Serial.println(sensorCO2.readFirmwareVersion());

  /* start continuous measurement with ambient pressure compensation */
     //CO2 level depends on ambient temperature, humidity & pressure (altitude)
     //range 700mBar...1400mBar & 0mBar to disable ambient pressure compensation
     //google average ambient pressure for the year in your area, in mBar
  sensorCO2.startContinuousMeasurement(1013);

  /* set measurement interval */
     //measurement interval range 2sec..1800sec
     //recommended measurement interval 2sec..60sec
     //modifying sampling interval reduces CO2 signal accuracy, causes out-of-spec measurements & CO2 signal recalibration is recomended
  //sensorCO2.setMeasurementInterval(5);

  /* enable/disable ASC algorithm */
     //ASC generates a reference value autonomously from historical sensor data as long as sensor in continuous measurement mode and conditions for ASC are fulfilled
     //true=continuous ASC on, false=continuous ASC off
  //sensorCO2.setAutoCalibration(true);

  /* set FRC value */
     //start continuous measurement mode & set 2sec measurement interval, put sensor in environment where CO2 concentration level is equal to target concentration, after 2 minutes set FRC with reference value
     //recomended environment temperature dirung FRC 0C..+50C at 5%..95%
     //CO2 concentration in fresh air 400ppm
  //sensorCO2.setManualCalibration(400);

  /* set temperature offset value */
     //sensor must be operated in desired enclosure at wanted sampling interval for 10 minutes for thermal stabilization
     //measure ambient temperature outside of the enclosure with a separate calibrated temperature sensor, calculate and apply offset
     //temperature offset range 0C..+70C with minimum step +0.01C
  //sensorCO2.setTemperatureOffset(0.01);

  /* set altitude compensation */
     //to activate ambient pressure compensation call "startContinuousMeasurement()" again with pressure equal to 0mBar
     //altitude compensation range -3100m...+3300m
     //google your altitude
  //sensorCO2.setAltitude(150);


  /* reset */
     //last chance to make it work
     //soft reset puts sensor in the same state as after power-up
     //automatically reloads all calibrations & saved settings (ASC or FRC, temperature offset, etc)
  if (sensorCO2.getCommandStatus() != SCD30_I2C_NO_ERROR) {sensorCO2.reset();}
}


/************************************************************************************/
/*
    loop()

    Main loop

    NOTE:
    - outdoor air is usually between 350ppm..480ppm in normal geographic locations
    - library returns 99999 for CO2, T & RH if a communication error occurs or CRC
      doesn't match

    - CO2 response time (t63) 20sec at measurement interval range 2sec
    - SHT31 humidity response time (t63) 8sec
    - SHT31 temperature response time (t63) 10sec
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

  delay(10000);                                           //rule of thumb, delay == sensor measurement interval x 4..5 times, e.g. 5sec x 5 = 25sec
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
