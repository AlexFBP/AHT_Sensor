/*
  AHT_Sensor - A Humidity Library for Arduino.

  Supported Sensor modules:
    AHT_Sensor Module - https://www.aliexpress.com/item/33002710848.html
    AHT_Sensor-Breakout Module - http://www.moderndevice.com/products/sht21-humidity-sensor

  Created by Thinary Eletronic at Modern Device on April 2019.
  
  Modified by www.misenso.com on October 2011:
    - code optimisation
    - compatibility with Arduino 1.0

 * This file is part of Thinary_AHT_Sensor.
 *
 * Thinary_AHT_Sensor is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or(at your option) any later version.
 *
 * Sodaq_SHT2x is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with Sodaq_SHT2x.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <math.h>

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>
#include "AHT_Sensor.h"

// Specify the constants for water vapor and barometric pressure.
#ifndef WATER_VAPOR
#define WATER_VAPOR 17.62f
#endif
#ifndef BAROMETRIC_PRESSURE
#define BAROMETRIC_PRESSURE 243.5f
#endif

Sensor_CMD eSensorCalibrateCmd[2][3]    = {{0xE1, 0x08, 0x00}, {0xBE, 0x08, 0x00}};
Sensor_CMD eSensorNormalCmd[3]          = {0xA8, 0x00, 0x00};
Sensor_CMD eSensorMeasureCmd[3]         = {0xAC, 0x33, 0x00};
Sensor_CMD eSensorResetCmd              = 0xBA;

/******************************************************************************
 * Global Functions
 ******************************************************************************/
AHT_Sensor_Class::AHT_Sensor_Class(HUM_Sensor_type _AHT_Sensor_type) {
    AHT_Sensor_type = _AHT_Sensor_type;
}

boolean AHT_Sensor_Class::begin(HUM_SENSOR_ADD _AHT_Sensor_address)
{
    AHT_Sensor_address = _AHT_Sensor_address;
    Serial.println("\x54\x68\x69\x6E\x61\x72\x79\x20\x45\x6C\x65\x74\x72\x6F\x6E\x69\x63\x20\x41\x48\x54\x31\x30\x20\x4D\x6F\x64\x75\x6C\x65\x2E");
    Wire.begin(AHT_Sensor_address);
    Wire.beginTransmission(AHT_Sensor_address);
    Wire.write(eSensorCalibrateCmd[AHT_Sensor_type], 3);
    Wire.endTransmission();
    Serial.println("https://thinaryelectronic.aliexpress.com");
    delay(100);
    if((readStatus()&0x68) == 0x08)
        this->_available = true;
    else
    {
        this->_available = false;
    }
    return this->_available;
}

/**********************************************************
 * measure
 *  Performs one single shot temperature and relative humidity measurement.
 *
 *  @return boolean - Read sensor susccesed(true) or fault(false)
 **********************************************************/
boolean AHT_Sensor_Class::measure(CRC_type _check_CRC)
{
    uint8_t temp[7] = {0}, crc = 0xFF;

    Wire.beginTransmission(AHT_Sensor_address);
    Wire.write(eSensorMeasureCmd, 3);
    Wire.endTransmission();
    delay(100);

    Wire.requestFrom(AHT_Sensor_address, 7);

    for(uint8_t i = 0; Wire.available() > 0; i++)
    {
        temp[i] = Wire.read();
    }

    this->_status = temp[0];
    this->_h_data = temp[1];
    this->_h_data <<= 8;
    this->_h_data |= temp[2];
    this->_h_data <<= 8;
    this->_h_data |= temp[3];
    this->_h_data >>= 4;
    this->_t_data = temp[3] & 0x0F;
    this->_t_data <<= 8;
    this->_t_data |= temp[4];
    this->_t_data <<= 8;
    this->_t_data |= temp[5];
    
    if(this->_status & 0x80) return false;

    if(_check_CRC) {
        for(uint8_t byte_cnt = 0; byte_cnt < 6; byte_cnt++)
        {
            crc ^= temp[byte_cnt];
            for(uint8_t i = 8; i > 0; i--)
            {
                uint8_t crc_8 = crc&0x80;
                crc <<= 1;
                if(crc_8) crc ^= 0x31;
            }
        }
        if(crc != temp[6]) return false;
    }
    
    return true;
}

/**********************************************************
 * GetHumidity
 *  Gets the current humidity from the sensor.
 *
 * @return float - The relative humidity in %RH
 **********************************************************/
float AHT_Sensor_Class::GetHumidity(void)
{
    if (this->_h_data == 0) {
        return 0;                       // Some unrealistic value
    }

    return this->_h_data * 100 / 1048576;
}

/**********************************************************
 * GetTemperature
 *  Gets the current temperature from the sensor.
 *
 * @return float - The temperature in Deg C, Kelvin K, Fahrenheit F
 **********************************************************/
float AHT_Sensor_Class::GetTemperature(Temperature_type _temperature_type)
{
    float temprature = ((200.0 * this->_t_data) / 1048576.0) - 50;

    switch(_temperature_type) {
        case(Celsius):      break;
        case(Kelvin) :      temprature += 273.15;break;
        case(Fahrenheit):   temprature = temprature * 1.8 + 32.0;break;
    }

    return temprature;
}

/**********************************************************
 * GetDewPoint
 *  Gets the current dew point based on the current humidity and temperature
 *
 * @return float - The dew point in Deg C
 **********************************************************/
float AHT_Sensor_Class::GetDewPoint(void)
{
  float humidity = GetHumidity();
  float temperature = GetTemperature();

  // Calculate the intermediate value 'gamma'
  float gamma = log(humidity / 100) + WATER_VAPOR * temperature / (BAROMETRIC_PRESSURE + temperature);
  // Calculate dew point in Celsius
  float dewPoint = BAROMETRIC_PRESSURE * gamma / (WATER_VAPOR - gamma);

  return dewPoint;
}

/**********************************************************
 * readStatus
 *  Read the current status of the sensor
 * 
 * @return unsigned char - The sensor status byte
 **********************************************************/
unsigned char AHT_Sensor_Class::readStatus(void)
{
    Wire.requestFrom(AHT_Sensor_address, 1);
    this->_status = Wire.read();
    return this->_status;
}

/**********************************************************
 * Reset
 *  Reset the sensor
 * 
 * 
 **********************************************************/
void AHT_Sensor_Class::Reset(void)
{
    Wire.beginTransmission(AHT_Sensor_address);
    Wire.write(eSensorResetCmd);
    Wire.endTransmission();
    delay(20);
}
