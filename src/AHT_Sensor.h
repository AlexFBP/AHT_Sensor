/*
  AHT_Sensor - A Humidity Library for Arduino.

  Supported Sensor modules:
    AHT_Sensor Module - https://www.aliexpress.com/item/33002710848.html

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
 * Thinary_AHT_Sensor is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with Thinary_AHT_Sensor.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#ifndef AHT_Sensor_H
#define AHT_Sensor_H

#include <stdint.h>

typedef enum {
    eAHT_SensorAddress_Low     = 0x38,
    eAHT_SensorAddress_High    = 0x39,
} HUM_SENSOR_ADD;

typedef enum {
    AHT10 = 0x00,
    AHT20 = 0x01,
    AHT21 = 0x01,
} HUM_Sensor_type;

typedef enum {
    No_CRC  = 0,
    CRC     = 1,
} CRC_type;

typedef enum {
    Celsius     = 1,
    Kelvin      = 2,
    Fahrenheit  = 3,
} Temperature_type;

typedef unsigned char Sensor_CMD;

class AHT_Sensor_Class
{
    private:
        HUM_SENSOR_ADD AHT_Sensor_address;
        HUM_Sensor_type AHT_Sensor_type;

        bool          _available = false;
        uint8_t       _status;
        uint32_t 			_t_data = 0;
        uint32_t      _h_data = 0;
    public:
        AHT_Sensor_Class(HUM_Sensor_type _AHT_Sensor_type = AHT20);
        boolean         begin(HUM_SENSOR_ADD _AHT_Sensor_address = eAHT_SensorAddress_Low);
        bool            available()       { return this->_available; }
        boolean         measure(CRC_type _check_CRC = No_CRC);
        float           GetHumidity();
        float           GetTemperature(Temperature_type _temperature_type = Celsius);
        float           GetDewPoint();
        uint8_t         readStatus();
        void            Reset();
};

#endif
