#include <Wire.h>
#include <AHT_Sensor.h>

AHT_Sensor_Class aht20(AHT20);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  if(aht20.begin(eAHT_SensorAddress_Low))
    Serial.println("Init AHT20 Sucess.");
  else
    Serial.println("Init AHT20 Failure.");
}

void loop() {
  // put your main code here, to run repeatedly:
  aht20.measure(No_CRC);
  Serial.println("//Thinary Eletronic AHT20 Module//");
  Serial.println("https://thinaryelectronic.aliexpress.com");
  Serial.println(String("")+"Humidity    (%RH):\t"+aht20.GetHumidity()+"%");
  Serial.println(String("")+"Temperature (°C) :\t"+aht20.GetTemperature()+"°C");
  Serial.println(String("")+"Dewpoint    (°C) :\t"+aht20.GetDewPoint()+"°C");
  delay(1000);
}
