#include <Wire.h>
#include <AHT_Sensor.h>

AHT_Sensor_Class aht10(AHT10);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  if(aht10.begin(eAHT_SensorAddress_Low))
    Serial.println("Init AHT10 Sucess.");
  else
    Serial.println("Init AHT10 Failure.");
}

void loop() {
  // put your main code here, to run repeatedly:
  aht10.measure(No_CRC);
  Serial.println("//Thinary Eletronic AHT10 Module//");
  Serial.println("https://thinaryelectronic.aliexpress.com");
  Serial.println(String("")+"Humidity    (%RH):\t"+aht10.GetHumidity()+"%");
  Serial.println(String("")+"Temperature (°C) :\t"+aht10.GetTemperature()+"°C");
  Serial.println(String("")+"Dewpoint    (°C) :\t"+aht10.GetDewPoint()+"°C");
  delay(1000);
}
