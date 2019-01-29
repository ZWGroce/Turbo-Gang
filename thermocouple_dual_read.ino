#include <Adafruit_MAX31856.h>

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31856 kthermo = Adafruit_MAX31856(6,7,8,9);
Adafruit_MAX31856 tthermo = Adafruit_MAX31856(2,3,4,5);


void setup() {
  Serial.begin(9600);
  Serial.println("Thermocouple test");

  tthermo.begin();
  kthermo.begin();

  tthermo.setThermocoupleType(MAX31856_TCTYPE_T);
  kthermo.setThermocoupleType(MAX31856_TCTYPE_K);

  tthermo.setColdJunctionFaultThreshholds(-100, 100);
  tthermo.setTempFaultThreshholds(0, 150);

  kthermo.setColdJunctionFaultThreshholds(-100, 100);
  kthermo.setTempFaultThreshholds(0, 2000);
}

void loop() {
  Serial.print("T-Type Thermocouple Temp: "); 
  Serial.println(tthermo.readThermocoupleTemperature());

  Serial.print("K-Type Thermocouple Temp: "); 
  Serial.println(kthermo.readThermocoupleTemperature());
  
  delay(50);
}
