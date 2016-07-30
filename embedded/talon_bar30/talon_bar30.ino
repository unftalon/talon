/* Sketch for testing the Bar30 Pressure Sensor.
    Michael otero
    UNF TALON
*/

#include <MS5837.h>
#include <Wire.h>

MS5837 bar30;

void setup() {
  Serial.begin(9600);

  Serial.println("Starting");

  Wire.begin();

  sensor.init();

  sensor.setFluidDensity(997); // kg/m^3 (997 freshwater, 1029 for seawater)

}

void loop() {
  sensor.read();

  Serial.print("Pressure: ");
  Serial.print(sensor.pressure());
  Serial.println(" mbar");

  Serial.print("Temperature: ");
  Serial.print(sensor.temperature());
  Serial.println(" deg C");

  Serial.print("Depth: ");
  Serial.print(sensor.depth());
  Serial.println(" m");

  Serial.print("Altitude: ");
  Serial.print(sensor.altitude());
  Serial.println(" m above mean sea level");

  delay(1000);

}
