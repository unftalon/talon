/* Sketch for testing the Bar30 Pressure Sensor.
    Michael otero
    UNF TALON
*/

#include <MS5837.h>
#include <i2c_t3.h>

MS5837 bar30;

void setup() {
  Serial.begin(9600);

  Serial.println("Starting");

  Wire.begin();

  bar30.init();

  bar30.setFluidDensity(1); // kg/m^3 (997 freshwater, 1029 for seawater)

}

void loop() {
  bar30.read();

  Serial.print("Pressure: ");
  Serial.print(bar30.pressure());
  Serial.println(" mbar");

  Serial.print("Temperature: ");
  Serial.print(bar30.temperature());
  Serial.println(" deg C");

  Serial.print("Depth: ");
  Serial.print(bar30.depth());
  Serial.println(" m");

  Serial.print("Altitude: ");
  Serial.print(bar30.altitude());
  Serial.println(" m above mean sea level");

  delay(1000);

}
