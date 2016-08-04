/* Sketch for testing the Bar30 Pressure Sensor.
    Michael otero
    UNF TALON
*/

#include <ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <MS5837.h>
#include <i2c_t3.h>

// ROS things ================================================
ros::NodeHandle nh;
sensor_msgs::FluidPressure bar30Value; // this  name will change later
ros::Publisher bar30sens("bar30sens", &bar30Value);

MS5837 bar30;

void setup() {

  nh.initNode();
  nh.advertise(bar30sens);

  // open serials
  Serial.begin(9600);
  Serial.println("Starting");
  Wire.begin();

  // initialize the bar30 sensor
  bar30.init();

  // set the fluid density
  // not sure if floats are accepted here, need to check documentation
  bar30.setFluidDensity(1); // kg/m^3 (997 freshwater, 1029 for seawater, 1 for air(??))

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

  
  bar30Value.fluid_pressure = bar30.depth(); // assigning the value we get to the thing that will be published
  bar30Value.variance = 0;
  bar30sens.publish( &bar30Value );
  nh.spinOnce();
}
