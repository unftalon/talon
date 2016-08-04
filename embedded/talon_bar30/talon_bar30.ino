/* Sketch for reading from the Blue Robotics Bar30 Sensor.
    Michael otero
    UNF TALON
*/

#include <ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Temperature.h>
#include <MS5837.h>
#include <i2c_t3.h>

// ROS things ================================================
ros::NodeHandle nh;

sensor_msgs::FluidPressure bar30pressure; // this  name will change later
sensor_msgs::Temperature bar30temp;

ros::Publisher bar30pressure_pub("/sensors/pressure", &bar30pressure);
ros::Publisher bar30temp_pub("/sensors/water_temp", &bar30temp);
// ===========================================================

MS5837 bar30;

void setup() {

  nh.initNode();
  nh.advertise(bar30pressure_pub);
  nh.advertise(bar30temp_pub);

  // open serials
  Serial.begin(9600);
  Serial.println("Starting");
  Wire.begin();

  // initialize the bar30 sensor
  bar30.init();

  // set the fluid density
  // not sure if floats are accepted here, need to check documentation
  bar30.setFluidDensity(1.2754); // kg/m^3 (997 freshwater, 1029 for seawater, 1.2754 for air(??))

}


void loop() {
  bar30.read();

  // convert the temp from C to freedom units
  float bar30TempF = (bar30.temperature()*1.8)+32;

  // convert mbar to pa (because that's what sensor_msgs/Temperature expects)
  // not sure how much this matters
  float bar30PressPa = (bar30.pressure()*100); // 1mbar = 100Pa
  Serial.println(bar30TempF);

  // this delay probably doesn't need to be here
  // but until i understand the ramifications of letting this go faster, i'll leave it.
  delay(1000); 

  bar30pressure.fluid_pressure = bar30PressPa; // assigning the value we get to the thing that will be published
  bar30pressure.variance = 0; // unknown for now

  bar30temp.temperature = bar30TempF;
  bar30temp.variance = 0; // unknown for now
  bar30pressure_pub.publish( &bar30pressure );
  bar30temp_pub.publish( &bar30temp );
  nh.spinOnce();
}
