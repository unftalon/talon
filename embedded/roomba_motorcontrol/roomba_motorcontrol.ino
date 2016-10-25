/* sketch for talking to the arduino motor controller
 *    Michael Otero
 *    UNF TALON
 *    forked from talon_bar30.ino
 *    
 *    TODO:
 *    - set this up to command motors
 *    - set this up to get info from encoders (or maybe a teensy will do this?)
 */


#include <ros.h>
#include <AFMotor.h>
// include any message types here

// ROS things ================================================
ros::NodeHandle nh;

// declare publisher/subscriber topic names here
sensor_msgs::FluidPressure bar30pressure;
sensor_msgs::Temperature bar30temp;

ros::Publisher bar30pressure_pub("/sensors/pressure", &bar30pressure);
ros::Publisher bar30temp_pub("/sensors/water_temp", &bar30temp);
// ===========================================================

void setup() {
  // initalize the ROS node
  nh.initNode();

  // advertise the topics
  nh.advertise(bar30pressure_pub);
  nh.advertise(bar30temp_pub);

  // set up the motor interface
  AF_DCMotor motor3(3); // define motor on channel 4 with 1KHz default PWM
  AF_DCMotor motor4(4);
}

void loop() {
  /* main loop
   *  do all of the things here
   *  
   */

   // publish both of the values to their corresponding ROS topics.
   bar30pressure_pub.publish( &bar30pressure );
   bar30temp_pub.publish( &bar30temp );

  // spin the ros system once. this is important, don't remove it. should be at end of loop.
   nh.spinOnce();
}

