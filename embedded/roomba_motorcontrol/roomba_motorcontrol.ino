/* sketch for talking to the arduino motor controller
 *    Michael Otero
 *    UNF TALON
 *    forked from talon_bar30.ino
 *    
 *    TODO:
 *    - set this up to command motors
 *    - set this up to get info from encoders (or maybe a teensy will do this?)
 *    
 *    some notes:
 *    - will probably subscribe to a geometry_msgs/Twist topic and translate to motor movement
 */


#include <ros.h>     // include the ROS library so we can talk to the ROS network
#include <AFMotor.h> // include the adafruit motor shield library
#include <geometry_msgs/Twist.h>
// include any necessary message types here

// ROS things ================================================
ros::NodeHandle nh;

// declare publisher/subscriber topic names here
//sensor_msgs::FluidPressure bar30pressure;
//sensor_msgs::Temperature bar30temp;
geometry_msgs::Twist cmd_vel;

//ros::Publisher bar30pressure_pub("/sensors/pressure", &bar30pressure);
//ros::Publisher bar30temp_pub("/sensors/water_temp", &bar30temp);

// ===========================================================

/* setup an array to hold the 6 Twist values
 *  {linear.x, linear.y, linear.z, angular.x, angular.y, angular.z}
 */
int vel[6];
#define base_width 0.5; // width of wheel base. should be actually measured.

  // set up the motor interface
AF_DCMotor motorLeft(3); // define motor on channel 4 with 1KHz default PWM
AF_DCMotor motorRight(4);

void cmd_cb( const geometry_msgs::Twist& cmd_vel) {
  // things go here for callback
  vel[0] = cmd_vel.linear.x;
  vel[5] = cmd_vel.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmd_cb);

void setup() {
  // initalize the ROS node
  nh.initNode();
  nh.subscribe(sub);

  // advertise the topics
  // nh.advertise(bar30pressure_pub);



  
  //Serial.begin(9600);

  // run the motor test function to make sure this works.
  motorTest();
}

void motorTest() {
  //Serial.println("Testing motors...");
  motorRight.setSpeed(255);
  motorLeft.setSpeed(255);
  motorRight.run(FORWARD);
  motorLeft.run(FORWARD);
  delay(1000);

  motorRight.run(RELEASE);
  motorLeft.run(RELEASE);
}

void loop() {
  /* main loop
   *  do all of the things here
   *  
   */

   int right = vel[0] + vel[5]*base_width;
   int left = vel[0] - vel[5]*base_width;
   //nh.loginfo(left);
   motorRight.setSpeed(right);
   motorRight.run(FORWARD);

   //Serial.print("Right value: ");
   //Serial.println(right);
   //Serial.print("Left value: ");
   //Serial.println(left);

   // publish both of the values to their corresponding ROS topics.
   //bar30pressure_pub.publish( &bar30pressure );
   //bar30temp_pub.publish( &bar30temp );

  // spin the ros system once. this is important, don't remove it. should be at end of loop.
   nh.spinOnce();
}

