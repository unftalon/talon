/* sketch for talking to the arduino motor controller
 *    Michael Otero
 *    UNF TALON
 *    forked from talon_bar30.ino
 *    
 *    TODO:
 *    - set this up to command motors
 *    - set this up to get info from encoders (or maybe a teensy will do this?)
 *    - maybe move all of the computing to a python node.
 *    
 *    some notes:
 *    - will probably subscribe to a geometry_msgs/Twist topic and translate to motor movement
 */


#include <ros.h>                      // include the ROS library so we can talk to the ROS network
#include <AFMotor.h>                  // include the adafruit motor shield library
#include <geometry_msgs/Twist.h>      // include the ROS message type "Twist" see the wiki for more details.

// ROS things ================================================
ros::NodeHandle nh;

// declare publisher/subscriber topic types and names here
geometry_msgs::Twist cmd_vel;

// setup publishers here
//ros::Publisher bar30pressure_pub("/sensors/pressure", &bar30pressure);
//ros::Publisher bar30temp_pub("/sensors/water_temp", &bar30temp);

// ===========================================================


/* setup an array to hold the 6 Twist values
 *  {linear.x, linear.y, linear.z, angular.x, angular.y, angular.z}
 *  NOTE: for a two-wheel differential drive vehicle, all except linear.x and angular.z will be zero
 */
int vel[6];

// measure the width of the wheel base and divide that by two, then put it here.
#define base_width 1; 

// create a motor object for each motor.
AF_DCMotor motorLeft(3);  // define motor on channel 3 with 1KHz default PWM
AF_DCMotor motorRight(4); // define motor on channel 4 with 1KHz default PWM

// cmd_cb is a function called every time information comes in from the cmd_vel ROS topic.
// it will extract the values of interest and assign them to global variables for use later.
void cmd_cb( const geometry_msgs::Twist& cmd_vel) {

  // with a differential drive robot, we are only interested in the forward velocity and the pivot velocity.
  vel[0] = cmd_vel.linear.x;
  vel[5] = cmd_vel.angular.z;
}// cmd_vel

// set up the subscriber topic type and topic name. For some reason this has to be done after its callback
// maybe it doesn't, maybe i'm missing something. but for now its here.
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmd_cb);

// standard arduino setup function
void setup() {
  
  // initalize the ROS node and open the subscriber
  nh.initNode();
  nh.subscribe(sub);

  // advertise the published topics
  // nh.advertise(bar30pressure_pub);

  // can't use Serial() while using ROS since there is only one com port
  // figure out a way to send debug messages to ROS console
  //Serial.begin(9600);

  // run the motor test function to make sure motor connectivity is good.
  motorTest();
}// setup()

// quick function to run the motors and confirm they are working. this can be run during setup() or any other time
void motorTest() {
  //Serial.println("Testing motors...");

  // set the speed and direction for both motors
  motorRight.setSpeed(255);
  motorLeft.setSpeed(255);
  motorRight.run(FORWARD);
  motorLeft.run(FORWARD);
  delay(1000);

  // after a brief delay, stop both motors
  motorRight.run(RELEASE);
  motorLeft.run(RELEASE);
}// motorTest

void loop() {
  /* main loop
   *  do all of the things here
   *  
   */

   /* below will be the continious conversion of the input command to the motor command.
    *  there should be some conditional statements to figure out mapping the values
    *  to reverse, forward, and all of the above.
    */
    
    // these values will be between -255 and 255
   int rightCmd = vel[0] + vel[5]*base_width; // base_width might make more sense if it was called "trim" or something
   int leftCmd = vel[0] - vel[5]*base_width;
   
   // conditional statements to determine which way to run the motors. that moment when you see a  6 successive if statements.
   if (rightCmd > 0) {
      motorRight.run(FORWARD);
   }
   else if (rightCmd < 0) {
      motorRight.run(BACKWARD);
   }
   else if (rightCmd == 0) {
      motorRight.run(RELEASE);
   }

   if (leftCmd > 0) {
      motorLeft.run(FORWARD);
   }
   else if (leftCmd < 0) {
      motorLeft.run(BACKWARD);
   }
   else if (leftCmd == 0) {
      motorLeft.run(RELEASE);
   }
   
   //nh.loginfo(left); // this is supposed to print information to the ROS console but not sure how it works yet.
   motorRight.setSpeed(abs(rightCmd));
   motorLeft.setSpeed(abs(leftCmd));

   //Serial.print("Right value: ");
   //Serial.println(right);
   //Serial.print("Left value: ");
   //Serial.println(left);

   // publish any topics as necessary here
   //bar30pressure_pub.publish( &bar30pressure);

  // spin the ros system once. this is important, don't remove it. should be at end of loop.
   nh.spinOnce();
}// main loop

