/*
 * TALON Thruster Driver Sketch
 * derived from the rosserial Servo Control Example
 *
 * This sketch runs through a test sequence for the thrusters and communicates statuses to ROS.
 * 
 * Currently designed for Arduino Uno for the configuration on the TALON I sub
 *
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * information on connecting and communicating with the Afro ESCs (known as Basic ESC from BlueRobotics)
 * can be found here:
 * http://docs.bluerobotics.com/besc/
 */


#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
//#include <geometry_msgs/Twist.h>

// define the thruster pin assignments. if you are confused about what these words are, you need to get your sea legs!
#define TH_BOW_SWAY 0
#define TH_BOW_HEAVE 1
#define TH_PORT_SURGE 9
#define TH_STARBOARD_SURGE 3
#define TH_STERN_SWAY 4
#define TH_STERN_HEAVE 5

// the microsends needed for "stop" signal, which initalizes the ESCs.
int ESC_initms = 1500;

// the amount of time to delay after ESC initalizing, in seconds (probably don't need to change this)
int ESC_initDelay = 1;

// Instantiate the node handle, allows us to create publishers and subscribers
ros::NodeHandle  nh;

// create the servo instances, one for each ESC
Servo ESC_BOW_SWAY;
Servo ESC_BOW_HEAVE;
Servo ESC_PORT_SURGE;
Servo ESC_STARBOARD_SURGE;
Servo ESC_STERN_SWAY;
Servo ESC_STERN_HEAVE;

// create the callback function using the twist message type, then name it.
void servo_cb( const geometry_msgs::Twist& cmd_msg) {

  // create variables for each controllable degree of freedom.
  //
  // NOTE: for TALON I, we only have 5DOF (no ROLL).
  // also, until we integrate a dual stick setup, there is no intuitive way
  // to TILT since ANGULAR_Y and LINEAR_X would typically be the same
  // "forwards backwards" on a joystick.
  //
  // also, I realized i'm not sure about angular+linear yet, lets do that later.
  int LINEAR_Z_HEAVE = map(cmd_msg.linear.z, -180, 180, 1100, 1900);
  int LINEAR_X_SURGE = map(cmd_msg.linear.x, -180, 180, 1100, 1900);
  int LINEAR_Y_SWAY = map(cmd_msg.linear.y, -180, 180, 1100, 1900);
  //int ANGULAR_Z_YAW = map(cmd_msg.angular.z, -180, 180, 1100, 1900);

  // send the commands to the thruster pairs.

  // HEAVE pair
  ESC_BOW_HEAVE.writeMicroseconds(LINEAR_Z_HEAVE);
  ESC_STERN_HEAVE.writeMicroseconds(LINEAR_Z_HEAVE);

  // SURGE pair
  ESC_PORT_SURGE.writeMicroseconds(LINEAR_X_SURGE);
  ESC_STARBOARD_SURGE.writeMicroseconds(LINEAR_X_SURGE);

  // SWAY pair
  ESC_BOW_SWAY.writeMicroseconds(LINEAR_Y_SWAY);
  ESC_STERN_SWAY.writeMicroseconds(LINEAR_Y_SWAY);

  // YAW PAIR
  // i don't think we can YAW yet

  // blink the LED to say we got something
  digitalWrite(13, HIGH - digitalRead(13)); //toggle led
}

// setup the subscriber with topic name "NAME" and type "geometry_msgs" and the name of the cb function
ros::Subscriber<geometry_msgs::Twist> sub("servo", servo_cb);

// arduino setup function, runs once.


// for setting up the ESCs, using the servo library.
void initESCs() {
  // attach the ESCs to the pins
  ESC_BOW_HEAVE.attach(TH_BOW_HEAVE);
  ESC_BOW_SWAY.attach(TH_BOW_SWAY);
  ESC_PORT_SURGE.attach(TH_PORT_SURGE);
  ESC_STARBOARD_SURGE.attach(TH_STARBOARD_SURGE);
  ESC_STERN_HEAVE.attach(TH_STERN_HEAVE);
  ESC_STERN_SWAY.attach(TH_STERN_SWAY);

  // here we have to send "stop" signal to all ESCs to initialize them
  // NOTE: i don't know if this works yet, need to test with actual ESCs

  // HEAVE pair
  ESC_BOW_HEAVE.writeMicroseconds(ESC_initms);
  ESC_STERN_HEAVE.writeMicroseconds(ESC_initms);

  // SURGE pair
  ESC_PORT_SURGE.writeMicroseconds(ESC_initms);
  ESC_STARBOARD_SURGE.writeMicroseconds(ESC_initms);

  // SWAY pair
  ESC_BOW_SWAY.writeMicroseconds(ESC_initms);
  ESC_STERN_SWAY.writeMicroseconds(ESC_initms);
  delay(ESC_initDelay*1000);
}

void setup() {
  // setup our favorite built in LED for diagnostic purposes.
  pinMode(13, OUTPUT);
  
  // initialize ROS node handle, advertise any topics being published and subscribe to any topics we want to listen to
  nh.initNode();
  nh.subscribe(sub);

  // run the ESC init sequence
  initESCs();

}

void loop() {
  // this keeps the node open
  nh.spinOnce();
  delay(1);
}
