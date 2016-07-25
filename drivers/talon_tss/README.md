# talon_tss
Driver for the YEI Tech Three-Space Sensor IMU. Import ```threespace_api.py``` into your script to gain all the TSS functions.
# Usage
In order to use this package, you must run setup.py threespace-master archive to install the tss libraries. Most people can figure out how to just have the libraries all in the same package and not installed into your system, but I'm just a mech engineering student who only knows how to bash his head on the keyboard until python hacks itself into working my way.
- To run, type ```rosrun talon_tss imu_node.py```
# TODO
- write functionality so this will automatically know what port the sensor is connected to
- figure out how to not require installation of the yei tss sensor.
