#!/usr/bin/env python2
import threespace as ts_api


print "Imported threespace, now lets connect to com port..."

#try:
device = ts_api.TSUSBSensor(com_port='/dev/ttyACM0')
#except:
#	print "Failed to connect on /dev/ttyACM0" 

if device is not None:
    ## Now we can start getting information from the device.
    ## The class instances have all of the functionality that corresponds to the
    ## 3-Space Sensor device type it is representing.
    print("It looks like we've connected.")
    print("==================================================")
    print("Getting the filtered tared quaternion orientation.")
    quat = device.getTaredOrientationAsQuaternion()
    if quat is not None:
        print(quat)
    print("==================================================")
    print("Getting the raw sensor data.")
    data = device.getAllRawComponentSensorData()
    if data is not None:
        print("[%f, %f, %f] --Gyro\n"
              "[%f, %f, %f] --Accel\n"
              "[%f, %f, %f] --Comp" % data)
    print("==================================================")
    print("Getting the LED color of the device.")
    led = device.getLEDColor()
    if led is not None:
        print(led)
    print("==================================================")
    
    ## Now close the port.
    device.close()
