# tesis_v2
This repository contains code and instructions for the development of an autonomous mobile beach cleaner robot version 2. The robot is designed to clean beaches and is controlled using a Jetson nano. The software stack is built on the ROS Melodic distribution.

## STATUS:
all the sensors works well. However, when using roboclaw drivers, I still don't know how to differentiate between the wheels drivers and mechanism drivers.

## Roboclaw 

[roboclaw rep](https://github.com/sonyccd/roboclaw_ros/tree/master)

Copy the nodes files (this repo) into the roboclaw repo.

## IMU BNO055
For using the BNO055 IMU sensor with ROS, follow this tutorial: [How to Publish IMU Data Using ROS and the BNO055 IMU Sensor](https://automaticaddison.com/how-to-publish-imu-data-using-ros-and-the-bno055-imu-sensor/).

## GPS UBLOX

For using a UBLOX GPS receiver with ROS Kinetic, follow this tutorial: [nmea_navsat_driver](http://wiki.ros.org/nmea_navsat_driver). use melodic branch. 

## USB CAM
Just run this command. The repository [usb_cam]( http://wiki.ros.org/usb_cam) has been having issues to build. 

```bash
sudo apt install ros-melodic-usb-cam
```

## BEACH CLEANER ROBOT:
Don't forget to build your workspace and to update your .bashrc 
```bash
catkin_make
```

to test each sensor separately: 
```
roslaunch paquete_prueba imu.launch
roslaunch paquete_prueba roboclaw_mechanism.launch
roslaunch paquete_prueba roboclaw2.launch
roslaunch paquete_prueba usb_cam_test.launch
roslaunch nmea_navsat_driver nmea_serial_driver.launch
```

To run the beach cleaner robot with all the included sensors, use the following command on your  Jetson:
```
roslaunch paquete_prueba beachcleaner_bringup.launch
```
To control the robot using a twist message publisher on your computer, run:

```
roslaunch teleoperation teleop_key.launch
```
