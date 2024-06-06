# tesis_v2
This repository contains code and instructions for the development of an autonomous mobile beach cleaner robot version 2. The robot is designed to clean beaches and is controlled using a Jetson nano. The software stack is built on the ROS Melodic distribution.

## STATUS:
all the sensors works well. However, when using roboclaw drivers, I still don't know how to differentiate between the wheels drivers and mechanism drivers.
```bash
dmesg | grep tty
```

## Roboclaw 

[roboclaw rep](https://github.com/sonyccd/roboclaw_ros/tree/master)

Copy the nodes files (this repo) into the roboclaw repo.

## IMU BNO055
For using the BNO055 IMU sensor with ROS, follow this tutorial: [How to Publish IMU Data Using ROS and the BNO055 IMU Sensor](https://automaticaddison.com/how-to-publish-imu-data-using-ros-and-the-bno055-imu-sensor/).

## GPS UBLOX

For using a UBLOX GPS receiver with ROS Melodic, follow this tutorial: [nmea_navsat_driver](http://wiki.ros.org/nmea_navsat_driver). use melodic branch. 

## USB CAM
Just run this command. 

```bash
sudo apt install ros-melodic-usb-cam
ls /dev | greo video
sudo apt install ros-melodic-perception
```
The repository [usb_cam]( http://wiki.ros.org/usb_cam) has been having issues to build. 

## paquete_prueba:

```bash
catkin_create_pkg beach_cleaner rospy roscpp usb_cam std_msgs image_transport cv_bridge sensor_msgs geometry_msgs
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

## EXTRAS:

Maybe this forum could help you to enable permission of serial ports

https://unix.stackexchange.com/questions/25258/ttyusb0-permission-changes-after-restart

```bash
cd etc/udev/rules.d
sudo nano my-newrule.rules

KERNEL=="ttyACM0", MODE="0666"
KERNEL=="ttyACM1", MODE="0666"
KERNEL=="ttyACM2", MODE="0666"
KERNEL=="ttyUSB0", MODE="0666"

sudo reboot
```
