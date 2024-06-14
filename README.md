# tesis_v2
This repository contains code and instructions for the development of an autonomous mobile beach cleaner robot version 2. The robot is designed to clean beaches and is controlled using a Jetson nano. The software stack is built on the ROS Melodic distribution on jetson nano. Some codes are built on python3 and use them in your computer Ros noetic. 

## STATUS:
all the sensors works well. However, when using roboclaw drivers, I still don't know how to differentiate between the wheels drivers and mechanism drivers.
```bash
dmesg | grep tty
```
## Starting

```bash
cd ~/catkin_ws/src
git clone https://github.com/markopuch/tesis_v2.git
git clone https://github.com/sonyccd/roboclaw_ros.git
git clone -b melodic-devel https://github.com/ros-drivers/nmea_navsat_driver.git
git clone https://github.com/dheera/ros-imu-bno055.git
sudo apt install ros-melodic-usb-cam
cd
cp  ~/catkin_ws/src/tesis_v2/nodes/roboclaw2.py ~/catkin_ws/src/roboclaw_ros/roboclaw_node/nodes
cp  ~/catkin_ws/src/tesis_v2/nodes/roboclaw_mechanism.py ~/catkin_ws/src/roboclaw_ros/roboclaw_node/nodes
cd ~/catkin_ws/
catkin_make
```

### Roboclaw 
[roboclaw rep](https://github.com/sonyccd/roboclaw_ros/tree/master)

Copy the nodes files (this repo) into the roboclaw repo.

```bash
dmesg | grep tty
sudo apt-get update
cd ~/catkin_ws/src
git clone https://github.com/sonyccd/roboclaw_ros.git
cd ~/catkin_ws/
catkin_make
```

### IMU BNO055
For using the BNO055 IMU sensor with ROS, follow this tutorial: [How to Publish IMU Data Using ROS and the BNO055 IMU Sensor](https://automaticaddison.com/how-to-publish-imu-data-using-ros-and-the-bno055-imu-sensor/).

```bash
sudo i2cdetect -r -y 1
sudo apt-get update
sudo apt-get install libi2c-dev
cd ~/catkin_ws/src
git clone https://github.com/dheera/ros-imu-bno055.git
cd ~/catkin_ws/
catkin_make --only-pkg-with-deps imu_bno055
rosdep update
rosdep check imu_bno055
sudo reboot
sudo apt-get install ros-melodic-rviz-imu-plugin
```

## GPS UBLOX
For using a UBLOX GPS receiver with ROS Melodic, follow this tutorial: [nmea_navsat_driver](http://wiki.ros.org/nmea_navsat_driver). use melodic branch. 

```bash
dmesg | grep tty
sudo apt-get update
cd ~/catkin_ws/src
git clone -b melodic https://github.com/ros-drivers/nmea_navsat_driver.git
cd ~/catkin_ws/
catkin_make
```

### USB CAM
Just run this command. 

```bash
sudo apt install ros-melodic-usb-cam
ls /dev | greo video
sudo apt install ros-melodic-perception
```
The repository [usb_cam]( http://wiki.ros.org/usb_cam) has been having issues to build. 

### paquete_prueba:
If you want can change the name by using this command. This will create a new ros package and you can move all the files from "paquete_prueba"

```bash
catkin_create_pkg beach_cleaner rospy roscpp usb_cam std_msgs image_transport cv_bridge sensor_msgs geometry_msgs
```
In src, there are files that do image processing. 

## BEACH CLEANER ROBOT:

Don't forget to build your workspace and to update your .bashrc.

```bash
catkin_make
```

to test each sensor separately: 
```bash
roslaunch paquete_prueba imu.launch
roslaunch paquete_prueba roboclaw_mechanism.launch
roslaunch paquete_prueba roboclaw2.launch
roslaunch paquete_prueba usb_cam_test.launch
roslaunch nmea_navsat_driver nmea_serial_driver.launch
```

To control the robot using a twist message publisher on your computer, run:
```bash
roslaunch teleoperation teleop_key.launch
```
Also you can test de teloperation keyword in your jetson, just by changing the header "python3" to "python".

To detect bottles:

```bash
roslaunch paquete_prueba bottle_detection.launch
``` 

To collect data: 

```bash
roslaunch teleoperation data_collection.launch
```

To run the beach cleaner robot with all the included sensors and data collection, use the following command on your  Jetson:
```bash
roslaunch paquete_prueba beachcleaner_bringup.launch
```

## CODES:

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
