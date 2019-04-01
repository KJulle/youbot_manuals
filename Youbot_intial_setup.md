# Setting up KUKA Youbot

**NB! This tutorial is for setting up the youBot's onboard computer from scratch. For a tutorial on how to use the robot go [here](https://github.com/ut-ims-robotics/youbot/tree/kinetic) instead.**

## Installing the operating system and ROS

First install Ubuntu 16.04 on the computer. There should be a flash drive with Ubuntu 16.04 in the room. If not, you can download the 64-bit desktop version from [here](http://releases.ubuntu.com/16.04/) and put it on a flash drive.

- Install Ubuntu 16.04 using [this tutorial](https://tutorials.ubuntu.com/tutorial/tutorial-install-ubuntu-desktop-1604#0).

- Install the full desktop version of ROS Kinetic using [this tutorial](http://wiki.ros.org/kinetic/Installation/Ubuntu).

**NB! The passwords and names for the youBot and the access point are in the IMS Lab wiki, so please use the same passwords and names that are given there or modify the current ones!**

## Making a workspace

First make a catkin workspace for your ROS packages and compile it.

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now source the setup file. So you do not have to source it every time you open a new terminal window, write the source command in the .bashrc file.

```bash
$ sudo gedit ~/.bashrc
```

This will open the text editor. Add this line to the end of the file:

```bash
source ~/catkin_ws/devel/setup.bash
```

Save the file and exit.

## Installing ROS packages needed for running the robot

First install needed packages for the robot with apt-get.

```bash
$ sudo apt-get install ros-kinetic-youbot-driver ros-kinetic-pr2-msgs ros-kinetic-brics-actuator ros-kinetic-moveit git
```

Now go to catkin_ws/src and download the packages there with git.

```bash
$ cd ~/catkin_ws/src
$ git clone -b kinetic --recursive https://github.com/ut-ims-robotics/youbot.git
```

Now you should be able to compile the workspace.

```bash
$ cd ~/catkin_ws
$ catkin_make
```

## Setting up the EtherCAT connection

First give the required rights for EtherCAT access.

```bash
$ cd ~/catkin_ws
$ sudo setcap cap_net_raw+ep devel/lib/youbot_driver_ros_interface/youbot_driver_ros_interface
```
 Next set up the right ethernet device as the etherCAT connection. Enter the command to see all connected ethernet devices.
 
 ```bash
$ ifconfig
```
 
 You should see multiple devices. Since the EtherCAT is connected to an external USB network card, the name should be something like enx00e04c2151d2 (note that this might not be exactly the same, you will have to check it). This is your EthernetDevice.
 
 Now open the Youbot EtherCAT configuration file.
 
 ```bash
$ sudo gedit /opt/ros/kinetic/share/youbot_driver/config/youbot-ethercat.cfg
```
There is a line that says:

 ```bash
EthernetDevice = eth0
```

Instead of eth0, insert the device name that you got from ifconfig.

## Setting up the access point

For easier remote access we will set up the youBot as an access point.

To do that you will have to open Network Connections.

Click Add and choose Wi-Fi.

Next, under the Wi-Fi tab choose Mode: Hotspot and Device(there is only one available). Enter the SSID and connection name as youbot-wifi.

Under the General tab, check that the option Automatically connect to this network when it is available is checked.

Under the Wi-Fi Security tab choose Security: WEP 40/128-bit Key enter the password.

Click Save.

Now on the next reboot, the system should automatically make and connect to this network.






 









