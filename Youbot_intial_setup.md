# Setting up KUKA Youbot

This tutorial contains instructions on how to set up the KUKA Youbot onboard computer.

## Installing the operating system and ROS

First you will have to install Ubuntu 16.04 on the computer. There should be a flash drive with Ubuntu 16.04 in the room. If not, you can download it from [here](https://www.ubuntu.com/download/alternative-downloads) and put it on a flash drive.

- Install Ubuntu 16.04 using [this tutorial](https://tutorials.ubuntu.com/tutorial/tutorial-install-ubuntu-desktop-1604#0).

- Install the full desktop version of ROS Kinetic using [this tutorial](http://wiki.ros.org/kinetic/Installation/Ubuntu).

## Making a workspace

First you will have to make a catkin workspace for your ROS packages and compile it.

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now you will have to source the setup file. So you don't have to source it every time you open a new terminal we will write the source command in the .bashrc file.

```bash
$ sudo gedit .bashrc
```

This will open the text editor. You will need to add this line to the end of the file:

```bash
source ~/catkin_ws/devel/setup.bash
```

Save the file and exit.

## Installing ROS packages needed for running the robot

First we will install needed packages for the robot with apt-get.

```bash
$ sudo apt-get install ros-kinetic-youbot-driver ros-kinetic-pr2-msgs ros-kinetic-brics-actuator ros-kinetic-moveit git
```

Now go to catkin_ws/src and download the packages there with git.

```bash
$ cd ~/catkin_ws/src
$ git clone http://github.com/youbot/youbot_description.git -b kinetic-devel
$ git clone https://github.com/youbot/youbot_driver_ros_interface.git -b indigo-devel
$ git clone https://github.com/svenschneider/youbot-manipulation.git -b indigo
```

Now you should be able to compile the workspace.

```bash
$ cd ~/catkin_ws
$ catkin_make
```

## Setting up the EtherCAT connection

First you will have to set the required rights for EtherCAT access.

```bash
$ cd ~/catkin_ws
$ sudo setcap cap_net_raw+ep devel/lib/youbot_driver_ros_interface/youbot_driver_ros_interface
```
 Next you will have to set up the right ethernet device as the etherCAT connection. Enter the command to see all connected ethernet devices.
 
 ```bash
$ ifconfig
```
 
 You should see multiple devices. Since the EtherCAT is connected to an external USB network card, the name should be something like INSERT DEVICE NAME. This is your EthernetDevice.
 
 Now open the Youbot EtherCAT configuration file.
 
 ```bash
$ sudo gedit /opt/ros/kinetic/share/youbot_driver/config/youbot-ethercat.cfg
```
There is a line that says:

 ```bash
EthernetDevice = eth0
```

Instead of eth0, insert the device name that you got from ifconfig.




 









