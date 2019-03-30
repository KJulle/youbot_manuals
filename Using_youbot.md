# Using KUKA youBot

Instructions on how to move the arm and drive around with the robot.

## 0 Things you will need
* KUKA youBot - this robot has an onboard computer in it, so it will be referred to as such.

* A computer with Ubuntu 16.04 and ROS Kinetic - referred to as a remote computer.

* Access to IMS Lab wiki.

If you need to install Ubuntu 16.04 or ROS Kinetic, the tutorials are here:

- Install Ubuntu 16.04 using [this tutorial](https://tutorials.ubuntu.com/tutorial/tutorial-install-ubuntu-desktop-1604#0).

- Install the full desktop version of ROS Kinetic using [this tutorial](http://wiki.ros.org/kinetic/Installation/Ubuntu).

---

```diff
- Red text example.
```

## 1 Setting up the remote computer

Everything in this chapter will be done on the remote computer and all the commands (lines beginning with $) will have to be entered in the terminal.

The first step is to download all the needed packages.

```bash
$ sudo apt-get install ros-kinetic-youbot-driver ros-kinetic-pr2-msgs ros-kinetic-brics-actuator ros-kinetic-moveit git ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-gazebo-ros-control
```

Next, you will need a catkin workspace. If you do not have it, use the following commands to create it.

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now source the setup file. So you would not have to source it every time you open a new terminal we will write the source command in the .bashrc file.

```bash
$ gedit ~/.bashrc
```

This will open the text editor. You will need to add this line to the end of the file if it not already there:

```bash
source ~/catkin_ws/devel/setup.bash
```

Save the file and exit.

Before continuing you will have to close and reopen all the terminal windows for the sourcing to work.

Now clone the youbot repository to our catkin workspace.

```bash
$ cd ~/catkin_ws/src
$ git clone -b kinetic --recursive https://github.com/ut-ims-robotics/youbot.git
```

Now you should be able to compile the workspace.

```bash
$ cd ~/catkin_ws
$ catkin_make
```

If this completed without any errors, you can go to the next step.

---

## 2 Working on a simulated youBot

To simulate the youBot you will only need the remote computer as set up in chapter 1.

The only thing you will have to do is to run a gazebo simulation which will visualize the simulated robot and environment. This should only be run when you wish to run the simulated robot.

```bash
$ roslaunch youbot_gazebo_robot youbot.launch 
```

To continue working with the simulated robot look under chapter 4 and 5.

---

## 3 Working on the real youBot
### 3.1 Powering the robot

Before turning on the robot you will have to connect the external power supply or battery. If you're not planning to drive the robot around, you should use only the external power supply to power the robot. The external power supply connects to a 24V input on the top of the base of the robot. The battery slides into the slot on the side of the robot and connects to a connector there.

To charge the battery connect both the battery and external power supply to the robot at the same time.

#### `NB! ALWAYS DISCONNECT BATTERY WHEN NOT IN USE!`

### 3.2 Turning on the robot 

Press the "ON/OFF"-button on the top of the robot's base for a few seconds to turn on the power for the system. An orange light on the display will indicate that the system has been powered on successfully. Now you can turn on the motors and/or PC. To do this, press and hold the power button and when the screen shows `PC on`, release the button to turn on the PC. To turn on the motors, repeat the same step, only now release the button when the screen shows `Motor on`.

The arm has to be turned on separately. After the motors have been turned on find the red button on the base of the arm. Press it to turn on the arm. The button will turn green when the arm is on. 

**NB! When you turn off the power of the arm, it will NOT hold its position and will collapse, so make sure to support the arm when turning it off.**

### 3.3 Turning off the robot 

Before turning off the robot you should first shut down the onboard computer. If you are logged in with ssh, you can do this by entering:

```bash
$ sudo shutdown -h now
```

To turn off the system press and hold the "ON/OFF"-button and release when the screen shows `System off`. You should also power off the PC manually before turning off the system.





### 3.3 Establishing an ssh connection

**NB! For the next steps the robot and your remote computer have to be in the same network.**

By default the robot needs to be connected to the youBot's access point named youbot-wifi.

**NB! The password for the access point and the youBot's onboard computer are in IMS Lab wiki.**
To access the 

If you're connected you can establish

To establish an ssh connection, enter this to your remote computer's new terminal window: 

```bash
$ ssh youbot@10.42.0.1
```

Use the password provided in IMS Lab wiki.

Now you should be logged into the onboard computer. That means that when you enter commands in the terminal window you have the ssh connection in, the commands will be run on the robot's onboard computer.//do sth w/ that 

If you need more terminal windows with an ssh connection, just use the same command above to  log in.


### 3.4 Setting up a remote ROS connection

First open a new terminal window (don't close the old one).

Now set up the remote connection. For that you will need to know the youBot's and the remote computer's IP addresses.

The youBot's IP address should be 10.42.0.1.

To find the remote computer's IP address enter this command to the terminal:

```bash
$ ifconfig
```

Now you should see multiple ethernet devices. The device you need should be wlp2s0 or something similar. You will need the inet addr of that device which should be something like 10.42.x.x, where the x's are specific to your computer.

One you know the youBot's IP address and the remote computer's IP address, you will have to specify ROS master aka youBot IP address on the remote computer:

```bash
$ export ROS_MASTER_URI=http://10.42.0.1:11311
```

Next, specify the remote computer's own IP (change the IP to the one you got from your remote computer):

```bash
$ export ROS_IP=10.42.x.x
```

Now check if your IP addresses are actually changed:

```bash
$ echo $ROS_MASTER_URI
$ echo $ROS_IP
```

See if the outputs are the same as you entered previously. If not, enter the commands again.

If you need multiple terminals with a ROS master, you just need to insert the two export commands in a new terminal window.// saab ka bashrcsse b

### 3.5 Running the youBot driver

For this driver to work, the youBot's motors and arm have to be turned on.

To work with the real youBot you will first have to run the youBot driver. This must be run on the youBot's onboard computer, so if you're using a remote computer you will have to run this in the terminal with an ssh connection.

This will also start roscore so you do not have to run that manually.

```bash
$ roslaunch youbot_driver_ros_interface youbot_driver.launch
```

Now your robot has been set up so you can continue

---

## 4 Driving the robot

**NB! If you're using the real youBot and the robot is on the table, make sure it is on a platform so it can't actually move.**

To control the robot, run keyboard_teleop. If you are using the real youBot, run this in the terminal window where you set your ROS master in.

The instructions on which keys to press will be displayed in the terminal window.

```bash
$ rosrun youbot_driver_ros_interface youbot_keyboard_teleop.py
```

Now you should be able to control the wheels with your keyboard. Note that the keyboard teleop only works when the terminal window where you gave the command is active.

---

## 5 Moving the robot arm

To move the arm, you need to launch this demo file. If you're using the real youBot, run this in the terminal window where you set your ROS master in.

```bash
$ roslaunch youbot_moveit demo.launch
```

By default this will open RViz, where you should now be able to see the robot model.

**NB! Make sure the robot model arm position in RViz is the same as in real life before proceeding.**

To move the arm, first you can drag the marker on the top of the robot arm in rviz to the wanted position. In the conext tab you can select a planning library (for example RRTkConfigDefault). 


Next, on the planning tab press *Plan*. 

** NB! Always plan before executing. **

Now you can see the trajectory the arm will make when moving into the wanted position. Make sure that the planned trajectory will not damage the robot.

To execute the planned trajectory, press *Execute*.

**Note:** Every time you plan/execute the trajectory, make sure that "Select Start State" option is "current", in section "Query" under tab "Planning". Otherwise the robot will plan from previous starting state, which may end up with damaging movement.

