# Using KUKA Youbot

Instructions on how to move the arm and drive around with the robot.

## Powering the robot

Before turning on the robot you will have to connect the external power supply or battery. If you're not planning to drive the robot around, you should use only the external power supply to power the robot. The external power supply connects to a 24V input on the top of the base of the robot. The battery slides into the slot on the side of the robot and connects to a connector there.

To charge the battery connect both the battery and external power supply to the robot at the same time.

**NB! Always disconnect the battery when not in use.**

## Turning on the robot

Press the "ON/OFF"-button for a few seconds to turn on the power for the system. An orange light on the display will indicate that the System has been powered on successfully. If you press and hold the ON/OFF-button again, you can turn off and on other systems of the robot. Release the button when the screen shows the PC off/Motors off. You will need to turn on the motors and PC separately.

The arm has to be turned on separately. When the motors are on, there should be a button on the arm with a red light. To turn on the arm, press the button. The light is green when the arm is turned on.

**NB! When you turn off the power of the arm, it will NOT hold its position and will collapse, so make sure to support the arm when turning it off.**

To turn off the system press and hold the same button and release when the screen shows ??? (peab järgi kontrollima mida see display päriselt näitab)

## Driving the robot

To control the robot wheels with your keyboard you have to first run the Youbot driver. This also launches roscore, so you don't have to do that yourself.

```bash
$ roslaunch youbot_driver_ros_interface youbot_driver.launch
```

To control the robot, run keyboard teleop.

```bash
$ rosrun youbot_driver_ros_interface youbot_keyboard_teleop.py
```

Now you should be able to control the wheels with your keyboard. Note that the keyboard teleop only works when the terminal window where you gave the command is active.

## Moving the robot arm

You must have the youbot driver running before doing any of the following commands.

To move the arm, enter:

```bash
$ roslaunch youbot_moveit real_demo.launch
```

By default this will open rviz, where you should now be able to see the robot model.

**NB! Make sure the robot model arm position in rviz is the same as in real life before proceeding.**

To move the arm, first you can drag the marker on the top of the robot arm in rviz to the wanted position. In the conext tab you can select a planning library(for example RRTkConfigDefault). 

Next, on the planning tab press *Plan*. Now you can see the trajectory the arm will make when moving into the wanted position. Make sure that the planned trajectory will not damage the robot.

To execute the planned trajectory, press *Execute*.

**Note:** Every time you plan/execute the trajectory, make sure that  "Select Start State" option is "current", in section "Query" under tab "Planning". Otherwise the robot will plan from previous starting state, which may end up with damaging movement.

