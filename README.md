irobotcreate2ros
===
Ros node for iRobot Create 2.

Prerequisites
---
* [ros serial](http://wiki.ros.org/serial)
* [ros joy](http://wiki.ros.org/joy)

Compiling
---
Just clone the repository in the src folder of a catkin workspace. Then run catkin_make.

To generate the model you have to launch in the model folder the dedicated script:
```
source generate_model.sh
```

To simulate the robot in Gazebo you first have to add this lines to your bashrc:

```
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(rospack find irobotcreate2)
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:$(rospack find irobotcreate2)
```

Usage
---
---
```
rosrun irobotcreate2 irobotcreate2
```
to run the basic software and have access to the following topics:

- /battery
- /bumper
- /buttons
- /cliff
- /cmd_vel
- /digit_leds
- /ir_bumper
- /ir_character
- /leds
- /mode
- /odom
- /play_song
- /rosout
- /rosout_agg
- /song
- /tf
- /wheel_drop

you can read sensors (/battery, /buttons, /bumper, ...) and send commands (/cmd_vel, ...).

---
```
roslaunch irobotcreate2 irobot_joy.launch
```
to run both the basic software and ros joy to move the robot with a controller.

<!--[Instructions](cad/laser_support/instructions.md)-->
