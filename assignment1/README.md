# Experimental robotics laboratory - Assignment 1

## Introduction
This is the repository for the first assignment of the Robotics engineering "Experimental robotics laboratory" course, a.y. 2020/2021, held in the University of Genoa. This work is focused on the implemetation of a ROS-based finite state machine (FSM) nested within a more complex architecture. This project has been coded with Python 2.7 and it has been tested on ROS-kinetic.

## Setting
The theme is based on a robot (MiRo), which wanders around and plays with the user. Given a configurable 2D playing ground, MiRo moves randomly around, until it decides to go to sleep in his house (a configurable predetermined location). Then he wakes up and starts its normal behaviour again (i.e. wandering). An user can interact with the robot, asking him to <<**play**>> a simple game: if MiRo is awake, it will reach the user location (yet another configurable parameter) and then wait for him to send him to a random point via a pointing geture. After some time MiRo gets bored, and goes back to its normal state. 

## Architecture
The architecture is made of three components: **Person**, **Command manager** and **Control**:

<div align="center">
  <img src="https://github.com/andreabradpitto/Experimental-robotics-laboratory/blob/main/assignment1/images/architecture.png">
</div>

The first one simulates a person interacting with MiRo; the user can say <<**play**>> in order to ask the robot to play a simple game, and can use pointing gestures to indicate MiRo a location to reach. Then the Command manager is the component which is devoted to the implementation of the finite state machine representing the robot behaviours. Finally, the Control component has the duty of simulating physical delays that, in real life, a robot would require in order to move from one position to another one. All the comunications between components are carried out via publish-subscribe techniques.
This is the intenal structure of the finite state machine:
  
<div align="center">
  <img src="https://github.com/andreabradpitto/Experimental-robotics-laboratory/blob/main/assignment1/images/fsm%20states.png">
</div>

MiRo starts in the **Sleep** state.<br/>
Topics involved:

```
control_topic
gesture_topic
motion_over_topic
play_topic
```

The message types used are:

```
std_msgs/String
assignment1/Coordinates
```

The latter of the two is of course a custom one, which has been coded for this project and is shipped with this package itself. Standard messages could have been used, but this new type creation had also been created as an extra exercise, in order to get more acquainted with the ROS environment. <\br>
Finally, here are all the parameters loaded in the ROS parameter server:

```
state       // parameter specifying robot current state
MiRo/x      // parameter specifying robot current position (x coordinate)
MiRo/y      // parameter specifying robot current position (y coordinate)
map/x_max   // parameter specifying the maximum x-axis value for the map
map/y_max   // parameter specifying the maximum y-axis value for the map
home/x      // parameter specifying robot home position (x coordinate)
home/y      // parameter specifying robot home position (y coordinate)
person/x    // parameter specifying user's position (x coordinate)
person/y    // parameter specifying user's position (y coordinate)
sim_scale   // parameter used to scale simulation velocity
```
All of these, as already pointed out, can be adjusted before runtime.

## Assumptions
In order to simplify the problem, it is assumed that the user is aware of the robot current state, and that they will only ask MiRo to play when necessary conditions are met (i.e. when it is in **Normal** state). Of course, all the wait times are random and do not resemble reality in a correct way: there has not been made any consideration about the fact that, for instance, a longer movement, distance-wise, takes a greater amount of time in the real world, wit respect to a shorter trip. So another assumption is that travel distances are not directly linked with the time the robot takes to perform them. It is also assumed that the user does not try to "break" the robot, as in fact its simulation is only allowed to say the <<**play**>> keyword and the to point a location in the playing ground. Finally, the user is assumed to use gestures only when the robot is in **Play** behavior, and to point only after the voice command.

## Limitations
Most, if not all, of the assumptions descripted above are indeed limits of the implementation. In addition to those, there is little to no control over input parameters (e.g. one can set a starting robot position outside of the playing field), DA FINIRE

## Instructions

In order to run the code, simply open a terminal, move to the *assignment1* directory (which should be put in a catkin workspace), and type:

```
roslaunch assignment.launch
```

This will launch all the three nodes with the parameters defined in the launche file itself, with the addition of the SMACH viewer tool, which is useful to visually keep track of the FSM state transitions. The parameters inside the launch file can be adjusted as desired.

## Extra

<ins>[This](http://consequentialrobotics.com/miro-beta#:~:text=MiRo%20is%20a%20fully%20programmable,suited%20for%20developing%20companion%20robots.)</ins> is the that has been shown us during the assignment presentation, and that we would probably have used if we could have worked in the University. Sadly, due time is still inside a COVID-19 outbreak.

## Cose da fare
1. Brief introduction (couple of sentences)
2. Software architecture and states diagrams
(a paragraph of description each, plus a list describing ROS
messages and parameters)
3. Packages and file list (to navigate the repository based on 2)
4. Installation and running procedure
(including all the steps to display the robot’s behavior)
5. Working hypothesis and environment (1 or 2 paragraph)
6. System’s features (1 or 2 paragraph)
7. System’s limitations (1 or 2 paragraph)
8. Possible technical Improvements (1 or 2 paragraph)
9. Authors and contacts (at least the email)

## Authors
All the files in this repository belong to [Andrea Pitto](https://github.com/andreabradpitto).
