# Experimental robotics laboratory - Assignment 2

## Introduction
This is the repository for the second assignment of the Robotics engineering "Experimental robotics laboratory" course, a.y. 2020/2021, held in the University of Genoa. This work is focused on the implemetation of a ROS-based finite state machine (FSM) nested within a more complex architecture. This project has been coded with [Python 2.7](https://www.python.org/download/releases/2.7/) and it has been tested on [ROS Kinetic](http://wiki.ros.org/kinetic).

## Setting
The theme is based on a robotic dog, which wanders around and plays with the a ball moved or hidden by the user. Through [Gazebo](http://gazebosim.org/), the idea is to build upon the finite state machine created for the [Assignment 1](https://github.com/andreabradpitto/Experimental-robotics-laboratory/tree/main/assignment1) and implement a graphical version of the moving robot, while also modifying its behaviors, to better match its new "dog" representation. The final result should indeed loosely resemble a real pet.<br/>
The world used, which is delimited by fences, is a 10x10 square and can be referred to as the dog playing field. Inside the square, other than the robotic dog, there are also an human mannequin and its chair, which have been added just to give some flavour to the environment. The dog has its home (see Sleep state) near the human model by default, but that can be changed in the parameters. Here is an image of the playing field:

<div align="center">
  <img src="https://github.com/andreabradpitto/Experimental-robotics-laboratory/blob/main/assignment2/images/playing%20field.png">
</div>

The robotic dog can have 3 states. The first one is **Sleep**, which triggers after a certain amount of time while being in **Normal** state: the dog will simply get back home and sleep for a bit. **Normal** is the state in which the robot wanders over the playing field. Finally, the **Play** state triggers from **Normal** when the dog sees a ball: it then start following it. The robo-dog will adjust its distance with respect to the sphere in order not to collide. If the dog sees the ball stading still (i.e. the throw effect by the human is over), it starts moving the head from left to right, and then starts following the ball again. As the human can hide the ball from time to time, If the robot loses track of the ball for long enough, it will get back to the normal state.
This is the model of the robotics dog:

<div align="center">
  <img src="https://github.com/andreabradpitto/Experimental-robotics-laboratory/blob/main/assignment2/images/robotic%20dog.png">
</div>

It loves big green balls

## Architecture

<div align="center">
  <img src="https://github.com/andreabradpitto/Experimental-robotics-laboratory/blob/main/assignment2/images/architecture%20and%20topics.png">
</div>

The architecture is made of five components:
- **Human**: it is implemented by [human.py](scripts/human.py), which is used to simulate the dog owner. This component randomly whooses whether the human decides to move the ball along (throw) the playing field, or to hide it
- **Dog FSM**: it is implemented by [dog_fsm.py](scripts/dog_fsm.py), which is used to handle the robotic dog's FSM internal architecture
- **Dog control**: it is implemented by [dog_control.py](scripts/dog_control.py), which is used to have the robotic dog reaching a random position during the Normal state, or its home during the Sleep state
- **Dog control ball**: it is implemented by [dog_control_ball.py](scripts/dog_control_ball.py), which is used to make the dog follow the ball, or to look for it if it has lost track of the green sphere. It also makes the robotic dog turn its head when he perceives that the ball has stopped
- **Go to point ball**: it is implemented by [go_to_point_ball.py](scripts/go_to_point_ball.py), and is used to move the ball along the playing field. Goal positions are issued randomly by [human.py](scripts/human.py)


<div align="center">
  <img src="https://github.com/andreabradpitto/Experimental-robotics-laboratory/blob/main/assignment2/images/fsm.png">
</div>

The dog starts in the **Sleep** state.<br/>
Topics involved:

- `control_topic`: topic used by the FSM to order the **Dog_control** component to start simulating a movement
- `motion_over_topic`: topic whose duty is to inform the FSM when the motion is over or interrupted by the sight of the ball
- `ball_control_topic`: topic used by **Dog control ball** to communicate with **Dog FSM**: it sends information when the ball is first spotted by the robot, and then when the dog eventually loses track of it

The actions used are:

- `assignment2.msg/PlanningAction`: a simple action whose goals are of type **geometry_msgs/PoseStamped**

The message types used are:

- `std_msgs.msg/Int64`: imported message type consisting in an integer
- `assignment2.msg/Coordinates`: message made of two integers **x** and **y**

The latter of the two is of course a custom one, which has been coded for this project and is shipped with this package itself. Standard messages could have been used, but this new type creation had also been created as an extra exercise, in order to get more acquainted with the ROS environment.<br/>
Finally, here are all the parameters loaded in the ROS parameter server:

- `state`: parameter specifying robot current state
- `dog/x`: parameter specifying robot current position (x coordinate)
- `dog/y`: parameter specifying robot current position (y coordinate)
- `map/x_max`: parameter specifying the maximum x-axis value for the map
- `map/y_max`: parameter specifying the maximum y-axis value for the map
- `map/x_min`: parameter specifying the minimum x-axis value for the map
- `map/y_min`: parameter specifying the minimum y-axis value for the map
- `home/x`: parameter specifying robot home position (x coordinate)
- `home/y`: parameter specifying robot home position (y coordinate)
- `sim_scale`: parameter used to scale simulation velocity
- `ball_detected`: parameter used to specify whether the ball is detected or not

All of these, as already pointed out, can be adjusted before runtime.

## Requirements
In order to run this piece of software, it is needed to have a Linux distribution that supports (and has them installed):
- [ROS Kinetic](http://wiki.ros.org/kinetic)
- [SMACH](http://wiki.ros.org/smach)
- [OpenCV](https://opencv.org/)
- A [Python](https://www.python.org/) interpreter

## Instructions
It might be needed to make all the nodes executable, before actually being able to run them. So, after reaching this assignment's folder via a terminal, type:

```
chmod +x src/*
```

In order to run the code, put the *assignment2* directory in your workspace, build, then open a terminal and type:

```
roslaunch assignment.launch
```

## Limitations and possible improvements
This project has been a lot time-consuming for me. This is the source of part of the limitations of my particular solution of this assignment. Here are some of them:

- First of all, due to time constraints, I had no time to properly test the code, and I just stopped developing as soon as it was working. Thus, I did not stress my code with randomness by any means, and I did not deal with some singularities, like the ones generated from random goal positions crossing the mannequin. I guest I could have set its collision size to zero, but as that was part of the given code, I think that would have meant cheating a bit.
- Another simplification I made was thanks to forcing maximum and minimum playing field sizes to 8 and -8, so that the robot would almost always avoid the chair and mannequin models, as well as the fences. I could have implemented a force field technique for those world limit, or maybe I could have used OpenCV with another hue (grey, in this case) in order to have the robot steer away from those fences.
- The robotic dog model is really schematic: a lot of improvements could have been made in order to make it resemble a dog
- The code comments are not detailed enough: some "while" loops and "if" conditions would benefit, in terms of clarity, from a richer explanation
- I am definitely still lacking an effective planning strategy, which led me to waste a lot of time composing and deleting several lines of code, but also to a final architecture which is sub-optimal to say the least. For this assignment, having as much as 5 nodes are too much; I could have, for example, adapted [go_to_point_ball.py](scripts/go_to_point_ball.py) in order to also handle the robot movements via actions. I could have, in any case, merged the two control schemes in one script only, as this would have simplified a lot the communications with the FSM.
- The robot is completely unaware of the ball movement while he is moving its head: this was not specified by the assignment, but I think that implementing a way to make it able to adapt to ball movements also during that tiem frame would be nice

## Authors
All the files in this repository belong to [Andrea Pitto](https://github.com/andreabradpitto).<br/>
Contact: [s3942710@studenti.unige.it](mailto:s3942710@studenti.unige.it).
