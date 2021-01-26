# Experimental robotics laboratory - Assignment 3

## Introduction
This is the repository for the third and last assignment of the Robotics engineering "Experimental robotics laboratory" course, a.y. 2020/2021, held in the University of Genoa. This work is focused on the implemetation of a ROS-based finite state machine (FSM) nested within a more complex architecture. This project has been coded with [Python 2.7](https://www.python.org/download/releases/2.7/) and it has been tested on [ROS Kinetic](http://wiki.ros.org/kinetic).

## Setting

Like for the previous assignment, [Assignment 2](https://github.com/andreabradpitto/Experimental-robotics-laboratory/tree/main/assignment2), the main subject of the project is a robotic dog. This time, it is free to wander inside a house, comprising 6 different rooms: the entrance, the closet, the living room, the kitchen, the bathroom, and the bedroom. Each of these room locations is schematically represented by a colored ball lying on the ground, so that the dog can recognize them. In order to avoid ambiguities, each ball has a different color; following the same order mentioned above, the colors are: blue, red, green, yellow, magenta and black. Event though the house is not procedurally generated (i.e. it is always the same model), every time the simulation is started, the robot does not know where these room are, so it has to learn the house planimetry by exploration. The only thing it knows, indeed, is the association of each colored ball to its respective room, so that when the robotic dog sees e.g. the yellow ball, it knows that that room is the kitchen.<br/>
Inside the house there is, other than the walls defining each room's border, also a human mannequin model, which represents the dog owner. The robot is in fact listening for human orders, and eager to play with them. The dog, in normal conditions, wanders randomly around the house, looking for undiscovered colored balls; when these are spotted, it reaches them and stores the coordinates of the corresponding room, so that this info can be used later on. When the human asks the robot to play (ideally, by actually telling the robot the word "play"), the dog reaches them, awaits for the human to tell a room name, and then it tries to reach it. Of course, diferent behaviors have been implmented whether the robot already knows the location it has been instructed to reach or not. When the robotic dog reaches the required room, it outputs a vocal acknowledgement, and finally gets back to the owner. When the robot gets tired, it gets back to its charging dock (also referred as "home") and waits for the battery to recharge. This behavior is the real life pet equivalent of sleeping; this parallelism is the one responsible for the state's actual name, as it will be shown later on in this document. The last behavior is the one devoted to the house exploration, for when the robot is looking for a precise still-to-be-discovered ball: this happens we the human wants it to reach a room that is not yet present in its database.<br/>
The code implemented will show a real-time simulation of the assignment over both [Gazebo](http://gazebosim.org/) and [RViz](http://wiki.ros.org/rviz). The human model and the balls will be present only on the former, while the latter is the one that will show the robotic dog's perspective, i.e. all the explored map so far, and the obstacles (the walls or the human model) that it discovered. Gazebo will always show the whole house, so that it is possbile to check if the robot knowledge matches the real planimetry of the environment. In addition, the voice messages shared between the human and the robotic dog will be displayed on terminal.

### The house
Here is a picture of the house:

<div align="center">
  <img src="https://github.com/andreabradpitto/Experimental-robotics-laboratory/blob/main/assignment2/images/playing%20field.png">
</div>

It is comprised inside a 10x10 square centered a the origin of the three axes. The environment can be thus effectively reduced to a bidimensional scenario, spreading over the xy plane (or, equivalently, on the z = 0 one). The robot initial position, which is also assumed to be its home location, is set in the launch file to be near the human mannequin by default: the latter is in *(-6, 8.5)*, while the former is in *(-5, 8)*.

### The robotic dog

The robot I implemented is composed by basic geometric shapes, while trying to find a medium between looks and functionality. Indeed, it features two eyes, in order to resemble a real dog, but only one of them holds a sensor, i.e. a camera, which is used to spot the colored balls: the other one has no purpose other than giving some context. The other sensor present on the robot is a laser scanner, which is mounted on its main chassis and is used to detect the obstacles in front of it. It is positioned much lower with respect to the camera in order to guarantee the best obstacle detection as possible: the tests I made clearly showed that having the two robot "eyes" as the two sensor spots was much harder to implement successfully.<br/>
Here is a picture the robotic dog:

<div align="center">
  <img src="https://github.com/andreabradpitto/Experimental-robotics-laboratory/blob/main/assignment2/images/robotic%20dog.png">
</div>

It is a two-wheeled differential drive robot. Apart from the wheels, all the joints are fixed, so the dog cannot, for intstance, rotate its head to look around.

### The colored balls

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

...qua dire gli algoritmi che uso e spiegare explore_lite inglobato...

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
Finally, here are the parameters (strictly related to the robotic dog) loaded in the ROS parameter server:

- `state`: parameter specifying robot current state
- `map/x_max`: parameter specifying the maximum x-axis value for the map
- `map/y_max`: parameter specifying the maximum y-axis value for the map
- `map/x_min`: parameter specifying the minimum x-axis value for the map
- `map/y_min`: parameter specifying the minimum y-axis value for the map
- `home/x`: parameter specifying robot home position (x coordinate)
- `home/y`: parameter specifying robot home position (y coordinate)
- `sim_scale`: parameter used to scale simulation velocity
- `new_ ball_detected`: parameter used to specify whether a new ball has been detected or not
- `unknown_ball`: parameter used to identify which ball has to be looked for
- `room_list`: list of the available rooms
- `play_task_status`: parameter used to specify **Play** state progress<br/>
                      A value of *0* means that the state is not active or at in initialization phase<br/>
                      A *1* stands for it being in progress<br/>
                      A value of *2* means that it has completed one iteration
                      The last value is also used by the **Find** state, in case the location of the room asked by the human was not present in the robot's database
- `blue/x`: parameter used to specify the x coordinate of the blue ball, once discovered 
- `blue/y`: parameter used to specify the y coordinate of the blue ball, once discovered
- `red/x`: parameter used to specify the x coordinate of the red ball, once discovered
- `red/y`: parameter used to specify the y coordinate of the red ball, once discovered
- `green/x`: parameter used to specify the x coordinate of the green ball, once discovered
- `green/y`: parameter used to specify the y coordinate of the green ball, once discovered
- `yellow/x`: parameter used to specify the x coordinate of the yellow ball, once discovered
- `yellow/y`: parameter used to specify the y coordinate of the yellow ball, once discovered
- `magenta/x`: parameter used to specify the x coordinate of the magenta ball, once discovered
- `magenta/y`: parameter used to specify the y coordinate of the magenta ball, once discovered
- `black/x`: parameter used to specify the x coordinate of the black ball, once discovered
- `black/y`: parameter used to specify the y coordinate of the black ball, once discovered

All of these, as already pointed out, can be adjusted before runtime.<br/>
Finally, here is the list of all the files and folders featured in this package:

<div align="center">
  <img src="https://github.com/andreabradpitto/Experimental-robotics-laboratory/blob/main/assignment2/images/tree.png">
</div>

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

In order to run the code, put the *assignment3* directory in your workspace, build, then open a terminal and type:

```
roslaunch assignment.launch
```

This will start the simulation on both Gazebo and RViz.

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