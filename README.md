# Software for Image Guided Robotics

ROS software controlling a surgical robot (lung biopsy).

7 dof input chart

| | + | - |
|--:|:-:|:-:|
|&#948;x|&#8593;|&#8595;|
|&#948;y|&#8594;|&#8592;|
|&#948;z|z|x|
|&#948;roll|r|f|
|&#948;pitch|t|g|
|&#948;yaw|y|h|
|&#948;needle|i|o|

## Required packages

- [pyrep](https://github.com/stepjam/PyRep)


## How to run

At root directory of the ROS workspace (commonly ```~/catkin_ws/```), run

```
roslaunch software_interface igr.launch
```
