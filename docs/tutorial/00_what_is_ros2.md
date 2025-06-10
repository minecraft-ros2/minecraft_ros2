# What is ROS 2?

ROS stands for Robot Operating System, which provides essential functionalities for robot development.
- plumbing (communication)
- tools (toolset)
- capabilities (functional set)
- ecosystem (ecosystem)

## plumbing (communication)
- Loose coupling of ROS nodes (= individual programs)
    - Easy to relocate
- New types can be defined by combining basic types
    - Includes bool, int, double, as well as arrays
    - Standard messages include time information
- DDS is used, and communication partners between programs are automatically resolved

### topic
- Asynchronous
- pub/sub communication (many-to-many)

### service
- Synchronous
- one-to-one

### action
- Service with feedback
- Feedback is asynchronous

### parameter
- Multivariate dictionary

## tools (toolset)
### Colcon
- Build system
- Can build various languages such as C++, C, Python, Java at once

### launch
- Launch multiple programs simultaneously
- Can also specify parameters

### rqt, rviz
- Visualization tools

## capabilities (functional set)
- Over 2000 libraries
- Just search for the library of the desired sensor and use it immediately

### TF
- 3D pose and chain relationships
- Time management

### ros2_control
- Hardware abstraction
- Allows experimenting with various types of control

### Navigation 2
- Map-based localization, path planning, tracking, and obstacle avoidance
- 2D autonomous navigation

### MoveIt
- Manipulation control

## ecosystem (ecosystem)
- Convenient tools and abundant libraries are all open source
- Well-established maintenance and sharing mechanisms
- Interaction through ROSCon, ROS Discourse, etc.
- Free presentations and networking through ROS Japan Users Group (rosjp)
- ROSConJP is the Japanese version of ROSCon
