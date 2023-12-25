# IAFSM-UR5-Pick-and-Place-simulation
## Prerequisites :
1. ROS1 noetic
2. MoveIt
3. Rviz
4. Gazebo

## Setup
1. Extract all the packages into your src folder and build them
2. Configure Moveit in your ROS workspace
   
### Building the Workspace
Use the following commands in your terminal:

```bash
cd /colcon_ws
colcon build
```
### Launching the Moveit Planning scene 
Use the following commands in a new terminal:

```bash
roslaunch newxxx_combined_moveit_config demo.launch
```


