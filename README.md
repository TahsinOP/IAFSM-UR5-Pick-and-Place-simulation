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
cd /catkin_ws
catkin build
```
### Launching the Moveit Planning scene and Gazebo world 
Use the following commands in a new terminal:
To launch the planning scene 
```bash
roslaunch newxxx_combined_moveit_config demo.launch
```
To launch the Gazebo world 
```bash
roslaunch newxxx_combined_moveit_config gazebo.launch
```

### Running the python scripts 
Make to sure the add the scripts in CmakeLists and then run the following commands in terminal :
To run the simulation in the planning scene : 
``` bash
roslaunch newxxx_combined_moveit_config pick_and_place.py
```
To run the simulation in Gazebo world : 
``` bash
roslaunch newxxx_combined_moveit_config pick_and_place_gazebo.py

https://github.com/TahsinOP/IAFSM-UR5-Pick-and-Place-simulation/assets/117567813/5a5be761-8d11-4b91-b4d0-5a7d42157339


