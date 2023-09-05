#!/usr/bin/env python
import os

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
import math


from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import LinkState


from gazebo_msgs.srv import GetModelState


from gazebo_msgs.srv import SpawnModel

from gazebo_msgs.srv import SetLinkState

from tf.transformations import quaternion_from_euler
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes

FRAME_ID = 'base_link'
(X, Y, Z, W) = (0, 1, 2, 3)
OPEN = 0.9

OBJECT_POSITIONS = {'target_1': [0.05, 0.35, 0.3]}
PICK_ORIENTATION_EULER = [-math.pi / 2, 0, 0]
PLACE_ORIENTATION_EULER = [-math.pi / 2, 0, -math.pi / 2]
SCENE = moveit_commander.PlanningSceneInterface()

def degrees_to_radians(degrees):
    return [math.radians(angle) for angle in degrees]

def attach_object(scene, object_name, gripper_links_to_attach):
    scene.attach_box(gripper_links_to_attach[0], object_name, touch_links=gripper_links_to_attach)

def detach_object(scene, object_name):
    scene.remove_attached_object( name=object_name)

def attach_object(model_name, link_name, gripper_links_to_attach):
    set_link_state = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)

    link_state = LinkState()
    link_state.link_name = link_name

    # Set the desired pose for the attached link
    link_state.pose.position.x = 0.0  # Adjust as needed
    link_state.pose.position.y = 0.0  # Adjust as needed
    link_state.pose.position.z = 0.0  # Adjust as needed

    try:
        response = set_link_state(link_state)
        if not response.success:
            rospy.logerr("Failed to set link state for attachment")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def load_sdf_model(model_name, sdf_path, pose):
    spawn_model_proxy = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    with open(sdf_path, 'r') as sdf_file:
        sdf_xml = sdf_file.read()

    model_pose = geometry_msgs.msg.Pose()
    model_pose.position.x = pose[0]
    model_pose.position.y = pose[1]
    model_pose.position.z = pose[2]
    model_pose.orientation.w = 1.0

    spawn_model_proxy(model_name, sdf_xml, '', model_pose, 'world')


# Example usage:

def add_collision_objects():
    sdf_path_floor_limit = 'floor_limit.sdf'
    sdf_path_target_1 = 'target_1.sdf'
    sdf_path_target_1_surface = 'target_1_surface.sdf'
    sdf_path_place_structure = 'place_structure.sdf'

    pose_floor_limit = [0, 0, -0.1]
    pose_target_1 = [-1.2, 0.0, 0.2]
    pose_target_1_surface = [0.6, 0.0, 0.1025]
    pose_place_structure = [0.0, 0.55, 0.0327]


    load_sdf_model('target_1', sdf_path_target_1, pose_target_1)
    load_sdf_model('target_1_surface', sdf_path_target_1_surface, pose_target_1_surface)
    load_sdf_model('place_structure', sdf_path_place_structure, pose_place_structure)



def main():
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('pick_and_place_demo', anonymous=True)

        robot_arm_group = moveit_commander.MoveGroupCommander("arm_group")
        robot_gripper_group = moveit_commander.MoveGroupCommander("gripper_group")

        open_gripper_pose = robot_gripper_group.get_named_target_values("semi_open_gripper")
        closed_gripper_pose = robot_gripper_group.get_named_target_values("semi_closed_gripper")

        # ... (previous code)

        # Adjust this value for proper gripping force

# ... (rest of your code)

        pick_joint_angles = [90, 35, 0, -49, 0]
        pick_pose = (degrees_to_radians(pick_joint_angles))
        
        CLOSE = [16,-16] 
        close_pose = (degrees_to_radians(CLOSE))

        place_joint_angles = [180, 40, 0, -57, 0]
        place_pose = (degrees_to_radians(place_joint_angles))

        home_pose = [0,0,0,0,0]

        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        wooden_box_model = 'wood_cube_10cm'
        wooden_box_link = 'link'
        wooden_box_state = get_model_state(wooden_box_model, '')
        wooden_box_pose = wooden_box_state.pose
        

        # ... (rest of your code remains unchanged)

        # Modify collision object creation
        #add_collision_objects()

        target_name = 'target_1'
        gripper_links_to_attach = ["gripper_finger1_finger_tip_link", "gripper_finger2_finger_tip_link"]
        gripper_links_to_detach = ["gripper_finger1_finger_tip_link", "gripper_finger2_finger_tip_joint"]

      
        robot_arm_group.set_joint_value_target(pick_pose)
        robot_arm_group.go(wait=True)
        robot_arm_group.stop()

        robot_gripper_group.set_joint_value_target(closed_gripper_pose)
        robot_gripper_group.go(wait=True)
        robot_gripper_group.stop()




        #attach_object(wooden_box_model, wooden_box_link, gripper_links_to_attach)

        robot_arm_group.set_joint_value_target(place_pose)
        robot_arm_group.go(wait=True)
        robot_arm_group.stop()

        robot_gripper_group.set_joint_value_target(open_gripper_pose)
        robot_gripper_group.go(wait=True)
        robot_gripper_group.stop()



      
        #detach_object(SCENE, target_name)

        #SCENE.attach_box("place_structure", target_name, touch_links=["place_structure"])


        robot_arm_group.set_joint_value_target(home_pose)
        robot_arm_group.go(wait=True)
        robot_arm_group.stop()

        # ... (rest of your code remains unchanged)

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
