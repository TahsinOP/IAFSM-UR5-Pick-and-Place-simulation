#!/usr/bin/env python

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

from tf.transformations import quaternion_from_euler
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes


FRAME_ID = 'base_link'
(X, Y, Z, W) = (0, 1, 2, 3)
OPEN = 0.9

OBJECT_POSITIONS = {'target_1': [0.05, 0.35, 0.3]}
PICK_ORIENTATION_EULER = [-math.pi / 2, 0, 0]
PLACE_ORIENTATION_EULER = [-math.pi / 2, 0, -math.pi / 2]
SCENE = moveit_commander.PlanningSceneInterface()


def create_collision_object(id, dimensions, pose):
    object = CollisionObject()
    object.id = id
    object.header.frame_id = FRAME_ID

    solid = SolidPrimitive()
    solid.type = solid.BOX
    solid.dimensions = dimensions
    object.primitives = [solid]

    object_pose = Pose()
    object_pose.position.x = pose[X]
    object_pose.position.y = pose[Y]
    object_pose.position.z = pose[Z]

    object.primitive_poses = [object_pose]
    object.operation = object.ADD
    return object

def add_collision_objects():
    floor_limit = create_collision_object(id='floor_limit',
                                          dimensions=[10, 10, 0.2],
                                          pose=[0, 0, -0.1])
    #table_1 = create_collision_object(id='table_1',
                                      #dimensions=[0.1, 0.1, 0.1],
                                      #pose=[0.6, -0.01, 0.09])
    #table_2 = create_collision_object(id='table_2',
                                      #dimensions=[0.1, 0.1, 0.1],
                                      #pose=[0.3, 0.45, 0.1])
    target_1 = create_collision_object(id='target_1',
                                       dimensions=[0.05, 0.05, 0.05],
                                       pose=[0.6 , 0.0, 0.2])
    target_1_surface = create_collision_object(id='target_1_surface',
                                               dimensions=[0.5, 0.3, 0.15],
                                               pose=[0.6 , 0.0, 0.1025])  # Below the target_1 box
    
    place_structure = create_collision_object(id='place_structure',
                                              dimensions=[0.25, 0.2, 0.1],
                                              pose=[0.0, 0.55, 0.0327])  # Below the place_pose

    SCENE.add_object(floor_limit)
    #SCENE.add_object(table_1)
    #SCENE.add_object(table_2)
    SCENE.add_object(target_1)
    SCENE.add_object(target_1_surface)
    SCENE.add_object(place_structure)

def degrees_to_radians(degrees):
    return [math.radians(angle) for angle in degrees]

# Function to create a pose from joint angles
def create_pose(joint_angles):
    pose = geometry_msgs.msg.Pose()
    pose.position.x = 0.6  # Adjust the x-coordinate of the pose
    pose.position.y = 0.0  # Adjust the y-coordinate of the pose
    pose.position.z = 0.2  # Adjust the z-coordinate of the pose
    # For orientation, you may need to adjust the quaternion or use Euler angles based on your setup
    # For simplicity, I'm setting a default orientation (facing downwards)
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 1.0
    return pose

    

def attach_object(scene, object_name, gripper_links_to_attach):
    scene.attach_box(gripper_links_to_attach[0], object_name, touch_links=gripper_links_to_attach)

def detach_object(scene, object_name):
    scene.remove_attached_object( name=object_name)


def main():
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('pick_and_place_demo', anonymous=True)

        robot_arm_group = moveit_commander.MoveGroupCommander("arm_group")
        robot_gripper_group = moveit_commander.MoveGroupCommander("gripper_group")

        open_gripper_pose = robot_gripper_group.get_named_target_values("semi_open_gripper")
        closed_gripper_pose = robot_gripper_group.get_named_target_values("semi_closed_gripper")

        pick_joint_angles = [90, 30, 0, -47, 0]
        pick_pose = (degrees_to_radians(pick_joint_angles))
        
        CLOSE = [16,-16] 
        close_pose = (degrees_to_radians(CLOSE))

        place_joint_angles = [180, 40, 0, -57, 0]
        place_pose = (degrees_to_radians(place_joint_angles))

        home_pose = [0,0,0,0,0]
        

        add_collision_objects()

        target_name = 'target_1'
        gripper_links_to_attach = ["gripper_finger1_finger_tip_link", "gripper_finger2_finger_tip_link"]
        gripper_links_to_detach = ["gripper_finger1_finger_tip_joint", "gripper_finger2_finger_tip_joint"]

      
        robot_arm_group.set_joint_value_target(pick_pose)
        robot_arm_group.go(wait=True)
        robot_arm_group.stop()

        robot_gripper_group.set_joint_value_target(closed_gripper_pose)
        robot_gripper_group.go(wait=True)
        robot_gripper_group.stop()




        attach_object(SCENE, target_name, gripper_links_to_attach)

        robot_arm_group.set_joint_value_target(place_pose)
        robot_arm_group.go(wait=True)
        robot_arm_group.stop()

        robot_gripper_group.set_joint_value_target(open_gripper_pose)
        robot_gripper_group.go(wait=True)
        robot_gripper_group.stop()



      
        detach_object(SCENE, target_name)

        #SCENE.attach_box("place_structure", target_name, touch_links=["place_structure"])


        robot_arm_group.set_joint_value_target(home_pose)
        robot_arm_group.go(wait=True)
        robot_arm_group.stop()


        moveit_commander.roscpp_shutdown()

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':

    main()