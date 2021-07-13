#!/usr/bin/env python

# References
# ----------
# http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html


import sys
import copy
import rospy
import moveit_commander
from movo_action_clients.gripper_action_client import GripperActionClient

from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface

import geometry_msgs.msg

_upper_body_joints = ["right_shoulder_pan_joint",
                      "right_shoulder_lift_joint",
                      "right_arm_half_joint",
                      "right_elbow_joint",
                      "right_wrist_spherical_1_joint",
                      "right_wrist_spherical_2_joint",
                      "right_wrist_3_joint",
                      "left_shoulder_pan_joint",
                      "left_shoulder_lift_joint",
                      "left_arm_half_joint",
                      "left_elbow_joint",
                      "left_wrist_spherical_1_joint",
                      "left_wrist_spherical_2_joint",
                      "left_wrist_3_joint",
                      "linear_joint",
                      "pan_joint",
                      "tilt_joint"]
                      
# Head looking straight
#
pose1 = [-1.1, -1.5, 0.1, -2.1, 0.0, 0.6, -1.69,
                       1.1, 1.5, -0.1, 2.1, 0.0, -0.6, 1.69,
                       0.14, 0, 0]
#pose 2: move to the top
pose2 = [0.6, -1.5, 0.1, -2.1, 1.57, 0, -1.69,
                       -0.6, 1.5, -0.1, 2.1, -1.57, 0, 1.69,
                       0.14, 0, 0]
#pose 3: differentiate two arms and move across
# pose3 = [0.6, -1.1, 0.1, -2.7, 1.57, 0, -1.69,
#                        -0.6, 0.9, -0.1, 2.3, -1.57, 0, 1.69,
#                        0.14, 0, 0]
pose3_1 = [0.6, -1.5, 0.1, -1.8, 1.57, 0, -1.69,
                       -0.6, 1.5, -0.1, 2.2, -1.57, 0, 1.69,
                       0.14, 0, 0]

pose3_2 = [0.6, -0.4, 0.1, -1.8, 1.57, 0, -1.69,
                       -0.6, 0.65, -0.1, 2.2, -1.57, 0, 1.69,
                       0.14, 0, 0]

pose4 = [0.6, -0.4, 0.1, -1.8, 1.57, 0.85, -1.69,
                       -0.6, 0.65, -0.1, 2.2, -1.57, -0.85, 1.69,
                       0.14, 0, 0]

# pose5 = [-1.5, 0.65, -0.8, -2.2, 1.4, 1.8, -1.69,
#                        1.5, 0.65, 0.8, 2.2, -1.4, -1.8, 1.69,
#                        0.14, 0, 0]
pose5_1 = [-1.5, -0.4, 0.1, -1.8, 1.57, 0.85, -1.69,
                       1.5, 1, -0.1, 2.4, -1.57, -0.85, 1.69,
                       0.14, 0, 0]
pose5_2 = [-1.5, -0.65, -0.8, -2.2, 1.4, 1.9, -1.69,
                       1.5, 0.65, 0.8, 2.2, -1.4, -1.9, 1.69,
                       0.14, 0, 0]
pose6_1 = [-0.75, -0.8, -0.8, -2.2, 1.7, 1.1, -1.69,
                       0.75, 0.8, 0.8, 2.2, -1.7, -1.1, 1.69,
                       0.14, 0, 0]
pose6_2 = [-0.2, -1.2, -0.4, -2.2, 1.7,0 , -1.69,
                       0.2, 1.2, 0.4, 2.2, -1.7, 0, 1.69,
                       0.14, 0, 0]
pose6_3 = [0.4, -1.3, -0.2, -2.1, 1.7, -0.8 , -1.69,
                       -0.4, 1.3, 0.2, 2.1, -1.7, 0.8, 1.69,
                       0.14, 0, 0]
pose7_1 = [0.4, -1.3, 0, -1.8, 1.7, -0.8 , -1.69,
                       -0.4, 1.3, 0, 2.3, -1.7, 0.8, 1.69,
                       0.14, 0, 0]
pose7_2 = [0.4, -0.6, 0.2, -1.8, 1.7, -1 , -1.69,
                       -0.4, 0.9, -0.2, 2.3, -1.8, 1, 1.69,
                       0.14, 0, 0]
pose8 = [0.4, -0.6, 0.2, -1.8, 1.7, 1 , -1.69,
                       -0.4, 0.9, -0.2, 2.3, -1.8, -1, 1.69,
                       0.14, 0, 0]
pose9_1 = [0, -0.6, 0.2, -1.8, 1.7, 1.4 , -1.69,
                       0, 0.9, -0.2, 2.3, -1.8, -1.4, 1.69,
                       0.14, 0, 0]
pose9_2 = [0.2, -0.1, 0.2, -1.5, 1.7, 1.7 , -1.69,
                       0.2, 0.1, -0.5, 2, -1.8, -1.7, 1.69,
                       0.14, 0, 0]                       
# pose5_1 = [-1.5, -0.4, 0.1, -1.8, 1.57, 0.85, -1.69,
#                        1.5, 0.8, -0.1, 2.3, -1.57, -0.85, 1.69,
#                        0.14, 0, 0]
# default_pose_tucked = [-1.595, -1.5, 0.40, -2.612, 0.0, 0.496, -1.69,
#                        1.595, 1.5, -0.4, 2.612, 0.0, -0.496, 1.69,
#                        0.14, 0, 0]
# default_pose_tucked = [-1.595, -1.5, 0.40, -2.612, 0.0, 0.496, -1.69,
#                                     1.595, 1.5, -0.4, 2.612, 0.0, -0.496, 1.69,
#                                     0.14, 0, -0.6]
if __name__=="__main__":
    rospy.init_node('movo_moveit_test',
                    anonymous=False)

    moveit_commander.roscpp_initialize(sys.argv)

    scene = moveit_commander.PlanningSceneInterface()

    lgripper = GripperActionClient('left')
    rgripper = GripperActionClient('right')
    gripper_closed = 0.00
    gripper_open = 0.165
    
    larm_group = moveit_commander.MoveGroupCommander("left_arm")
    rarm_group = moveit_commander.MoveGroupCommander("right_arm")
    upper_body = moveit_commander.MoveGroupCommander("upper_body")

    move_group = MoveGroupInterface("upper_body", "base_link")
    lmove_group = MoveGroupInterface("left_arm", "base_link")
    rmove_group = MoveGroupInterface("right_arm", "base_link")

    print("Done spinning up MoveIt!")
    # upper_body.go(joints=[])
    # larm_group.
    

    while not rospy.is_shutdown():
        print("doing L0_goto_upper_body_joints")
        result = move_group.moveToJointPosition(_upper_body_joints, pose1, 0.005, wait=True)

        print("error code: ", result.error_code.val)
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            break
    print("successful, Moving to Pose 2")
    # raw_input()
    while not rospy.is_shutdown():
        print("doing L0_goto_upper_body_joints")
        result = move_group.moveToJointPosition(_upper_body_joints, pose2, 0.005, wait=True)

        print("error code: ", result.error_code.val)
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            break
    print("successful, Moving to Pose 3")
    # raw_input()

    while not rospy.is_shutdown():
        print("doing L0_goto_upper_body_joints")
        result = move_group.moveToJointPosition(_upper_body_joints, pose3_1, 0.005, wait=True)

        print("error code: ", result.error_code.val)
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            break

    while not rospy.is_shutdown():
        print("doing L0_goto_upper_body_joints")
        result = move_group.moveToJointPosition(_upper_body_joints, pose3_2, 0.005, wait=True)

        print("error code: ", result.error_code.val)
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            break  
    print("successful, Moving to Pose 4")

    # raw_input()
    while not rospy.is_shutdown():
        print("doing L0_goto_upper_body_joints")
        result = move_group.moveToJointPosition(_upper_body_joints, pose4, 0.005, wait=True)

        print("error code: ", result.error_code.val)
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            break  
    print("successful, Moving to Pose 5")
    # raw_input()
    while not rospy.is_shutdown():
        print("doing L0_goto_upper_body_joints")
        result = move_group.moveToJointPosition(_upper_body_joints, pose5_1, 0.005, wait=True)

        print("error code: ", result.error_code.val)
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            break
    print("successful, Moving to Pose 5_2")    

    while not rospy.is_shutdown():
        print("doing L0_goto_upper_body_joints")
        result = move_group.moveToJointPosition(_upper_body_joints, pose5_2, 0.005, wait=True)

        print("error code: ", result.error_code.val)
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            break  

    print("successful, Moving to Pose 6_1")

    # raw_input()
    while not rospy.is_shutdown():
        print("doing L0_goto_upper_body_joints")
        result = move_group.moveToJointPosition(_upper_body_joints, pose6_1, 0.005, wait=True)

        print("error code: ", result.error_code.val)
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            break  
    print("successful, Moving to Pose 6_2")  

    # raw_input()
    while not rospy.is_shutdown():
        print("doing L0_goto_upper_body_joints")
        result = move_group.moveToJointPosition(_upper_body_joints, pose6_2, 0.005, wait=True)

        print("error code: ", result.error_code.val)
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            break  
    print("successful, Moving to Pose 6_3")  

    # raw_input()
    while not rospy.is_shutdown():
        print("doing L0_goto_upper_body_joints")
        result = move_group.moveToJointPosition(_upper_body_joints, pose6_3, 0.005, wait=True)

        print("error code: ", result.error_code.val)
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            break  
    print("successful, Moving to Pose 7_1")
    # raw_input()
    while not rospy.is_shutdown():
        print("doing L0_goto_upper_body_joints")
        result = move_group.moveToJointPosition(_upper_body_joints, pose7_1, 0.005, wait=True)

        print("error code: ", result.error_code.val)
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            break  
    print("successful, Moving to Pose 7_2")    
    # raw_input()
    while not rospy.is_shutdown():
        print("doing L0_goto_upper_body_joints")
        result = move_group.moveToJointPosition(_upper_body_joints, pose7_2, 0.005, wait=True)

        print("error code: ", result.error_code.val)
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            break  
    print("successful, Moving to Pose 8")
    # raw_input()
    while not rospy.is_shutdown():
        print("doing L0_goto_upper_body_joints")
        result = move_group.moveToJointPosition(_upper_body_joints, pose8, 0.005, wait=True)

        print("error code: ", result.error_code.val)
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            break  
    print("successful, Moving to Pose 9_1")    
    # raw_input()
    while not rospy.is_shutdown():
        print("doing L0_goto_upper_body_joints")
        result = move_group.moveToJointPosition(_upper_body_joints, pose9_1, 0.005, wait=True)

        print("error code: ", result.error_code.val)
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            break  
    print("successful, Moving to Pose 9_2")      
    # raw_input()
    while not rospy.is_shutdown():
        print("doing L0_goto_upper_body_joints")
        result = move_group.moveToJointPosition(_upper_body_joints, pose9_2, 0.005, wait=True)

        print("error code: ", result.error_code.val)
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            break  
    print("successful, Moving to Pose 9_2")     