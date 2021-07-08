#!/usr/bin/env python
import rospy
import sys
import copy
import tf
from hrclib_client_v6 import odyssey_Interface
import geometry_msgs.msg 
import math
import moveit_commander
from movo_action_clients.gripper_action_client import GripperActionClient
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
import time
# import zl_ods_contact

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
default_pose_tucked = [-1.595, -1.5, 0.1, -2.612, 0.0, 0.496, -1.69,
                       1.595, 1.5, -0.1, 2.612, 0.0, -0.496, 1.69,
                       0.14, 0, 0]

# class temp_pickbolt_client(object):
#     def __init__(self):
#         rospy.init_node('transform_subscriber', anonymous= True)
#         rospy.Subscriber("/marker",geometry_msgs.msg.Transform,self.marker_CallBack)
        
#         self.msg=None
#     def marker_CallBack(self,msg):
#         print(msg)
#         self.msg=msg


# class move_to_tuck(object):
#     def __init__(self):
#         _upper_body_joints = ["right_shoulder_pan_joint",
#                       "right_shoulder_lift_joint",
#                       "right_arm_half_joint",
#                       "right_elbow_joint",
#                       "right_wrist_spherical_1_joint",
#                       "right_wrist_spherical_2_joint",
#                       "right_wrist_3_joint",
#                       "left_shoulder_pan_joint",
#                       "left_shoulder_lift_joint",
#                       "left_arm_half_joint",
#                       "left_elbow_joint",
#                       "left_wrist_spherical_1_joint",
#                       "left_wrist_spherical_2_joint",
#                       "left_wrist_3_joint",
#                       "linear_joint",
#                       "pan_joint",
#                       "tilt_joint"]
                      
# # Head looking straight
#         default_pose_tucked = [-1.595, -1.5, 0.1, -2.612, 0.0, 0.496, -1.69,
#                             1.595, 1.5, -0.1, 2.612, 0.0, -0.496, 1.69,
#                             0.14, 0, 0]
#         lgripper = GripperActionClient('left')
#         rgripper = GripperActionClient('right')
#         gripper_closed = 0.00
#         gripper_open = 0.165
        
#         larm_group = moveit_commander.MoveGroupCommander("left_arm")
#         rarm_group = moveit_commander.MoveGroupCommander("right_arm")
#         upper_body = moveit_commander.MoveGroupCommander("upper_body")

#         move_group = MoveGroupInterface("upper_body", "base_link")
#         lmove_group = MoveGroupInterface("left_arm", "base_link")
#         rmove_group = MoveGroupInterface("right_arm", "base_link")
#         self._upper_body_joints=_upper_body_joints
#         self.default_pose_tucked=default_pose_tucked
#         self.move_group=move_group

#     def move_to_initial_pose(self):


#         print("Done spinning up MoveIt!")

        
#         while not rospy.is_shutdown():
#             print("doing L0_goto_upper_body_joints")
#             result = self.move_group.moveToJointPosition(self._upper_body_joints, self.default_pose_tucked, 0.005, wait=True)

#             print("error code: ", result.error_code.val)
#             if result.error_code.val == MoveItErrorCodes.SUCCESS:
#                 break

#         print("successful")

#     def __del__(self):
#         pass
#temp_pickbolt_client.marker_CallBack()

if __name__=="__main__":
    rospy.init_node("Test_all",
                    anonymous=False)
    # moveit_commander.roscpp_initialize(sys.argv)
    # scene = moveit_commander.PlanningSceneInterface()
    # tuck = move_to_tuck()
    # tuck.move_to_initial_pose()
    # del tuck
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
        result = move_group.moveToJointPosition(_upper_body_joints, default_pose_tucked, 0.005, wait=True)


        print("error code: ", result.error_code.val)
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            break
    


    print("successful")
    time.sleep(3)
    ods = odyssey_Interface()

    print "=========== left arm out =========="
    # raw_input()
    # -pi/2 --> rotate relatively to x axis of base_link
    ods._L0_single_task_move_safe("left",[0.444+0.3, 0.130, 0.8],
                                        [-math.pi/2, 0, 0],
                                        [20 for i in range(6)])
    time.sleep(3)
    print "=========== right arm out =========="
    # raw_input()
    ods._L0_single_task_move_safe("right",[0.444+0.3, -0.130, 0.8],
                                        [-math.pi/2, 0, 0],
                                        [20 for i in range(6)])
    time.sleep(3)
    print "=========== right gripper open =========="
    # raw_input()
    ods._L0_gripper("right",1)
    time.sleep(3)
    print "=========== left gripper open =========="
    # raw_input()
    ods._L0_gripper("left",1)
    time.sleep(3)
    
    print "=========== dual gripper close =========="
    # raw_input()
    ods._L0_dual_set_gripper(0)
    time.sleep(3)

    print "=========== dual arm back =========="
    # raw_input()
    ods._L0_dual_jp_move_safe_relate(
        jp_r=[0, -0.2, 0, 0, -0.06, 1.8, 0], rmaxforce=[10, 10, 10, 5, 5, 5],
        jp_l=[0, 0.2, 0, 0, 0.06, -1.8, 0], lmaxforce=[10, 10, 10, 5, 5, 5],
        duration=3)
    # ods._L0_dual_jp_move_safe_relate([-1.595, -1.6, 0.1, -2.612, 0.0, 0.496, -1.69],[1.595, 1.6,-0.1,2.612,0.0,-0.496,1.69],[20 for i in range(6)],[20 for i in range(6)],5)
    

    # ods.grip("right",1)
    # ods.grip("right",0)
    # ods.grip("right",1)
    # ods.grip("right",0)


############################ Pick Bolt ##################################################################
    # ods=odyssey_Interface()
    # ods._L0_single_task_move_safe("right",[0.7263-0.1, -0.309544, 0.93824+0.2],
    #                               [0.2+math.pi/2, math.pi / 2, -math.pi / 2],
    #                               [20 for i in range(6)])
    # ods._L0_single_task_move_safe("right",[ods.marker0[0]-0.03,ods.marker0[1]-0.05,ods.marker0[2]+0.2],
    #                               [0.2+math.pi/2, math.pi / 2, -math.pi / 2],
    #                               [20 for i in range(6)])
    # ods.grip("right",1)
    # ods.single_move_relate(arm="right",move=[0,0,-0.09],maxforce=[25 for i in range(6)],time=2)
    # ods.grip("right",0)
    # ods.single_move_relate(arm="right",move=[0,0,0.09],maxforce=[25 for i in range(6)],time=2)


#########################################################################################################

############################### Follow Marker ###########################################################
    # while not rospy.is_shutdown():
    #     ods=odyssey_Interface()
    #     ods._L0_single_task_move_safe("right",[0.7263-0.1, -0.309544, 0.93824+0.2],
    #                             [0.2+math.pi/2, math.pi / 2, -math.pi / 2],
    #                             [20 for i in range(6)])
    #     # ods.go_upper_default_jp(pick_pos_dict,hard=False,f=30)
    #     ods._L0_single_task_move_safe("right",[ods.marker0[0],ods.marker0[1],ods.marker0[2]+0.3],
    #                               [0.2+math.pi/2, math.pi / 2, -math.pi / 2],
    #                               [20 for i in range(6)])
    #     rospy.Rate(0.1).sleep()
#########################################################################################################


#     ods.go_upper_default_jp(posdict=ods.gval.default_pose_screw_rightarm_observe,hard=False,f=15)
#     #ods.go_upper_default_jp(posdict=ods.gval.default_pose_ready_insert_and_screw,hard=False,f=15)
#     print(ods.rpose)#[0.7375103572717412, -0.25048753446663785, 1.2731132364376299
# #    ods.get_rpose
#     print('The position of marker0:', ods.marker0)#[0.9548131752342998, -0.30152793713899934, 0.9200864503752137]
# #    ods.arm_cart_move(arm="right",pos=[ods.marker0[0],ods.marker0[1],ods.marker0[2]+0.1],orn=[0.2+math.pi/2, math.pi / 2, -math.pi / 2],maxforce=[15 for i in range(6)])

# #    ods._L0_single_task_move_safe(arm="right",pos=[ods.rpose[0],ods.rpose[1],ods.rpose[2]+0.1],orn=[0, math.pi / 2, -math.pi / 2],maxforce=[15 for i in range(6)])

#     ods._L0_single_task_move_safe("right",[0.7263, -0.309544, 0.93824+0.2],
#                                   [0.1, math.pi / 2, -math.pi / 2],
#                                   [50 for i in range(6)],hard=True)
                                
#     ods.go_upper_default_jp(posdict=ods.gval.default_pose_screw_rightarm_observe,hard=False,f=15)
#    ods._L0_single_task_move_safe("right",[0.74,-0.25,1.17],[0, math.pi / 2, -math.pi / 2],  [20 for i in range(6)])
    # print(ods.rpose)#[0.7375103572717412, -0.25048753446663785, 1.2731132364376299
    #tp=temp_pickbolt_client()
       
#    print(ods.lpose)
#    print(ods.rpose)
#    print(ods.get_jp_dict())
    # ods.get
    # ods.go_upper_default_jp(pick_pos_dict,hard=False,f=30)
    # ods.get_jp_dict()
    # ods.go_upper_default_jp(pick_pos_dict,hard=False,f=30)

    # ods.set_grippers(0)
    # ods.set_grippers(1)
    # ods.grip(rl="left",v=1)
    # ods.grip(rl="left",v=0)
    # ods.grip(rl="left",v=1)



def subscriber():
    rospy.init_node('transform_subscriber', anonymous= True)
    rospy.Subscriber()
