import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
rospy.init_node('movo_moveit_test',
                anonymous=False)

moveit_commander.roscpp_initialize(sys.argv)

scene = moveit_commander.PlanningSceneInterface()

# lgripper = GripperActionClient('left')
# rgripper = GripperActionClient('right')
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


# while not rospy.is_shutdown():
#     print("doing L0_goto_upper_body_joints")
#     result = move_group.moveToJointPosition(_upper_body_joints, default_pose_tucked, 0.005, wait=True)


#     print("error code: ", result.error_code.val)
#     if result.error_code.val == MoveItErrorCodes.SUCCESS:
#         break
goal = upper_body.get_current_joint_values()
print('Joint name is:')
print(upper_body.get_joints())
print('current joint values is:')
print(goal)
# ['linear_joint', 
# 'left_shoulder_pan_joint', 'left_shoulder_lift_joint', 'left_arm_half_joint', 'left_elbow_joint', 'left_wrist_spherical_1_joint', 'left_wrist_spherical_2_joint', 'left_wrist_3_joint', 'left_ee_fixed_joint', 
# 'pan_joint', 'tilt_joint', 
# 'right_shoulder_pan_joint', 'right_shoulder_lift_joint', 'right_arm_half_joint', 'right_elbow_joint', 'right_wrist_spherical_1_joint', 'right_wrist_spherical_2_joint', 'right_wrist_3_joint', 'right_ee_fixed_joint']
i = 0
# 1 - 7 LEFT, 10 - 16 RIGHT
while (i < 5):
    goal[2] = goal[2] - 0.1
    goal[11] = goal[11] + 0.1
    goal[6] = goal[6] + 0.5
    goal[15] = goal[15] - 0.5
    goal[7] = goal[7] + pi
    goal[16] = goal[16] + pi
    upper_body.go(goal,wait=True)
    upper_body.stop()
    goal[2] = goal[2] + 0.1
    goal[11] = goal[11] - 0.1
    goal[6] = goal[6] - 0.5
    goal[15] = goal[15] + 0.5
    goal[7] = goal[7] - pi
    goal[16] = goal[16] - pi
    upper_body.go(goal,wait=True)
    upper_body.stop()
    i = i+1
