# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import math

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("sauce", anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "right_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)
display_trajectory_publisher = rospy.Publisher(
    "/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20
)
planning_frame = move_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print("============ Printing robot state")
print(robot.get_current_state())
print("")



## BEGIN_SUB_TUTORIAL plan_to_pose
##
## Planning to a Pose Goal
## ^^^^^^^^^^^^^^^^^^^^^^^
## We can plan a motion for this group to a desired pose for the
## end-effector:
print("============ Moving to initial state ============")
raw_input()
pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = 0.707107
pose_goal.orientation.x = 0
pose_goal.orientation.y = 0.707107
pose_goal.orientation.z = 0
pose_goal.position.x = 0.7
pose_goal.position.y = -0.4
pose_goal.position.z = 0.7

move_group.set_pose_target(pose_goal)

## Now, we call the planner to compute the plan and execute it.
plan = move_group.go(wait=True)
# Calling `stop()` ensures that there is no residual movement
# move_group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
move_group.clear_pose_targets()



waypoints = []
step_size = math.pi/10
step = (2 * math.pi )/step_size
radius = 0.125
print (step)
wpose = move_group.get_current_pose().pose
center_x = wpose.position.x -radius
center_y = wpose.position.y
# for k in range(10):
#     for i in range(20):
#         angle = i *step_size
#         wpose.position.x = center_x + radius * math.cos(angle)
#         wpose.position.y = center_y - radius * math.sin(angle)
#         waypoints.append(copy.deepcopy(wpose))


#     (plan, fraction) = move_group.compute_cartesian_path(
#         waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
#     )  # jump_threshold
#     # display_trajectory = moveit_msgs.msg.DisplayTrajectory()
#     # display_trajectory.trajectory_start = robot.get_current_state()
#     # display_trajectory.trajectory.append(plan)
#     # Publish
#     # display_trajectory_publisher.publish(display_trajectory)


#     move_group.execute(plan, wait=True)
for i in range(200):
    angle = i *step_size
    wpose.position.x = center_x + radius * math.cos(angle)
    wpose.position.y = center_y - radius * math.sin(angle)
    waypoints.append(copy.deepcopy(wpose))
# print(waypoints)
# waypoints_final = []
# for i in range(10):
#     waypoints_final.append(copy.deepcopy(waypoints))
# print(waypoints_final)
# for i in range(10):
(plan, fraction) = move_group.compute_cartesian_path(
    waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
)  # jump_threshold
# display_trajectory = moveit_msgs.msg.DisplayTrajectory()
# display_trajectory.trajectory_start = robot.get_current_state()
# display_trajectory.trajectory.append(plan)
# Publish
# display_trajectory_publisher.publish(display_trajectory)

print("============ Calculation finished, start stiring ============")
raw_input()
move_group.execute(plan, wait=True)
    