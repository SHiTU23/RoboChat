"""
Errors:
syntax for giving the position
import scripts
"""
### Query72: pick the blue cube and placec it above the leftest cube 

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
from moveit_commander import PlanningSceneInterface, MoveGroupCommander
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AllowedCollisionMatrix, AllowedCollisionEntry
import shape_msgs.msg
from go_and_pick_object import *

# Initialize ROS and MoveIt components
rospy.init_node("pick_and_place_RobotController", anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)

scene = PlanningSceneInterface()
exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
exectute_trajectory_client.wait_for_server()

arm_move_group_interface = MoveGroupCommander('ur5_arm')
move_group_interface_gripper = MoveGroupCommander('gripper')
GRIPPER_LINKS = ["robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link"]
# Convert pixel coordinates to robot coordinates
def pixel_to_sim(px, py):
    sim_x1, sim_y1 = 0.0502, 0.6592
    sim_x2, sim_y2 = 0.3928, 0.7176
    pixel_x1, pixel_y1 = 332, 127
    pixel_x2, pixel_y2 = 475, 96
    pixel_w = 25.0
    sim_w = 0.6
    sim_x = sim_x1 + (px - pixel_x1) * (sim_x2 - sim_x1) / (pixel_x2 - pixel_x1)
    sim_y = sim_y1 + (py - pixel_y1) * (sim_y2 - sim_y1) / (pixel_y2 - pixel_y1)
    return round(sim_x, 1), round(sim_y, 1)

blue_cube_x, blue_cube_y = pixel_to_sim(445, 187)
leftmost_cube_x, leftmost_cube_y = pixel_to_sim(332, 96)

# # Move robot to initial table pose
# robot_set_pose(exectute_trajectory_client, arm_move_group_interface, [1.5447, -1.5447, 1.5447, -1.5794, -1.5794, 0.0])

# # Add collision objects to planning scene
# add_cubeObject_to_trajectory(scene, arm_move_group_interface, ["robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link"], 'blue_cube', [blue_cube_x, blue_cube_y, 1.045 - 1.21])
# add_cubeObject_to_trajectory(scene, arm_move_group_interface, ["robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link"], 'leftmost_cube', [leftmost_cube_x, leftmost_cube_y, 1.045 - 1.21])

# current_pose = arm_move_group_interface.get_current_pose("ee_link")
# target_pose = Pose()
# target_pose.orientation = current_pose.pose.orientation
# target_pose.position.x = blue_cube_x
# target_pose.position.y = blue_cube_y
# target_pose.position.z = 0.2
# # Move robot above the blue cube
# robot_set_pose(exectute_trajectory_client, arm_move_group_interface, target_pose)
# target_pose.position.z -= 0.2
# # Lower robot to grasp the blue cube
# robot_set_pose(exectute_trajectory_client, arm_move_group_interface, target_pose)

# Close gripper to pick the blue cube
control_the_gripper(exectute_trajectory_client, move_group_interface_gripper, 'close')

# Attach the blue cube to the gripper
# scene.attach_box('blue_cube', ["robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link"])
scene.attach_box(
        link=GRIPPER_LINKS[0], 
        name='blue_cube',
        touch_links=[
            GRIPPER_LINKS[0],
            GRIPPER_LINKS[1]
        ]
    )
# Move robot back to table pose
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, [1.5447, -1.5447, 1.5447, -1.5794, -1.5794, 0.0])

current_pose = arm_move_group_interface.get_current_pose("ee_link")
target_pose = Pose()
target_pose.orientation = current_pose.pose.orientation
target_pose.position.x = leftmost_cube_x
target_pose.position.y = leftmost_cube_y
target_pose.position.z = 0.2
# Move robot above the leftmost cube
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, target_pose)
target_pose.position.z -= 0.2

# Lower robot to place the blue cube
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, target_pose)

# Open gripper to release the blue cube
control_the_gripper(exectute_trajectory_client, move_group_interface_gripper, 'open')

# Detach the blue cube from the gripper
scene.remove_attached_object('blue_cube')

# Move robot back to table pose
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, [1.5447, -1.5447, 1.5447, -1.5794, -1.5794, 0.0])