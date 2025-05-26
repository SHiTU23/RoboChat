### Query: pick the leftest cube
"""
Errors:
import scripts
attach object
"""

# Import necessary libraries
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
from moveit_commander import PlanningSceneInterface, MoveGroupCommander
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from image_coordinates_to_robots_coordinates import pixel_to_sim
from go_and_pick_object import *
from pick_object import attache_object_to_gripper

# Initialize ROS and MoveIt
rospy.init_node("pick_and_place_RobotController", anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)

# Set up interfaces
scene = PlanningSceneInterface()
exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
exectute_trajectory_client.wait_for_server()

arm_move_group_interface = MoveGroupCommander('ur5_arm')
move_group_interface_gripper = MoveGroupCommander('gripper')

# Convert pixel coordinates to robot coordinates
sim_x, sim_y = pixel_to_sim(px=332, py=127)

# Move robot to Table Pose
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, [1.5447, -1.5447, 1.5447, -1.5794, -1.5794, 0.0])

# Add cube to trajectory plan
add_cubeObject_to_trajectory(scene, arm_move_group_interface, ["robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link"], "leftmost_cube", [sim_x, sim_y, 1.045 - 1.21])

# Move robot above the cube
current_pose = arm_move_group_interface.get_current_pose("ee_link")
target_pose = Pose()
target_pose.orientation = current_pose.pose.orientation
target_pose.position.x = sim_x
target_pose.position.y = sim_y
target_pose.position.z = 0.2
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, target_pose)

# Lower the robot to grasp the cube
target_pose.position.z -= 0.2
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, target_pose)

# Close the gripper
control_the_gripper(exectute_trajectory_client, move_group_interface_gripper, 'close')

# Attach cube to gripper
# scene.attach_box("leftmost_cube", SolidPrimitive(), ["robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link"])
attache_object_to_gripper(scene, ["robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link"], "leftmost_cube")
# Move robot back to Table Pose
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, [1.5447, -1.5447, 1.5447, -1.5794, -1.5794, 0.0])