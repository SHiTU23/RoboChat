### Query: pick the blue cube and place it on top of the yellow cube
"""
error:
    Imports
    conversion
"""


# Import necessary libraries
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
from image_coordinates_to_robots_coordinates import pixel_to_sim

# Initialize ROS node and MoveIt Commander
rospy.init_node("pick_and_place_RobotController", anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)

# Define constants and initialize interfaces
PLANNING_GROUP_ARM = 'ur5_arm'
PLANNING_GROUP_GRIPPER = 'gripper'
GRIPPER_LINKS = ["robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link"]
TABLE_POSE = [1.5447, -1.5447, 1.5447, -1.5794, -1.5794, 0.0]
BOX_DIMENSION = [0.06, 0.06, 0.06]
BOX_Z_POSE = 1.045
ROBOT_Z_POSE = 1.21

scene = PlanningSceneInterface()
exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
exectute_trajectory_client.wait_for_server()

arm_move_group_interface = MoveGroupCommander(PLANNING_GROUP_ARM)
move_group_interface_gripper = MoveGroupCommander(PLANNING_GROUP_GRIPPER)

# Move robot to Table Pose
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, TABLE_POSE)

# Convert object locations from pixel to robot coordinates
sim_x1, sim_y1 = 0.0502, 0.6592
sim_x2, sim_y2 = 0.3928, 0.7176
pixel_x1, pixel_y1 = 332, 127
pixel_x2, pixel_y2 = 475, 96
pixel_w = 25.0
sim_w = 0.6
scale_factor = sim_w / pixel_w

blue_cube_pixel = (445, 187)
yellow_cube_pixel = (379, 75)

blue_cube_position = pixel_to_sim(blue_cube_pixel[0], blue_cube_pixel[1])
yellow_cube_position = pixel_to_sim(yellow_cube_pixel[0], yellow_cube_pixel[1])

# Add blue cube to trajectory
add_cubeObject_to_trajectory(scene, arm_move_group_interface, GRIPPER_LINKS, 'blue_cube', blue_cube_position)

# Move robot above blue cube
current_pose = arm_move_group_interface.get_current_pose("ee_link")
target_pose = Pose()
target_pose.orientation = current_pose.pose.orientation
target_pose.position.x = blue_cube_position[0]
target_pose.position.y = blue_cube_position[1]
target_pose.position.z = 0.2
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, target_pose)

# Lower robot to pick blue cube
target_pose.position.z -= 0.2
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, target_pose)

# Close gripper and attach blue cube
control_the_gripper(exectute_trajectory_client, move_group_interface_gripper, 'close')
scene.attach_box(GRIPPER_LINKS[0], 'blue_cube', touch_links=GRIPPER_LINKS)

# Move robot to Table Pose
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, TABLE_POSE)

# Add yellow cube to trajectory
add_cubeObject_to_trajectory(scene, arm_move_group_interface, GRIPPER_LINKS, 'yellow_cube', yellow_cube_position)

# Move robot above yellow cube
target_pose.position.x = yellow_cube_position[0]
target_pose.position.y = yellow_cube_position[1]
target_pose.position.z = 0.2
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, target_pose)

# Lower robot to place blue cube
target_pose.position.z -= 0.2
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, target_pose)

# Open gripper and detach blue cube
control_the_gripper(exectute_trajectory_client, move_group_interface_gripper, 'open')
scene.remove_attached_object(GRIPPER_LINKS[0], 'blue_cube')

# Move robot to Table Pose
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, TABLE_POSE)