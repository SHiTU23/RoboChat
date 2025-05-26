###  place the yellow cube at the same position
"""
eroor:
imports
"""


# Import necessary libraries
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
from moveit_commander import PlanningSceneInterface, MoveGroupCommander
from geometry_msgs.msg import Pose
import shape_msgs.msg
from go_and_pick_object import *
from image_coordinates_to_robots_coordinates import pixel_to_sim

# Initialize ROS node and MoveIt interfaces
rospy.init_node("pick_and_place_RobotController", anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)

scene = PlanningSceneInterface()
exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
exectute_trajectory_client.wait_for_server()

arm_move_group_interface = MoveGroupCommander('ur5_arm')
move_group_interface_gripper = MoveGroupCommander('gripper')

# Constants
TABLE_POSE = [1.5447, -1.5447, 1.5447, -1.5794, -1.5794, 0.0]
BOX_DIMENSION = [0.06, 0.06, 0.06]
BOX_Z_POSE = 1.045
ROBOT_Z_POSE = 1.21
GRIPPER_LINKS = ["robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link"]

# Object details
object_name = 'yellow_cube'
pixel_x, pixel_y = 379, 75

# Convert pixel coordinates to robot coordinates
# sim_x1, sim_y1 = 0.0502, 0.6592
# sim_x2, sim_y2 = 0.3928, 0.7176
# pixel_x1, pixel_y1 = 332, 127
# pixel_x2, pixel_y2 = 475, 96
# pixel_w = 25.0
# sim_w = 0.6

# sim_x = sim_x1 + ((pixel_x - pixel_x1) / pixel_w) * sim_w
# sim_y = sim_y1 + ((pixel_y - pixel_y1) / pixel_w) * sim_w
sim_x, sim_y = pixel_to_sim(pixel_x, pixel_y)
object_position = [sim_x, sim_y]

# Move robot to Table Pose
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, TABLE_POSE)

# Add the yellow cube to the trajectory
add_cubeObject_to_trajectory(scene, arm_move_group_interface, GRIPPER_LINKS, object_name, object_position)

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
arm_move_group_interface.set_pose_target(target_pose)
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, target_pose)

# Close the gripper
control_the_gripper(exectute_trajectory_client, move_group_interface_gripper, 'close')

# Attach the cube to the gripper
scene.attach_box(
    link=GRIPPER_LINKS[0], 
    name=object_name,
    touch_links=[
        GRIPPER_LINKS[0],
        GRIPPER_LINKS[1]
    ]
)
rospy.sleep(1)

# Move robot to placement position
placement_pose = Pose()
placement_pose.orientation = current_pose.pose.orientation
placement_pose.position.x = sim_x
placement_pose.position.y = sim_y
placement_pose.position.z = 0.2
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, placement_pose)

# Lower the robot to place the cube
placement_pose.position.z -= 0.2
arm_move_group_interface.set_pose_target(placement_pose)
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, placement_pose)

# Open the gripper
control_the_gripper(exectute_trajectory_client, move_group_interface_gripper, 'open')

# Detach the cube
scene.remove_attached_object(link=GRIPPER_LINKS[0], name=object_name)

# Return robot to Table Pose
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, TABLE_POSE)