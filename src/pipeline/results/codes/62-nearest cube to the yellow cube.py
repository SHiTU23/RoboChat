### Query62: Pick the nearest cube to the yellow cube and place it on the left side of the blue cube
"""
Errors:
    - The scripts for functions was not imported
    - attache object to gripper was not correct
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
from go_and_pick_object import *

# Initialize ROS and MoveIt
rospy.init_node("pick_and_place_RobotController", anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)

# Define robot groups and constants
PLANNING_GROUP_ARM = 'ur5_arm'
PLANNING_GROUP_GRIPPER = 'gripper'
GRIPPER_LINKS = ["robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link"]
TABLE_POSE = [1.5447, -1.5447, 1.5447, -1.5794, -1.5794, 0.0]

# Initialize interfaces
scene = PlanningSceneInterface()
exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
exectute_trajectory_client.wait_for_server()
arm_move_group_interface = MoveGroupCommander(PLANNING_GROUP_ARM)
move_group_interface_gripper = MoveGroupCommander(PLANNING_GROUP_GRIPPER)

# Move robot to table pose
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, TABLE_POSE)

# Convert object locations from pixel to simulation coordinates
def pixel_to_sim(px, py):
    sim_x1, sim_y1 = 0.0502, 0.6592
    sim_x2, sim_y2 = 0.3928, 0.7176
    pixel_x1, pixel_y1 = 332, 127
    pixel_x2, pixel_y2 = 475, 96
    scale_x = (sim_x2 - sim_x1) / (pixel_x2 - pixel_x1)
    scale_y = (sim_y2 - sim_y1) / (pixel_y2 - pixel_y1)
    offset_x = sim_x1 - pixel_x1 * scale_x
    offset_y = sim_y1 - pixel_y1 * scale_y
    sim_x = px * scale_x + offset_x
    sim_y = py * scale_y + offset_y
    return round(sim_x, 4), round(sim_y, 4)

nearest_cube_location = pixel_to_sim(399, 128)
placement_location = pixel_to_sim(408, 189)

# Add the cube to the trajectory plan
add_cubeObject_to_trajectory(scene, arm_move_group_interface, GRIPPER_LINKS, 'nearest_cube', nearest_cube_location)

# Move robot above the cube
current_pose = arm_move_group_interface.get_current_pose("ee_link")
t_pose = Pose()
t_pose.orientation = current_pose.pose.orientation
t_pose.position.x = nearest_cube_location[0]
t_pose.position.y = nearest_cube_location[1]
t_pose.position.z = 0.2
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, t_pose)

# Lower robot to grasp the cube
t_pose.position.z -= 0.2
arm_move_group_interface.set_pose_target(t_pose)
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, t_pose)

# Close the gripper
control_the_gripper(exectute_trajectory_client, move_group_interface_gripper, 'close')

# Attach the cube to the gripper
# attache_object_to_gripper(GRIPPER_LINKS, 'nearest_cube')
scene.attach_box(
    link=GRIPPER_LINKS[0], 
    name='nearest_cube',
    touch_links=[
        GRIPPER_LINKS[0],
        GRIPPER_LINKS[1]
    ]
)
rospy.sleep(1)

# Return to table pose
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, TABLE_POSE)

# Move robot above the placement location
t_pose.position.x = placement_location[0]
t_pose.position.y = placement_location[1]
t_pose.position.z = 0.2
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, t_pose)

# Lower robot to place the cube
t_pose.position.z -= 0.2
arm_move_group_interface.set_pose_target(t_pose)
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, t_pose)

# Open the gripper
control_the_gripper(exectute_trajectory_client, move_group_interface_gripper, 'open')

# Detach the cube from the gripper
scene.remove_attached_object(GRIPPER_LINKS[0], 'nearest_cube')

# Return to table pose
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, TABLE_POSE)