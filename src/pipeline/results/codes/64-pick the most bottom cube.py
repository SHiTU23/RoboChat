### Query64: pick the most bottom cube --- Works in all steps 
"""
Errors: 
    - The scripts for functions was not imported
    - pixel to sim conversion was not correct
"""



# Import necessary libraries
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
from geometry_msgs.msg import Pose
import shape_msgs.msg
from moveit_commander import PlanningSceneInterface, MoveGroupCommander
from go_and_pick_object import *
from image_coordinates_to_robots_coordinates import pixel_to_sim

# Initialize ROS node and MoveIt commander
rospy.init_node("pick_and_place_RobotController", anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)

# Set up planning scene and action client
scene = PlanningSceneInterface()
exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
exectute_trajectory_client.wait_for_server()

# Define planning groups
PLANNING_GROUP_ARM = 'ur5_arm'
PLANNING_GROUP_GRIPPER = 'gripper'
GRIPPER_LINKS = ["robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link"]
TABLE_POSE = [1.5447, -1.5447, 1.5447, -1.5794, -1.5794, 0.0]
BOX_DIMENSION = [0.06, 0.06, 0.06]
BOX_Z_POSE = 1.045
ROBOT_Z_POSE = 1.21
object_name = 'box_unit'

# Object location in image coordinates
pixel_x, pixel_y = 445, 187


# Convert image coordinates to robot coordinates
sim_x , sim_y = pixel_to_sim(pixel_x, pixel_y)

sim_z = BOX_Z_POSE - ROBOT_Z_POSE  # Adjust for height
object_position = [sim_x, sim_y, sim_z]

# Initialize MoveGroup interfaces
arm_move_group_interface = MoveGroupCommander(PLANNING_GROUP_ARM)
move_group_interface_gripper = MoveGroupCommander(PLANNING_GROUP_GRIPPER)

# Move robot to table pose
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, TABLE_POSE)

# Add object to trajectory plan
add_cubeObject_to_trajectory(scene, arm_move_group_interface, GRIPPER_LINKS, object_name, object_position)

# Move robot above the object
current_pose = arm_move_group_interface.get_current_pose("ee_link").pose
target_pose = Pose()
target_pose.orientation = current_pose.orientation
target_pose.position.x = object_position[0]
target_pose.position.y = object_position[1]
target_pose.position.z = 0.2  # 20 cm above the object
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, target_pose)

# Lower robot to the object
target_pose.position.z -= 0.2  # Lower by 20 cm
arm_move_group_interface.set_pose_target(target_pose)
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, target_pose)

# Close the gripper to pick the object
control_the_gripper(exectute_trajectory_client, move_group_interface_gripper, 'close')

# Attach the object to the gripper
scene.attach_box(
    link=GRIPPER_LINKS[0],
    name=object_name,
    touch_links=GRIPPER_LINKS
)
rospy.sleep(1)

# Return robot to table pose
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, TABLE_POSE)