### Every thing was perfect
### Query66: pick the blue cube and place it at the left side of its previous position 
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
from moveit_commander import PlanningSceneInterface, MoveGroupCommander
from geometry_msgs.msg import Pose
from image_coordinates_to_robots_coordinates import pixel_to_sim
from gripper_control_func import control_the_gripper
from go_to_an_object_func import robot_set_pose
from pick_object import add_cubeObject_to_trajectory
from pick_object import attache_object_to_gripper

# Initialize ROS and MoveIt
rospy.init_node("pick_and_place_RobotController", anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)

PLANNING_GROUP_ARM = 'ur5_arm'
PLANNING_GROUP_GRIPPER = 'gripper'
TABLE_POSE = [1.5447, -1.5447, 1.5447, -1.5794, -1.5794, 0.0]
GRIPPER_LINKS = ["robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link"]

scene = PlanningSceneInterface()
exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
exectute_trajectory_client.wait_for_server()

arm_move_group_interface = MoveGroupCommander(PLANNING_GROUP_ARM)
move_group_interface_gripper = MoveGroupCommander(PLANNING_GROUP_GRIPPER)

# Convert pixel coordinates to robot coordinates
converted_x_blue, converted_y_blue = pixel_to_sim(445, 187)
converted_x_new, converted_y_new = pixel_to_sim(480, 189)

# Move robot to Table Pose
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, TABLE_POSE)

# Add blue cube to trajectory plan
add_cubeObject_to_trajectory(scene, arm_move_group_interface, GRIPPER_LINKS, 'blue_cube', [converted_x_blue, converted_y_blue])

# Move robot above the blue cube
current_pose = arm_move_group_interface.get_current_pose("ee_link")
target_pose = Pose()
target_pose.orientation = current_pose.pose.orientation
target_pose.position.x = converted_x_blue
target_pose.position.y = converted_y_blue
target_pose.position.z = 0.2
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, target_pose)

# Lower robot to grasp the blue cube
target_pose.position.z -= 0.2
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, target_pose)

# Close gripper to pick the cube
control_the_gripper(exectute_trajectory_client, move_group_interface_gripper, 'close')

# Attach cube to gripper
attache_object_to_gripper(scene, GRIPPER_LINKS, 'blue_cube')

# Move robot back to Table Pose
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, TABLE_POSE)

# Add new position to trajectory plan
add_cubeObject_to_trajectory(scene, arm_move_group_interface, GRIPPER_LINKS, 'new_position', [converted_x_new, converted_y_new])

# Move robot above the new position
target_pose.position.x = converted_x_new
target_pose.position.y = converted_y_new
target_pose.position.z = 0.2
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, target_pose)

# Lower robot to place the cube
target_pose.position.z -= 0.2
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, target_pose)

# Open gripper to release the cube
control_the_gripper(exectute_trajectory_client, move_group_interface_gripper, 'open')

# Detach cube from gripper
scene.remove_attached_object(GRIPPER_LINKS[0], 'blue_cube')

# Move robot back to Table Pose
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, TABLE_POSE)
