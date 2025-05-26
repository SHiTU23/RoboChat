### Query63: pick the red cube and place it on the left side of its previous position 
"""
Errors:
    - The scripts for functions was not imported
    - attache object to gripper was not correct
"""


#  Import necessary libraries
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
from moveit_commander import PlanningSceneInterface, MoveGroupCommander
from geometry_msgs.msg import Pose
import shape_msgs.msg
from go_and_pick_object import *
# Initialize ROS node and MoveIt Commander
rospy.init_node("pick_and_place_RobotController", anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)

# Define constants
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

# Step 1: Move to Table Pose
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, TABLE_POSE)

# Step 2: Convert pixel coordinates to simulation coordinates
def pixel_to_sim(px, py):
    sim_x1, sim_y1 = 0.0502, 0.6592
    sim_x2, sim_y2 = 0.3928, 0.7176
    pixel_x1, pixel_y1 = 332, 127
    pixel_x2, pixel_y2 = 475, 96
    pixel_w = 25.0
    sim_w = 0.6
    sim_x = sim_x1 + (px - pixel_x1) * (sim_x2 - sim_x1) / (pixel_x2 - pixel_x1)
    sim_y = sim_y1 + (py - pixel_y1) * (sim_y2 - sim_y1) / (pixel_y2 - pixel_y1)
    return sim_x, sim_y

red_cube_sim_x, red_cube_sim_y = pixel_to_sim(475, 96)
target_sim_x, target_sim_y = pixel_to_sim(437, 103)

# Step 3: Add the red cube to the trajectory
object_name = 'red_cube'
object_position = [red_cube_sim_x, red_cube_sim_y]
add_cubeObject_to_trajectory(scene, arm_move_group_interface, GRIPPER_LINKS, object_name, object_position)

# Step 4: Move above the red cube
current_pose = arm_move_group_interface.get_current_pose("ee_link")
target_pose = Pose()
target_pose.orientation = current_pose.pose.orientation
target_pose.position.x = red_cube_sim_x
target_pose.position.y = red_cube_sim_y
target_pose.position.z = 0.2
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, target_pose)

# Step 5: Lower to pick the red cube
target_pose.position.z -= 0.2
arm_move_group_interface.set_pose_target(target_pose)
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, target_pose)

# Step 6: Close the gripper
control_the_gripper(exectute_trajectory_client, move_group_interface_gripper, 'close')

# Step 7: Attach the red cube to the gripper
scene.attach_box(
    link=GRIPPER_LINKS[0], 
    name=object_name,
    touch_links=[
        GRIPPER_LINKS[0],
        GRIPPER_LINKS[1]
    ]
)
rospy.sleep(1)


# Step 8: Move back to Table Pose
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, TABLE_POSE)

# Step 9: Move above the target position
target_pose.position.x = target_sim_x
target_pose.position.y = target_sim_y
target_pose.position.z = 0.2
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, target_pose)

# Step 10: Lower to place the red cube
target_pose.position.z -= 0.2
arm_move_group_interface.set_pose_target(target_pose)
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, target_pose)

# Step 11: Open the gripper
control_the_gripper(exectute_trajectory_client, move_group_interface_gripper, 'open')

# Step 12: Detach the red cube
scene.remove_attached_object(GRIPPER_LINKS[0], object_name)

# Step 13: Move back to Table Pose
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, TABLE_POSE)