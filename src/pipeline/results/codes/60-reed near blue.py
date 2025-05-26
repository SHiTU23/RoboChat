### Query: pick the red cube and place it near to the blue cube 
"""
Errors:
    adding collision object to the path planning
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
from image_coordinates_to_robots_coordinates import pixel_to_sim
from go_to_an_object_func import robot_set_pose, add_cubeObject_to_trajectory
from gripper_control_func import control_the_gripper

# Initialize ROS node and MoveIt interfaces
rospy.init_node("pick_and_place_RobotController", anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)

PLANNING_GROUP_ARM = 'ur5_arm'
PLANNING_GROUP_GRIPPER = 'gripper'
GRIPPER_LINKS = ["robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link"]
TABLE_POSE = [1.5447, -1.5447, 1.5447, -1.5794, -1.5794, 0.0]

scene = PlanningSceneInterface()
exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
exectute_trajectory_client.wait_for_server()

arm_move_group_interface = MoveGroupCommander(PLANNING_GROUP_ARM)
move_group_interface_gripper = MoveGroupCommander(PLANNING_GROUP_GRIPPER)

# Convert object positions from image coordinates to robot coordinates
red_cube_sim_coords = pixel_to_sim(475, 96)
blue_cube_sim_coords = pixel_to_sim(480, 225)

# Step 1: Move the robot to the Table Pose
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, TABLE_POSE)

# Step 2: Add the red cube to the trajectory plan
# collision_object = moveit_msgs.msg.CollisionObject()
# collision_object.header.frame_id = arm_move_group_interface.get_planning_frame()
# collision_object.id = "red_cube"
# primitive = shape_msgs.msg.SolidPrimitive()
# primitive.type = primitive.BOX
# primitive.dimensions = [0.06, 0.06, 0.06]
# box_pose = Pose()
# box_pose.orientation.w = 1.0
# box_pose.position.x = red_cube_sim_coords[0]
# box_pose.position.y = red_cube_sim_coords[1]
# box_pose.position.z = 1.045 - 1.21
# collision_object.primitives.append(primitive)
# collision_object.primitive_poses.append(box_pose)
# collision_object.operation = collision_object.ADD
# scene.add_box(collision_object.id, collision_object)
add_cubeObject_to_trajectory(scene, arm_move_group_interface, GRIPPER_LINKS, "red_cube", [red_cube_sim_coords[0], red_cube_sim_coords[1]])

# Step 3: Move above the red cube
target_pose = Pose()
target_pose.orientation = arm_move_group_interface.get_current_pose("ee_link").pose.orientation
target_pose.position.x = red_cube_sim_coords[0]
target_pose.position.y = red_cube_sim_coords[1]
target_pose.position.z = 0.2
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, target_pose)

# Step 4: Lower the robot to grasp the red cube
target_pose.position.z -= 0.2
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, target_pose)

# Step 5: Close the gripper
control_the_gripper(exectute_trajectory_client, move_group_interface_gripper, 'close')

# Step 6: Attach the red cube to the gripper
scene.attach_box(
    link=GRIPPER_LINKS[0],
    name="red_cube",
    touch_links=GRIPPER_LINKS
)

# Step 7: Move back to the Table Pose
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, TABLE_POSE)

# Step 8: Move above the blue cube for placement
placement_pose = Pose()
placement_pose.orientation = arm_move_group_interface.get_current_pose("ee_link").pose.orientation
placement_pose.position.x = blue_cube_sim_coords[0]
placement_pose.position.y = blue_cube_sim_coords[1]
placement_pose.position.z = 0.2
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, placement_pose)

# Step 9: Lower the robot to place the red cube
placement_pose.position.z -= 0.2
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, placement_pose)

# Step 10: Open the gripper to release the red cube
control_the_gripper(exectute_trajectory_client, move_group_interface_gripper, 'open')

# Step 11: Detach the red cube
scene.remove_attached_object(GRIPPER_LINKS[0], name="red_cube")

# Step 12: Move back to the Table Pose
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, TABLE_POSE)