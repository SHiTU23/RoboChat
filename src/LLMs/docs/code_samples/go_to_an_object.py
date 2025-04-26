"""
This code script shows an example of controling the robot in ROS to go to an object.
To move to an object position, the object should be added to the trajectory and not considered as a collision object.
The important step is to add the object to the trajectory plan to not consider it as a collision object. SO the robot can move to the object position.
"""

#Include the necessary libraries 
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
from moveit_commander import PlanningSceneInterface, MoveGroupCommander
import shape_msgs.msg
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AllowedCollisionMatrix, AllowedCollisionEntry


def robot_set_pose(exectute_trajectory_client, group_name, target_pose):
    """
    Moves the robot to a specific position.
    @param: exectute_trajectory_client: the action client to execute the trajectory
    @param: group_name: the name of the links to move: 'gripper' or 'ur5_arm'
    @param: target_pose: can be predefined names: 'home', 'open', 'closed' or A list of robot joints values.
    """
    ### set pre-defined pose like 'home'
    if type(target_pose) == str:
        group_name.set_named_target(target_pose)
    
    ### set joint or pose values in a list
    elif type(target_pose) == list:
        ### set joint values
        if len(target_pose) == 6:
            group_name.set_joint_value_target(target_pose)

    ### set the position by giving the Pose
    else:
        group_name.set_pose_target(target_pose)
        
    # Plan the motion
    plan = group_name.plan()

    if isinstance(plan, tuple):
        plan = plan[1]

    if plan.joint_trajectory.points:
        rospy.loginfo("Plan found, executing trajectory...")
        # Create a goal message object for the action server
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        exectute_trajectory_client.send_goal(goal)
        exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[32m' + f"Now at Pose: Position {target_pose}" + '\033[0m')
    else:
        rospy.logerr(f"No valid plan found. The robot could not reach the {target_pose}.")
    rospy.sleep(2)


PLANNING_GROUP_ARM = 'ur5_arm'
PLANNING_GROUP_GRIPPER = 'gripper'
TABLE_POSE = [1.5447, -1.5447, 1.5447, -1.5794, -1.5794, 0.0]
BOX_DIMENSION = [0.06, 0.06, 0.06]
BOX_Z_POSE = 1.045
ROBOT_Z_POSE = 1.21
object_position = [0.3, 0.5]

rospy.init_node("pick_and_place_RobotController", anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)

scene = PlanningSceneInterface()
exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
exectute_trajectory_client.wait_for_server()

arm_move_group_interface = MoveGroupCommander(PLANNING_GROUP_ARM)
move_group_interface_gripper = MoveGroupCommander(PLANNING_GROUP_GRIPPER)


### move the robot to the Table Pose
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, TABLE_POSE)

### add the object to the trajectory plan to not consider it as a collision object
collision_object = moveit_msgs.msg.CollisionObject()
collision_object.header.frame_id = arm_move_group_interface.get_planning_frame()
collision_object.id = 'box_unit' 

primitive = shape_msgs.msg.SolidPrimitive()
primitive.type = primitive.BOX
primitive.dimensions = BOX_DIMENSION

box_pose = Pose()
box_pose.orientation.w = 1.0
box_pose.position.x = object_position[0]
box_pose.position.y = object_position[1]
box_pose.position.z = BOX_Z_POSE - ROBOT_Z_POSE

collision_object.pose = box_pose
collision_object.primitives.append(primitive)
collision_object.primitive_poses.append(box_pose)
collision_object.operation = collision_object.ADD

scene.add_box(collision_object.id, collision_object, size=BOX_DIMENSION)

acm = AllowedCollisionMatrix()
acm.entry_names.append(collision_object.id)
acm.entry_names.append("robotiq_85_left_finger_tip_link")
acm.entry_names.append("robotiq_85_right_finger_tip_link")  

entry_blue_box = AllowedCollisionEntry(enabled=True)
entry_finger_tip_left = AllowedCollisionEntry(enabled=True)
entry_finger_tip_right = AllowedCollisionEntry(enabled=True)

acm.entry_values.append([True, True, True])  # "blue_box" with all other links
acm.entry_values.append([True, True, True])  # "robotiq_85_left_finger_tip_link"
acm.entry_values.append([True, True, True])  # "robotiq_85_right_finger_tip_link"

rospy.loginfo("*** CollisionMatrix is Allowed") 

current_pose = arm_move_group_interface.get_current_pose("ee_link")

target_pose = Pose()
target_pose.orientation = current_pose.pose.orientation
target_pose.position.x = object_position[0]
target_pose.position.y = object_position[1]
target_pose.position.z = 0.2

### move the robot above the box
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, target_pose)

### lower the robot to place gripper around the box
target_pose.position.z -= 0.2
arm_move_group_interface.set_pose_target(target_pose)
robot_set_pose(exectute_trajectory_client, arm_move_group_interface,target_pose)