"""
This code script controls the robot in ROS to go to a specific pose and pick an object.
The steps are:
1. First the robot should be placed in TABLE_POSE to be in the initial position.
2. MOve to the object position. For this, the object should be added to the trajectory and not considered as a collision object.
3. Close the gripper to pick the object.
4. The object should be attached to the grippers to be able to move it.

The important steps are
 1. add the object to the trajectory plan to not consider it as a collision object. SO the robot can move to the object position.
 2. attach the object to the gripper to be able to move it.
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

def control_the_gripper(exectute_trajectory_client, gripper_group, gripper_position='open'):
    """
    Moves the robot's gripper in a modified pose

    @param: gripper_position: 'open' or 'close' 
    """
    if gripper_position == 'open':
        robot_set_pose(exectute_trajectory_client, gripper_group, 'open')

    elif gripper_position == 'close':
        target_gripper_position = 0.26
        rospy.loginfo(f"gripper position to fit object width: {target_gripper_position}")

        gripper_joint_positions = gripper_group.get_current_joint_values()

        for i in range(len(gripper_joint_positions)):
            gripper_joint_positions[i] = target_gripper_position
            print(f"gripper_joint_positions[i] = target_gripper_position: {gripper_joint_positions[i]} = {target_gripper_position}")

        gripper_group.set_joint_value_target(gripper_joint_positions)
        plan = gripper_group.plan()

        if isinstance(plan, tuple):
            plan = plan[1]

        if plan.joint_trajectory.points:
            rospy.loginfo("Plan found, executing trajectory...")
            # Create a goal message object for the action server
            goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
            goal.trajectory = plan
            exectute_trajectory_client.send_goal(goal)
            exectute_trajectory_client.wait_for_result()
            rospy.loginfo('\033[32m' + f"Now at Pose: Position {gripper_joint_positions}" + '\033[0m')
        else:
            rospy.logerr(f"No valid plan found. The robot could not reach the {gripper_joint_positions}.")
    rospy.sleep(1)

def add_cubeObject_to_trajectory(scene, group_link, gripper_links, object_name, object_position):
    """
    Adding the object to the trajectory plan to not consider it as a collision object. SO the robot can move to the object position.
    """
    ### add the object to the trajectory plan to not consider it as a collision object
    collision_object = moveit_msgs.msg.CollisionObject()
    collision_object.header.frame_id = group_link.get_planning_frame()
    collision_object.id = object_name

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
    acm.entry_names.append(gripper_links[0])
    acm.entry_names.append(gripper_links[1])  

    entry_blue_box = AllowedCollisionEntry(enabled=True)
    entry_finger_tip_left = AllowedCollisionEntry(enabled=True)
    entry_finger_tip_right = AllowedCollisionEntry(enabled=True)

    acm.entry_values.append([True, True, True])  # "blue_box" with all other links
    acm.entry_values.append([True, True, True])  # robot gripper link name
    acm.entry_values.append([True, True, True])  # robot the other gripper link name

    rospy.loginfo("*** CollisionMatrix is Allowed") 
   


PLANNING_GROUP_ARM = 'ur5_arm'
PLANNING_GROUP_GRIPPER = 'gripper'
GRIPPER_LINKS = ["robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link"]
TABLE_POSE = [1.5447, -1.5447, 1.5447, -1.5794, -1.5794, 0.0]
BOX_DIMENSION = [0.06, 0.06, 0.06]
BOX_Z_POSE = 1.045
ROBOT_Z_POSE = 1.21
object_name = 'box_unit'
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
add_cubeObject_to_trajectory(scene, arm_move_group_interface, GRIPPER_LINKS, object_name, object_position)

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


### close the gripper
control_the_gripper(exectute_trajectory_client, move_group_interface_gripper, 'close')

### attache the object to the gripper
scene.attach_box(
    link=GRIPPER_LINKS[0], 
    name=object_name,
    touch_links=[
        GRIPPER_LINKS[0],
        GRIPPER_LINKS[1]
    ]
)
rospy.sleep(1)

### take the obj to table pose
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, TABLE_POSE)