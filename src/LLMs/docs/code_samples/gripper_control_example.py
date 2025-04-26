"""
This code script shows example of controls the robot's gripper in ROS.
First the robot should be placed in TABLE_POSE to be in the initial position.
In the function `control_the_gripper`, the gripper can be controlled by predefined poses: 'open' and 'close'.
The gripper can be controlled by predefined pose for opening the gripper use 'open'. But for closing the gripper, the gripper position should be 0.26.
The gripper should be Open at first.
"""

#Include the necessary libraries 
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
from moveit_commander import PlanningSceneInterface, MoveGroupCommander

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


PLANNING_GROUP_ARM = 'ur5_arm'
PLANNING_GROUP_GRIPPER = 'gripper'
TABLE_POSE = [1.5447, -1.5447, 1.5447, -1.5794, -1.5794, 0.0]

rospy.init_node("pick_and_place_RobotController", anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)

scene = PlanningSceneInterface()
exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
exectute_trajectory_client.wait_for_server()

arm_move_group_interface = MoveGroupCommander(PLANNING_GROUP_ARM)
move_group_interface_gripper = MoveGroupCommander(PLANNING_GROUP_GRIPPER)

### move the robot to the Table Pose
robot_set_pose(exectute_trajectory_client, arm_move_group_interface, TABLE_POSE)

### open the gripper
control_the_gripper(exectute_trajectory_client, move_group_interface_gripper, 'open')

### close the gripper
control_the_gripper(exectute_trajectory_client, move_group_interface_gripper, 'close')
