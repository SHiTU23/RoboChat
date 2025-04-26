"""
This code script controls the robot in ROS to go to a specific pose.
The target position can be given in [x, y, z] or robot's joint values or Pose format or with predefined names in the simulation: 'home', 'open', 'closed'.
"""

#Include the necessary libraries 
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
import actionlib
from moveit_commander import PlanningSceneInterface, MoveGroupCommander

def set_pose_values(group_name, destination, orientation=None):
    """
    Moves the robot to a specific position and orientation.
    
    @param: destination: A list of [x, y, z] for the target position in Cartesian space.
    @param: orientation: A list of [roll, pitch, yaw] in radians for the target orientation. Optional.
    """
    # Create a Pose object for the target
    target_pose = Pose()
    target_pose.position.x = destination[0]
    target_pose.position.y = destination[1]
    target_pose.position.z = destination[2]

    # If orientation is provided, convert roll, pitch, yaw to a quaternion
    if orientation:
        roll, pitch, yaw = orientation
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        target_pose.orientation.x = quaternion[0]
        target_pose.orientation.y = quaternion[1]
        target_pose.orientation.z = quaternion[2]
        target_pose.orientation.w = quaternion[3]
    else:
        # Use the current orientation if none is provided
        current_pose = group_name.get_current_pose().pose
        target_pose.orientation = current_pose.orientation

    # Set the target pose
    group_name.set_pose_target(target_pose)

def robot_set_pose(exectute_trajectory_client, group_name, target_pose):
    """
    Moves the robot to a specific position.
    @param: exectute_trajectory_client: the action client to execute the trajectory
    @param: group_name: the name of the links to move: 'gripper' or 'ur5_arm'
    @param: target_pose: can be predefined names: 'home', 'open', 'closed'; A list of robot joints values; A list of [x, y, z] or Pose.
    """
    ### set pre-defined pose like 'home'
    if type(target_pose) == str:
        group_name.set_named_target(target_pose)
    
    ### set joint or pose values in a list
    elif type(target_pose) == list:
        ### robot's joint values sare give for the target pose
        if len(target_pose) == 6:
            group_name.set_joint_value_target(target_pose)
        ### target pose given in [x, y, z] format
        elif len(target_pose) == 3: 
            set_pose_values(group_name, target_pose)
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


