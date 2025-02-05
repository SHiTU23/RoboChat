#! /usr/bin/env python

#Include the necessary libraries 
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import AllowedCollisionMatrix, AllowedCollisionEntry, AttachedCollisionObject, CollisionObject
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 
import actionlib
from math import pi
import os
import numpy as np
import shape_msgs.msg
from tf.transformations import quaternion_from_euler
from moveit_commander import PlanningSceneInterface, MoveGroupCommander, planning_scene_interface
import shape_msgs
from moveit_msgs.srv import GetPlanningScene
import threading
import time

## self._eef_link = self._group.get_end_effector_link()

class robot_controller:

    PLANNING_GROUP_ARM = 'ur5_arm'
    PLANNING_GROUP_GRIPPER = 'gripper'

    TABLE_POSE = [1.5447, -1.5447, 1.5447, -1.5794, -1.5794, 0.0]
    
    CURRENT_DIR = os.path.dirname(__file__)
    def __init__(self):
        rospy.init_node("pick_and_place_RobotController", anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.Subscriber('camera1/rgb/image_raw', Image, self._image_callback)
        self._cv_bridge = CvBridge()
        self.cv_image = None
        self.imageSave_counter = 0

        self._scene = PlanningSceneInterface()

        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self.move_group_interface_arm = MoveGroupCommander(self.PLANNING_GROUP_ARM)
        self.move_group_interface_gripper = MoveGroupCommander(self.PLANNING_GROUP_GRIPPER)

        self.image_counter = 0


    def robot_at_homePose(self):
        self.robot_set_pose(self.move_group_interface_arm, 'home')
        rospy.sleep(1)

    def robot_set_pose(self, group_name, target_pose):
        """
        Moves the robot to a specific position.
        
        @param: group_name: the name of the links to move: 'gripper' or 'ur5_arm'
        @param: target_pose: can be predefined names: 'home', 'open', 'closed'; A list of robot joints values; 
                A list of [x, y, z] or Pose.
        """
        ### set pre-defined pose like 'home'
        if type(target_pose) == str:
            group_name.set_named_target(target_pose)
        
        ### set joint or pose values in a list
        elif type(target_pose) == list:
            ### set joint values
            if len(target_pose) == 6:
                group_name.set_joint_value_target(target_pose)

            ### set position values: [x, y, z]
            elif len(target_pose) == 3:
                self.set_pose_values(group_name, target_pose)

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
            self._exectute_trajectory_client.send_goal(goal)
            self._exectute_trajectory_client.wait_for_result()
            rospy.loginfo('\033[32m' + f"Now at Pose: Position {target_pose}" + '\033[0m')
        else:
            rospy.logerr(f"No valid plan found. The robot could not reach the {target_pose}.")

    def set_pose_values(self, group_name, destination, orientation=None):
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
    
    def close_gripper_to_fit_obj(self, target_gripper_position = 0.26):
        """
        Moves the robot's gripper in a modified pose

        @param: target_gripper_position: default value 0.26
        """
        rospy.loginfo(f"gripper position to fit object width: {target_gripper_position}")

        gripper_joint_positions = self.move_group_interface_gripper.get_current_joint_values()

        for i in range(len(gripper_joint_positions)):
            gripper_joint_positions[i] = target_gripper_position
            print(f"gripper_joint_positions[i] = target_gripper_position: {gripper_joint_positions[i]} = {target_gripper_position}")

        self.move_group_interface_gripper.set_joint_value_target(gripper_joint_positions)
        plan = self.move_group_interface_gripper.plan()

        if isinstance(plan, tuple):
            plan = plan[1]

        if plan.joint_trajectory.points:
            rospy.loginfo("Plan found, executing trajectory...")
            # Create a goal message object for the action server
            goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
            goal.trajectory = plan
            self._exectute_trajectory_client.send_goal(goal)
            self._exectute_trajectory_client.wait_for_result()
            rospy.loginfo('\033[32m' + f"Now at Pose: Position {gripper_joint_positions}" + '\033[0m')
        else:
            rospy.logerr(f"No valid plan found. The robot could not reach the {gripper_joint_positions}.")
        rospy.sleep(1)

    def pick_and_place_obj(self, from_pose, to_pose):
        """
        Moves the robot to obj position, picks it, move the obj to the target pose, place it there.
        Frist robot is on the table pose.
        
        @param: from_pose: A list of [x, y] position of the object, obtained from image.
        @param: to_pose: A list of [x, y] to place the obj.
        """
        BOX_DIMENSION = [0.06, 0.06, 0.06]
        BOX_Z_POSE = 1.045
        ROBOT_Z_POSE = 1.21

        ### place robot in table pose
        self.robot_set_pose(self.move_group_interface_arm, self.TABLE_POSE)
        rospy.sleep(1)

        ### add collision obj to be included in planned path; don't consider collision
        collision_object = moveit_msgs.msg.CollisionObject()
        collision_object.header.frame_id = self.move_group_interface_arm.get_planning_frame()
        collision_object.id = 'box_unit' 

        primitive = shape_msgs.msg.SolidPrimitive()
        primitive.type = primitive.BOX
        primitive.dimensions = BOX_DIMENSION

        box_pose = Pose()
        box_pose.orientation.w = 1.0
        box_pose.position.x = from_pose[0]
        box_pose.position.y = from_pose[1]
        box_pose.position.z = BOX_Z_POSE - ROBOT_Z_POSE

        collision_object.pose = box_pose
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = collision_object.ADD

        self._scene.add_box(collision_object.id, collision_object, size=BOX_DIMENSION)

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

        current_pose = self.move_group_interface_arm.get_current_pose("ee_link")

        target_pose1 = Pose()
        target_pose1.orientation = current_pose.pose.orientation
        target_pose1.position.x = from_pose[0]
        target_pose1.position.y = from_pose[1]
        target_pose1.position.z = 0.2

        ### above the box
        self.robot_set_pose(self.move_group_interface_arm,target_pose1)
        rospy.sleep(1)

        ### lower the robot to place gripper around the box
        target_pose1.position.z -= 0.2
        self.move_group_interface_arm.set_pose_target(target_pose1)
        self.robot_set_pose(self.move_group_interface_arm,target_pose1)
        rospy.sleep(1)
        
        ###########################################
        ######          PICK OBJ            #######
        ###########################################

        ### close the gripper
        self.close_gripper_to_fit_obj()

        ### attach the obj
        self._scene.attach_box(
            link="robotiq_85_left_finger_tip_link",  # Adjust as needed
            name=collision_object.id,
            touch_links=[
                "robotiq_85_left_finger_tip_link",
                "robotiq_85_right_finger_tip_link"
            ]
        )
        rospy.sleep(1)

        ### take the obj to table pose
        self.robot_set_pose(self.move_group_interface_arm, self.TABLE_POSE)
        rospy.sleep(1)

        ###########################################
        ######          PLACE OBJ           #######
        ###########################################

        ### take the obj to the target pose
        current_pose = self.move_group_interface_arm.get_current_pose("ee_link")
        target_pose2 = Pose()
        target_pose2.orientation = current_pose.pose.orientation
        target_pose2.position.x = to_pose[0]
        target_pose2.position.y = to_pose[1]
        target_pose2.position.z = 0.2

        ### move the robot to the target pose
        self.robot_set_pose(self.move_group_interface_arm, target_pose2)
        rospy.sleep(1)

        ### lower the robot closer to the table
        target_pose2.position.z -= 0.1
        self.robot_set_pose(self.move_group_interface_arm, target_pose2)
        rospy.sleep(1)

        ###########################################
        ######         OPEN GRIPPER         #######
        ###########################################

        ### open the gripper
        self.robot_set_pose(self.move_group_interface_gripper, 'open')
        rospy.sleep(1)

        ### move the robot back to the table pose
        ### take the obj to table pose
        self.robot_set_pose(self.move_group_interface_arm, self.TABLE_POSE)
        rospy.sleep(1)


    ###########################################
    ######              IMAGE           #######
    ###########################################

    def _image_callback(self, msg):        
        try:
            self.image_counter += 1
            self.cv_image = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("image", self.cv_image)

            last_time = time.time()
            interval = 3 # sec

            while time.time() - last_time < interval:
                self.cv_image = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                cv2.imshow("image", self.cv_image)

 
            cv2.imwrite(f"{self.CURRENT_DIR}/images/_image{self.image_counter}.jpg", self.cv_image)
            rospy.loginfo("+++++Image saved +++++")

        except CvBridgeError as e:
            rospy.logerr(f"CbBridgeError: {e}")




    def object_detector(self):
        """
            Gives the x, y of the center point of the box
        """
        self.show_image()
        rospy.sleep(1)
        if self.cv_image is not None:
            detected_objects = []
            hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
            min_hsv_value = np.array([118, 90, 84])
            max_hsv_value = np.array([125, 234, 159])

            mask_range = cv2.inRange(hsv, min_hsv_value, max_hsv_value)
            mask = cv2.adaptiveThreshold(mask_range, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 3, 3)
            # cv2.imshow("mask", mask)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                cv2.polylines(self.cv_image, [cnt], True, [255, 0, 0], 1)

                area = cv2.contourArea(cnt)
                if area > 20:
                    rospy.loginfo(f"the object area is: {area} ")
                    detected_objects.append(cnt)

            rospy.loginfo(f"detected obj BB: {detected_objects}")

            objs = []

            for obj in detected_objects:
                rect = cv2.minAreaRect(obj)
                objs.append(rect)
                (x_center, y_center), (w, h), orientation = rect
                print(f"x: {x_center}, y: {y_center}, theta:{orientation}")
                print(f"w: {w}, h: {h}")

                box = cv2.boxPoints(rect)
                box = np.int0(box)

                cv2.polylines(self.cv_image, [box], True, (0, 255, 0), 1)

                ###convert pixel values to world frame
                x, y = self.pixel2world_conversion(x_center, y_center)
                rospy.loginfo(f"obj_center:({x}, {y})")
                cv2.putText(self.cv_image, "world_x: {}".format(round(x, 1)) + " y: {}".format(round(y,1)), (int(x_center), int(y_center)), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0),1)
                cv2.circle(self.cv_image, (int(x_center), int(y_center)), 1, (255,0,0), thickness=-1)
            
            self.show_image()
            return x, y

    def pixel2world_conversion(self, x, y):
        ### these values are taken from an object presented in the scene
        referenceObj_pixel_x = 447.0
        referenceObj_pixel_y = 183.5

        referenceObj_actual_x = 0.3
        referenceObj_actual_y = 0.5

        refernceObj_pixel_w = 25.0
        refernceObj_actual_w = 0.6

        pixel2mm = refernceObj_pixel_w / refernceObj_actual_w

        #condition top right of the imgage
        if (x > referenceObj_pixel_x) and (y < referenceObj_pixel_y):
            worldFrame_y = referenceObj_actual_y - (x-referenceObj_pixel_x)/pixel2mm
            worldFrame_x = referenceObj_actual_x +(referenceObj_pixel_y - y)/pixel2mm
        
        #condition bottom right of the imgage
        elif (x > referenceObj_pixel_x) and (y > referenceObj_pixel_y):
            worldFrame_y = referenceObj_actual_y - (x-referenceObj_pixel_x)/pixel2mm  
            worldFrame_x = referenceObj_actual_x - (y - referenceObj_pixel_y)/pixel2mm  
        
        #condition bottom left of the imgage
        elif (x < referenceObj_pixel_x) and (y > referenceObj_pixel_y): 
            worldFrame_y = referenceObj_actual_y + (referenceObj_pixel_x-x)/pixel2mm  
            worldFrame_x = referenceObj_actual_x - (y - referenceObj_pixel_y)/pixel2mm 

        #condition top left of the imgage
        elif (x < referenceObj_pixel_x) and (y < referenceObj_pixel_y):
            worldFrame_y = referenceObj_actual_y + (referenceObj_pixel_x-x)/pixel2mm
            worldFrame_x = referenceObj_actual_x + (referenceObj_pixel_y - y)/pixel2mm

        elif (x == referenceObj_pixel_x) and (y == referenceObj_pixel_y):
            worldFrame_x = referenceObj_actual_x
            worldFrame_y = referenceObj_actual_y

        return worldFrame_x, worldFrame_y

    def show_image(self):
        if self.cv_image is not None:
            cv2.imshow("camera_image", self.cv_image)
            cv2.waitKey(1)

    # Class Destructor
    def __del__(self):
        #When the actions are finished, shut down the moveit commander
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[95m' + "Object of class MyRobot Deleted." + '\033[0m')


def main():
    controller = robot_controller()

    while not rospy.is_shutdown():
        # object_position = [0.3, 0.5]

        ## place the robot in the home pose
        controller.robot_at_homePose()

        '''
        ## scan the scene
        x, y= controller.object_detector()
        object_position = [x, y]
        print(f"detected obj pose is: ({x}, {y})")
        ## place the obj in the same place as you picked it
        controller.pick_and_place_obj(object_position, object_position)
        '''




        #call the function to set the position to "zero_pose"
        # arm.set_pose("home")
        # x, y = arm.object_detector()
        # #Wait for 2 seconds
        # rospy.sleep(2)
        # #Open the gripper or end effector
        # # hand.set_pose("open")
        # # rospy.sleep(1)

        # # arm.set_pose_values([0.5,0.2,0.3])
        # arm.set_joint_values(arm.TABLE_ROBOT_JOINTCONFIG)
        # rospy.sleep(1)



        ### search for objects
        # x, y = arm.object_detector()
        # print(f"in loopx:{type(x)},y:{y} ")
        # y = 0.5
        # x = 0.3
        # arm.set_pose_values([x, y, 0.05])
        # rospy.sleep(1)

        # arm.set_pose_values([x, y, 0.03])
        # rospy.sleep(1)

        # #Open the gripper or end effector
        # hand.set_pose("closed")
        # rospy.sleep(1)

    #delete the arm object at the end of code
    # del arm
    # del hand
	


if __name__ == '__main__':
    main()


