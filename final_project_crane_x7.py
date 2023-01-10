#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import actionlib
import math
import random
from geometry_msgs.msg import Point, Pose
from gazebo_msgs.msg import ModelStates
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from tf.transformations import quaternion_from_euler, euler_from_quaternion

gazebo_model_states = ModelStates()

def callback(msg):
    global gazebo_model_states
    gazebo_model_states = msg


def yaw_of(object_orientation):
    # convert quaternion to euler angles and return yaw angle
    euler = euler_from_quaternion(
        (object_orientation.x, object_orientation.y,
        object_orientation.z, object_orientation.w))

    return euler[2]


def main():
    global gazebo_model_states
    
    # the name of the object to grab
    OBJECT_NAME_LIST=["wood_cube_5cm","wood_cube_5cm_clone","wood_cube_5cm_clone_clone"]
    
    # hand opening/closing angle when grabbing
    GRIPPER_OPEN = 1.2   
    # hand opening/closing angle during installation
    GRIPPER_CLOSE = 0.42           
    # hand height when approaching
    APPROACH_Z = 0.15             
    # hand height when grabbing               
    PICK_Z = 0.12  
    # hand height when leaving
    LEAVE_Z = 0.15                

   
    sub_model_states = rospy.Subscriber("gazebo/model_states", ModelStates, callback, queue_size=1)
    
    # control arm group
    arm = moveit_commander.MoveGroupCommander("arm")
    # set the velocity
    arm.set_max_velocity_scaling_factor(0.4) 
    # set the acceleration
    arm.set_max_acceleration_scaling_factor(1.0) 
    # hand initialization
    gripper = actionlib.SimpleActionClient("crane_x7/gripper_controller/gripper_cmd", GripperCommandAction)
    gripper.wait_for_server()
    gripper_goal = GripperCommandGoal()
    gripper_goal.command.max_effort = 4.0

    rospy.sleep(1.0) 

    ########################################################################
    #############################  first cube  #############################
    ########################################################################
    
    OBJECT_NAME=OBJECT_NAME_LIST[0]
    print(OBJECT_NAME)

    # open your hand for when you were holding something
    gripper_goal.command.position = GRIPPER_OPEN
    gripper.send_goal(gripper_goal)
    gripper.wait_for_result(rospy.Duration(1.0))
    
    # use the "home" posture defined in SRDF
    arm.set_named_target("home")
    arm.go()
    rospy.sleep(1.0)
    
    # wait a certain amount of time
    sleep_time = 3.0
    print("Wait " + str(sleep_time) + " secs.")
    rospy.sleep(sleep_time)
    print("Start")
    
    # if object is on gazebo, do pick_and_place
    if OBJECT_NAME in gazebo_model_states.name:
        object_index = gazebo_model_states.name.index(OBJECT_NAME)
        # get object pose
        object_position = gazebo_model_states.pose[object_index].position
        object_orientation = gazebo_model_states.pose[object_index].orientation
        object_yaw = yaw_of(object_orientation) 

    # approach the object
    target_pose = Pose()
    target_pose.position.x = object_position.x
    target_pose.position.y = object_position.y
    target_pose.position.z = APPROACH_Z
    q = quaternion_from_euler(-math.pi, 0.0, object_yaw)
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]

    # go grab
    target_pose.position.z = PICK_Z
    arm.set_pose_target(target_pose)
    if arm.go() is False:
        print("Failed to grip an object.")
    rospy.sleep(1.0)
    gripper_goal.command.position = GRIPPER_CLOSE
    gripper.send_goal(gripper_goal)
    gripper.wait_for_result(rospy.Duration(1.0))

    # lift 
    target_pose.position.z = LEAVE_Z
    arm.set_pose_target(target_pose)
    if arm.go() is False:
        print("Failed to pick up an object.")
    rospy.sleep(1.0) 

    # move to installation position
    target_pose.position.x = 0.4
    target_pose.position.y = 0.0
    target_pose.position.z = 0.15
    q = quaternion_from_euler(-math.pi, 0.0, -math.pi/2.0)
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]

    # set up
    target_pose.position.z = PICK_Z
    arm.set_pose_target(target_pose)
    if arm.go() is False:
        print("Failed to place an object.")
    rospy.sleep(1.0)
    gripper_goal.command.position = GRIPPER_OPEN
    gripper.send_goal(gripper_goal)
    gripper.wait_for_result(rospy.Duration(1.0))
    
    # raise hand
    target_pose.position.z = LEAVE_Z
    arm.set_pose_target(target_pose)
    if arm.go() is False:
        print("Failed to leave from an object.")
    rospy.sleep(1.0)  
    
    ############################################################################
    #############################   second cube   ##############################
    ############################################################################

    OBJECT_NAME=OBJECT_NAME_LIST[1]
    print(OBJECT_NAME) 
    
    # open your hand for when you were holding something
    gripper_goal.command.position = GRIPPER_OPEN
    gripper.send_goal(gripper_goal)
    gripper.wait_for_result(rospy.Duration(1.0))
    
    # use the "home" posture defined in SRDF
    arm.set_named_target("home")
    arm.go()
    rospy.sleep(1.0)
    
    # wait a certain amount of time
    sleep_time = 3.0
    print("Wait " + str(sleep_time) + " secs.")
    rospy.sleep(sleep_time)
    print("Start") 
    
    # if object is on gazebo, do pick_and_place
    if OBJECT_NAME in gazebo_model_states.name:
        object_index = gazebo_model_states.name.index(OBJECT_NAME)
        # get object pose
        object_position = gazebo_model_states.pose[object_index].position
        object_orientation = gazebo_model_states.pose[object_index].orientation
        object_yaw = yaw_of(object_orientation) 
    
    # approach the object
    target_pose = Pose()
    target_pose.position.x = object_position.x
    target_pose.position.y = object_position.y
    target_pose.position.z = APPROACH_Z
    q = quaternion_from_euler(-math.pi, 0.0, object_yaw)
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3] 
    
    # go grab
    target_pose.position.z = PICK_Z
    arm.set_pose_target(target_pose)
    if arm.go() is False:
        print("Failed to grip an object.")
    rospy.sleep(1.0)
    gripper_goal.command.position = GRIPPER_CLOSE
    gripper.send_goal(gripper_goal)
    gripper.wait_for_result(rospy.Duration(1.0))
    
    # lift (ridica)
    target_pose.position.z = LEAVE_Z
    arm.set_pose_target(target_pose)
    if arm.go() is False:
        print("Failed to pick up an object.")
    rospy.sleep(1.0) 

    # move to installation position
    target_pose.position.x = 0.4
    target_pose.position.y = 0.0
    target_pose.position.z = 0.15
    q = quaternion_from_euler(-math.pi, 0.0, -math.pi/2.0)
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3] 

    # hand height when grabbing               
    PICK_Z_2= 0.18 

    # set up
    target_pose.position.z = PICK_Z_2
    arm.set_pose_target(target_pose)
    if arm.go() is False:
        print("Failed to place an object.")
    rospy.sleep(1.0)
    gripper_goal.command.position = GRIPPER_OPEN
    gripper.send_goal(gripper_goal)
    gripper.wait_for_result(rospy.Duration(1.0))
    
    # hand height when leaving
    LEAVE_Z_2 = 0.19

    # raise hand
    target_pose.position.z = LEAVE_Z_2
    arm.set_pose_target(target_pose)
    if arm.go() is False:
        print("Failed to leave from an object.")
    rospy.sleep(1.0)  

    ###########################################################################
    ###############################  third cube  ##############################
    ###########################################################################

    OBJECT_NAME=OBJECT_NAME_LIST[2]
    print(OBJECT_NAME) 
    
    # open your hand for when you were holding something
    gripper_goal.command.position = GRIPPER_OPEN
    gripper.send_goal(gripper_goal)
    gripper.wait_for_result(rospy.Duration(1.0))
    
    # use the "home" posture defined in SRDF
    arm.set_named_target("home")
    arm.go()
    rospy.sleep(1.0)
    
    # wait a certain amount of time
    sleep_time = 3.0
    print("Wait " + str(sleep_time) + " secs.")
    rospy.sleep(sleep_time)
    print("Start") 
    
    # if object is on gazebo, do pick_and_place
    if OBJECT_NAME in gazebo_model_states.name:
        object_index = gazebo_model_states.name.index(OBJECT_NAME)
        # get object pose
        object_position = gazebo_model_states.pose[object_index].position
        object_orientation = gazebo_model_states.pose[object_index].orientation
        object_yaw = yaw_of(object_orientation) 

    # approach the object
    target_pose = Pose()
    target_pose.position.x = object_position.x
    target_pose.position.y = object_position.y
    target_pose.position.z = APPROACH_Z
    q = quaternion_from_euler(-math.pi, 0.0, object_yaw)
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3] 
    
    # go grab
    target_pose.position.z = PICK_Z
    arm.set_pose_target(target_pose)
    if arm.go() is False:
        print("Failed to grip an object.")
    rospy.sleep(1.0)
    gripper_goal.command.position = GRIPPER_CLOSE
    gripper.send_goal(gripper_goal)
    gripper.wait_for_result(rospy.Duration(1.0))
    
    # lift 
    target_pose.position.z = LEAVE_Z
    arm.set_pose_target(target_pose)
    if arm.go() is False:
        print("Failed to pick up an object.")
    rospy.sleep(1.0) 
    
    # move to installation position
    target_pose.position.x = 0.4
    target_pose.position.y = 0.0
    target_pose.position.z = 0.15
    q = quaternion_from_euler(-math.pi, 0.0, -math.pi/2)
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3] 
    
    # hand height when grabbing               
    PICK_Z_3 = 0.24 

    # set up
    target_pose.position.z = PICK_Z_3
    arm.set_pose_target(target_pose)
    if arm.go() is False:
        print("Failed to place an object.")
    rospy.sleep(1.0)
    gripper_goal.command.position = GRIPPER_OPEN
    gripper.send_goal(gripper_goal)
    gripper.wait_for_result(rospy.Duration(1.0))
    
    # hand height when leaving
    LEAVE_Z_3 = 0.24  

    # raise hand
    target_pose.position.z = LEAVE_Z_3
    arm.set_pose_target(target_pose)
    if arm.go() is False:
        print("Failed to leave from an object.")
    rospy.sleep(1.0) 
    
    # returns to the "home" posture defined in SRDF
    arm.set_named_target("home")
    arm.go()
            

if __name__ == '__main__':
    rospy.init_node("pick_and_place_in_gazebo_example")

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
