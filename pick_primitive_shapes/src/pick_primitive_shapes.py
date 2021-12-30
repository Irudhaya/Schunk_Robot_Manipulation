#! /usr/bin/env python

import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from manipulation_msgs.msg import ObjectData
import tf
from tf.transformations import quaternion_from_euler
import time

def data_cb(msg):

	global height,objects
	height = msg.height
	objects = msg.objects

def pose_tf(name):

	source_frame = "/"+name
	target_frame = "/world"
	time = rospy.Time()

	t = tf.TransformListener()
	t.waitForTransform(target_frame,source_frame,time,rospy.Duration(5))
	object_t, object_r = t.lookupTransform(target_frame, source_frame, time)
	#print(object_t)
	#print(object_r)
	return (object_t,object_r)

def compute_poses(pose,size=0):
	
	poses=[]
	translation, rotation = pose
	pose1 = Pose()
	#pre-grasp pose assuming execution of a vertical grasp
	gripper_collision_clearance = 0.095 + 0.0789 + 0.2 #(pg70_palm, eef_pose_link, additional clearance)
	pose1.position.z = translation[2] + size  + gripper_collision_clearance  #(z_object_pose,height)

	rospy.loginfo("Pose of the arm_6_link in z: %f",pose1.position.z)
	#arm_orientation = [0.0,0.0,0.0,1.0]

	

	pose1.position.x, pose1.position.y,  = translation[0],translation[1]
	

	#assuming fixed orientation of the arm w.r.t world to enable vertical grasping 
	#(arm_6_link orientation is considered)
	# z
	# |  /y		            /x	
	# | /		           /   (arm_6_link)   (default orientation for grasping)
	# |/-------->x       |/-------->y
	# (world frame)	     |
	# 		     		 |z	
	orientation1 = pose1.orientation
	#defining the rotation of arm_6_link w.r.t world frame
	orientation_default = quaternion_from_euler(3.1415927,0.0,1.5707963) 
	#orientation_default = quaternion_from_euler(0,0,0)

	
	orientation1.x,orientation1.y,orientation1.z,orientation1.w = orientation_default[0], orientation_default[1], orientation_default[2], orientation_default[3] 

	#print("pre-grasp pose")
	#print(pose)
	poses.append(pose1)
	#print("Poses",poses)

	pose2 = Pose()
	pose2.position.x,pose2.position.y = translation[0], translation[1] 
	pose2.position.z = translation[2] + size + 0.095 + 0.0789 + 0.005 #(actual position at z, centre of the object, eef_link,gripper_link)
	orientation2 = pose2.orientation
	orientation2.x, orientation2.y, orientation2.z, orientation2.w = orientation_default[0], orientation_default[1], orientation_default[2], orientation_default[3]
	rospy.loginfo("Pose of the arm_6_link in z: %f",pose2.position.z)

	#print("graasp pose")
	#print(pose2)
	poses.append(pose2)

	#print(poses)
	
	###### grasp - pose
	return poses

def grasp_execution(poses):

	global move_group, gripper, eef

	for pose in poses:
		
		move_group.set_start_state_to_current_state()

		rospy.loginfo("Pose received to move the arm")
		move_group.set_pose_target(pose ,end_effector_link=eef)
		executed = move_group.go(wait=True)	

		if executed:
			rospy.loginfo("Arm moved to the pre-grasp position")
		else:
			rospy.loginfo("Arm failed to execute the movement")
			return False
		

	rospy.loginfo("RObot ready to grasp")
	gripper.set_named_target("gripper_close")
	gripper.go()
	
	return True

def retreat_grasp(pose):

	global move_group, gripper, eef

	rospy.loginfo("Object grasped and now retreating with the object")

	move_group.set_pose_target(pose ,end_effector_link=eef)
	executed = move_group.go(wait=True)

	gripper.set_named_target("gripper_open")
	gripper.go()

	rospy.loginfo("Grasping pipeline completed......")


def main():

	global object_name, move_group, gripper, eef

	rospy.init_node("object_manipulation")

	robot = moveit_commander.RobotCommander()
	group_names = robot.get_group_names()

	#arm group to plan
	move_group = moveit_commander.MoveGroupCommander(group_names[0]) 

	#object frame name (Cylinder object for the current world setup)
	object_name = rospy.get_param("/object_name","Cylinder") 

	rospy.loginfo("Robot set to grasp the object {}".format(object_name))
	rospy.loginfo("Waiting for the object data to compute the grasp pose....")
	object_data = rospy.wait_for_message("/object/data",ObjectData)

	move_group.set_named_target("home")
	move_group.go()
	
	#gripper group to control the opening and closing action of gripper
	gripper = moveit_commander.MoveGroupCommander(group_names[1])

	rospy.loginfo("Opening the gripper")
	gripper.set_named_target("gripper_open")
	gripper.go()

	goal_tolerance = 0.0005
	move_group.set_goal_position_tolerance(goal_tolerance)


	if move_group.has_end_effector_link():#check for the presence of end_effector in the group
		eef = move_group.get_end_effector_link()
		rospy.loginfo("Name of the end effector link %s",eef)

	while object_name not in object_data.name:
		object_data = rospy.wait_for_message("/object/data",ObjectData)

	if object_name in object_data.name:
		
		obj_id = object_data.name.index(object_name)
		obj_name = object_data.name[obj_id]
		obj_height =  object_data.height[obj_id]
		
		object_pose = pose_tf(object_name)
		grasp_poses = compute_poses(object_pose, size=obj_height)
		print(len(grasp_poses))
		object_grasped = grasp_execution(grasp_poses)

		if object_grasped:
			#same as pre-grasp pose
			retreat_grasp(grasp_poses[0])

		else:
			rospy.loginfo("Failed  to grasp the object")

	else:
		rospy.logwarn("No objects named %s exists in the scene",object_name)

main()
