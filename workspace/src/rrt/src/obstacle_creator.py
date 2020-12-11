#!/usr/bin/env python
import rospy, rospkg
import tf
import sys
import numpy as np
from geometry_msgs.msg import Twist, PoseArray, Pose, Quaternion, Point, Vector3
from rrt.msg import PointArray, Obstacle, PointForRRT
from rrt.srv import CreateObstacle, FollowPath, RunRRT
from tf.transformations import quaternion_matrix, euler_from_quaternion
import math

radius_of_robot = 0.177 #set based on model file
radius_of_block = 0.1 #set based on model file
k = 1 #2 

def load_gazebo_models(blockPose=Pose(position=Point(x=-3.5, y=1.5, z=0)),
						blockRefFrame="map",):

	# Get models' paths
	model_path = rospkg.RosPack().get_path('starter')+"/models/"

	# Load Block SDF
	block_xml = ''
	with open (model_path + "blocks_ob/model.sdf", "r") as block_file:
		block_xml=block_file.read().replace('\n', '')

	# Load Target SDF
	#target_xml = ''
	#with open (model_path + "target/model.sdf", "r") as target_file:
	#	target_xml=target_file.read().replace('\n', '')block

	# Spawn Block SDF
	rospy.wait_for_service('/gazebo/spawn_sdf_model')
	try:
		spawn_block_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
		block_resp_sdf = spawn_block_sdf("cyl_block", block_xml, "/",
							blockPose, blockRefFrame)
	except rospy.ServiceException, e:
		rospy.logerr("Spawn SDF service call failed: {0}".format(e))

def callback(message): 
	#create an obstacle at position obstacle.pose, with dimensions obstacle.dim 
	#if the obstacle is our moving object then make it a different colour 
	#return an obstacle message with buffer values added to the dimensions 
	obstacle = message.ob_out
	if not obstacle.is_obj_to_move: 
		#load_gazebo_models(blockPose=obstacle.pose)
		obstacle.dim.x += k * radius_of_block
		obstacle.dim.y += k * radius_of_block

	obstacle.dim.x += k * radius_of_robot
	obstacle.dim.y += k * radius_of_robot

	return obstacle

def listener():
	s = rospy.Service("create_obstacle", CreateObstacle, callback)
	rospy.spin()

	  
# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  
  rospy.init_node('obstacle_creator', anonymous=True)

  listener()