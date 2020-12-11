#!/usr/bin/env python
import rospy
import tf
import sys
import numpy as np
from geometry_msgs.msg import Twist, PoseArray, Pose, Quaternion, Point, Vector3
from rrt.msg import PointArray, Obstacle, PointForRRT
from rrt.srv import CreateObstacle, FollowPath, RunRRT
from tf.transformations import quaternion_matrix, euler_from_quaternion
import math

radius_of_block = 0.177 #set based on model file 
radius_of_robot = 0.1 #set based on model file

#distance of robots origin from blocks origin once it reorients 
dist_rob_block = radius_of_block + radius_of_robot + 0.1 # 0.1 margin of error

#For each node of that path:

# Now, based on the orientation of the first point of the object's path, 
	#find out the target position for the robot next to the block

# Next generate a path from the robot to the point 
# re orient robot to face the block

#push the robot along the path 

#repeat 
def get_next_target(curr_block_pos, next_block_pos): 
	#takes in where the block currently is: Point()
	#takes in where the block needs to go next: Point()
	#returns where the robot should go and how it should orient itself
	#returns the next point that robot will need to reorient at 

	x_diff = next_block_pos.x - curr_block_pos.x
	y_diff = next_block_pos.y - curr_block_pos.y
	angle_block_must_move_in = math.atan2(y_diff / x_diff)

	target_pos = Point()
	target_pos.x = curr_block_pos.x + (dist_rob_block * np.cos(np.pi+angle_block_must_move_in))
	target_pos.y = curr_block_pos.y + (dist_rob_block * np.sin(np.pi+angle_block_must_move_in))
	# target_pos.x = curr_block_pos.x - (dist_rob_block * np.cos(angle_block_must_move_in))
	# target_pos.y = curr_block_pos.y - (dist_rob_block * np.sin(angle_block_must_move_in))
	
	target_pos.z = 0

	next_endpnt = Point()
	next_endpnt.x = next_block_pos.x + (dist_rob_block * np.cos(np.pi+angle_block_must_move_in))
	next_endpnt.y = next_block_pos.y + (dist_rob_block * np.sin(np.pi+angle_block_must_move_in))

	next_endpnt.z = 0

	return target_pos, next_endpnt

def path(message): 
	rob_pos = message.points[0]
	block_pos = message.points[1]
	target_pos = message.points[2]
	
	
	# request = CreateObstacleRequest()
	#block object Obstacle msg with buffer 
	block_obstacle = Obstacle()
	block_obstacle.is_obj_to_move = 1
	block_obstacle.pose = Pose()
	block_obstacle.pose.position = block_pos
	block_obstacle.pose.orientation = Quaternion(0,0,0,1)
	block_obstacle.dim.x = 2*radius_of_block
	block_obstacle.dim.y = 2*radius_of_block
	block_obstacle.dim.z = 2*radius_of_block
	#request.ob_in = block_obstacle
	rospy.wait_for_service("create_obstacle")
	try:
		obstacle_creator = rospy.ServiceProxy("create_obstacle", CreateObstacle)
		block_obs = obstacle_creator(block_obstacle)
	except rospy.ServiceException, e:
		rospy.logerr("Spawn SDF service call failed: {0}".format(e))

	# First, generate a path from the block to the target
	rrt_block_path_find = PointForRRT()
	rrt_block_path_find.start = block_pos 
	rrt_block_path_find.target = target_pos
	rrt_block_path_find.obstacles = []

	
	rospy.wait_for_service("run_rrt")
	try:
		rrt_runner = rospy.ServiceProxy("run_rrt", RunRRT)
		block_to_tgt_resp = rrt_runner(rrt_block_path_find)

	except rospy.ServiceException, e:
		rospy.logerr("Spawn SDF service call failed: {0}".format(e))

	block_to_tgt_pnts = block_to_tgt_resp.points #now we have block's path to the target

	for i in range(len(block_to_tgt_pnts) - 1):
		#find target for robot's reorientation
		robs_nxt_tgt, robs_nxt_endpnt = get_next_target(block_to_tgt_pnts[i], block_to_tgt_pnts[i + 1])
		# find path from robots current position to ^target
		rrt_reorient_path_find = PointForRRT()
		rrt_reorient_path_find.start = robs_pos
		rrt_reorient_path_find.target = robs_nxt_tgt
		block_obstacle.pose.position = block_to_tgt_pnts[i] #update this to check for blocks actual position
		rrt_reorient_path_find.obstacles = [block_obs]

		rospy.wait_for_service("run_rrt")
		try:
			rrt_runner = rospy.ServiceProxy("run_rrt", RunRRT)
			rob_reorient_resp = rrt_runner(rrt_reorient_path_find)
		except rospy.ServiceException, e:
			rospy.logerr("Spawn SDF service call failed: {0}".format(e))

		rob_reorient_resp.points.append(robs_nxt_endpnt)

		#move robot + block to the next point ! 
		rospy.wait_for_service("follow_path")
		try:
			controller_runner = rospy.ServiceProxy("follow_path", FollowPath)
			robs_pos = controller_runner(rob_reorient_resp)
		except rospy.ServiceException, e:
			rospy.logerr("Spawn SDF service call failed: {0}".format(e))


def listener_init():
	#Getting initial positions of robot, block and target
	rospy.Subscriber('rob_block_target', PointArray, path)
	rospy.spin()

	  
# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method
  #Run this program as a new node in the ROS computation graph 
  #called /turtlebot_controller.
  rospy.init_node('everything', anonymous=True)
  #controller()

	# First, generate a path from the object to the goal
  listener_init()

#For each node of that path:

# Now, based on the orientation of the first point of the object's path, 
	#find out the target position for the robot next to the block

# Next generate a path from the robot to the point 
# re orient robot to face the block

#push the robot along the path 

#repeat 