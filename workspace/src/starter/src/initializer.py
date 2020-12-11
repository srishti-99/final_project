#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import rospkg

#Import the Point message type from the /msg directory of
#the geometry_msgs package.
from geometry_msgs.msg import Point, Pose, Quaternion, PoseWithCovarianceStamped, PoseWithCovariance
from std_msgs.msg import Header
from gazebo_msgs.srv import SpawnModel, DeleteModel
from rrt.msg import PointArray

#Loads only the target and block gazebo models (launch file loads robot)
def load_gazebo_models(blockPose=Pose(position=Point(x=-3.5, y=1.5, z=0)),
						blockRefFrame="map",
						targetPose=Pose(position=Point(x=-2.5, y=-3.5, z=0)),
						targetRefFrame="map"):

	#Publish to /initialpose topic, so that rviz and gazebo match up
	initPosePub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)

	initPoseMsg = PoseWithCovarianceStamped()
	initPoseMsg.header = Header()
	initPoseMsg.pose = PoseWithCovariance()

	initPoseMsg.header.seq = 0
	initPoseMsg.header.stamp = rospy.Time.now()
	initPoseMsg.header.frame_id = "robot_description"

	initPoseMsg.pose.pose = Pose()
	initPoseMsg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
	initPoseMsg.pose.pose.position = Point()
	initPoseMsg.pose.pose.orientation = Quaternion()

	# TODO: change to inputted robot starting positions later
	initPoseMsg.pose.pose.position.x = -4.0
	initPoseMsg.pose.pose.position.y = 4.0
	initPoseMsg.pose.pose.position.z = 0
	initPoseMsg.pose.pose.orientation.x = 0
	initPoseMsg.pose.pose.orientation.y = 0
	initPoseMsg.pose.pose.orientation.z = 0
	initPoseMsg.pose.pose.orientation.w = 0

	#rate = rospy.Rate(1)
	initPosePub.publish(initPoseMsg)

	# Get models' paths
	model_path = rospkg.RosPack().get_path('starter')+"/models/"

	# Load Block SDF
	block_xml = ''
	with open (model_path + "cyl_ob/model.sdf", "r") as block_file:
		block_xml=block_file.read().replace('\n', '')

	# Load Target SDF
	#target_xml = ''
	#with open (model_path + "target/model.sdf", "r") as target_file:
	#	target_xml=target_file.read().replace('\n', '')

	# Spawn Block SDF
	rospy.wait_for_service('/gazebo/spawn_sdf_model')
	try:
		spawn_block_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
		block_resp_sdf = spawn_block_sdf("cyl_block", block_xml, "/",
							blockPose, blockRefFrame)
	except rospy.ServiceException, e:
		rospy.logerr("Spawn SDF service call failed: {0}".format(e))

	# Spawn Target SDF
	#rospy.wait_for_service('/gazebo/spawn_sdf_model')
	#try:
	#	spawn_target_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
	#	target_resp_sdf = spawn_target_sdf("target", target_xml, "/",
	#						targetPose, targetRefFrame)

	#except rospy.ServiceException, e:
	#	rospy.logerr("Spawn SDF Service call failed: {0}".format(e))

#Define the method which contains the main functionality of the node.
def initializer():

	#Create an instance of the rospy.Publisher object which we can 
	#use to publish messages to a topic. This publisher publishes 
	#messages of type geometry_msgs/Point to the topic /rob_init
	robInitPub = rospy.Publisher('rob_init', Point, queue_size=10)

	#Create instances of publishers for the /block_init topic
	#and the /target_init topics
	blockInitPub = rospy.Publisher('block_init', Point, queue_size=10)
	targetInitPub = rospy.Publisher('target_init', Point, queue_size=10)
	allInitPub = rospy.Publisher('rob_block_target', PointArray, queue_size=10)

	#raw_input takes in a user prompt to initialize positions
	#for the robot, block, and target, then publish to corresponding topics
	robPosInit = Point()
	robPosInit.x = float(raw_input("Robot starting x: "))
	robPosInit.y = float(raw_input("Robot starting y: "))
	robPosInit.z = 0
	robInitPub.publish(robPosInit)
	
	blockPosInit = Point()
	blockPosInit.x = float(raw_input("Block starting x: "))
	blockPosInit.y = float(raw_input("Block starting y: "))
	blockPosInit.z = 0
	blockInitPub.publish(blockPosInit)

	targetPosInit = Point()
	targetPosInit.x = float(raw_input("Target starting x: "))
	targetPosInit.y = float(raw_input("Target starting y: "))
	targetPosInit.z = 0
	targetInitPub.publish(targetPosInit)

	allInit = PointArray()
	allInit.points = [robPosInit, blockPosInit, targetPosInit]
	allInitPub.publish(allInit)

	#Load target and block gazebo modelstargetPosInit
	load_gazebo_models()

	# Loop until the node is killed with Ctrl-C
	#while not rospy.is_shutdown():
		# Construct a string that we want to publish
		# (In Python, the "%" operator functions similarly
		#  to sprintf in C or MATLAB)
	  
# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':

	#Run this program as a new node in the ROS computation graph 
	#called /initializer.
	rospy.init_node('initializer', anonymous=True)

	# Check if the node has received a signal to shut down
	# If not, run the initializer method
	try:
		initializer()
	except rospy.ROSInterruptException: pass

