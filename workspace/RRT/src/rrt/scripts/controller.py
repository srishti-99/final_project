#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import tf2_ros
import sys
import numpy as np
from geometry_msgs.msg import Twist, PoseArray, Pose, Quaternion
from tf.transformations import quaternion_matrix

#Define the method which contains the main functionality of the node.
def controller(message):
  """
  Controls a robot whose position is denoted by robot_frame,
  to go to a position denoted by target_frame
  Inputs:
  - robot_frame: the tf frame of the robot base.
  - target_frame: the tf frame of the desired position.
  """

  ################################### YOUR CODE HERE #############
  #Create a publisher and a tf buffer, which is primed with a tf listener
  #TODO: replace 'INPUT TOPIC' with the correct name for the ROS topic on which
  # the robot accepts velocity inputs.
  pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
  tfBuffer = tf2_ros.Buffer()
  tfListener = tf2_ros.TransformListener(tfBuffer)

  
  # Create a timer object that will sleep long enough to result in
  # a 10Hz publishing rate
  r = rospy.Rate(10) # 10hz

  K1 = 1
  K2 = 1

  epsilon_error = 0.5 #set error value 

  poses = message.poses
  for i in range(len(poses)):
    target_pose = poses[i]
    reached = False

    # Loop until the node is killed with Ctrl-C 
    while not reached: 
      robot_frame = '/odom' #TODO this needs to be a TF frame. I can't figure out how to create a TF frame and attach it to the gazebo turtlebot
      fixed_frame = 'map' #TODO this is currently the marker.header.frame_id from assignment.py. 
  
      trans = tfBuffer.lookup_transform(robot_frame, fixed_frame, rospy.Time()) 
      
      current_pose = Pose()
      current_point = Point()
      current_point.x = trans.transform.translation.x
      current_point.y = trans.transform.translation.y
      current_point.z = trans.transform.translation.z

      current_pose.position = current_point
      current_pose.orientation = trans.transform.rotation

      ###Another way to get current_pose MIGHT be to suscribe to the /odom topic. 
      ###I'm not sure how to subscribe to two topics. Maybe ^^ can be recieved at the 
      ###start of the while loop? 
      ###is it possible to just like "get" from a topic at an instance and then continue? idk..

      try:
        # Process trans to get your state error
        # Generate a control command to send to the robot
        relative_pose = target_pose - current_pose

        x_dot = np.sqrt((relative_pose.position.x)**2 + (relative_pose.position.y)**2)

        if x_dot < epsilon_error:
          reached = True
          break

        theta_dot = relative_pose.orientation.w 

        # x_dot = K1 * trans.transform.translation.x
        # theta_dot = K2 * trans.transform.translation.y
        print("X dot ", x_dot)
        print("Theta dot ", theta_dot)

        cmd = Twist()

        cmd.angular.x = 0.0
        cmd.linear.x = x_dot
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = theta_dot

        control_command = cmd

        #################################### end your code ###############

        pub.publish(control_command)

      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        reached = True #move on to next target? 

      # Use our rate object to sleep until it is time to publish again
      r.sleep()


def pose_to_trans(p):
    q = p.pose.orientation
    pos = p.pose.position
    T = quaternion_matrix([q.x,q.y,q.z,q.w])
    T[:3,3] = np.array([pos.x,pos.y,pos.z])
    return T

    

#Define the method which contains the node's main functionality
def listener():

    #Create a new instance of the rospy.Subscriber object which we can 
    #use to receive messages of type std_msgs/String from the topic /chatter_talk.
    #Whenever a new message is received, the method callback() will be called
    #with the received message as its first argument.
    rospy.Subscriber("path_points", PoseArray, controller)



    #Wait for messages to arrive on the subscribed topics, and exit the node
    #when it is killed with Ctrl+C
    rospy.spin()

      
# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method
  #Run this program as a new node in the ROS computation graph 
  #called /turtlebot_controller.
  rospy.init_node('turtlebot_controller', anonymous=True)

  listener()