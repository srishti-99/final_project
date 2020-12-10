#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import tf
import sys
import numpy as np
from geometry_msgs.msg import Twist, PoseArray, Pose, Quaternion, Point, Vector3
from rrt.msg import PointArray
from tf.transformations import quaternion_matrix, euler_from_quaternion
from math import atan2

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
  pub = rospy.Publisher('teleop_velocity_smoother/raw_cmd_vel', Twist, queue_size=10)
  #tfBuffer = tf.Buffer()
  tfListener = tf.TransformListener()

  
  # Create a timer object that will sleep long enough to result in
  # a 10Hz publishing rate
  freq = 2
  r = rospy.Rate(freq) # freq hz
  freq2 = 10
  r2 = rospy.Rate(freq2)

  K1 = 1
  K2 = 1

  pos_epsilon_error = 0.5 #set error value
  orientation_epsilon_error = 0.01
  x_diff_error = 0.05
  y_diff_error = 0.05

  zero_cmd = Twist()
  zero_cmd.linear = Vector3()
  zero_cmd.angular = Vector3()
  zero_cmd.linear.x = 0
  zero_cmd.linear.y = 0
  zero_cmd.linear.z = 0
  zero_cmd.angular.x = 0
  zero_cmd.angular.y = 0
  zero_cmd.angular.z = 0

  robot_frame = '/base_link' #TODO this needs to be a TF frame. I can't figure out how to create a TF frame and attach it to the gazebo turtlebot
  fixed_frame = '/map' #TODO this is currently the marker.header.frame_id from assignment.py. 

  # poses = [Pose(Point(-3, 1.5, 0), Quaternion(x=0, y=0, z=1, w=np.pi))] #message.poses
  target_points = message.points # [Point(-3, 1.5, 0)]
  for i in range(len(target_points)):
    target_point = target_points[i]
    reachedOrientation = False
    reachedPosition = False

    # Loop until the node is killed with Ctrl-C 
    blah = 0
    while not reachedPosition:
      while not reachedOrientation:
        print("Orientation iteration ", blah)
        blah += 1
    
        tfListener.waitForTransform(fixed_frame, robot_frame, rospy.Time(), rospy.Duration(4.0))
        trans, rot = tfListener.lookupTransform(fixed_frame, robot_frame, rospy.Time()) 
        
        #current_pose = Pose()
        current_point = Point()
        current_point.x = trans[0]
        current_point.y = trans[1]
        current_point.z = trans[2]
        print(rot)
        print(type(rot))
        #current_pose.position = current_point
        #current_quaternion = Quaternion()
        current_quaternion = rot
        #current_quaternion.x, current_quaternion.y, current_quaternion.z, current_quaternion.w = rot[0], rot[1], rot[2], rot[3]
        current_euler = euler_from_quaternion(rot)

        # given a point to move to, first orient in the direction of travel (no linear velocity)
        x_diff = target_point.x - current_point.x
        y_diff = target_point.y - current_point.y

        euclidean_dist = np.sqrt(x_diff**2 + y_diff**2)

        goal_angle = atan2(y_diff, x_diff) #np.arctan(y_diff / x_diff)
        change_in_angle = -current_euler[2] + goal_angle

        print("Goal angle is ", goal_angle)
        print("Change in angle is ", change_in_angle)
        print("x_diff is ", x_diff)
        print("y_diff is ", y_diff)
        print("Current quaternion is ", current_quaternion)
        print("Current euler is ", current_euler)

        if euclidean_dist < pos_epsilon_error or (np.abs(x_diff) < x_diff_error and np.abs(y_diff) < y_diff_error):
          reachedPosition = True
          pub.publish(zero_cmd)
          print("REACHED POSITION")
          r2.sleep()
          break

        if np.abs(change_in_angle) < orientation_epsilon_error:
          reachedOrientation = True
          print("REACHED ANGLE")
          pub.publish(zero_cmd)
          r.sleep()
          break

        cmd = Twist()
        cmd.linear = Vector3()
        cmd.angular = Vector3()

        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = change_in_angle # * freq * 2 #np.arctan(y/x) #y # theta_dot

        pub.publish(cmd)
        r.sleep()
        #pub.publish(cmd)
        #r.sleep()
        #pub.publish(cmd)
        #r.sleep()

        # now, move straight with 0 orientation (no angular velocity)

      if reachedPosition:
        break
      kj = 0
      reachedOrientation = False
      while kj < 10:

        print("Iteration ", kj)
        kj += 1

        tfListener.waitForTransform(robot_frame, fixed_frame, rospy.Time(), rospy.Duration(4.0))
        trans, rot = tfListener.lookupTransform(robot_frame, fixed_frame, rospy.Time()) 
        
        #current_pose = Pose()
        current_point = Point()
        current_point.x = trans[0]
        current_point.y = trans[1]
        current_point.z = trans[2]

        # given a point to move to, first orient in the direction of travel (no linear velocity)
        x_diff = target_point.x - current_point.x
        y_diff = target_point.y - current_point.y

        euclidean_dist = np.sqrt(x_diff**2 + y_diff**2)

        if euclidean_dist < pos_epsilon_error or (np.abs(x_diff) < x_diff_error and np.abs(y_diff) < y_diff_error):
          reachedPosition = True
          pub.publish(zero_cmd)
          print("REACHED POSITION")
          r2.sleep()
          break

        cmd2 = Twist()
        cmd2.linear = Vector3()
        cmd2.angular = Vector3()

        cmd2.linear.x = euclidean_dist
        cmd2.linear.y = 0.0
        cmd2.linear.z = 0.0
        cmd2.angular.x = 0.0
        cmd2.angular.y = 0.0
        cmd2.angular.z = 0.0

        pub.publish(cmd2)
        r2.sleep()



      # try:
      #   # Process trans to get your state error
      #   # Generate a control command to send to the robot
      #   x = target_pose.position.x - current_pose.position.x
      #   y = target_pose.position.y - current_pose.position.y


      #   x_dot = np.sqrt((x)**2 + (y)**2)

      #   if x_dot < epsilon_error:
      #     reached = True
      #     break

      #   theta_dot = target_pose.orientation.w - w[3]

      #   # x_dot = K1 * trans.transform.translation.x
      #   # theta_dot = K2 * trans.transform.translation.y
      #   print("X dot ", x_dot)
      #   print("Theta dot ", theta_dot)

      #   cmd = Twist()

      #   cmd.linear.x = K1 * x_dot
      #   cmd.linear.y = 0.0
      #   cmd.linear.z = 0.0
      #   cmd.angular.x = 0.0
      #   cmd.angular.y = 0.0
      #   cmd.angular.z = 1 #np.arctan(y/x) #y # theta_dot

      #   control_command = cmd

      #   #################################### end your code ###############

      #   pub.publish(control_command)

      # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      #   reached = True #move on to next target? 

      # # Use our rate object to sleep until it is time to publish again
      # r.sleep()


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
    rospy.Subscriber("path_points", PointArray, controller)



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
  #controller()

  listener()