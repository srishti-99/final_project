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
from rrt.msg import PointArray, Obstacle, PointForRRT
from rrt.srv import CreateObstacle, FollowPath, RunRRT, FollowPathResponse, CreateObstacleRequest, RunRRTRequest
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from tf.transformations import quaternion_matrix, euler_from_quaternion
from math import atan2
from everything import get_next_target

radius = 0.1 + 0.177 + 0.1

K1 = 1
K2 = 1

# radius = #radius around the block's position which is acceptable

pos_epsilon_error = 0.05 #set error value
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

def in_contact_with_block():

  current_rob_pos, current_quaternion = get_current_rob_pos()
  current_block_pos = get_current_block_pos()

  x_diff_2 = current_block_pos.x - current_rob_pos.x
  y_diff_2 = current_block_pos.y - current_rob_pos.y
  block_to_rob_dist = np.sqrt(x_diff_2**2 + y_diff_2**2)

  if block_to_rob_dist <= radius:
    return True
  else:
    return False

# returns the current block POSITION
def get_current_block_pos():
  rospy.wait_for_service('/gazebo/get_model_state')
  get_model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
  object_coordinates = get_model_coordinates("cyl_block", "/map")
  return object_coordinates.pose.position

# return the current robot POSITION, and quaternion
def get_current_rob_pos():
  # tfListener = tf.TransformListener()
  # tfListener.waitForTransform(fixed_frame, robot_frame, rospy.Time(), rospy.Duration(4.0))
  # trans, rot = tfListener.lookupTransform(fixed_frame, robot_frame, rospy.Time()) 

  # current_rob_pos = Point()
  # current_rob_pos.x = trans[0]
  # current_rob_pos.y = trans[1]
  # current_rob_pos.z = trans[2]

  rospy.wait_for_service('/gazebo/get_model_state')
  get_rob_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
  current_rob_info = get_rob_coordinates("mobile_base", "/map")
  current_rob_pos = current_rob_info.pose.position
  current_quaternion = current_rob_info.pose.orientation

  return current_rob_pos, current_quaternion

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

  # poses = [Pose(Point(-3, 1.5, 0), Quaternion(x=0, y=0, z=1, w=np.pi))] #message.poses
  should_be_in_contact = False
  target_points = message.points.points # [Point(-3, 1.5, 0)]
  for i in range(len(target_points)):
    print("ENTERING ITERATION ", i)
    target_point = target_points[i]
    #reachedOrientation = False
    reachedPosition = False
    old_target_point = None
    if i == len(target_points) - 1: #if we're on the last point
      should_be_in_contact = True

    while not reachedPosition:

      reachedPosition = reach_orientation(target_point)
      if reachedPosition:
        # if old_target_point != None:
        #   print("Are you entering here?")
        #   target_point = old_target_point
        #   old_target_point = None
        #   reachedPosition = False
        #   should_be_in_contact = True
        continue
        #else:
        #  break

      if not should_be_in_contact:
        # run normal algorithm
        reachedPosition = run_position(target_point)
        if reachedPosition:
          # if old_target_point != None:
          #   target_point = old_target_point
          #   old_target_point = None
          #   reachedPosition = False
          #   should_be_in_contact = True
          continue
          #else:
          #  break
      else: # push block
        while in_contact_with_block():
          print("IN CONTACT WITH BLOCK")
          if run_position(target_point):
            reachedPosition = True
            break


        # now go behind block
        if not in_contact_with_block():
          print("Reached here")
          #old_target_point = target_point
          tp, kya = get_next_target(get_current_block_pos(), target_points[i]) # get point right behind block
          
          # call RRT service to get path to target point
          rospy.wait_for_service("run_rrt")
          rrt_backtrack_runner = rospy.ServiceProxy("run_rrt", RunRRT)
          block_obstacle = Obstacle()
          block_obstacle.is_obj_to_move = 1
          block_obstacle.pose = Pose()
          block_obstacle.pose.position = get_current_block_pos()
          block_obstacle.pose.orientation = Quaternion(0,0,0,1)
          block_obstacle.dim.x = 2*0.1
          block_obstacle.dim.y = 2*0.1
          block_obstacle.dim.z = 2*0.1
          #request.ob_in = block_obstacle
          rospy.wait_for_service("create_obstacle")
          obstacle_creator = rospy.ServiceProxy("create_obstacle", CreateObstacle)
          block_resp = obstacle_creator(CreateObstacleRequest(block_obstacle))
          block_obs = block_resp.ob_out
          print("Finished creating block obstacle")

          rrt_rob_path_find = PointForRRT()
          rrt_rob_path_find.start, current_quaternion = get_current_rob_pos()
          rrt_rob_path_find.target = tp
          rrt_rob_path_find.obstacles = [block_obs]
          rob_to_tgt_resp = rrt_backtrack_runner(RunRRTRequest(rrt_rob_path_find))
          rob_to_tgt_pnts = rob_to_tgt_resp.path_to_follow.points #now we have block's path to the target
          # call controller_simple to move along that path
          controller_simple(rob_to_tgt_pnts)

          # continue from before
          should_be_in_contact = False



      # if reach_orientation(target_point): # succeeds if it happened to reach position in the process of reaching orientation
      #   reachedPosition = True
      #   break
      # reachedOrientation = True
      # # now, move straight with 0 orientation (no angular velocity)
      # if not contact:
      #   if reachedPosition:
      #     break
      #   kj = 0
      #   reachedOrientation = False
      #   while kj < 5:
      #     kj+=1
      #     if (run_position(target_point) == 1):
      #       reachedPosition = True
      #       break

      # else: 
      #   still_in_contact = True
      #   while not reachedPosition:
      #     if not reachedOrientation: 
      #       if reach_orientation(target_point): 
      #         reachedPosition = True
      #         break
      #       reachedOrientation = True

      #     while still_in_contact:
      #       ended_run_pos = run_position(target_point)
      #       if (ended_run_pos == 1):
      #         reachedPosition = True
      #         break

      #       elif (ended_run_pos == -1):
      #         still_in_contact = False
      #         break

      #     while not still_in_contact:
      #       #bring back to contact
      #       tfListener.waitForTransform(fixed_frame, robot_frame, rospy.Time(), rospy.Duration(4.0))
      #       trans, rot = tfListener.lookupTransform(fixed_frame, robot_frame, rospy.Time()) 

      #       current_point = Point()
      #       current_point.x = trans[0]
      #       current_point.y = trans[1]          
      #       current_point.z = trans[2]

      #       rospy.wait_for_service('/gazebo/get_model_state')
      #       get_model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
      #       object_coordinates = get_model_coordinates("cyl_block", "/map")
      #       current_block_pos = object_coordinates.pose
      #       x_diff_2 = current_block_pos.position.x - current_point.x
      #       y_diff_2 = current_block_pos.position.y - current_point.y
      #       block_to_rob_dist = np.sqrt(x_diff_2**2 + y_diff_2**2)

      #       if block_to_rob_dist <= radius:
      #         still_in_contact = True
      #         reachedOrientation = False
      #         break

      #       tgt, kya = get_next_target(current_block_pos.position, target_point)
      #       reachedtgt = False 

      #       #reached_ang = False

      #       while not reachedtgt:
      #         if reach_orientation(tgt):
      #           reachedtgt = True
      #           still_in_contact = True
      #           reachedOrientation = False
      #           #reached_ang = True
      #           break

      #         if reachedtgt:
      #           still_in_contact = True
      #           reachedOrientation = False
      #           break

      #         ctr = 0
      #         while ctr < 5:
      #           ctr+=1
      #           if (run_position(target_point) == 1):
      #             reachedtgt = True
      #             still_in_contact = True
      #             reachedOrientation = False
      #             break

      #   #################################### end your code ###############

      #   pub.publish(control_command)

      # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      #   reached = True #move on to next target? 

      # # Use our rate object to sleep until it is time to publish again
      # r.sleep()

  robot_final_position, current_quaternion = get_current_rob_pos()

  return FollowPathResponse(robot_final_position)


def controller_simple(list_of_points):
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
  target_points = list_of_points # [Point(-3, 1.5, 0)]
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

        goal_angle = atan2(y_diff, x_diff)
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


def run_position(target_point): 
  #returns False if not reached position and completed run 
  #returns True if reached position 
  pub = rospy.Publisher('teleop_velocity_smoother/raw_cmd_vel', Twist, queue_size=10)

  # Create a timer object that will sleep long enough to result in
  # a 10Hz publishing rate
  freq2 = 10
  r2 = rospy.Rate(freq2) #freq2 Hz

  for i in range(5):
    current_point, current_quaternion = get_current_rob_pos()

    # given a point to move to, first orient in the direction of travel (no linear velocity)
    x_diff = target_point.x - current_point.x
    y_diff = target_point.y - current_point.y

    euclidean_dist = np.sqrt(x_diff**2 + y_diff**2)

    if euclidean_dist < pos_epsilon_error or (np.abs(x_diff) < x_diff_error and np.abs(y_diff) < y_diff_error):
      pub.publish(zero_cmd)
      r2.sleep()
      return True

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
  return False



def reach_orientation(target_point):
  pub = rospy.Publisher('teleop_velocity_smoother/raw_cmd_vel', Twist, queue_size=10)
  #tfBuffer = tf.Buffer()
  tfListener = tf.TransformListener()

  # Create a timer object that will sleep long enough to result in
  # a 10Hz publishing rate
  freq = 2
  r = rospy.Rate(freq) # freq hz
  freq2 = 10
  r2 = rospy.Rate(freq2)

  reachedOrientation = False 
  fixed_frame = '/map'
  robot_frame = '/base_link'
  blah = 0
  while not reachedOrientation:
    print("Orientation iteration ", blah)
    blah += 1

    current_point, current_quaternion = get_current_rob_pos()
    current_quaternion = [current_quaternion.x, current_quaternion.y, current_quaternion.z, current_quaternion.w]
    current_euler = euler_from_quaternion(current_quaternion)

    # given a point to move to, first orient in the direction of travel (no linear velocity)
    x_diff = target_point.x - current_point.x
    y_diff = target_point.y - current_point.y

    euclidean_dist = np.sqrt(x_diff**2 + y_diff**2)

    goal_angle = atan2(y_diff, x_diff) #np.arctan(y_diff / x_diff)
    change_in_angle = -current_euler[2] + goal_angle

    # print("Goal angle is ", goal_angle)
    # print("Change in angle is ", change_in_angle)
    # print("x_diff is ", x_diff)
    # print("y_diff is ", y_diff)
    # print("Current quaternion is ", current_quaternion)
    # print("Current euler is ", current_euler[2])

    if euclidean_dist < pos_epsilon_error or (np.abs(x_diff) < x_diff_error and np.abs(y_diff) < y_diff_error):
      reachedPosition = True
      pub.publish(zero_cmd)
      r2.sleep()
      print("RETURNING TRUE, with target point ", target_point)
      return True

    if np.abs(change_in_angle) < orientation_epsilon_error:
      reachedOrientation = True
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
  return False


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
    s = rospy.Service("follow_path", FollowPath, controller)
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