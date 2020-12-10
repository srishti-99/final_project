#!/usr/bin/python2

from geometry_msgs.msg import Point, PoseArray, Pose, Quaternion
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from tree import Tree, TreeNode

import math
import random
import rospy
import numpy as np 

from rrt.msg import PointArray, PointForRRT

HZ = 5
STEP = 1.0

BOARD_CORNERS = [-5, 5, 5, -5]


def create_obstacles(obstacles):
	obs_list = []
	for i in range(len(obstacles)):
		ob = Marker()
		ob.type = ob.CUBE
		ob.header.frame_id = "map"
		ob.ns = "obstacles"
		ob.id = i
		ob.action = ob.ADD
		ob.scale.x, ob.scale.y, ob.scale.z = obstacles[i].dim.x, obstacles[i].dim.y, obstacles[i].dim.z
		ob.pose = obstacles[i].pose 
		ob.color.r, ob.color.g, ob.color.b, ob.color.a = 0.95, 0.95, 0.95, 1.0
		if ob.is_obj_to_move: 
			ob.color.r, ob.color.g, ob.color.b, ob.color.a = 0.5, 0.5, 0.5, 1.0
		ob.lifetime = rospy.Duration()
		ob.frame_locked = True 
		obs_list.append(ob)

	return obs_list


def create_robot(pos):
	rob = Marker()
	rob.type = rob.CUBE
	rob.header.frame_id = "map"
	rob.ns = "robot"
	rob.id = 0
	rob.action = rob.ADD
	rob.scale.x, rob.scale.y, rob.scale.z = 0.5, 0.5, 0.1
	rob.pose.position.x, rob.pose.position.y, rob.pose.position.z = pos.x, pos.y, pos.z
	rob.pose.orientation.w = 1.0
	rob.color.r, rob.color.g, rob.color.b, rob.color.a = 0.45, 0.45, 0.45, 1.0
	rob.lifetime = rospy.Duration()
	return rob


def create_target(pos):
	tgt = Marker()
	tgt.type = tgt.CUBE
	tgt.header.frame_id = "map"
	tgt.ns = "target"
	tgt.id = 0
	tgt.action = tgt.ADD
	tgt.scale.x, tgt.scale.y, tgt.scale.z = 1, 1, 0.1
	tgt.pose.position.x, tgt.pose.position.y, tgt.pose.position.z = pos.x, pos.y, pos.z
	tgt.pose.orientation.w = 1.0
	tgt.color.r, tgt.color.g, tgt.color.b, tgt.color.a = 1.00, 0.83, 0.45, 1.0
	tgt.lifetime = rospy.Duration()
	return tgt


def get_tree_edges_structure():
	edges = Marker()
	edges.type = edges.LINE_LIST
	edges.header.frame_id = "map"
	edges.ns = "tree_edges"
	edges.id = 0
	edges.action = edges.ADD
	edges.scale.x = 0.02
	edges.pose.orientation.w = 1.0
	edges.color.r, edges.color.g, edges.color.b, edges.color.a = 1.00, 0.95, 0.69, 1.00
	return edges


def get_point_structure():
	point = Marker()
	point.type = point.POINTS
	point.header.frame_id = "map"
	point.ns = "point"
	point.id = 0
	point.action = point.ADD
	point.scale.x, point.scale.y = 0.05, 0.05
	point.pose.orientation.w = 1.0
	point.color.r, point.color.g, point.color.b, point.color.a = 0.45, 0.31, 0.31, 1.00
	return point


def get_collision_edges_structure():
	edges = Marker()
	edges.type = edges.LINE_LIST
	edges.header.frame_id = "map"
	edges.ns = "collision_edges"
	edges.id = 0
	edges.action = edges.ADD
	edges.scale.x = 0.02
	edges.pose.orientation.w = 1.0
	edges.color.r, edges.color.g, edges.color.b, edges.color.a = 0.96, 0.67, 0.67, 1.00
	return edges


def get_path_edges_structure():
	edges = Marker()
	edges.type = edges.LINE_LIST
	edges.header.frame_id = "map"
	edges.ns = "path_edges"
	edges.id = 0
	edges.action = edges.ADD
	edges.scale.x = 0.06
	edges.pose.orientation.w = 1.0
	edges.color.r, edges.color.g, edges.color.b, edges.color.a = 0.35, 0.75, 0.75, 1.00
	return edges


def get_obstacles_lines(obstacles):
	lines = []
	lines.append((Point(BOARD_CORNERS[0], BOARD_CORNERS[2], 0), Point(BOARD_CORNERS[1], BOARD_CORNERS[2], 0)))
	lines.append((Point(BOARD_CORNERS[0], BOARD_CORNERS[3], 0), Point(BOARD_CORNERS[1], BOARD_CORNERS[3], 0)))
	lines.append((Point(BOARD_CORNERS[0], BOARD_CORNERS[2], 0), Point(BOARD_CORNERS[0], BOARD_CORNERS[3], 0)))
	lines.append((Point(BOARD_CORNERS[1], BOARD_CORNERS[2], 0), Point(BOARD_CORNERS[1], BOARD_CORNERS[3], 0)))
	for obst in obstacles:
		scale_x = obst.scale.x
		scale_y = obst.scale.y
		pos_x = obst.pose.position.x
		pos_y = obst.pose.position.y
		if obst.type == obst.CUBE:  # Only rectangular obstacles are supported
			# Only aligned obstacles are supported
			x_i = pos_x - scale_x / 2
			x_f = pos_x + scale_x / 2
			y_i = pos_y - scale_y / 2
			y_f = pos_y + scale_y / 2
			lines.append((Point(x_i, y_i, 0), Point(x_f, y_i, 0)))
			lines.append((Point(x_i, y_f, 0), Point(x_f, y_f, 0)))
			lines.append((Point(x_i, y_i, 0), Point(x_i, y_f, 0)))
			lines.append((Point(x_f, y_i, 0), Point(x_f, y_f, 0)))
	return lines


def get_target_lines(target):
	lines = []
	scale_x = target.scale.x
	scale_y = target.scale.y
	pos_x = target.pose.position.x
	pos_y = target.pose.position.y
	if target.type == target.CUBE:  # Only rectangular obstacles are supported
		# Only aligned obstacles are supported
		x_i = pos_x - scale_x / 2
		x_f = pos_x + scale_x / 2
		y_i = pos_y - scale_y / 2
		y_f = pos_y + scale_y / 2
		lines.append((Point(x_i, y_i, 0), Point(x_f, y_i, 0)))
		lines.append((Point(x_i, y_f, 0), Point(x_f, y_f, 0)))
		lines.append((Point(x_i, y_i, 0), Point(x_i, y_f, 0)))
		lines.append((Point(x_f, y_i, 0), Point(x_f, y_f, 0)))
	return lines


def orientation(p, q, r):
	"""
	https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
	:param p:
	:param q:
	:param r:
	:return:
	"""
	val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y)
	if val == 0:
		return 0
	return 1 if val > 0 else 2


def on_segment(p, q, r):
	"""
	https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
	:param p:
	:param q:
	:param r:
	:return:
	"""
	if min(p.x, r.x) <= q.x <= max(p.x, r.x) and min(p.y, r.y) <= q.y <= max(p.y, r.y):
		return True
	return False


def collides_line(point_i, point_e, line):
	p_i, p_e = line
	o1 = orientation(point_i, point_e, p_i)
	o2 = orientation(point_i, point_e, p_e)
	o3 = orientation(p_i, p_e, point_i)
	o4 = orientation(p_i, p_e, point_e)
	if o1 != o2 and o3 != o4:
		return True
	if o1 == 0 and on_segment(point_i, p_i, point_e):
		return True
	if o2 == 0 and on_segment(point_i, p_e, point_e):
		return True
	if o3 == 0 and on_segment(p_i, point_i, p_e):
		return True
	if o4 == 0 and on_segment(p_i, point_e, p_e):
		return True
	return False


def collides_object(point_i, point_e, lines):
	for line in lines:
		if collides_line(point_i, point_e, line):
			mid_point = Point()
			mid_point.x = (line[1].x + line[0].x)/2
			mid_point.y = (line[1].y + line[0].y)/2
			mid_point.z = 0
			return (True, mid_point)
	return (False, None)

def choose_next_point(target):
	next_point = Point()
	target_x = target.x
	target_y = target.y
	next_point.x = random.gauss(target_x, target_x/2)
	while not (next_point.x < BOARD_CORNERS[1] and next_point.x > BOARD_CORNERS[0]):
		next_point.x = random.gauss(target_x, target_x/2)

	next_point.y = random.gauss(target_y, target_y/2)
	while not (next_point.y < BOARD_CORNERS[2] and next_point.y > BOARD_CORNERS[3]):
		next_point.y = random.gauss(target_y, target_y/2)

	next_point.z = 0
	return next_point


def run_ros(message):
	rospy.init_node('ros_demo')
	# First param: topic name; second param: type of the message to be published; third param: size of queued messages,
	# at least 1
	chatter_pub = rospy.Publisher('some_chatter', String, queue_size=10)
	marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

	# Each second, ros "spins" and draws 20 frames
	loop_rate = rospy.Rate(20)  # 20

	frame_count = 0

	obstacles = create_obstacles(message.obstacles)

	obstacles_lines = get_obstacles_lines(obstacles)

	robot = create_robot(message.start) 

	target = create_target(message.target)

	target_lines = get_target_lines(target)

	point = get_point_structure()

	tree_edges = get_tree_edges_structure()

	p0 = Point(robot.pose.position.x, robot.pose.position.y, 0)

	tree = Tree(TreeNode(p0, 0))

	collision_edges = get_collision_edges_structure()

	found_path = False

	path_edges = get_path_edges_structure()

	drawed_path = False

	robot_reached = False

	path_published = False

	path_points = []

	obstacle_collision_count = 0

	change_normal_distribution = False

	annoying_obstacle_point = None

	while not rospy.is_shutdown():
		msg = "Frame index: %s" % frame_count

		rospy.loginfo(msg)

		chatter_pub.publish(msg)

		for obst in obstacles:
			obst.header.stamp = rospy.Time.now()
			marker_pub.publish(obst)

		robot.header.stamp = rospy.Time.now()

		target.header.stamp = rospy.Time.now()
		marker_pub.publish(target)

		point.header.stamp = rospy.Time.now()

		tree_edges.header.stamp = rospy.Time.now()

		collision_edges.header.stamp = rospy.Time.now()

		path_edges.header.stamp = rospy.Time.now()

		if frame_count % HZ == 0 and not found_path:

			rand_pnt = None
			if change_normal_distribution:
				rand_pnt = choose_next_point(annoying_obstacle_pnt)
				change_normal_distribution = False
			else:
				rand_pnt = choose_next_point(target.pose.position)

			#enters this if statements every 5 counts
			# rand_pnt = choose_next_point(target)
			#rand_pnt = Point()
			#rand_pnt.x = random.uniform(BOARD_CORNERS[0], BOARD_CORNERS[1])
			#rand_pnt.y = random.uniform(BOARD_CORNERS[3], BOARD_CORNERS[2])
			#rand_pnt.z = 0
			point.points = [rand_pnt]
			close_node = tree.get_closest_node(rand_pnt)
			close_pnt = close_node.point

			# https://math.stackexchange.com/questions/175896/finding-a-point-along-a-line-a-certain-distance-away-from-another-point
			total_dist = math.sqrt(math.pow(rand_pnt.x - close_pnt.x, 2) + math.pow(rand_pnt.y - close_pnt.y, 2))

			dist_ratio = STEP / total_dist

			new_pnt = Point()
			new_pnt.x = (1 - dist_ratio) * close_pnt.x + dist_ratio * rand_pnt.x
			new_pnt.y = (1 - dist_ratio) * close_pnt.y + dist_ratio * rand_pnt.y
			new_pnt.z = 0

			does_collide, collision_point = collides_object(close_pnt, new_pnt, obstacles_lines)
			if does_collide:
				collision_edges.points.append(close_pnt)
				collision_edges.points.append(new_pnt)
				obstacle_collision_count += 1

				if obstacle_collision_count == 2:
					change_normal_distribution = True
					obstacle_collision_count = 0
					annoying_obstacle_pnt = collision_point

			else:
				# Try to find straight path by comparing the distance between new_pnt and all parents
				new_check_pnt_cost = 1 + close_node.cost_to_parent
				check_node = close_node.parent
				potential_parent = close_node
				potential_cost = 1
				while check_node != None:
					cost = math.sqrt(math.pow(check_node.point.x - new_pnt.x, 2) + math.pow(check_node.point.y - new_pnt.y, 2))
					if cost <= new_check_pnt_cost:
						if not collides_object(check_node.point, new_pnt, obstacles_lines)[0]:
							potential_cost = cost
							potential_parent = check_node
							new_check_pnt_cost = cost

					check_node = check_node.parent
					if check_node:
						new_check_pnt_cost += check_node.cost_to_parent


				close_node = potential_parent
				last_node = tree.add_node(close_node, new_pnt)

				tree_edges.points.append(close_node.point)
				tree_edges.points.append(new_pnt)

			if collides_object(close_node.point, new_pnt, target_lines)[0]:
				found_path = True

		if found_path and not drawed_path:
			current_node = last_node
			while not current_node.is_root():
				path_points.append(current_node.point)
				path_edges.points.append(current_node.point)
				path_edges.points.append(current_node.parent.point)
				current_node = current_node.parent
			drawed_path = True

		if found_path and drawed_path and not path_published:
			# path_poses = []

			# for i in range(len(path_points) - 1):
			# 	current_point = path_points[i]
			# 	next_point = path_points[i + 1]
			# 	current_pose = Pose()
			# 	current_pose.position = current_point
			# 	current_quat = Quaternion()
			# 	current_quat.x = 0
			# 	current_quat.y = 0
			# 	current_quat.z = 1

			# 	prev_angle = 0 
			# 	if i > 0:
			# 		prev_angle = path_poses[i - 1].orientation.w

			# 	current_quat.w = np.arctan((next_point.y -current_point.y)/ (next_point.x - current_point.x)) - prev_angle
			# 	current_pose.orientation = current_quat
			# 	path_poses.append(current_pose)

			# path_pose_array = PoseArray()
			# path_pose_array.poses = path_poses
			# path_points_pub.publish(path_pose_array)
			point_array = PointArray()
			point_array.points = path_points
			point_array.points.reverse()
			marker_pub.publish(robot)
			marker_pub.publish(point)
			marker_pub.publish(tree_edges)
			marker_pub.publish(collision_edges)
			marker_pub.publish(path_edges)
			# path_points_pub.publish(point_array)
			path_published = True
			return point_array

		
		if frame_count % 2 == 0 and drawed_path and not robot_reached:
			robot.pose.position = path_points.pop()
			robot_reached = True if len(path_points) == 0 else False

		if robot_reached: 
			break

		marker_pub.publish(robot)
		marker_pub.publish(point)
		marker_pub.publish(tree_edges)
		marker_pub.publish(collision_edges)
		marker_pub.publish(path_edges)

		# Check if there is a subscriber. Here our subscriber will be Rviz
		while marker_pub.get_num_connections() < 1:
			if rospy.is_shutdown():
				return 0
			# rospy.logwarn_once("Please run Rviz in another terminal.")
			rospy.sleep(1)

		loop_rate.sleep()
		frame_count += 1


def listener():
	#rospy.Subscriber("rrt_start_target", PointForRRT, run_ros)
	s = rospy.Service("run_rrt", PointForRRT, run_ros)
	rospy.spin()


if __name__ == '__main__':
	# try:
	# 	run_ros()
	# except rospy.ROSInterruptException:
	# 	pass
	rospy.init_node('rrt_planner', anonymous=True)
	listener()