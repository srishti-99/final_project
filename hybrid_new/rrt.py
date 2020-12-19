import numpy as np

class PathPlanner:

    robot_vel = 1.0   # 1 m/s

    def __init__(self, initialPoint, objectDetection, dimensions):
        """ Takes in a seed configuration to start running the RRT from, along with an
        instance of an objectDetection object, and dimensions of the map.

        initialPoint (tuple): (x, y, theta)

        """

        self.initialPoint = initialPoint
        self.objectDetection = objectDetection
        self.dimensions = dimensions
        self.nodeCount = 0
        self.plan = {}
        contact = objectDetection.check_contact(seedPoint)
        if contact:
            print "Initial configuration of TurtleBot overlapping with object."
            sys.exit()

    def nh_build_rrt(self, K=100, epsilon=1):

        t = Graph()
        t.addVertex(self.q_init)
        q_nearest = None
        q_new = None
        for k in range(K):
            OBSTACLE_FREE = False
            while not OBSTACLE_FREE:
                q_rand = self.random_config(t)
                q_nearest, dist, _ = self.nearest_neighbour(q_rand, np.array(t.vertexMap.keys()))
                (u_s, u_p, q_new), ctrls_path = self.nh_steer(q_nearest, q_rand, epsilon)
                if self.within_boundary(q_new) \
                        and self.nh_obstacle_free(ctrls_path[(u_s, u_p)]) \
                        and t.getVertex(q_new) is None:
                    OBSTACLE_FREE = True
            edge = t.addUniEdge(q_nearest, q_new, self.robot_vel * epsilon, True)
            edge.ctrl = (u_s, u_p)
            edge.path = ctrls_path[(u_s, u_p)]
        return t

    def run_rrt(self, K=100, epsilon=1):

        graph = Graph()
        graph.addVertex(self.initialPoint)
        nearestNode = None
        newestNode = None

        for iteration in range(K):
            contact = False

            while not contact:
                randomPoint = self.random_config(graph)
                nodesInGraph = np.array(graph.vertexMap.keys())
                nearestNode, distance = self.nearest_neighbour(randomPoint, nodesInGraph)




    def contact_control(self, nearestNode, randomPoint, epsilon):
        """ All possible control inputs of the form (delta_speed, delta_angle) are generated and used
        to find the closest new node to the randomPoint.

        nearestNode (tuple) = (x, y, theta)
        randomPoint (tuple) = (x, y, theta)

        """

        L = 20.0  # Length between midpoints of front and rear axle of the car like robot
        # delta_speed, delta_angle = np.arange(0.1, 1.0), [-math.pi/2, -math.pi/4, 0, math.pi/4, math.pi/2] # Uncomment for predefined set
        # delta_speed = np.random.rand(3, size=10) #Uncomment for random sampling
        delta_angle = np.random.rand(3, size=10) #Uncomment for random sampling
        controls = list(itertools.product(delta_speed, delta_angle)) # Cartesian product of speeds and angles
        result = []
        control_paths = {c: [] for c in controls}

        # Select optimal control.

        for control in controls:
            newestNode = nearestNode

            for i in range(epsilon):
                change_speed = control[0]
                change_angle = control[1]

                newestNode = tuple(map(add, newestNode, self.euler(i, newestNode, change_speed, change_angle, L)))
                control_paths[control].append(newestNode)

            result.append((control, control[0], control[1], newestNode))
        new_nodes = [x[3] for x in result]
        _, _, index = self.nearest_neighbour(randomPoint, np.array(new_nodes))
        chosen_result = result[index]
        chosen_control = chosen_result[0]

        # Generate velocities to publish for control.

        nextNode = nearestNode
        for i in range(epsilon):
            change_speed = chosen_control[0]
            change_angle = chosen_control[1]

            nextNode = tuple(map(add, nextNode, self.euler(i, nextNode, change_speed, change_angle, L)))
            x_vel, y_vel, ang_vel = self.euler(i, nextNode, change_speed, change_angle, L)
            x, y, z = nextNode[0], nextNode[1], 0
            pose = np.array([x, y ,z, x_vel, y_vel, ang_vel])


        generated_pose = []
        return result[idx], control_paths

    def no_contact_control(self, nearestNode, randomPoint, epsilon):
        """ All possible control inputs of the form (delta_speed, delta_angle) are generated and used
        to find the closest new node to the randomPoint.

        nearestNode (tuple) = (x, y, theta)
        randomPoint (tuple) = (x, y, theta)

        """

        L = 20.0  # Length between midpoints of front and rear axle of the car like robot
        delta_speed, delta_angle = [-1.0, 1.0], [-math.pi/4, 0, math.pi/4]
        controls = list(itertools.product(delta_speed, delta_angle)) # Cartesian product of speeds and angles
        result = []
        control_paths = {c: [] for c in controls}

        # Select optimal control.

        for control in controls:
            newestNode = nearestNode

            for i in range(epsilon):
                change_speed = control[0]
                change_angle = control[1]

                newestNode = tuple(map(add, newestNode, self.euler(i, newestNode, change_speed, change_angle, L)))
                control_paths[control].append(newestNode)

            result.append((control, control[0], control[1], newestNode))
        new_nodes = [x[3] for x in result]
        _, _, index = self.nearest_neighbour(randomPoint, np.array(new_nodes))
        chosen_result = result[index]
        chosen_control = chosen_result[0]

        # Generate velocities to publish for control.

        nextNode = nearestNode
        for i in range(epsilon):
            change_speed = chosen_control[0]
            change_angle = chosen_control[1]

            nextNode = tuple(map(add, nextNode, self.euler(i, nextNode, change_speed, change_angle, L)))
            x_vel, y_vel, ang_vel = self.euler(i, nextNode, change_speed, change_angle, L)
            x, y, z = nextNode[0], nextNode[1], 0
            pose = np.array([x, y ,z, x_vel, y_vel, ang_vel])


        generated_pose = []
        return result[idx], control_paths

    def euler(self, i, node, change_speed, change_angle, L):
        return change_speed*math.cos(node[2]), change_speed*math.sin(node[2]), change_speed/L*math.tan(change_angle)

    def holonomic_extend(self, q_nearest, q_rand, epsilon=None):
        # TODO check if the new config overshoots the q_rand

        controls = [1.0]
        if epsilon is None:
            # Integrate till it reach q_rand
            dist = self.euc_distance(q_nearest, q_rand)
            epsilon = int(round(float(dist)/controls[0]))

        ctrls_path = {c: [] for c in controls}
        dx, dy, _ = np.array(q_rand) - np.array(q_nearest)
        theta = math.atan2(dy, dx)
        for ctrl in controls:
            q_new = q_nearest
            for t_i in range(epsilon):
                x_new = q_new[0] + ctrl * math.cos(theta)
                y_new = q_new[1] + ctrl * math.sin(theta)
                q_new = x_new, y_new, _
                ctrls_path[ctrl].append(q_new)
        return q_new, ctrls_path

    def random_config(self, graph):
        # Returns a random configuration tuple (x, y, theta)
        # Second entry in x, y tuple should be maximum dimensions of Gazebo
        x, y= self.dimensions[0], self.dimensions[1]
        while True:
            randomPoint = random.uniform(0, x), random.uniform(0, y), random.uniform(-math.pi, math.pi)
            if graph.getVertex(randomPoint) is None:
                return randomPoint
            else:
                print 'random_config:: randomPoint exists in RRT, picking another!'

    def nearest_neighbour(self, randomPoint, nodesInGraph):
        """ Takes in a randomPoint tuple generated by the RRT and computes the closest nodes
        currently present in the graph.

        randomPoint (tuple): (x, y, theta)

        nodesInGraph (array): [point1, point2, point3, ....] -> each point of the form of a tuple (x, y, theta)

        """
        xy_array, theta_array = nodesInGraph[:, 0:2], nodesInGraph[:, 2]
        # TODO Include Orientation into distance metric
        theta = [math.cos(q[2]), math.sin(q[2])]
        theta_array = [np.cos(theta_array[:]), np.sin(theta_array[:])]
        dist = np.linalg.norm(xy_array - randomPoint[0:2], axis=1)
        index = np.argmin(dist)
        nearestPoint = tuple(nodesInGraph[index])
        distance = dist[index]
        return nearestPoint, distance, index

    def nh_obstacle_free(self, points):
        for p in points:
            if self.cDetector.is_point_colliding(p):
                print "obstacle_free:: path/point collides!"
                return False
        return True

    def within_boundary(self, point):
        if 0 <= point[0] < 800 and 0 <= point[1] < 800:
            return True
        else:
            return False

    def euc_distance(self, start_pos, end_pos):
        return np.linalg.norm(np.array(start_pos) - np.array(end_pos))

    def rad_wrap(self, radian):
        """
        Takes an angle in radians of any range and returns an angle from
        the half open interval of [-3.14, 3.14) in radians.
        Note: If (-3.14, 3.14] is desired, then multiply the result of the below expression with -1
        :param radian:
        :return:
        """
        return (radian + np.pi) % (2 * np.pi) - np.pi

    def rad_diff(self, rad1, rad2):
        """
        Returns difference in angle between rad1 and rad2 in counterclockwise direction as radians
        :param rad1:
        :param rad2:
        :return:
        """
        rad1 = self.rad_wrap(rad1)
        rad2 = self.rad_wrap(rad2)
        if rad1 <= rad2:
            return rad2 - rad1
        else:
            return 2 * np.pi - (rad1 - rad2)

    def k_nearest_neighbour(self, t, q, max_dist=50):
        q_tree = np.array(t.vertexMap.keys())
        dist = np.linalg.norm(q_tree - q, axis=1)
        knn_keys = q_tree[np.where(dist <= max_dist)]
        return [t.vertexMap[tuple(k)] for k in knn_keys]

    def t2_maneuver(self, q_init, theta_goal):
        """
        Change q_init orientation to q_goal orientation by performing type 2 maneuvers
        NOTE: One observation made while coding was, when the u_speed is high, the expected angle is getting skipped or
        had to give huge approximation by rounding off. So I decided to do the maneuvers at very slow u_speed
        :param q_init:
        :param theta_goal:
        :return:
        """
        # TODO wrap q_init theta, or expect circles in maneuvers.
        del2_theta = self.rad_diff(q_init[2], theta_goal)
        del_theta = del2_theta/2

        L = 20.0  # Length between midpoints of front and rear axle of the car like robot
        u_speed, u_phi = [-0.1, 0.1], [-math.pi/4, 0, math.pi/4]

        distance = 0    # Cost of the t2_maneuver path
        q_new = q_init
        TURN1_DONE =False
        turn1_path = []
        line_start = None
        while not TURN1_DONE:
            distance += u_speed[1] * 1
            q_new = tuple(map(add, q_new, self.euler(1, q_new, u_speed[0], u_phi[0], L)))
            turn1_path.append(q_new)
            if round(q_new[2], 2) == round(q_init[2] + del_theta, 2) \
                    or round(self.rad_wrap(q_new[2]), 2) == round(self.rad_wrap(q_init[2] + del_theta), 2):
                TURN1_DONE = True
                line_start = q_new[0], q_new[1], self.rad_wrap(q_new[2])

        q_new = (q_init[0], q_init[1], theta_goal)
        TURN2_DONE = False
        turn2_path = []
        line_goal = None
        while not TURN2_DONE:
            distance += u_speed[1] * 1
            q_new = tuple(map(add, q_new, self.euler(1, q_new, u_speed[1], u_phi[0], L)))
            turn2_path.append(q_new)
            if round(q_new[2], 2) == round(q_init[2] + del_theta, 2)\
                    or round(self.rad_wrap(q_new[2]), 2) == round(self.rad_wrap(q_init[2] + del_theta), 2):
                TURN2_DONE = True
                line_goal = q_new[0], q_new[1], self.rad_wrap(q_new[2])

        q_new = line_start
        line_path = []
        for t in range(int(self.euc_distance(line_start, line_goal))):
            distance += 1
            q_new = tuple(map(add, q_new, self.euler(t, q_new, 1.0, u_phi[1], L)))
            line_path.append(q_new)
        return turn1_path, line_path, turn2_path[::-1], distance

    def t1_maneuver(self, q_init, paral_dist=1, direction='Left'):
        phi_max = math.pi/4
        L = 20.0
        u_speed = [-0.1, 0.1]
        rho_min = L/np.tan(phi_max)
        d = paral_dist
        del_theta = math.acos(2*rho_min / (d + 2*rho_min))

        cw_rot = np.array([[0, 1], [-1, 0]])
        acw_rot = np.array([[0, -1], [1, 0]])
        q_unit = np.array([math.cos(q_init[2]), math.sin(q_init[2])])
        cw_q_unit = np.dot(cw_rot, q_unit)
        acw_q_unit = np.dot(acw_rot, q_unit)
        cw_point = d * cw_q_unit
        cw_point = (cw_point[0] + q_init[0], cw_point[1] + q_init[1], q_init[2])
        acw_point = d * acw_q_unit
        acw_point = (acw_point[0] + q_init[0], acw_point[1] + q_init[1], q_init[2])

        if direction == 'Left':
            u_phi = [-math.pi/4, 0, math.pi/4]
            q_goal = cw_point
            turn_goal_theta = q_init[2] + del_theta
        else:
            u_phi = [math.pi/4, 0, -math.pi/4]
            q_goal = acw_point
            turn_goal_theta = q_init[2] - del_theta

        distance = 0    # Cost of the t1_maneuver path
        q_new = q_init
        TURN1_DONE =False
        turn1_path = []
        line_start = None
        while not TURN1_DONE:
            distance += u_speed[1] * 1
            q_new = tuple(map(add, q_new, self.euler(1, q_new, u_speed[1], u_phi[2], L)))
            turn1_path.append(q_new)
            if round(q_new[2], 2) == round(turn_goal_theta, 2) \
                    or round(self.rad_wrap(q_new[2]), 2) == round(self.rad_wrap(turn_goal_theta), 2):
                TURN1_DONE = True
                line_start = q_new[0], q_new[1], self.rad_wrap(q_new[2])

        q_new = (q_goal[0], q_goal[1], q_init[2])
        TURN2_DONE = False
        turn2_path = []
        line_goal = None
        while not TURN2_DONE:
            distance += u_speed[1] * 1
            q_new = tuple(map(add, q_new, self.euler(1, q_new, u_speed[0], u_phi[0], L)))
            turn2_path.append(q_new)
            if round(q_new[2], 2) == round(turn_goal_theta, 2) \
                or round(self.rad_wrap(q_new[2]), 2) == round(self.rad_wrap(turn_goal_theta), 2):
                TURN2_DONE = True
                line_goal = q_new[0], q_new[1], self.rad_wrap(q_new[2])

        q_new = line_start
        line_path = []
        for t in range(int(self.euc_distance(line_start, line_goal))):
            distance += 1
            q_new = tuple(map(add, q_new, self.euler(t, q_new, -1.0, u_phi[1], L)))
            line_path.append(q_new)
        turn2_path = turn2_path[::-1]
        turn2_path.append((round(q_goal[0]), round(q_goal[1]), q_goal[2]))
        return turn1_path, line_path, turn2_path, distance

    def is_forward(self, q_nearest, q_goal):
        q_fwd, q_bwd = q_nearest, q_nearest
        for t in range(10):
            q_fwd = tuple(map(add, q_fwd, self.euler(t, q_fwd, 1.0, 0.0, 20.0)))
            q_bwd = tuple(map(add, q_bwd, self.euler(t, q_bwd, -1.0, 0.0, 20.0)))
        _, _, idx = self.nearest_neighbour(q_goal, np.array([q_fwd, q_bwd]))
        if idx == 0:
            return True
        else:
            return False

    def is_left(self, q_init, q_goal):
        cw_rot = np.array([[0, 1], [-1, 0]])
        acw_rot = np.array([[0, -1], [1, 0]])
        q_unit = np.array([math.cos(q_init[2]), math.sin(q_init[2])])
        cw_q_unit, acw_q_unit = np.dot(cw_rot, q_unit), np.dot(acw_rot, q_unit)
        cw_point, acw_point = 5 * cw_q_unit, 5 * acw_q_unit
        cw_point = (cw_point[0] + q_init[0], cw_point[1] + q_init[1], q_init[2])
        acw_point = (acw_point[0] + q_init[0], acw_point[1] + q_init[1], q_init[2])
        _, _, idx = self.nearest_neighbour(q_goal, np.array([cw_point, acw_point]))
        if idx == 0:
            return True
        else:
            return False

    def nh_reach_goal(self, q_nearest, q_goal):
        distance = 0    # Cost of t1_march_t2 path

        # Type 2 to reach goal orientation inplace
        t2_paths = self.t2_maneuver(q_nearest, q_goal[2])
        distance += t2_paths[3]

        # March Forward/Backward to closest point
        march_path = []
        q_fwd = q_nearest[0], q_nearest[1], q_goal[2]
        q_bwd = q_nearest[0], q_nearest[1], q_goal[2]
        march_path.append(q_fwd)
        prev_dist = self.euc_distance(q_fwd, q_goal)
        if self.is_forward(q_fwd, q_goal):
            q_fwd = tuple(map(add, q_fwd, self.euler(1, q_fwd, 1.0, 0.0, 20.0)))
            march_path.append(q_fwd)
            curr_dist = self.euc_distance(q_fwd, q_goal)
            while curr_dist < prev_dist:
                q_fwd = tuple(map(add, q_fwd, self.euler(1, q_fwd, 1.0, 0.0, 20.0)))
                prev_dist = curr_dist
                curr_dist = self.euc_distance(q_fwd, q_goal)
                march_path.append(q_fwd)
        else:
            q_bwd = tuple(map(add, q_bwd, self.euler(1, q_bwd, -1.0, 0.0, 20.0)))
            march_path.append(q_bwd)
            curr_dist = self.euc_distance(q_bwd, q_goal)
            while curr_dist < prev_dist:
                q_bwd = tuple(map(add, q_bwd, self.euler(1, q_bwd, -1.0, 0.0, 20.0)))
                prev_dist = curr_dist
                curr_dist = self.euc_distance(q_bwd, q_goal)
                march_path.append(q_bwd)

        march_path = march_path[:-1]
        distance += len(march_path)

        # Type1 Left or Right to reach goal
        t1_paths = []
        paral_dist = self.euc_distance(march_path[-1], q_goal)
        if self.is_left(march_path[-1], q_goal):
            q_left = march_path[-1]
            # while not self.x_or_y_equal(q_left, q_goal):
            t1_steps = self.t1_maneuver(q_left, paral_dist, direction='Left')
            t1_paths.append(t1_steps[0:3])
            q_left = t1_steps[2][-1]
            distance += t1_steps[3]
        else:
            q_right = march_path[-1]
            # while not self.x_or_y_equal(q_right, q_goal):
            t1_steps = self.t1_maneuver(q_right, paral_dist, direction='Right')
            t1_paths.append(t1_steps[0:3])
            q_right = t1_steps[2][-1]
            distance += t1_steps[3]

        return t2_paths[0:3], march_path, t1_paths[0:3], distance

    def reach_goal(self, t, q_goal):
        q_nearest, dist, _ = self.nearest_neighbour(q_goal, np.array(t.vertexMap.keys()))
        q_new, ctrls_path = self.holonomic_extend(q_nearest, q_goal)
        if self.nh_obstacle_free(ctrls_path[1]) and t.getVertex(q_goal) is None:
            t.addUniEdge(q_nearest, q_goal, 1.0 * len(ctrls_path), True)
        else:
            print 'Path to goal collides. Try more samples'
            sys.exit()
        return t.getVertex(q_goal)

    def append_t1_m_t2(self, t2_paths, march_path, t1_paths):
        '''
        Accumulate type 1, marching and type 2 paths into single list for representing it as single path
        :param t2_paths:
        :param march_path:
        :param t1_paths:
        :return:
        '''
        final_path = []
        for l in t2_paths:
            for i in l:
                final_path.append(i)
        for i in march_path:
            final_path.append(i)
        for tup in t1_paths:
            for l in tup:
                for i in l:
                    final_path.append(i)
        return final_path
