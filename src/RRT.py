import numpy as np
import sys
from math import sin, cos, atan2, sqrt, asin, acos
import random
import rospy

def rotz(theta):
    R = np.array([[cos(theta), -sin(theta), 0],
                  [sin(theta), cos(theta), 0],
                  [0, 0, 1]])
    return R

def mod2pi(phi):
    mod = phi%(2 * np.pi)
    return mod

class RRT(object):
    def __init__(self, wpp_start, wpp_end, R_min, area_map):

        self.map = area_map
        self.map_shape = self.map.shape[0]
        self.start_node = rrtNode(0, None, wpp_start)
        self.end_node = rrtNode(0, None, wpp_end)
        self.nodes = set()
        self.nodes.add(self.start_node)
        self.end_nodes = set()
        self.R_min = R_min
        self.num_valid_paths = 0
        self.path_elevation = self.start_node.pos[2]

    def get_map_index(self, pt_x, pt_y):
        map_x = pt_x + self.map_shape / 2
        map_y = pt_y + self.map_shape / 2

        return map_x, map_y

    def find_path(self):

        if np.linalg.norm(self.end_node.pos-self.start_node.pos)<5*self.R_min and self.check_dubins_path(self.start_node,self.end_node):
            return [self.convertToFloats(self.start_node), self.convertToFloats(self.end_node)]

        iterations = 0

        while self.num_valid_paths < 5:
            valid_flag = True
            # pick a random point
            x = random.random() * self.map_shape - self.map_shape / 2
            y = random.random() * self.map_shape - self.map_shape / 2

            # find the nearest node
            nearest = None
            nearest_dist = self.map_shape**3
            chi = 0

            for node in self.nodes:
                dist, ang = node.dist_to(x, y)
                if dist < nearest_dist and dist > 3*self.R_min:
                    nearest_dist = dist
                    nearest = node
                    chi = ang

            if nearest is None:
                continue

            dist_e, ang_e = self.end_node.dist_to(x,y)

            if dist_e < 3*self.R_min:
                continue

            # take a step in the right direction
            if dist > 3.2*self.R_min:
                x_test = nearest.pos.item(0) + cos(chi) * 3.2*self.R_min
                y_test = nearest.pos.item(1) + sin(chi) * 3.2*self.R_min
            else:
                x_test = x
                y_test = y
            # rospy.logwarn(dist)
            # rospy.logwarn(x)
            # rospy.logwarn(y)
            # rospy.logwarn(chi)
            # rospy.logwarn(dist > 3.2*self.R_min)
            # rospy.logwarn(x_test)
            # rospy.logwarn(y_test)
            pos_test = np.array([[x_test], [y_test], self.path_elevation])
            nearest_dist, chi = nearest.dist_to(x_test,y_test)
            new_node = rrtNode(nearest.cost + nearest_dist, nearest, [pos_test,chi,nearest.Va])

            # get the dubins path between the two
            valid_flag = self.check_dubins_path(nearest, new_node)

            # if valid point add a new node
            if valid_flag:
                # add the point to valid nodes stack
                self.nodes.add(new_node)
                #------------GOOD TO HERE-----------
                # add the path to the map

                # find path to end node
                # dubinsParameters(ps, chi_s, pe, chi_e)
                # check if path to end is valid
                to_end_valid = self.check_dubins_path(new_node, self.end_node)
                # if valid add end node
                if to_end_valid:
                    dist_end, ang_end = new_node.dist_to(self.end_node.pos.item(0), self.end_node.pos.item(1))
                    # return dist_end
                    new_end = rrtNode(new_node.cost + dist_end, new_node, [self.end_node.pos, self.end_node.chi, self.end_node.Va],end_node=True)
                    self.end_nodes.add(new_end)
                    # increment count
                    self.num_valid_paths += 1
        waypoints = self.find_shortest_path()
        return waypoints


    def collision(self, node_s, node_e):

        dubinsPath = self.dubinsParameters(node_s.pos, node_s.chi, node_e.pos, node_e.chi)

        return True

    def check_dubins_path(self, start_node, end_node):
        valid_path = True

        path_points = self.dubins_points(start_node, end_node, 10)

        if path_points == False:
            return False

        for point in path_points:
            pt_x = int(point.item(0))
            pt_y = int(point.item(1))

            if pt_x >= self.map_shape / 2:
                pt_x = self.map_shape / 2 - 1
            elif pt_x <= -self.map_shape / 2:
                pt_x = -self.map_shape / 2 + 1

            if pt_y >= self.map_shape / 2:
                pt_y = self.map_shape / 2 - 1
            elif pt_y <= -self.map_shape / 2:
                pt_y = -self.map_shape / 2 + 1

            map_x, map_y = self.get_map_index(pt_x,pt_y)

            if self.map[map_x][map_y] >= -1 * self.path_elevation:
                valid_path = False
                break

        return valid_path

    def dubins_points(self, start, end, delta):
        """
        Find points along a dubins path
        :param start: rrtnode for the start point
        :param end: rrtnode for the end point
        :param delta: step size along the path
        :return: list of points
        """
        dp_out = self.dubinsParameters(start.pos, start.chi, end.pos, end.chi)
        # L, cs, lam_s, ce, lam_e, z1, q1, z2, z3, q3 = self.dubinsParameters(start.pos, start.chi, end.pos, end.chi)
        if dp_out == 0:
            return False

        L, cs, lam_s, ce, lam_e, z1, q1, z2, z3, q3 = dp_out

        points = self.points_along_circle(cs, start.pos, z1, delta)
        points += self.points_along_path(z1, z2, delta)
        points += self.points_along_circle(ce, z2, z3, delta)
        return points

    def points_along_path(self, start_pos, end_pos, delta):
        """
        return points along a path separated by a distance delta
        :param start_pos: path start position
        :param end_pos: path end position
        :param delta: distance between each point
        :return: list points along the path
        """
        points = []
        q = end_pos - start_pos
        L = np.linalg.norm(q)
        q = q / L
        for i in range(int(L/delta) + 1):
            points.append(start_pos + i * delta * q)
        points.append(end_pos)
        return points

    def points_along_circle(self, center, start_pos, end_pos, delta):
        """
        Calculates points along a circular arc segment defined by a start, end and center
        position spaced out by a distance of delta
        :param center:
        :param start_pos:
        :param end_pos:
        :param delta:
        :return:
        """
        rad = np.linalg.norm(center-start_pos)
        start_ang = atan2(start_pos.item(0)-center.item(0), start_pos.item(1)-center.item(1))
        end_ang = atan2(end_pos.item(0)-center.item(0), end_pos.item(1)-center.item(1))
        theta_step = delta / rad
        num_steps = int((end_ang-start_ang)/theta_step)
        points = []
        for i in range(num_steps):
            x = center.item(0) + rad * cos(start_ang + i * theta_step)
            y = center.item(1) + rad * sin(start_ang + i * theta_step)
            z = center.item(2)
            points.append(np.array([x, y, z]))
        points.append(np.array([end_pos.item(0), end_pos.item(1), end_pos.item(2)]))
        return points

    def convertToFloats(self, node):
        return [float(node.pos[0]),float(node.pos[1]),float(node.pos[2]),node.chi,node.Va]

    def dubinsParameters(self, ps, chi_s, pe, chi_e):
        dist = np.linalg.norm(ps - pe)

        if dist < 3.0*self.R_min:
            # return 'dist too short'
            return 0

        e1 = np.array([[1], [0], [0]])
        # return dist, e1
        crs = np.add(ps,self.R_min*np.matmul(rotz(np.pi/2), np.array([[cos(chi_s)], [sin(chi_s)], [0]])))
        cls = ps + self.R_min*np.matmul(rotz(-np.pi/2), np.array([[cos(chi_s)], [sin(chi_s)], [0]]))
        cre = pe + self.R_min*np.matmul(rotz(np.pi/2), np.array([[cos(chi_e)], [sin(chi_e)], [0]]))
        cle = pe + self.R_min*np.matmul(rotz(-np.pi/2), np.array([[cos(chi_e)], [sin(chi_e)], [0]]))

        #compute length for case 1 rsr
        ang = atan2(cre.item(1)-crs.item(1), cre.item(0)-crs.item(0))
        L1 = np.linalg.norm(crs-cre) + mod2pi(2 * np.pi + mod2pi(ang - np.pi / 2) - mod2pi(chi_s - np.pi / 2)) \
             + self.R_min*mod2pi(2 * np.pi + mod2pi(chi_e - np.pi / 2) - mod2pi(ang - np.pi / 2))

        # Compute length for case 2 rsl
        ang = atan2(cle.item(1)-crs.item(1), cle.item(0)-crs.item(0))
        l = np.linalg.norm(cle - crs)
        try:
            ang2 = ang - np.pi / 2 + asin((2 * self.R_min) / l)
            L2 = np.sqrt(l ** 2 - 4 * self.R_min ** 2) + self.R_min * mod2pi(2 * np.pi + mod2pi(ang2) - mod2pi(chi_s - np.pi / 2)) \
                 + self.R_min * mod2pi(2 * np.pi + mod2pi(ang2 + np.pi) - mod2pi(chi_e + np.pi / 2))
        except ValueError:
            L2 = 9999


        # Compute length for case 3 lsr
        ang = atan2(cre.item(1)-cls.item(1), cre.item(0)-cls.item(0))
        l = np.linalg.norm(cre-cls)
        try:
            ang2 = acos((2 * self.R_min) / l)
            L3 = np.sqrt(l ** 2 - 4 * self.R_min ** 2) + self.R_min * mod2pi(2 * np.pi + mod2pi(ang2) - mod2pi(chi_s - np.pi / 2)) \
                 + self.R_min * mod2pi(2 * np.pi + mod2pi(ang2 + np.pi) - mod2pi(chi_e + np.pi / 2))
        except ValueError:
            L3 = 9999

        # Compute length for case 4 lsl
        ang = atan2(cle.item(1)-cls.item(1), cle.item(0)-cls.item(0))
        L4 = np.linalg.norm(cls-cle) + self.R_min * mod2pi(2 * np.pi + mod2pi(chi_s + np.pi / 2) - mod2pi(ang + np.pi / 2)) \
             + self.R_min * mod2pi(2 * np.pi + mod2pi(ang + np.pi / 2) - mod2pi(chi_e + np.pi / 2))

        lengths = [L1, L2, L3, L4]
        if min(lengths) == L1:
            cs = crs
            lam_s = 1
            ce = cre
            lam_e = 1
            q1 = (ce - cs) / np.linalg.norm(ce - cs)
            z1 = cs + self.R_min * np.matmul(rotz(-np.pi/2), q1)
            z2 = ce + self.R_min * np.matmul(rotz(-np.pi/2), q1)

        elif min(lengths) == L2:
            cs = crs
            lam_s = 1
            ce = cle
            lam_e = -1
            l = np.linalg.norm(ce - cs)

            if l < 2*self.R_min:
                return 0

            ang = atan2(ce.item(1) - cs.item(1), ce.item(0) - cs.item(0))
            ang2 = ang - np.pi/2 + asin((2 * self.R_min) / l)
            q1 = np.matmul(rotz(ang2 + np.pi/2), e1)
            z1 = cs + self.R_min * np.matmul(rotz(ang2), e1)
            z2 = ce + self.R_min * np.matmul(rotz(ang2 + np.pi), e1)

        elif min(lengths) == L3:
            cs = cls
            lam_s = -1
            ce = cre
            lam_e = 1
            l = np.linalg.norm(ce - cs)

            if l < 2*self.R_min:
                return 0

            ang = atan2(ce.item(1) - cs.item(1), ce.item(0) - cs.item(0))
            ang2 = acos((2 * self.R_min) / l)
            q1 = np.matmul(rotz(ang + ang2 - np.pi/2), e1)
            z1 = cs + self.R_min * np.matmul(rotz(ang + ang2), e1)
            z2 = ce + self.R_min * np.matmul(rotz(ang + ang2 -np.pi), e1)

        # elif min(lengths) == L4:
        else:
            cs = cls
            lam_s = -1
            ce = cle
            lam_e = -1
            q1 = (ce - cs) / np.linalg.norm(ce - cs)
            z1 = cs + self.R_min * np.matmul(rotz(np.pi/2), q1)
            z2 = ce + self.R_min * np.matmul(rotz(np.pi/2), q1)

        z3 = pe
        q3 = np.matmul(rotz(chi_e), e1)

        # return [cs,lam_s,ce,lam_e]
        return [min(lengths), cs, lam_s, ce, lam_e, z1, q1, z2, z3, q3]

    def find_shortest_path(self):
        node_path = []
        waypoint_path = []
        shortest_node = None
        length = 99999999999999999999
        for node in self.end_nodes:
            if node.cost < length:
                shortest_node = node
                length = node.cost
        node_path.append(shortest_node)
        rospy.logwarn(shortest_node.cost)
        while node_path[-1].parent is not None:
            node_path.append(node_path[-1].parent)
        node_path.reverse()
        node_path = self.smooth_path(node_path)
        for node in node_path:
            waypoint_path.append(self.convertToFloats(node))
        return waypoint_path

    def smooth_path(self, node_path):
        node_counter = 0
        smooth_path = [node_path[0]]
        end_reached = False
        while not end_reached:
            furthest_possible = node_counter
            for i in range(node_counter+1, len(node_path)):
                if self.check_dubins_path(smooth_path[-1], node_path[i]):
                    furthest_possible = i
            smooth_path.append(node_path[furthest_possible])
            node_counter = furthest_possible
            if smooth_path[-1].end_node:
                end_reached = True
        return smooth_path

    def check_path(self, start, end):
        valid_path = True
        path_points = self.points_along_path(start, end, 10)
        for point in path_points:
            pt_x = int(point.item(0))
            pt_y = int(point.item(1))

            if pt_x >= self.map_shape / 2:
                pt_x = self.map_shape / 2 - 1
            elif pt_x <= -self.map_shape / 2:
                pt_x = -self.map_shape / 2 + 1

            if pt_y >= self.map_shape / 2:
                pt_y = self.map_shape / 2 - 1
            elif pt_y <= -self.map_shape / 2:
                pt_y = -self.map_shape / 2 + 1

            map_x, mapy_y = self.get_map_index(pt_x,pt_y)

            if self.map[map_x][map_y] >= -1 * self.path_elevation:
                valid_path = False
                break
        return valid_path

class rrtNode(object):
    """
    Class to hold the nodes for the RRT path planning algorythm
    """
    def __init__(self, cost, parent, waypoint, end_node=False):
        """
        :param cost: path distance to get to the node
        :param parent: previous node in the path chain
        :param waypoint: (x, y, h, angle) the location/course angle of node
                         will contain Va, just ignore it
        """
        self.cost = cost
        self.parent = parent
        self.pos = waypoint[0]
        self.chi = waypoint[1]
        self.Va = waypoint[2]
        self.end_node = end_node

    def dist_to(self, x, y):
        """
        Calculate the distance from the node to an xy location
        elevation is assumed to be the same as the node
        :param x: x location
        :param y: y location
        :return: distance between the node and the location
        """
        node_x = self.pos[0]
        node_y = self.pos[1]
        dist = sqrt((node_x-x)**2 + (node_y-y)**2)
        ang = atan2(y-node_y, x-node_x)
        return dist, ang
