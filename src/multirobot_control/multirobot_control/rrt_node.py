'''
This class defines a Rapidly-exploring Random Tree (RRT).

Lots of stuff referenced from 
https://colab.research.google.com/github/RussTedrake/underactuated/blob/master/exercises/planning/rrt_planning/rrt_planning.ipynb#scrollTo=YeJOgoXh-QJd
'''
import numpy as np

# Testing only
import matplotlib.pyplot as plt

from typing import Tuple, List

from multirobot_control.math_utils import dist_to_aabb, get_intersection

class RRT:
    class Node:
        def __init__(self, pos: Tuple[float, float], parent=None, cost=np.inf) -> None:
            self._pos = np.array(pos)         # 2d array of [x, y]
            self._parent = parent   # parent node for traversal
            self._cost = cost       # cost of getting to that particular node from its parent

    def __init__(self, start_pos: Tuple[float, float], goal_pos: Tuple[float, float], 
        obstacle_list: Tuple[float, float, float, float], 
        bounds:Tuple[float, float, float, float],
        path_bias:float=0.3, it_lim:int=2000, node_lim:int=10000,
        max_extend_length:float=0.2, safety_radius:float=0.2, robot_radius:float=0.35,
        connect_circle_dist:float=1.0,
    ) -> None:
        '''
        Initializes a RRT* graph.

        Args:
        - Mapping args:
            - start_pos: starting position on the map
            - goal_pos: end position on the map
            - obstacle_list: in this case, list of Axis-Aligned bounding boxes 
            that are the obstacles in the environment.
            - bounds: boundaries of the workspace (x0, y0, x1, y1 where x0<x1, y0<y1)

        - Config args
            - path_bias: % chance of exploring straight towards the goal instead of randomly
            - it_lim: iteration limit before giving up
            - node_lim: maximum num nodes in the system
            - max_extend_length: how far to extend the path across waypoints
            - safety_radius: how far away from an obstacle to plan 
            - robot_radius: 'size' of robot
            - connect_circle_dist: area to search around new node for graph rewiring
        '''

        self.start = np.array(start_pos)
        self.goal = np.array(goal_pos)
        self.node_list = [self.Node(start_pos, None, 0)]
        self.goal_node = self.Node(goal_pos, None, np.inf)
        self.obstacle_list = obstacle_list
        self.bounds = bounds
        
        self.x_width = (self.bounds[2] - self.bounds[0])
        self.y_width = (self.bounds[3] - self.bounds[1])
        self.x_ctr = (self.bounds[2] + self.bounds[0]) / 2
        self.y_ctr = (self.bounds[3] + self.bounds[1]) / 2

        self.path_bias = path_bias
        self.it_lim = it_lim
        self.node_lim = node_lim
        self.max_extend_length = max_extend_length
        self.safety_radius = safety_radius
        self.robot_radius = robot_radius
        self.connect_circle_dist = connect_circle_dist

        # DEBUG ONLY
        self.fig = plt.figure()
        plt.ion()
        plt.show()
        
        plt.axes()
        plt.axis([self.bounds[0]-0.25, self.bounds[2]+0.25, self.bounds[1]-0.25, self.bounds[3]+0.25])

        # Plot start
        plt.plot( self.start[0], self.start[1], 'ro' )
        plt.draw()
        plt.plot( self.goal[0], self.goal[1], 'go' )
        plt.draw()
        plt.pause(0.001)

        for (x0, y0, x1, y1) in self.obstacle_list:
            rect = plt.Rectangle( (x0,y0), (x1-x0), (y1-y0), fc='gray' )
            plt.gca().add_patch(rect)
            plt.draw()
            plt.pause(0.001)

    def explore_one_step(self):
        '''
        Performs one instance of exploration.
        1. Randomly samples a point in free space / steps in the direction of the goal
        2. Find nearest neighbor in list of nodes
        3. See if straight-line path to list of nodes is obstacle-free
        4. Create node step distance away from its nearest neighbor and steps toward goal.
        '''
        # Proposed x, y coordinates
        prop_coords = np.array((0.0, 0.0))
        nearest_node = None
        new_node = None

        goal_valid = True

        # Look for a new x,y coordinate that is correct
        while True:
            # See if we should step in the direction of the goal
            if goal_valid and (self.path_bias > np.random.random()):
                prop_coords = self.goal
            else:
                # Randomly sample in the configuration space and check for collision-free point
                while True:
                    prop_x = (np.random.random()-0.5) * self.x_width + self.x_ctr
                    prop_y = (np.random.random()-0.5) * self.y_width + self.y_ctr
                    prop_coords = np.array((prop_x, prop_y))
                    
                    # If this returns false, means we collide with something.
                    if self.check_collision( prop_coords ):
                        break

            print(f"Proposed new point at {prop_coords[0]:.2f}, {prop_coords[1]:.2f}")

            # Find NN in node list
            nearest_node = self.get_nearest_node(self.node_list, prop_coords)
            print(f"Nearest node is at {nearest_node._pos[0]:.2f}, {nearest_node._pos[1]:.2f}")

            # Draw a line towards nearest node
            vect_to_nearest = prop_coords - nearest_node._pos
            dist_to_nearest = np.linalg.norm(vect_to_nearest)
            # If dist_to_nearest is closer than the max_extend_length, just use that distance instead
            # Else scale the vector to be the distance
            vect_to_nearest *= min(1, self.max_extend_length/dist_to_nearest)
            prop_coords = nearest_node._pos + vect_to_nearest
            print(f"Step towards new point at {prop_coords[0]:.2f}, {prop_coords[1]:.2f}")
            
            # Check if there are any obstacles along this new line.
            # If there are no obstacles, then we have found a valid new point!
            if self.check_line_intersection(nearest_node._pos, prop_coords) == False:
                print("New node added.\n")
                # If path between new_node and nearest_node is not in collision:
                # Connect node to best parent in near_inds
                near_idxs = self.get_near_idxs(prop_coords) # find close nodes
                # connect new node to the best parent in near_idxs
                new_node = self.choose_parent(near_idxs, prop_coords)
                # Rewire nodes in the proximity of new_node if it improves their costs
                self.rewire(new_node, near_idxs)
                break

            else:
                # If a step towards the goal results in a collision, dont do it again
                if goal_valid and np.all(prop_coords==self.goal):
                    goal_valid = False
            
                print("")

        # append new node
        assert new_node is not None, "[rrt_explore] new_node must not be None!"
        self.node_list.append( new_node )

    def explore(self):
        '''
        Plans the path from start to goal while avoiding obstacles. Repeatedly calls 
        `explore_one_step` till goal is found, or max iterations are reached.
        '''
        for i in range(self.it_lim):
            self.explore_one_step()

            # Check if node is close to target
            last_endpt = self.node_list[-1]._pos
            if np.linalg.norm( last_endpt - self.goal_node._pos ) <= self.max_extend_length:
                if self.check_line_intersection(last_endpt, self.goal_node._pos) == False:
                    # we found a path!
                    self.goal_node._parent = self.node_list[-1]
                    self.node_list.append(self.goal_node)
                    return self.get_path()

        print(f"Could not find a path from start {self.start[0]:.2f}, {self.start[1]:.2f} to end {self.goal[0]:.2f}, {self.goal[1]:.2f}")
        return None    # Cannot find a path

    def get_near_idxs(self, new_pos: np.ndarray):
        '''
        Find nodes close to the new node's pos.
        '''
        nnode = len(self.node_list) + 1
        r = self.connect_circle_dist * np.sqrt((np.log(nnode) / nnode))
        dlist = [np.sum(np.square((node._pos - new_pos))) for node in self.node_list]
        near_inds = [dlist.index(i) for i in dlist if i <= r ** 2]
        return near_inds

    def choose_parent(self, near_idxs, prop_coords):
        '''
        Choose parent of new node as the lowest resulting cost parent in near_idxs and
        new_node.cost to the corresponding minimal cost

        Return a corresponding new best node.
        '''
        min_cost = np.inf
        best_near_node = None

        # Go through all near nodes and evaluate them as potential parent nodes by
        for node_idx in near_idxs:
            prop_parent = self.node_list[node_idx]  # proposed parent
            # check if a connection would result in a collision
            if self.check_line_intersection(prop_parent._pos, prop_coords) != False:
                continue

            # evaluate the cost of the new_node if it had that near node as a parent
            cost = prop_parent._cost + np.linalg.norm( prop_parent._pos - prop_coords )

            if cost < min_cost:
                min_cost = cost
                best_near_node = prop_parent

        assert prop_parent is not None, "[rrt_explore] parent node must not be None!"
        # pick the parent resulting in the lowest cost, and return a corresponding node

        # DEBUG
        print(f"new node at {prop_coords[0]:.2f}, {prop_coords[1]:.2f}")
        plt.plot( prop_coords[0], prop_coords[1], 'ro' )
        # plt.plot( (prop_coords[0], prop_coords[1]), (best_near_node._pos[0], best_near_node._pos[1]), '-r' )
        plt.draw()
        plt.pause(0.001)

        return self.Node( prop_coords, best_near_node, min_cost )

    def rewire(self, new_node, near_idxs):
        '''
        Rewire near nodes to new node if it will result in a lower cost.
        '''
        # go through all near nodes and check whether rewiring them to the new_node ...
        for node_idx in near_idxs:
            prop_child = self.node_list[node_idx]  # proposed child
            # A) Not cause a collision and
            if self.check_line_intersection(prop_child._pos, new_node._pos) != False:
                continue
            # B) reduce their own cost.
            orig_cost = new_node._cost
            new_cost = np.linalg.norm(new_node._pos - prop_child._pos) + orig_cost
            if new_cost < prop_child._cost:
                prop_child._parent = new_node
        # If A and B are true, update the cost and parent properties of the node.
        
        self.propagate_cost_to_leaves(new_node)

    def check_collision(self, pos:Tuple[float, float]):
        '''
        Checks proposed point (x,y) if it will collide with any of the obstacles.

        Returns false if proposed point comes less than the safety radius of the robot.
        '''
        for obstacle in self.obstacle_list:
            obs_dist = dist_to_aabb(pos[0], pos[1], obstacle)
            if obs_dist < (self.robot_radius + self.safety_radius):
                return False

        return True

    @staticmethod
    def get_nearest_node(node_list, pos:np.ndarray):
        """Find the nearest node in node_list to proposed position"""
        dlist = [np.sum(np.square( pos - n._pos )) for n in node_list]
        minind = dlist.index(min(dlist))
        return node_list[minind]

    def check_line_intersection(self, line_start: np.ndarray, line_end: np.ndarray):
        '''
        Checks if proposed line from start to end will come close to any bounding box.
        
        - First we 'inflate' each bounding box (easy to do; as they are all axis-aligned).
        - Then we obtain 4 equations of lines for each side of the BB
        - Then we calculate the intersection point between the two lines (either solve for x or y)

        Args:
        - line_start, line_end (2d coords of line endpoints)

        Returns:
        - True if line intersects with something, False otherwise
        '''

        for (x0, y0, x1, y1) in self.obstacle_list:
            # Inflate obstacles by safety radius + robot_radius
            inflate_dist = self.safety_radius + self.robot_radius
            x0_ = x0-inflate_dist
            y0_ = y0-inflate_dist
            x1_ = x1+inflate_dist
            y1_ = y1+inflate_dist
            # 8 corners of inflated AABB
            c1 = np.array((x0, y0_))
            c2 = np.array((x0_, y0))
            c3 = np.array((x0_, y1))
            c4 = np.array((x0, y1_))
            c5 = np.array((x1, y1_))
            c6 = np.array((x1_, y1))
            c7 = np.array((x1_, y0))
            c8 = np.array((x1, y0_))
            
            # If all are False -> no intersection, valid line extension
            i1 = get_intersection( line_start, line_end, c1, c2 )
            i2 = get_intersection( line_start, line_end, c2, c3 )
            i3 = get_intersection( line_start, line_end, c3, c4 )
            i4 = get_intersection( line_start, line_end, c4, c5 )
            i5 = get_intersection( line_start, line_end, c5, c6 )
            i6 = get_intersection( line_start, line_end, c6, c7 )
            i7 = get_intersection( line_start, line_end, c7, c8 )
            i8 = get_intersection( line_start, line_end, c8, c1 )

            if i1 is not None or i2 is not None or i3 is not None or i4 is not None \
            or i5 is not None or i6 is not None or i7 is not None or i8 is not None :
                print(f"Intersection with obstacle @ {x0_:.2f}, {y0_:.2f}, {x1_:.2f}, {y1_:.2f}")
                return True

        return False

    def get_path(self):
        '''
        Traverse node_list to get path from first node to last node.
        Only call this when exploration is complete, otherwise will throw errors.
        '''
        path = [ self.node_list[-1] ]   # This should be the goal node
        while np.any(path[-1]._pos != self.start):
            path.append( path[-1]._parent )
            print("Appended point at", path[-1]._pos)

        path.reverse()

        assert np.all(path[0]._pos == self.start), f"Start position was {self.start[0]:.2f}, {self.start[1]:.2f} but start of path was {path[0]._pos[0]:.2f}, {path[0]._pos[1]:.2f}"
        assert np.all(path[-1]._pos == self.goal), f"Goal position was {self.goal[0]:.2f}, {self.goal[1]:.2f} but end of path was {path[-1]._pos[0]:.2f}, {path[-1]._pos[1]:.2f}"

        return path

    def propagate_cost_to_leaves(self, parent_node):
        """Recursively update the cost of the nodes"""
        for node in self.node_list:
            if node._parent == parent_node:
                orig_cost = parent_node._cost
                new_cost = np.linalg.norm(parent_node._pos - node._pos)
                node._cost = orig_cost + new_cost
                self.propagate_cost_to_leaves(node)