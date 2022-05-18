import numpy as np
from typing import List, Tuple

def dist_to_aabb(curr_x: float, curr_y: float, aabb: List[Tuple[float, float, float, float]]):
        '''
        Calculates the distance from the robot base (modelled as a circle) and any Axis-Aligned Bounding Box.
        AABBs are useful here because the shelf obstacles in the world are axis-aligned rectangles.
        
        Algorithm taken from https://learnopengl.com/In-Practice/2D-Game/Collisions/Collision-detection
        
        ---

        Args:
        curr_x
        curr_y
        aabb: [x0, y0, x1, y1]
        '''
        # First calculate the closest point to the circle on the AABB.
        # AABB coords are always (x1, y1, x2, y2) with x1<x2, y1<y2
        aabb_ctr_x = (aabb[0] + aabb[2]) / 2
        aabb_ctr_y = (aabb[1] + aabb[3]) / 2

        diff_vect_x = curr_x-aabb_ctr_x
        diff_vect_y = curr_y-aabb_ctr_y

        w = abs(aabb[2]-aabb[0])/2  # half-width of AABB
        h = abs(aabb[3]-aabb[1])/2  # half-height of AABB

        # Clamp diff_vect between +-w/h
        # This gives us the closest point from the AABB to the circle.
        diff_vect_x_clamped = max(min(diff_vect_x, w), -w)
        diff_vect_y_clamped = max(min(diff_vect_y, h), -h)

        # If both values are not w/h, it means that the center of the circle is within the AABB.
        # In our context this is very bad, (distance is too close!)
        if abs(diff_vect_x_clamped) != w and abs(diff_vect_y_clamped) != h:
            internal_dist = -np.hypot((diff_vect_x_clamped-curr_x),(diff_vect_y_clamped-curr_y))
            return internal_dist

        # Find the distance away from the closest point on the AABB to (curr_x, curr_y)
        diff_vect_x_prime = curr_x-(aabb_ctr_x+diff_vect_x_clamped)
        diff_vect_y_prime = curr_y-(aabb_ctr_y+diff_vect_y_clamped)

        dist_to_bot = np.hypot(diff_vect_x_prime, diff_vect_y_prime)

        return dist_to_bot

def get_intersection(a0, a1, b0, b1):
    '''
    Formula from: https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection

    Given two points on each line, returns their point of intersection, if it exists.
    Returns false otherwise.
    '''
    # (Px, Py) = x1 + t(x2-x1), y1 + t(y2-y1)

    # num = (x1   -x3   ) (y3   -y4   )-(y1   -y3   ) (x3   -x4   )
    t_num = (a0[0]-b0[0])*(b0[1]-b1[1])-(a0[1]-b0[1])*(b0[0]-b1[0])
    # den = (x1   -x2   ) (y3   -y4   )-(y1   -y2   ) (x3   -x4   )
    t_den = (a0[0]-a1[0])*(b0[1]-b1[1])-(a0[1]-a1[1])*(b0[0]-b1[0])
    
    print(t_num, t_den)

    # t = t_num/t_den
    # There exists an intersection if 0.0 <= t <= 1.0
    # Therefore check if t_num and t_den have the same sign and if t_num <= t_den
    # neatly also leaves out case if t_den == 0
    if (abs(t_num) <= abs(t_den)) \
        and ((t_num >=0 and t_den>0) or (t_num <= 0 and t_den <0)):
        # Calculate intersection and return it
        return a0 + (t_num/t_den)*(a1-a0)
    else:
        return False # no intersection