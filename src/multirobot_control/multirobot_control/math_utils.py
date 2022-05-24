import numpy as np
from typing import List, Tuple

def dist_to_aabb(curr_x: float, curr_y: float, aabb: List[Tuple[float, float, float, float]],
    get_closest_point:bool=False
    ):
        '''
        Calculates the distance from the robot base (modelled as a circle) and any Axis-Aligned Bounding Box.
        AABBs are useful here because the shelf obstacles in the world are axis-aligned rectangles.
        
        Algorithm taken from https://learnopengl.com/In-Practice/2D-Game/Collisions/Collision-detection
        
        ---

        Args:
        - curr_x
        - curr_y
        - aabb: [x0, y0, x1, y1]
        - get_closest_point: Additionally also returns the closest point on the AABB to the 
            given point.
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
            # internal_dist = -np.hypot((diff_vect_x_clamped-curr_x),(diff_vect_y_clamped-curr_y))
            internal_dist = -np.hypot(diff_vect_x_clamped, diff_vect_y_clamped)

            if get_closest_point:
                # Closest point expanding on X.
                # Take sign of closest x/y to be the 'closer' side of the AABB box
                if diff_vect_x_clamped > 0:
                    closest_x = np.array((aabb_ctr_x + w, aabb_ctr_y + diff_vect_y_clamped))
                else:
                    closest_x = np.array((aabb_ctr_x - w, aabb_ctr_y + diff_vect_y_clamped))
                
                # Closest point expanding on Y
                if diff_vect_y_clamped > 0:
                    closest_y = np.array((aabb_ctr_x + diff_vect_x_clamped, aabb_ctr_y + h))
                else:
                    closest_y = np.array((aabb_ctr_x + diff_vect_x_clamped, aabb_ctr_y - h))

                dist_closest_x = np.linalg.norm(closest_x - np.array((curr_x, curr_y)) )
                dist_closest_y = np.linalg.norm(closest_y - np.array((curr_x, curr_y)) )

                # Return the closest point to the edge of the bounding box.
                if dist_closest_x < dist_closest_y:
                  return internal_dist, closest_x
                else:
                  return internal_dist, closest_y
            else:
                return internal_dist

        # Find the distance away from the closest point on the AABB to (curr_x, curr_y)
        diff_vect_x_prime = curr_x-(aabb_ctr_x+diff_vect_x_clamped)
        diff_vect_y_prime = curr_y-(aabb_ctr_y+diff_vect_y_clamped)

        dist_to_bot = np.hypot(diff_vect_x_prime, diff_vect_y_prime)

        if get_closest_point:
            return dist_to_bot, np.array((
                aabb_ctr_x + diff_vect_x_clamped,
                aabb_ctr_y + diff_vect_y_clamped
            ))
        else:
            return dist_to_bot

def get_intersection(a0: np.ndarray, a1: np.ndarray, b0: np.ndarray, b1: np.ndarray):
    '''
    Formula from: https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection

    Given two points on each line, returns their point of intersection, if it exists.
    Returns None otherwise.
    '''
    # (Px, Py) = x1 + t(x2-x1), y1 + t(y2-y1)
    # num = (x1   -x3   ) (y3   -y4   )-(y1   -y3   ) (x3   -x4   )
    t_num = (a0[0]-b0[0])*(b0[1]-b1[1])-(a0[1]-b0[1])*(b0[0]-b1[0])
    # den = (x1   -x2   ) (y3   -y4   )-(y1   -y2   ) (x3   -x4   )
    t_den = (a0[0]-a1[0])*(b0[1]-b1[1])-(a0[1]-a1[1])*(b0[0]-b1[0])

    if t_den == 0:
        return None # Not possible to divide by 0

    # (Px, Py) = x3 + u(x4-x3), y3 + u(y4-y3)
    # num = (x1   -x3   ) (y1   -y2   )-(y1   -y3   ) (x1   -x2   )
    u_num = (a0[0]-b0[0])*(a0[1]-a1[1])-(a0[1]-b0[1])*(a0[0]-a1[0])
    # den = (x1   -x2   ) (y3   -y4   )-(y1   -y2   ) (x3   -x4   )
    u_den = t_den

    t = t_num/t_den
    u = u_num/u_den

    t_int = a0 + t*(a1-a0)
    u_int = b0 + u*(b1-b0)

    # There exists an intersection if 0.0<=t<=1.0, 0.0<=u<=1.0
    if t>=0.0 and t<=1.0 and u>=0.0 and u<=1.0:
        # Calculate intersection and return it.
        assert np.allclose(t_int, u_int, atol=1e-4), f"Int 1: {t_int[0]:.4f}, {t_int[1]:.4f} | Int 2: {u_int[0]:.4f}, {u_int[1]:.4f}"
        return t_int
    else:
        return None # no intersection