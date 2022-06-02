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
                w += 0.01   # Add a small amount so this passes lt/gt comparisons elsewhere
                h += 0.01

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

def get_point_on_connecting_line(line_start:np.ndarray, line_end:np.ndarray, dist:float) -> np.ndarray:
    '''
    Given two points that make up a line, `line_start` and `line_end`, calculate the x,y
    coordinates of the point on the line a distance `dist` away from line_start.
    '''
    line = line_end-line_start
    line /= np.linalg.norm(line)

    return line_start + line*dist

"""
def check_line_of_sight(line_start:Tuple[float,float], line_end:Tuple[float,float], 
                        obstacles:List[Tuple[float,float,float,float]]) -> bool:
    '''
    Checks if a line from `line_start` to `line_end` is blocked by any Axis-Aligned 
    Bounding Box modelled by `obstacles`.

    Returns `True` if there is a line of sight to any obstacle, and `False` otherwise.
    '''
    line_start_np = np.array(line_start)
    line_end_np = np.array(line_end)
    for x0, y0, x1, y1 in obstacles:
        p1 = np.array((x0,y0))  # The 4 corners of the bounding box
        p2 = np.array((x0,y1))
        p3 = np.array((x1,y1))
        p4 = np.array((x1,y0))

        i1 = get_intersection(line_start_np, line_end_np, p1, p2)
        i2 = get_intersection(line_start_np, line_end_np, p2, p3)
        i3 = get_intersection(line_start_np, line_end_np, p3, p4)
        i4 = get_intersection(line_start_np, line_end_np, p4, p1)
        
        if i1 is None and i2 is None and i3 is None and i4 is None:
            continue
        else:
            return False

    return True
"""

def check_line_of_sight(line_start: Tuple[float,float], line_end: Tuple[float,float], 
            obstacles:List[Tuple[float,float,float,float]], 
            safety_radius:float=0.1, robot_radius:float=0.35,
            waypoint:bool=True):
    '''
    Checks if proposed line from start to end will come close to any bounding box.
    
    - First we 'inflate' each bounding box (easy to do; as they are all axis-aligned).
    - Then we obtain 4 equations of lines for each side of the BB
    - Then we calculate the intersection point between the two lines (either solve for x or y)

    Args:
    - line_start, line_end: 2d coords of line endpoints
    - Waypoint(bool) if the current node is at a start or end point, we relax the collision rules.
        Else it is a waypoint, and the typical safety radius rules apply.

    Returns:
    - The closest intersection point to line_start, and the x,y coordinates of the offending obstacle
    - True otherwise
    '''

    line_start = np.array(line_start)
    line_end = np.array(line_end)

    for (x0, y0, x1, y1) in obstacles:
        # Inflate obstacles by safety radius + robot_radius.
        # The AABBs are 'inflated' like this:
        #   __________
        #  /.        .\ where the dots are the orignal coordinates
        # |           | of the AABB.
        # \.________./
        # This allows the robots to navigate around the narrow corridors of shelves.

        if waypoint==True:
            inflate_dist = safety_radius + robot_radius
        else:
            inflate_dist = robot_radius
            # inflate_dist = 0.0

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
            # Return closest point of intersection
            intersection_list = [i1,i2,i3,i4,i5,i6,i7,i8]
            valid_i = [i for i in intersection_list if i is not None]
            min_dist = np.inf
            closest_intersection = valid_i[0]
            for intersection in valid_i:
                dist = np.linalg.norm(line_start-intersection)
                if dist < min_dist:
                    min_dist = dist
                    closest_intersection = intersection

            return (closest_intersection, (x0_+x1_)/2, (y0_+y1_)/2)

    return True

def check_collision(pos:Tuple[float, float], obstacles:List[Tuple[float,float,float,float]],
    safety_radius:float=0.1, robot_radius:float=0.35,
    use_safety_radius:bool=True):
    '''
    Args:
    - pos (x,y)
    - use_safety_radius: If safety radius is to be added to the inflation radius.

    Checks proposed point (x,y) if it will collide with any of the obstacles.
    
    First inflates obstacles by the safety radius, and returns false if 
    the proposed point lies within the expanded obstacle (a would-be collision)

    Returns a valid point that lies outside an obstacle. This assumes that the proposed
    point only collides with one obstacle (probably a valid assumption)
    '''
    eff_safety_radius = (robot_radius + safety_radius) if use_safety_radius else robot_radius

    for obstacle in obstacles:
        obstacle_expanded = (
            obstacle[0] - eff_safety_radius,
            obstacle[1] - eff_safety_radius,
            obstacle[2] + eff_safety_radius,
            obstacle[3] + eff_safety_radius
        )
        obs_dist, closest_point = dist_to_aabb(pos[0], pos[1], obstacle_expanded, get_closest_point=True)
        # Use copysign here because there is the possibility of -0.0 being returned
        if np.copysign(1, obs_dist) < 0:
            return False, closest_point

    return True, pos