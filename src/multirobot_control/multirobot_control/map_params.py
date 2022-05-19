# This is specific to the "warehouse" map, `worlds/factory_world2.world`

# Has 3 cols and 5 rows of shelves of dimension 4x0.8m, spaced 1.5m apart.
# Assume that the goals should be around ~40cm from the shelves.

# Have goals like so:
#    x    x    x
# |---------------|
# |-------O-------|
# |---------------|
#    x    x    x
# Spaced around each shelf, and O is the origin of each shelf

# Obstacles are considered as rectangles to use the AABB <-> Circle collision-detection algorithm later.

from typing import List, Tuple

GOAL_ARRAY: List[Tuple[float, float]] = []
OBSTACLE_ARRAY: List[Tuple[float, float, float, float]] = []

# Iterate through shelves
total_x = 3
total_y = 5
for x_coord in range(3):
    for y_coord in range(5):
        # Iterate through each shelf

        x = x_coord - 1 # Iterates from -1, 0, 1
        y = y_coord - 2 # Iterates from -2, 0, 2

        x_pos = x*5.5   # Values taken from Gazebo
        y_pos = y*2.3
        
        # Goals are (x,y) coords
        GOAL_ARRAY.append((x_pos+1, y_pos+0.8))
        GOAL_ARRAY.append((x_pos, y_pos+0.8))
        GOAL_ARRAY.append((x_pos-1, y_pos+0.8))
        GOAL_ARRAY.append((x_pos+1, y_pos-0.8))
        GOAL_ARRAY.append((x_pos, y_pos-0.8))
        GOAL_ARRAY.append((x_pos-1, y_pos-0.8))

        # Obstacles are ((x1,y1,x2,y2)) coords, where x1<x2, y1<y2
        # Based on the size (4x0.8) of the shelves
        OBSTACLE_ARRAY.append( (x_pos-2, y_pos-0.4, x_pos+2, y_pos+0.4) )

# Iterate through walls
wall_x = 9.1
wall_y = 6.6
OBSTACLE_ARRAY.append( ( wall_x-0.1, -wall_y,  wall_x+0.1, wall_y) )
OBSTACLE_ARRAY.append( (-wall_x-0.1, -wall_y, -wall_x+0.1, wall_y) )
OBSTACLE_ARRAY.append( (-wall_x,  wall_y-0.1, wall_x,  wall_y+0.1) )
OBSTACLE_ARRAY.append( (-wall_x, -wall_y-0.1, wall_x, -wall_y+0.1) )

OBSTACLE_BOUND = (-9.1, -6.6, 9.1, 6.6)

# Debugging
# for aabb in OBSTACLE_ARRAY:
#     print(f"x0:{aabb[0]:.2f} y0:{aabb[1]:.2f} x1:{aabb[2]:.2f} y1:{aabb[3]:.2f} ")
# print("-----")
# print("-----")
# for aabb in GOAL_ARRAY:
#     print(f"x0:{aabb[0]:.2f} y0:{aabb[1]:.2f}")
