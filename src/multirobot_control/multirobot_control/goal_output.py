
from dataclasses import dataclass
from typing import Dict, Tuple, List

@dataclass
class goal_output():
    goal_coords: Tuple[float, float]
    start_coords: Tuple[float, float] = (0,0)
    distance_travelled: float = -1
    num_waypoints: int = -1
    start_time: float = -1
    plan_time: float = -1
    completion_time: float = -1

    def to_dict(self) -> Dict :
        ''' Returns the output as a dictionary, so that we can easily write to YAML log file later. '''
        return {
            'goal_coords' : self.goal_coords,
            'start_coords' : self.start_coords,
            'distance_travelled' : self.distance_travelled,
            'num_waypoints' : self.num_waypoints,
            'start_time' : self.start_time,
            'plan_time': self.plan_time,
            'completion_time' : self.completion_time,
        }

@dataclass
class result_summary():
    avg_num_completed_goals: float = None
    avg_waypoint_dist: float = None
    avg_dist_travelled: float = None
    avg_total_time: float = None
    avg_plan_time: float = None
    avg_move_time: float = None