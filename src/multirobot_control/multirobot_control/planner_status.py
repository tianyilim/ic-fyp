from enum import Enum, auto

# Make sure this is the same order as `planner_action_interfaces.srv.GetPlannerStatus.srv`

class PlannerStatus(Enum):
    PLANNER_EXEC = auto()
    PLANNER_PLAN = auto()
    PLANNER_READY = auto()