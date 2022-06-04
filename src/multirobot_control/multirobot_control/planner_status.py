from enum import IntEnum, auto

# Make sure this is the same order as `planner_action_interfaces.srv.GetPlannerStatus.srv`

class PlannerStatus(IntEnum):
    PLANNER_EXEC = 0
    PLANNER_PLAN = 1
    PLANNER_READY = 2
    PLANNER_DEFERRED = 3
    PLANNER_EXEC_JOINT = 4
    PLANNER_ABORT = 5