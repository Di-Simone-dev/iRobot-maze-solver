import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from maze_solver.helpers import *
from custom_msg.action import ActuatorMove
from maze_solver import config

class MoveForwardTremaux(Behaviour):
    def __init__(self, name="Move Forward Trémaux"):
        super().__init__(name)
        self.BB = self.attach_blackboard_client(name=self.name)
        self.BB.register_key(key="current_position", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="goal_position", access=py_trees.common.Access.READ)
        self.BB.register_key(key="paused", access=py_trees.common.Access.READ)
        self.BB.register_key(key="kidnapped", access=py_trees.common.Access.READ)
        self.BB.register_key(key="hazard", access=py_trees.common.Access.READ)
        self.BB.register_key(key="ir_sensors", access=py_trees.common.Access.READ)
        self.BB.register_key(key="lidar_scan", access=py_trees.common.Access.READ)
        self.BB.register_key(key="heading", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="reached_exit", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="last_action", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="visited", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="allow_visit_fallback", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="algorithm_mode", access=py_trees.common.Access.READ)
        self.BB.register_key(key="pledge_counter", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="heading_global", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="busy", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="actuator_movement_action_client", access=py_trees.common.Access.READ)

    def update(self):
        if self.BB.get("reached_exit"):
            return Status.SUCCESS
        
        current_position = self.BB.get("current_position")
        chosen_dir = self.BB.get("chosen_direction")
        
        self.BB.set("busy", True)
        
        target = forward_cell(current_position, chosen_dir)
        
        visits_key = f"visits_{current_position}"
        visits = self.BB.get(visits_key) if self.BB.exists(visits_key) else 0
        self.BB.set(visits_key, visits + 1)
        
        self.BB.set("current_position", target)

        # Movement
        goal_msg = ActuatorMove.Goal()
        goal_msg.type = "Distance"
        goal_msg.distance = config.MOVEMENT_DISTANCE
        goal_msg.max_speed = config.MOVEMENT_SPEED
        actuator_movement_action_client = self.BB.get("actuator_movement_action_client")
        future = actuator_movement_action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.end_movement_callback)


        self.BB.set("last_action", f"Move {chosen_dir}° → {target}")
        visited = self.BB.get("visited")
        visited.add(target)
        self.BB.set("visited", visited)
        
        if target == self.BB.get("goal_position"):
            self.BB.set("reached_exit", True)
        
        return Status.SUCCESS
    
    def end_movement_callback(self, future):
        self.BB.set("busy", False)