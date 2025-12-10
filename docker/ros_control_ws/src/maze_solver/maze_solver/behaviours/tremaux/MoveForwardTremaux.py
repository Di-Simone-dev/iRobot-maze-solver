from py_trees.behaviour import Behaviour
from py_trees.common import Status
from helpers import *

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

    def update(self):
        if self.BB.get("reached_exit"):
            return Status.SUCCESS
        
        current_position = self.BB.get("current_position")
        chosen_dir = self.BB.get("chosen_direction")
        
        if chosen_dir is None:
            return Status.FAILURE
        
        target = forward_cell(current_position, chosen_dir)
        
        if not is_free(target):
            return Status.FAILURE
        
        visits_key = f"visits_{current_position}"
        visits = self.BB.get(visits_key) if self.BB.exists(visits_key) else 0
        self.BB.set(visits_key, visits + 1)
        
        self.BB.set("current_position", target)
        self.BB.set("heading", chosen_dir)  # Aggiorna orientamento
        self.BB.set("last_action", f"Move {chosen_dir}° → {target}")
        visited = self.BB.get("visited")
        visited.add(target)
        self.BB.set("visited", visited)
        
        if target == self.BB.get("goal_position"):
            self.BB.set("reached_exit", True)
        
        return Status.SUCCESS