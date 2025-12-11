import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from maze_solver.helpers import *

class ChooseUnvisitedPath(Behaviour):
    def __init__(self, name="Choose Unvisited Path"):
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
        self.BB.register_key(key="maze_walls", access=py_trees.common.Access.WRITE)

    def update(self):
        current_pos = self.BB.get("current_position")
        heading = self.BB.get("heading")
        candidates = neighbor_cells_with_headings(current_pos, heading)
        
        best_dir = None
        min_visits = float('inf')
        
        for h, cell in candidates:
            key = f"visits_{cell}"
            self.BB.register_key(key=key, access=py_trees.common.Access.WRITE)
            if self.BB.exists(key):
                visits = self.BB.get(key)
            else:
                self.BB.set(key, 0)
                visits = 0
                
            if visits < min_visits and is_free(self, cell):
                min_visits = visits
                best_dir = h
        
        if best_dir is not None:
            self.BB.set("chosen_direction", best_dir)
            #self.feedback_message = f"Scelto: {best_dir}"
            return Status.SUCCESS
        return Status.FAILURE
                
        