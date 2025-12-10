import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from helpers import *

class BacktrackIfDeadEnd(Behaviour):
    def __init__(self, name="BacktrackIfDeadEnd"):
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
        current_pos = self.BB.get("current_position")
        heading = self.BB.get("heading")
        
        if current_pos is None or heading is None:
            return Status.FAILURE
        
        candidates = neighbor_cells_with_headings(current_pos, heading)
        
        # Controlla se tutte le direzioni sono state visitate più di 1 volta (dead end)
        is_dead_end = True
        for h, cell in candidates:
            if not is_free(cell):  # Salta muri
                continue
            key = f"visits_{cell}"
            if self.BB.exists(key):
                visits = self.BB.get(key)
            else:
                self.BB.set(key, 0)
                visits = 0
                
            if visits < 2:  # C'è ancora un percorso da esplorare
                is_dead_end = False
                break
        
        if not is_dead_end:
            return Status.FAILURE  # Non è dead end, continua esplorazione
        
        valid_candidates = [(h, cell) for h, cell in candidates if is_free(cell)]
        if not valid_candidates:
            return Status.FAILURE
        
        best_dir = min(valid_candidates, 
               key=lambda x: (
                   self.BB.get(f"visits_{(current_pos, x[0])}") 
                   if self.BB.exists(f"visits_{(current_pos, x[0])}") 
                   else 0
               ))[0]
        
        self.BB.set("chosen_direction", best_dir)
        #self.feedback_message = f"Dead end! Backtrack su {best_dir}"
        return Status.SUCCESS