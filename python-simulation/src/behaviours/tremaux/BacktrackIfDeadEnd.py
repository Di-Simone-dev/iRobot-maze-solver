import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from helpers import *

class BacktrackIfDeadEnd(Behaviour):
    def __init__(self, name="BacktrackIfDeadEnd"):
        super().__init__(name)
        self.blackboard = BB

    def update(self):
        pose = self.blackboard.get("pose")
        heading = self.blackboard.get("heading")
        
        if pose is None or heading is None:
            return Status.FAILURE
        
        candidates = neighbor_cells_with_headings(pose, heading)
        
        # Controlla se tutte le direzioni sono state visitate più di 1 volta (dead end)
        is_dead_end = True
        for h, cell in candidates:
            if not is_free(cell):  # Salta muri
                continue
            key = f"visits_{cell}"
            if self.blackboard.exists(key):
                visits = self.blackboard.get(key)
            else:
                self.blackboard.set(key, 0)
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
                   self.blackboard.get(f"visits_{(pose, x[0])}") 
                   if self.blackboard.exists(f"visits_{(pose, x[0])}") 
                   else 0
               ))[0]
        
        self.blackboard.set("chosen_direction", best_dir)
        #self.feedback_message = f"Dead end! Backtrack su {best_dir}"
        return Status.SUCCESS