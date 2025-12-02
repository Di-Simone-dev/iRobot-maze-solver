import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from helpers import *

class ChooseUnvisitedPath(Behaviour):
    def __init__(self, name="Choose Unvisited Path"):
        super().__init__(name)
        self.blackboard = BB

    def update(self):
        pose = self.blackboard.get("pose")
        heading = self.blackboard.get("heading")
        candidates = neighbor_cells_with_headings(pose, heading)
        
        best_dir = None
        min_visits = float('inf')
        
        for h, cell in candidates:
            key = f"visits_{cell}"
            if self.blackboard.exists(key):
                visits = self.blackboard.get(key)
            else:
                self.blackboard.set(key, 0)
                visits = 0
                
            if visits < min_visits and is_free(cell):
                min_visits = visits
                best_dir = h
        
        if best_dir is not None:
            self.blackboard.set("chosen_direction", best_dir)
            #self.feedback_message = f"Scelto: {best_dir}"
            return Status.SUCCESS
        return Status.FAILURE
                
        