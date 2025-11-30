from py_trees.behaviour import Behaviour
from py_trees.common import Status
from helpers import *

class MoveForwardTremaux(Behaviour):
    def __init__(self, name="Move Forward Trémaux"):
        super().__init__(name)
        self.blackboard = BB

    def update(self):
        if self.blackboard.get("reached_exit"):
            return Status.SUCCESS
        
        pose = self.blackboard.get("pose")
        chosen_dir = self.blackboard.get("chosen_direction")
        
        if chosen_dir is None:
            return Status.FAILURE
        
        target = forward_cell(pose, chosen_dir)
        
        if not is_free(target):
            return Status.FAILURE
        
        visits_key = f"visits_{pose}"
        visits = self.blackboard.get(visits_key) if self.blackboard.exists(visits_key) else 0
        self.blackboard.set(visits_key, visits + 1)
        
        self.blackboard.set("pose", target)
        self.blackboard.set("heading", chosen_dir)  # Aggiorna orientamento
        self.blackboard.set("last_action", f"Move {chosen_dir}° → {target}")
        visited = self.blackboard.get("visited")
        visited.add(target)
        self.blackboard.set("visited", visited)
        
        if target == self.blackboard.get("goal"):
            self.blackboard.set("reached_exit", True)
        
        return Status.SUCCESS