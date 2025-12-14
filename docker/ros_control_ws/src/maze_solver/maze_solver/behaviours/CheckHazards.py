import py_trees
from maze_solver.helpers import *

class CheckHazards(py_trees.behaviour.Behaviour):

    def __init__(self, name="Check Hazards"):
        super().__init__(name)
        self.BB = self.attach_blackboard_client(name=self.name)
        self.BB.register_key(key="hazards", access=py_trees.common.Access.READ)
        self.BB.register_key(key="logger", access=py_trees.common.Access.READ)
        

    def update(self):
        self.BB.get("logger").info(f"Hazards: {self.BB.get("hazards")}")
        if len(self.BB.get("hazards")) > 0:
            self.BB.get("logger").info("System has hazards!")
            return py_trees.common.Status.SUCCESS
        self.BB.get("logger").info("System has NO hazards!")
        return py_trees.common.Status.FAILURE