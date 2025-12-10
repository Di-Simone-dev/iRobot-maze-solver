import py_trees
from helpers import *

class CheckHazards(py_trees.behaviour.Behaviour):

    def __init__(self, name="Check Hazards"):
        super().__init__(name)
        self.BB = self.attach_blackboard_client(name=self.name)
        self.BB.register_key(key="hazards", access=py_trees.common.Access.READ)
        

    def update(self):
        if len(BB.get("hazards")) > 0:
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS