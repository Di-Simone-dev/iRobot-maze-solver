import py_trees
from helpers import *

class IsPaused(py_trees.behaviour.Behaviour):

    def __init__(self, name="Is Paused"):
        super().__init__(name)
        self.BB = self.attach_blackboard_client(name=self.name)
        self.BB.register_key(key="paused", access=py_trees.common.Access.READ)
        

    def update(self):
        if BB.get("paused"):
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS