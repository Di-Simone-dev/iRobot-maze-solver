import py_trees
from helpers import *
import sys

class IsPaused(py_trees.behaviour.Behaviour):

    def __init__(self, name="Is Busy"):
        super().__init__(name)
        self.BB = self.attach_blackboard_client(name=self.name)
        self.BB.register_key(key="busy", access=py_trees.common.Access.READ)
        

    def update(self):
        if BB.get("busy"):
            sys.sleep(50)
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS