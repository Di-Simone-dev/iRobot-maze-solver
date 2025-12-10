import py_trees
from helpers import *

class CheckKidnap(py_trees.behaviour.Behaviour):

    def __init__(self, name="Check Kidnap"):
        super().__init__(name)
        self.BB = self.attach_blackboard_client(name=self.name)
        self.BB.register_key(key="is_kidnapped", access=py_trees.common.Access.READ)
        

    def update(self):
        if BB.get("is_kidnapped"):
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS