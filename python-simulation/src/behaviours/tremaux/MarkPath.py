import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from helpers import *

class MarkPath(Behaviour):
    def __init__(self, name="Mark Path"):
        super().__init__(name)
        self.blackboard = BB

    def update(self):
        current_pos = self.blackboard.get("pose")
        key = f"visits_{current_pos}"
        if self.blackboard.exists(key):
            visits = self.blackboard.get(key)
        else:
            visits = 0
        self.blackboard.set(key, visits + 1)
        #self.feedback_message = f"Posizione {current_pos} marcata: {visits + 1}"
        return Status.SUCCESS
                
        