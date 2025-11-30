from py_trees.behaviour import Behaviour
from helpers import *
from py_trees.common import Status

class AlgorithmSelector(Behaviour):
    def __init__(self, name, mode):
        super().__init__(name)
        self.mode = mode

    def update(self):
        if BB.get("algorithm_mode") == self.mode:
            return Status.SUCCESS
        else:
            return Status.FAILURE