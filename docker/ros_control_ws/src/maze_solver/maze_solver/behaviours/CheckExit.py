import py_trees
from helpers import *

#Si controlla di essere arrivati all'uscita
class CheckExit(py_trees.behaviour.Behaviour):
    def __init__(self, name="Check Exit"):
        super().__init__(name)

    def update(self):
        if BB.get("pose") == BB.get("goal"):
            BB.set("reached_exit", True)
            BB.set("last_action", "Reached Exit!")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE