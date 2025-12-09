import py_trees
from py_trees.common import Status
from helpers import *

class MoveForward(py_trees.behaviour.Behaviour):
    """
    Moves forward:
    - Prefers unvisited cells.
    - If no unvisited options existed (allow_visit_fallback=True), allows moving into visited to escape loops.
    - Records visited after moving.
    """
    def __init__(self, name="Move Forward"):
        super().__init__(name)

    def update(self):
        if BB.get("reached_exit"):
            return py_trees.common.Status.SUCCESS

        #print("MOVIMENTO")
        pose = BB.get("pose")
        heading = BB.get("heading")
        target = forward_cell(pose, heading)


        if not is_free(target):
            BB.set("last_action", f"Blocked forward at {target}")
            
            return py_trees.common.Status.FAILURE

        visited = BB.get("visited")
        allow_visit_fallback = BB.get("allow_visit_fallback")


        #TEST
        BB.set("allow_visit_fallback", False)  # reset after movement
        # If target visited and no fallback allowed, fail to re-choose direction
        """ if (target in visited) and not allow_visit_fallback:
            #print("visita riuscita")
            BB.set("last_action", f"Forward visited {target} → reselect direction")
            return py_trees.common.Status.FAILURE """

        # Move
        BB.set("pose", target)  #MOVIMENTO EFFETTIVO DEL ROBOT
        visited.add(target)        
        BB.set("visited", visited)
        BB.set("last_action", f"Move to {target}")
        BB.set("allow_visit_fallback", False)  # reset after movement

        if target == BB.get("goal"):
            BB.set("reached_exit", True)
        return Status.SUCCESS