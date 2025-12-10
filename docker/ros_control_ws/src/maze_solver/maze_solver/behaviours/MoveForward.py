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
        self.BB = self.attach_blackboard_client(name=self.name)
        self.BB.register_key(key="reached_exit", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="current_position", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="heading", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="last_action", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="visited", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="allow_visit_fallback", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="goal_position", access=py_trees.common.Access.READ)
        

    def update(self):
        if self.BB.get("reached_exit"):
            return py_trees.common.Status.SUCCESS

        #print("MOVIMENTO")
        current_position = self.BB.get("current_position")
        heading = self.BB.get("heading")
        target = forward_cell(current_position, heading)


        if not is_free(target):
            self.BB.set("last_action", f"Blocked forward at {target}")
            return py_trees.common.Status.FAILURE

        visited = self.BB.get("visited")
        allow_visit_fallback = self.BB.get("allow_visit_fallback")


        #TEST
        self.BB.set("allow_visit_fallback", False)  # reset after movement
        # If target visited and no fallback allowed, fail to re-choose direction
        """ if (target in visited) and not allow_visit_fallback:
            #print("visita riuscita")
            BB.set("last_action", f"Forward visited {target} → reselect direction")
            return py_trees.common.Status.FAILURE """

        # Move
        self.BB.set("current_position", target)  #MOVIMENTO EFFETTIVO DEL ROBOT
        visited.add(target)
        self.BB.set("visited", visited)
        self.BB.set("last_action", f"Move to {target}")
        self.BB.set("allow_visit_fallback", False)  # reset after movement

        if target == self.BB.get("goal_position"):
            self.BB.set("reached_exit", True)
        return Status.SUCCESS