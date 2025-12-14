import py_trees
from maze_solver.helpers import *

class ChooseDirection(py_trees.behaviour.Behaviour):

    def __init__(self, name="Choose Direction"):
        super().__init__(name)
        self.BB = self.attach_blackboard_client(name=self.name)
        self.BB.register_key(key="current_position", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="heading", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="allow_visit_fallback", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="visited", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="last_action", access=py_trees.common.Access.WRITE)

        self.BB.register_key(key="grid_size", access=py_trees.common.Access.READ)

    def update(self):
        current_position = self.BB.get("current_position")
        heading = self.BB.get("heading")
        visited = self.BB.get("visited")
        grid_size = self.BB.get("grid_size")

        candidates = neighbor_cells_with_headings(current_position, heading)

        # Try free & unvisited
        for h, cell in candidates:
            if is_free(self, cell, grid_size) and (cell not in visited):
                self.BB.set("heading", h)
                self.BB.set("allow_visit_fallback", False)
                self.B.set("last_action", f"Choose heading {h}° → unvisited {cell}")
                return py_trees.common.Status.SUCCESS

        # Fallback: any free (visited allowed)
        for h, cell in candidates:
            if is_free(self, cell, grid_size):
                self.BB.set("heading", h)
                self.BB.set("allow_visit_fallback", True)  # permit moving into visited to avoid deadlock
                self.BB.set("last_action", f"Choose heading {h}° → free {cell} (visited fallback)")
                return py_trees.common.Status.SUCCESS

        self.BB.set("last_action", "No free neighbor available")
        return py_trees.common.Status.FAILURE