import py_trees
from helpers import *

class ChooseDirection(py_trees.behaviour.Behaviour):

    def __init__(self, name="Choose Direction"):
        super().__init__(name)

    def update(self):
        pose = BB.get("pose")
        heading = BB.get("heading")
        visited = BB.get("visited")

        candidates = neighbor_cells_with_headings(pose, heading)

        # Try free & unvisited
        for h, cell in candidates:
            if is_free(cell) and (cell not in visited):
                BB.set("heading", h)
                BB.set("allow_visit_fallback", False)
                BB.set("last_action", f"Choose heading {h}° → unvisited {cell}")
                return py_trees.common.Status.SUCCESS

        # Fallback: any free (visited allowed)
        for h, cell in candidates:
            if is_free(cell):
                BB.set("heading", h)
                BB.set("allow_visit_fallback", True)  # permit moving into visited to avoid deadlock
                BB.set("last_action", f"Choose heading {h}° → free {cell} (visited fallback)")
                return py_trees.common.Status.SUCCESS

        BB.set("last_action", "No free neighbor available")
        return py_trees.common.Status.FAILURE