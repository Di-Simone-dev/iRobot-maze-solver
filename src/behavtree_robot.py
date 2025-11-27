# bt_maze_explore_unblock_gui.py
import tkinter as tk
import py_trees
import py_trees.display  # NEW: for DOT/SVG export
import random

def save_bt_svg(root, filename="maze_solver_bt"):
    """
    Usa la vecchia API render_dot_tree per generare un file DOT e convertirlo in SVG.
    """
    # Qui passo il nodo radice, non l'oggetto BehaviourTree
    py_trees.display.render_dot_tree(
        root,
        name=filename,   # nome base del file
        target_directory=".",  # cartella corrente
        format="svg"     # formato di output
    )
    print(f"SVG salvato come {filename}.svg")


# ----------------------------
# Config
# ----------------------------
GRID_SIZE = 10
CELL_SIZE = 40

# ----------------------------
# Blackboard
# ----------------------------
BB = py_trees.blackboard.Blackboard()
BB.set("pose", (0, 0))
BB.set("goal", (GRID_SIZE - 1, GRID_SIZE - 1))
BB.set("maze_walls", set())            # blocked cells {(r,c)}
BB.set("heading", 90)                  # 0=N, 90=E, 180=S, 270=W
BB.set("reached_exit", False)
BB.set("last_action", "Idle")
BB.set("visited", {(0, 0)})            # visited cells set
BB.set("allow_visit_fallback", False)  # allows moving into visited if no unvisited options exist

# ----------------------------
# Maze generation
# ----------------------------
def generate_walls(density=0.22):
    start = (0, 0)
    goal = (GRID_SIZE - 1, GRID_SIZE - 1)
    walls = set()
    for r in range(GRID_SIZE):
        for c in range(GRID_SIZE):
            if (r, c) in [start, goal]:
                continue
            if random.random() < density:
                walls.add((r, c))
    BB.set("maze_walls", walls)

# ----------------------------
# Helpers
# ----------------------------
def forward_cell(pose, heading):
    r, c = pose
    if heading == 0:     return (r - 1, c)
    if heading == 90:    return (r, c + 1)
    if heading == 180:   return (r + 1, c)
    if heading == 270:   return (r, c - 1)
    return pose

def in_bounds(cell):
    r, c = cell
    return 0 <= r < GRID_SIZE and 0 <= c < GRID_SIZE

def is_free(cell):
    return in_bounds(cell) and (cell not in BB.get("maze_walls"))

def left_of(heading):    return (heading - 90) % 360
def right_of(heading):   return (heading + 90) % 360
def back_of(heading):    return (heading + 180) % 360

def neighbor_cells_with_headings(pose, heading):
    return [
        (heading, forward_cell(pose, heading)),                # forward
        (left_of(heading),  forward_cell(pose, left_of(heading))),
        (right_of(heading), forward_cell(pose, right_of(heading))),
        (back_of(heading),  forward_cell(pose, back_of(heading))),
    ]

# ----------------------------
# Behaviours
# ----------------------------
class CheckExit(py_trees.behaviour.Behaviour):
    def __init__(self, name="Check Exit"):
        super().__init__(name)

    def update(self):
        if BB.get("pose") == BB.get("goal"):
            BB.set("reached_exit", True)
            BB.set("last_action", "Reached Exit!")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class ChooseDirection(py_trees.behaviour.Behaviour):
    """
    Strict priority for unvisited:
    1) Pick heading toward a free & unvisited neighbor (order: F, L, R, B).
    2) If none exist, enable allow_visit_fallback and pick any free neighbor (order: F, L, R, B).
    3) If none free, fail.
    Only sets heading; movement happens in MoveForward.
    """
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

        pose = BB.get("pose")
        heading = BB.get("heading")
        target = forward_cell(pose, heading)

        if not is_free(target):
            BB.set("last_action", f"Blocked forward at {target}")
            return py_trees.common.Status.FAILURE

        visited = BB.get("visited")
        allow_visit_fallback = BB.get("allow_visit_fallback")

        # If target visited and no fallback allowed, fail to re-choose direction
        if (target in visited) and not allow_visit_fallback:
            BB.set("last_action", f"Forward visited {target} → reselect direction")
            return py_trees.common.Status.FAILURE

        # Move
        BB.set("pose", target)
        visited.add(target)
        BB.set("visited", visited)
        BB.set("last_action", f"Move to {target}")
        BB.set("allow_visit_fallback", False)  # reset after movement

        if target == BB.get("goal"):
            BB.set("reached_exit", True)
        return py_trees.common.Status.SUCCESS

# ----------------------------
# Build behaviour tree (reactive, unblocked)
# ----------------------------
def build_tree():
    # Sequence ensures we always choose heading before moving
    root = py_trees.composites.Sequence("Explorer Unblocked", memory=True)
    root.add_children([
        py_trees.composites.Selector("Exit or Choose", memory=True, children=[
            CheckExit("Check Exit"),
            ChooseDirection("Choose Direction")
        ]),
        MoveForward("Move Forward"),
    ])
    return root

# ----------------------------
# Export BT SVG (NEW)
# ----------------------------
def save_bt_svg(root, filename="maze_solver_bt.svg"):
    """
    Generate a DOT graph of the behaviour tree and save it as SVG.
    Requires Graphviz installed and available (dot).
    """
    graph = py_trees.display.generate_dot_tree(
        root=root,
        with_blackboard_variables=True,
        collapse_decorators=False
    )
    graph.write(filename, format="svg")
    print(f"Saved behaviour tree SVG to: {filename}")

# ----------------------------
# GUI
# ----------------------------
class MazeGUI:
    def __init__(self, root_node):
        self.tree = py_trees.trees.BehaviourTree(root_node)
        self.root_node = root_node  # keep a reference for SVG export
        self.win = tk.Tk()
        self.win.title("Exploration Maze Solver GUI 10x10 (Unvisited Priority + Unblock)")

        self.canvas = tk.Canvas(self.win, width=GRID_SIZE*CELL_SIZE, height=GRID_SIZE*CELL_SIZE)
        self.canvas.pack(padx=10, pady=10)

        ctrl = tk.Frame(self.win)
        ctrl.pack(pady=6)
        tk.Button(ctrl, text="Start", command=self.start).pack(side=tk.LEFT, padx=4)
        tk.Button(ctrl, text="Step", command=self.step).pack(side=tk.LEFT, padx=4)
        tk.Button(ctrl, text="Reset Maze", command=self.reset_maze).pack(side=tk.LEFT, padx=4)
        tk.Button(ctrl, text="Save BT SVG", command=self.save_svg).pack(side=tk.LEFT, padx=4)  # NEW: button

        self.status = tk.StringVar()
        tk.Label(self.win, textvariable=self.status).pack(fill=tk.X, padx=10, pady=4)

        self.log = tk.StringVar()
        tk.Label(self.win, textvariable=self.log, fg="blue").pack(fill=tk.X, padx=10, pady=4)

        generate_walls(density=0.22)
        self.running = False

        self.draw_maze()
        self.update_status()

        self.win.after(200, self.loop)

    def draw_maze(self):
        self.canvas.delete("all")
        walls = BB.get("maze_walls")
        pose = BB.get("pose")
        goal = BB.get("goal")
        heading = BB.get("heading")
        visited = BB.get("visited")

        for r in range(GRID_SIZE):
            for c in range(GRID_SIZE):
                x1, y1 = c * CELL_SIZE, r * CELL_SIZE
                x2, y2 = x1 + CELL_SIZE, y1 + CELL_SIZE

                fill = "white"
                if (r, c) in walls:
                    fill = "black"
                elif (r, c) == goal:
                    fill = "red"
                elif (r, c) == pose:
                    fill = "lime green"
                elif (r, c) in visited:
                    fill = "#d3d3d3"  # light gray for visited

                self.canvas.create_rectangle(x1, y1, x2, y2, fill=fill, outline="gray")

        # orientation arrow over robot
        pr, pc = pose
        cx = pc * CELL_SIZE + CELL_SIZE / 2
        cy = pr * CELL_SIZE + CELL_SIZE / 2
        if heading == 0:
            self.canvas.create_line(cx, cy, cx, cy - 15, arrow=tk.LAST, width=3)
        elif heading == 90:
            self.canvas.create_line(cx, cy, cx + 15, cy, arrow=tk.LAST, width=3)
        elif heading == 180:
            self.canvas.create_line(cx, cy, cx, cy + 15, arrow=tk.LAST, width=3)
        elif heading == 270:
            self.canvas.create_line(cx, cy, cx - 15, cy, arrow=tk.LAST, width=3)

    def update_status(self):
        pose = BB.get("pose")
        goal = BB.get("goal")
        reached = BB.get("reached_exit")
        visited_count = len(BB.get("visited"))
        self.status.set(f"Pose: {pose} | Goal: {goal} | Exit: {reached} | Visited: {visited_count}")
        self.log.set(f"Last action: {BB.get('last_action')}")

    def start(self):
        self.running = True

    def step(self):
        self.tree.tick()
        self.draw_maze()
        self.update_status()

    def reset_maze(self):
        BB.set("pose", (0, 0))
        BB.set("goal", (GRID_SIZE - 1, GRID_SIZE - 1))
        BB.set("heading", 90)
        BB.set("reached_exit", False)
        BB.set("last_action", "Maze reset")
        BB.set("visited", {(0, 0)})
        BB.set("allow_visit_fallback", False)
        generate_walls(density=0.22)
        self.draw_maze()
        self.update_status()
        self.running = False

    def save_svg(self):
        # NEW: export current behaviour tree to SVG
        save_bt_svg(self.root_node, filename="maze_solver_bt.svg")

    def loop(self):
        if self.running and not BB.get("reached_exit"):
            self.tree.tick()
            self.draw_maze()
            self.update_status()
            if BB.get("reached_exit"):
                self.running = False
        self.win.after(200, self.loop)

    def run(self):
        self.win.mainloop()



# ----------------------------
# Export BT SVG
# ----------------------------
def save_bt_svg(root, filename="maze_solver_bt.svg"):
    graph = py_trees.display.generate_dot_tree(
        root=root,
        with_blackboard_variables=True,
        collapse_decorators=False
    )
    graph.write(filename, format="svg")
    print(f"Saved behaviour tree SVG to: {filename}")

# ----------------------------

# ----------------------------
# Main
# ----------------------------
if __name__ == "__main__":
    root = build_tree()
    tree = py_trees.trees.BehaviourTree(root)
    #save_bt_svg(root, filename="maze_solver_bt")
    py_trees.display.render_dot_tree(root, name="maze")


    gui = MazeGUI(root)
    gui.run()