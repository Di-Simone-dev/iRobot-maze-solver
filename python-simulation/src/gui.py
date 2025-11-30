# ----------------------------
# GUI
# ----------------------------

import tkinter as tk
import py_trees
from config import *
from helpers import *
from mazegenerator import *
import os #per la gestione delle cartelle nei salvataggi dei diagrammi

class MazeGUI:
    def __init__(self, root_node):
        self.tree = py_trees.trees.BehaviourTree(root_node)
        self.root_node = root_node  # keep a reference for SVG export
        self.win = tk.Tk()
        self.win.title("Maze solver simulator GUI")

        self.canvas = tk.Canvas(self.win, width=GRID_SIZE*CELL_SIZE, height=GRID_SIZE*CELL_SIZE)
        self.canvas.pack(padx=10, pady=10)

        ctrl = tk.Frame(self.win)
        ctrl.pack(pady=6)
        tk.Button(ctrl, text="Start", command=self.start).pack(side=tk.LEFT, padx=4)
        tk.Button(ctrl, text="Stop", command=self.stop).pack(side=tk.LEFT, padx=4)
        tk.Button(ctrl, text="Step", command=self.step).pack(side=tk.LEFT, padx=4)
        tk.Button(ctrl, text="Reset Maze", command=self.reset_maze).pack(side=tk.LEFT, padx=4)
        tk.Button(ctrl, text="New Maze", command=self.new_maze).pack(side=tk.LEFT, padx=4)
        tk.Button(ctrl, text="Pledge", command=self.select_pledge).pack(side=tk.LEFT, padx=4)
        tk.Button(ctrl, text="Tremaux", command=self.select_tremaux).pack(side=tk.LEFT, padx=4)
        tk.Button(ctrl, text="Save BT SVG", command=self.save_svg).pack(side=tk.LEFT, padx=4) 

        self.status = tk.StringVar()
        tk.Label(self.win, textvariable=self.status).pack(fill=tk.X, padx=10, pady=4)

        self.log = tk.StringVar()
        tk.Label(self.win, textvariable=self.log, fg="blue").pack(fill=tk.X, padx=10, pady=4)

        generate_walls()
        self.running = False

        self.draw_maze()
        self.update_status()

        self.win.after(UPDATE_PERIOD, self.loop)   #AGGIORNAMENTO OGNI 100ms (10 FPS)

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
        algo = BB.get("algorithm_mode")
        #self.status.set(f"Pose: {pose} | Goal: {goal} | Exit: {reached} | Visited: {visited_count}")
        self.status.set(f"Pose: {pose} | Goal: {goal} | Exit: {reached} | Visited: {visited_count} | Algorithm: {algo}")
        self.log.set(f"Last action: {BB.get('last_action')}")

    def start(self):
        self.running = True

    def step(self):   #Lo step aumenta di 1 tick e aggiunta la schermata e lo status
        self.tree.tick()
        self.draw_maze()
        self.update_status()

    def new_maze(self):
        BB.set("pose", START)
        BB.set("goal", GOAL)
        BB.set("heading", 90)
        BB.set("reached_exit", False)
        BB.set("last_action", "New Maze")
        BB.set("visited", {START})
        BB.set("allow_visit_fallback", False)
        BB.set("pledge_counter", 0)
        generate_walls()
        self.draw_maze()
        self.update_status()
        self.running = False

    def reset_maze(self):
        BB.set("pose", START)
        BB.set("goal", GOAL)
        BB.set("heading", 90)
        BB.set("reached_exit", False)
        BB.set("last_action", "Maze reset")
        BB.set("visited", {START})
        BB.set("allow_visit_fallback", False)
        BB.set("pledge_counter", 0)
        self.draw_maze()
        self.update_status()
        self.running = False

    def loop(self):
        if self.running and not BB.get("reached_exit"):

            #Questi 3 equivalgono allo step
            self.tree.tick()
            self.draw_maze()
            self.update_status()

            if BB.get("reached_exit"):#STOP SOLO SE RAGGIUNGE L'USCITA
                self.running = False
        self.win.after(UPDATE_PERIOD, self.loop)

    def stop(self):
        self.running = False

    def run(self):
        self.win.mainloop()

    def save_svg(self):
        #py_trees.display.render_dot_tree(self.root_node, name="maze", target_directory="./graphical_trees")
        output_dir = "./graphical_trees"
        os.makedirs(output_dir, exist_ok=True)  # crea la cartella se non esiste
        py_trees.display.render_dot_tree(
            self.root_node,
            name="maze",
            target_directory=output_dir
        )


    def select_pledge(self):
        BB.set("algorithm_mode", "pledge")
        self.update_status()
    
    def select_tremaux(self):
        BB.set("algorithm_mode", "tremaux")
        self.update_status()
