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

#L'inserimento di randomizzazione all'interno della generazione risulta ideale per:
#Avere uno start non necessariamente al bordo del labirinto
#Settare la stazione di docking allo spawn e far tornare il robot alla stazione per la ricarica
#la generazione di maze (percorsi di dimensione 1) o di semplici mappe (implementazione attuale) con densità variabile
#ma tenendo a mente di non tralasciare la raggiungibilità dell'obiettivo




# ----------------------------
# Configurazioni
# ----------------------------
GRID_SIZE = 21  #Numero di celle nel lato della griglia quadrata (DISPARI)
CELL_SIZE = 30  #Dimensione celle in PIXEL
DENSITY = 0.20  #Densità di muri  [0,1] 
#MAZE = True
START = (1, 1)# RANDOMIZZABILE idealmente
GOAL = (GRID_SIZE - 2, GRID_SIZE - 2)
# ----------------------------
# Blackboard (memoria condivisa tra i nodi)
# ----------------------------
BB = py_trees.blackboard.Blackboard()
BB.set("pose", START)   #SET POSIZIONE INIZIALE
BB.set("goal", GOAL)     #SET OBIETTIVO
BB.set("maze_walls", set())            # blocked cells {(r,c)}
BB.set("heading", 90)                  # 0=N, 90=E, 180=S, 270=W SET ORIENTAMENTO INIZIALE
BB.set("reached_exit", False)
BB.set("last_action", "Idle")
BB.set("visited", {START})            # visited cells set  (Visita dello start)
BB.set("allow_visit_fallback", False)  # allows moving into visited if no unvisited options exist

#SCELTA ALGORITMO (HARDCODATA PER ORA)
BB.set("algorithm_mode", "pledge")
BB.set("pledge_counter", 0)
BB.set("heading_global", 180)   # o la direzione che vuoi come riferimento

# ----------------------------
# Maze generation, chiamato al init e al reset, ha densità parametrica
# ----------------------------
def generate_walls(density=DENSITY):
    reachable = False
    while(not reachable):#piccolo trucchetto per ottenere un maze risolvibile dal robot
        start = START
        goal = (GRID_SIZE - 2, GRID_SIZE - 2)#hardcodato ma randomizzabile

        # Tutte le celle inizialmente muri
        walls = {(r, c) for r in range(GRID_SIZE) for c in range(GRID_SIZE)}
        walls.remove(start)
        walls.remove(goal)

        visited = set()
        stack = [start]

        # Direzioni (N, S, E, W)
        directions = [(-1, 0), (1, 0), (0, 1), (0, -1)]

        while stack:
            current = stack[-1]
            visited.add(current)

            # Trova vicini non visitati a distanza 2 (per mantenere corridoi larghi 1)
            neighbors = []
            for dr, dc in directions:
                nr, nc = current[0] + 2*dr, current[1] + 2*dc
                if 0 <= nr < GRID_SIZE and 0 <= nc < GRID_SIZE   and (nr, nc) not in visited:
                    neighbors.append((nr, nc))

            if neighbors:
                next_cell = random.choice(neighbors)
                # Rimuovi il muro tra current e next_cell
                wall = (current[0] + (next_cell[0] - current[0]) // 2,
                        current[1] + (next_cell[1] - current[1]) // 2)
                if wall in walls:
                    walls.remove(wall)
                if next_cell in walls:
                    walls.remove(next_cell)
                stack.append(next_cell)
            else:
                stack.pop()

        BB.set("maze_walls", walls)
        #print(walls)
        #reachable = True
        reachable = is_reachable(START,GOAL,walls)
        #print(reachable)

#Con questo sono sicuro che il maze sia risolvibile
def is_reachable(start, goal, walls):
    from collections import deque
    q = deque([start])
    visited = {start}
    directions = [(1,0),(-1,0),(0,1),(0,-1)]
    while q:
        r,c = q.popleft()
        if (r,c) == goal:
            return True
        for dr,dc in directions:
            nr,nc = r+dr, c+dc
            if 0 <= nr < GRID_SIZE and 0 <= nc < GRID_SIZE:
                if (nr,nc) not in walls and (nr,nc) not in visited:
                    visited.add((nr,nc))
                    q.append((nr,nc))
    return False

# ----------------------------
# Helpers
# ----------------------------

def left_cell(pose, heading):
    return forward_cell(pose, (heading - 90) % 360)

def right_cell(pose, heading):
    return forward_cell(pose, (heading + 90) % 360)

def forward_cell(pose, heading):  #VISUALIZZAZIONE CELLA IN BASE A POSIZIONE E ORIENTAMENTO
    r, c = pose
    if heading == 0:     return (r - 1, c)
    if heading == 90:    return (r, c + 1)
    if heading == 180:   return (r + 1, c)
    if heading == 270:   return (r, c - 1)
    return pose

def leftforward_cell(pose, heading):
    fwd = forward_cell(pose, heading)
    return left_cell(fwd, heading)

def rightforward_cell(pose, heading):
    fwd = forward_cell(pose, heading)
    return right_cell(fwd, heading)

#POSTERIORI
def backward_cell(pose, heading):
    return forward_cell(pose, (heading + 180) % 360)

def backleft_cell(pose, heading):
    back = backward_cell(pose, heading)
    return left_cell(back, heading)

def backright_cell(pose, heading):
    back = backward_cell(pose, heading)
    return right_cell(back, heading)



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
class CheckExit(py_trees.behaviour.Behaviour):#Si controlla di essere arrivati all'uscita
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


        #TEST
        BB.set("allow_visit_fallback", False)  # reset after movement
        # If target visited and no fallback allowed, fail to re-choose direction
        """ if (target in visited) and not allow_visit_fallback:
            print("visita riuscita")
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
        return py_trees.common.Status.SUCCESS

# ----------------------------
# Build behaviour tree (reactive, unblocked)
# ----------------------------
""" def build_tree():   
    # Sequence ensures we always choose heading before moving
    root = py_trees.composites.Sequence("Explorer Unblocked", memory=True)
    root.add_children([
        py_trees.composites.Selector("Exit or Choose", memory=True, children=[
            CheckExit("Check Exit"),
            ChooseDirection("Choose Direction")
        ]),
        MoveForward("Move Forward"),
    ])
    return root """

def PledgeSubTree(name="Pledge"):
    return py_trees.composites.Sequence(name, memory=True, children=[
        UpdateDeviationCounter("Update Counter"),
        py_trees.composites.Selector("Aligned or Follow", memory=True, children=[
            py_trees.composites.Sequence("Aligned Straight", memory=True, children=[
                AlignedWithGlobal("Check Alignment"),
                ChooseStraightIfFree("Choose Straight"),
            ]),
            FollowWall("Follow Wall"),
        ]),
        MoveForward("Move Forward"),
    ])



#STEP 1 DEL PLEDGE
class UpdateDeviationCounter(py_trees.behaviour.Behaviour):
    def __init__(self, name="Update Counter"):
        super().__init__(name)

    def update(self):
        counter = BB.get("pledge_counter") if BB.exists("pledge_counter") else 0
        last_action = BB.get("last_action") if BB.exists("last_action") else ""

        if "Turn Left" in last_action: counter += 1
        elif "Turn Right" in last_action: counter -= 1

        BB.set("pledge_counter", counter)
        # don't overwrite last_action used by other nodes
        BB.set("pledge_counter_log", f"Pledge counter: {counter}")
        return py_trees.common.Status.SUCCESS

#STEP 2 DEL PLEDGE
#controlla se l'allineamento globale (deciso all'inizio sta venendo seguito, altrimenti faila)
class AlignedWithGlobal(py_trees.behaviour.Behaviour):
    def __init__(self, name="Check Alignment"):
        super().__init__(name)

    def update(self):
        return (py_trees.common.Status.SUCCESS
                if BB.get("pledge_counter") == 0 and BB.get("heading") == BB.get("heading_global")
                else py_trees.common.Status.FAILURE)
    

class ChooseStraightIfFree(py_trees.behaviour.Behaviour):
    def __init__(self, name="Choose Straight"):
        super().__init__(name)

    def update(self):
        pose = BB.get("pose")
        heading = BB.get("heading_global")  # force global heading when aligned
        
        target = forward_cell(pose, heading)
        print(pose,target)
        if is_free(target):
            BB.set("heading", heading)
            BB.set("allow_visit_fallback", False)
            BB.set("last_action", f"Straight on global heading → {target}")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


#entro se NON posso andare dritto
class FollowWall(py_trees.behaviour.Behaviour):
    def __init__(self, name="Follow Wall"):
        super().__init__(name)

    def update(self):
        pose = BB.get("pose")
        heading = BB.get("heading")
        global_heading = BB.get("heading_global")
        counter = BB.get("pledge_counter")

        visited = BB.get("visited")
        forward = forward_cell(pose, heading)
        left = left_cell(pose, heading)
        right = right_cell(pose, heading)
        backright = backright_cell(pose,heading)
        backleft = backleft_cell(pose,heading)
        back = backward_cell(pose,heading)
        # Priority: Left if blocked, Right if wall-follow, else forward or global
        #se devo seguire il muro ma è terminato in questo istante (rightforward libera) allora vado a dx
        #DA DEBUGGARE
        #basterebbe gestire i casi di muro da seguire a dx, muro da seguire a sx e tornare indietro in caso contrario
        #MA NON DEVO SCEGLIERE DI SEGUIRE MURI "GIà VISITATI"
        # and not (right in visited)  and not (left in visited)
        if is_free(right) and not is_free(backright):
            BB.set("heading", (heading + 90) % 360)
            BB.set("last_action", "Turn Right (wall follow)")
            print("RIGHT111 seguo il muro a destra" )
        elif is_free(forward):
            BB.set("heading", heading)
            BB.set("last_action", "Continue forward")
            print("VADO DRITTO")
        elif is_free(left) and not is_free(backleft):
            BB.set("heading", (heading - 90) % 360)
            BB.set("last_action", "Turn Right (wall follow)")
            print("LEFT111 seguo il muro a sinistra" )

        elif is_free(back):# non dovrebbe servire questo controllo
            BB.set("heading", (heading - 180) % 360)
            BB.set("last_action", "Going back")
            print("TORNO INDIETRO")
        else:
            BB.set("last_action", "Wall ended → no free path")
            print("FAIL")
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS

""" #STEP 3 DEL PLEDGE        
class CorrectHeading(py_trees.behaviour.Behaviour):
    def __init__(self, name="Correct Heading"):
        super().__init__(name)

    def update(self):
        counter = BB.get("pledge_counter")
        heading = BB.get("heading")
        global_heading = BB.get("heading_global")

        if counter == 0 and heading == global_heading:
            BB.set("last_action", "Corrected heading → resume straight")
            return py_trees.common.Status.SUCCESS

        BB.set("last_action", "Still correcting deviation")
        return py_trees.common.Status.FAILURE """
    
def TremauxSubTree(name="Trémaux"):
    return py_trees.composites.Sequence(name, memory=True, children=[
        #METODI DA IMPLEMENTARE 
        #ChooseUnvisitedPath("Choose Path"),
        #MarkPath("Mark Path"),
        #MoveForward("Move Forward"),
        #BacktrackIfDeadEnd("Backtrack")

        #COMPORTAMENTO DI DEFAULT
        py_trees.composites.Selector("Exit or Choose", memory=True, children=[
            CheckExit("Check Exit"),
            ChooseDirection("Choose Direction")
        ]),
        MoveForward("Move Forward"),
    ])

def build_tree():
    root = py_trees.composites.Sequence("Maze Solver", memory=True)

    pledge_subtree = PledgeSubTree("Pledge Algorithm")
    tremaux_subtree = TremauxSubTree("Trémaux Algorithm")

    choose_algorithm = py_trees.composites.Selector("Choose Algorithm", memory=True, children=[
        py_trees.composites.Sequence("Pledge Branch", memory=True, children=[
            AlgorithmSelector("Is Pledge", "pledge"),
            PledgeSubTree("Pledge Algorithm")
        ]),
        py_trees.composites.Sequence("Tremaux Branch", memory=True, children=[
            AlgorithmSelector("Is Tremaux", "tremaux"),
            TremauxSubTree("Tremaux Algorithm")
        ])
    ])



    root.add_children([
        py_trees.composites.Selector("Exit or Explore", memory=True, children=[
            CheckExit("Check Exit"),
            choose_algorithm
        ])
    ])

    return root

class AlgorithmSelector(py_trees.behaviour.Behaviour):
    def __init__(self, name, mode):
        super().__init__(name)
        self.mode = mode

    def update(self):
        bb = py_trees.blackboard.Blackboard()
        if bb.get("algorithm_mode") == self.mode:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


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
        self.win.title("Maze solver simulator GUI")

        self.canvas = tk.Canvas(self.win, width=GRID_SIZE*CELL_SIZE, height=GRID_SIZE*CELL_SIZE)
        self.canvas.pack(padx=10, pady=10)

        ctrl = tk.Frame(self.win)
        ctrl.pack(pady=6)
        tk.Button(ctrl, text="Start", command=self.start).pack(side=tk.LEFT, padx=4)
        tk.Button(ctrl, text="Step", command=self.step).pack(side=tk.LEFT, padx=4)
        tk.Button(ctrl, text="Reset Maze", command=self.reset_maze).pack(side=tk.LEFT, padx=4)
        tk.Button(ctrl, text="Stop", command=self.stop).pack(side=tk.LEFT, padx=4)
        #tk.Button(ctrl, text="Save BT SVG", command=self.save_svg).pack(side=tk.LEFT, padx=4)  # NEW: button

        self.status = tk.StringVar()
        tk.Label(self.win, textvariable=self.status).pack(fill=tk.X, padx=10, pady=4)

        self.log = tk.StringVar()
        tk.Label(self.win, textvariable=self.log, fg="blue").pack(fill=tk.X, padx=10, pady=4)

        generate_walls(density=DENSITY)
        self.running = False

        self.draw_maze()
        self.update_status()

        self.win.after(200, self.loop)   #AGGIORNAMENTO OGNI 200ms (5 FPS)

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

    def step(self):   #Lo step aumenta di 1 tick e aggiunta la schermata e lo status
        self.tree.tick()
        self.draw_maze()
        self.update_status()

    def reset_maze(self):
        BB.set("pose", START)
        BB.set("goal", GOAL)
        BB.set("heading", 90)
        BB.set("reached_exit", False)
        BB.set("last_action", "Maze reset")
        BB.set("visited", {START})
        BB.set("allow_visit_fallback", False)
        generate_walls(density=DENSITY)  #Con il config
        self.draw_maze()
        self.update_status()
        self.running = False

    def save_svg(self):
        # NEW: export current behaviour tree to SVG
        save_bt_svg(self.root_node, filename="maze_solver_bt.svg")

    def loop(self):
        if self.running and not BB.get("reached_exit"):

            #Questi 3 equivalgono allo step
            self.tree.tick()
            self.draw_maze()
            self.update_status()


            if BB.get("reached_exit"):#STOP SOLO SE RAGGIUNGE L'USCITA
                self.running = False
        self.win.after(200, self.loop)

    def stop(self):
        self.running = False

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
    
    #per la produzione delle immagini
    #py_trees.display.render_dot_tree(root, name="maze")


    gui = MazeGUI(root)
    gui.run()





    """ elif not is_free(forward):
            BB.set("heading", (heading - 90) % 360)
            BB.set("last_action", "Turn Left (wall ahead)")
            print("LEFT1 non seguo il muro")
        elif is_free(right):
            BB.set("heading", (heading + 90) % 360)
            BB.set("last_action", "Turn Right (wall follow)")
            print("RIGHT1 seguo il muro" )
        elif is_free(forward):
            BB.set("heading", heading)
            BB.set("last_action", "Continue forward")
            print("VADO DRITTO" )
        elif counter == 0:
            BB.set("heading", global_heading)
            BB.set("last_action", "Wall ended → resume global heading")
            print("Muro terminato, riprendo orientamento globale" )"""