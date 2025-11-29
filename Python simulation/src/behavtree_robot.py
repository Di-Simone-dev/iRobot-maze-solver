import py_trees

from helpers import *   # import locale da helpers.py 
from config import *    # import locale da config.py
from gui import *       # import locale da gui.py

#L'inserimento di randomizzazione all'interno della generazione risulta ideale per:
#Avere uno start non necessariamente al bordo del labirinto
#Settare la stazione di docking allo spawn e far tornare il robot alla stazione per la ricarica
#la generazione di maze (percorsi di dimensione 1) o di semplici mappe (implementazione attuale) con densità variabile
#ma tenendo a mente di non tralasciare la raggiungibilità dell'obiettivo

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
BB.set("heading_global", BB.get("heading")) #la direzione che vuoi come riferimento, Pledge la richiede per il "tracking dei giri"

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
        return py_trees.common.Status.SUCCESS

# ----------------------------
# Build behaviour tree (reactive, unblocked)
# ----------------------------

#in questo sono mutualmente esclusive
def PledgeSubTree(name="Pledge"):
    if(BB.get("algorithm_mode")!="pledge"):
        return py_trees.common.Status.FAILURE
    else :
        return py_trees.composites.Sequence(
            name, memory=True, children=[
                UpdateDeviationCounter("Update Counter"),  #gestione contatori
                py_trees.composites.Selector(
                    "Aligned or Follow", memory=True, children=[
                        #questo è SUCCESSFULL solo se si procede nella direzione iniziale del pledge
                        py_trees.composites.Sequence(
                            "Aligned Straight", memory=True, children=[
                                AlignedWithGlobal("Check Alignment"),
                                MoveForward("Move Forward"),
                            ]
                        ),
                        py_trees.composites.Sequence(
                            "Follow OR Move", memory=True, children=[
                                FollowWall("Follow Wall"),#è successful solo se può procedere dritto senza girarsi
                                MoveForward("Move Forward"),#ovvero quando devo eseguire il movimento 
                            ]
                        ),
                    ]
                )
            ]
        )


#STEP 1 DEL PLEDGE
class UpdateDeviationCounter(py_trees.behaviour.Behaviour):
    def __init__(self, name="Update Counter"):
        super().__init__(name)

    def update(self):
        #print("")
        counter = BB.get("pledge_counter") if BB.exists("pledge_counter") else 0
        last_action = BB.get("last_action") if BB.exists("last_action") else ""
        #print("Pledge counter attuale", BB.get("pledge_counter"),last_action)
        if "Turn Left" in last_action: counter -= 1     #li ho invertiti dx => + , sx => - 
        elif "Turn Right" in last_action: counter += 1
        elif "Turn Back (2 times left)" in last_action: counter -= 2
        elif "Turn Back (2 times right)" in last_action: counter += 2
        
         
        BB.set("pledge_counter", counter)
        #print("Pledge counter aggiornato", BB.get("pledge_counter"),last_action)

        # don't overwrite last_action used by other nodes
        BB.set("pledge_counter_log", f"Pledge counter: {counter}")
        return py_trees.common.Status.SUCCESS

#Primo subtree DEL PLEDGE
#controlla se l'allineamento globale (deciso all'inizio sta venendo seguito, altrimenti faila)
class AlignedWithGlobal(py_trees.behaviour.Behaviour):
    def __init__(self, name="Check Alignment"):
        super().__init__(name)

    def update(self):
        if BB.get("pledge_counter") == 0 and BB.get("heading") == BB.get("heading_global"):
            #print("HEADING CORRETTO", BB.get("pledge_counter"))
            return py_trees.common.Status.SUCCESS
        else:
            #print("HEADING NON CORRETTO" , BB.get("pledge_counter"))
            return py_trees.common.Status.FAILURE

#Secondo subtree del Pledge
#controlla le rotazioni e se non è necessario farne ritorna successful per per permettere il movimento
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

        if(is_free(left) and is_free(right) and is_free(back) and "Turn" not in BB.get("last_action")):
            print("Scelta tra sx e dx con il seguente pledge counter", counter)
            print(left, right)
        if(is_free(left) and is_free(forward) and is_free(back) and "Turn" not in BB.get("last_action")):
            print("Scelta tra sx e dritto con il seguente pledge counter", counter)
            print(left,forward)
        if(is_free(right) and is_free(forward)and is_free(back) and "Turn" not in BB.get("last_action")):
            print("Scelta tra dx e dritto con il seguente pledge counter", counter)
            print(right, forward)
        #implemento il controllo sul pledge counter 
        #>0 prioritizzo il turn left, mentre <0 prioritizzo il turn right
        change_heading = True
        if BB.get("pledge_counter")>0:
            #print("priorittizo la sinistra")
            #controlli aggiuntivi necessari, se ha appena girato a sinistra senza muoversi non deve rifarlo
            if is_free(left) and not is_free(backleft) and "Turn Left" not in BB.get("last_action"):
                BB.set("heading", (heading - 90) % 360)
                BB.set("last_action", "Turn Left (wall follow)")
                #print("LEFT111 seguo il muro a sinistra" )
            #ALTRIMENTI DESTRA E POI CENTRO
            elif is_free(forward):
                BB.set("heading", heading)
                BB.set("last_action", "Continue forward")
                change_heading = False
                #print("VADO DRITTO")
            elif is_free(right) and not is_free(backright):
                BB.set("heading", (heading + 90) % 360)
                BB.set("last_action", "Turn Right (wall follow)")
                #print("RIGHT111 seguo il muro a destra" )

            elif is_free(back):# QUI DEVO CAPIRE come gestire il pledgecounter ipotizzo di girare a sx 2 volte
                #POSSO FARLO ANCHE IN 2 STEP (PROBABILMENTE PIù ADATTO)
                BB.set("heading", (heading - 180) % 360)
                BB.set("last_action", "Turn Back (2 times right)")
                #print("TORNO INDIETRO")
            else:
                BB.set("last_action", "Wall ended → no free path")
                #print("FAIL")
                return py_trees.common.Status.FAILURE


        #Qui invece prioritizzo la destra        
        elif BB.get("pledge_counter")<=0:
            #print("priorittizo la destra")
            #LE SVOLTE VANNO FATTE 1 SOLA VOLTA DI FILA 
            if is_free(right) and not is_free(backright) and "Turn Right" not in BB.get("last_action"):
                BB.set("heading", (heading + 90) % 360)
                BB.set("last_action", "Turn Right (wall follow)")
                #print("RIGHT111 seguo il muro a destra" )
            elif is_free(forward):
                BB.set("heading", heading)
                BB.set("last_action", "Continue forward")
                change_heading = False
                #print("VADO DRITTO")
            elif is_free(left) and not is_free(backleft):
                BB.set("heading", (heading - 90) % 360)
                BB.set("last_action", "Turn Left (wall follow)")
                #print("LEFT111 seguo il muro a sinistra" )

            elif is_free(back):# non dovrebbe servire questo controllo
                BB.set("heading", (heading - 180) % 360)
                BB.set("last_action", "Turn Back (2 times left)")
                #print("TORNO INDIETRO")
            else:
                BB.set("last_action", "Wall ended → no free path")
                #print("FAIL")
                return py_trees.common.Status.FAILURE

        if(change_heading == False):
            return py_trees.common.Status.SUCCESS   #se non giro mi muovo procedendo con il moveforward
        else:
            return py_trees.common.Status.FAILURE   #altrimenti resto fermo
    
def TremauxSubTree(name="Trémaux"):
    return py_trees.composites.Sequence(name, memory=True, children=[
        #METODI DA IMPLEMENTARE 
        #TREMAUX DOVREBBE AVERE QUESTI STEP
        #ChooseUnvisitedPath("Choose Path"),
        #MarkPath("Mark Path"),
        #MoveForward("Move Forward"),
        #BacktrackIfDeadEnd("Backtrack")

        #COMPORTAMENTO DI DEFAULT (DA CAMBIARE)
        py_trees.composites.Selector("Exit or Choose", memory=True, children=[
            CheckExit("Check Exit"),
            ChooseDirection("Choose Direction")
        ]),
        MoveForward("Move Forward"),
    ])

def build_tree():
    root = py_trees.composites.Sequence("Maze Solver", memory=True)

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
        if BB.get("algorithm_mode") == self.mode:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


# ----------------------------
# Main
# ----------------------------
if __name__ == "__main__":
    root = build_tree()
    tree = py_trees.trees.BehaviourTree(root)
    gui = MazeGUI(root)
    gui.run()

