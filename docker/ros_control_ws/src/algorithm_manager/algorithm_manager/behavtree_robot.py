import py_trees
from py_trees.composites import Sequence, Selector

from helpers import *   # import locale da helpers.py 
from config import *    # import locale da config.py
from gui import *       # import locale da gui.py
from behaviours import *

#L'inserimento di randomizzazione all'interno della generazione risulta ideale per:
#Avere uno start non necessariamente al bordo del labirinto
#Settare la stazione di docking allo spawn e far tornare il robot alla stazione per la ricarica
#la generazione di maze (percorsi di dimensione 1) o di semplici mappe (implementazione attuale) con densità variabile
#ma tenendo a mente di non tralasciare la raggiungibilità dell'obiettivo

# ADDED: la blackboard si dovrebbe poter gestire mezza localmente in ogni Behaviour, sta pure nell'esempio del prof e nella documentazione con le register_key, anche se così a funzionare funziona
BB.set("pose", START)    #SET POSIZIONE INIZIALE
BB.set("goal", GOAL)     #SET OBIETTIVO
BB.set("maze_walls", set())            # blocked cells {(r,c)}
BB.set("heading", 0)                  # 0=N, 90=E, 180=S, 270=W SET ORIENTAMENTO INIZIALE
BB.set("reached_exit", False)
BB.set("last_action", "Idle")
BB.set("visited", {START})            # visited cells set  (Visita dello start)
BB.set("allow_visit_fallback", False)  # allows moving into visited if no unvisited options exist

#SCELTA ALGORITMO (HARDCODATA PER ORA)
BB.set("algorithm_mode", "pledge")
BB.set("pledge_counter", 0)
BB.set("heading_global", BB.get("heading")) #la direzione che vuoi come riferimento, Pledge la richiede per il "tracking dei giri"

# ----------------------------
# Build behaviour tree (reactive, unblocked)
# ----------------------------

# In questo sono mutualmente esclusive
def PledgeSubTree(name="Pledge Algorithm"):
    # ADDED: Questo non va bene, sennò ritorna un failure e non un behaviour o simili
    # CONCORDO, era per un test sulla modifica del selector e superfluo attualmente

    root = Sequence(name, memory = True)
    align_or_follow_selector = Selector(name = "Aligned of Follow", memory = True)
    #aligned_straight_sequence = Sequence(name = "Aligned Straight", memory = True)
    follow_or_move_sequence = Sequence(name = "Follow Or Move", memory = True)
    
    update_deviation_counter = UpdateDeviationCounter("Update Counter")
    aligned_with_global = AlignedWithGlobal("Check Alignment")
    move_forward = MoveForward("Move Forward")
    follow_wall = FollowWall("Follow Wall")
    move_forward_2 = MoveForward("Move Forward 2")
    
    #root.add_children([update_deviation_counter, align_or_follow_selector])
    root.add_children([update_deviation_counter, follow_wall, move_forward_2])
    #align_or_follow_selector.add_children([aligned_straight_sequence, follow_or_move_sequence])
    #align_or_follow_selector.add_children([follow_or_move_sequence])
    #aligned_straight_sequence.add_children([aligned_with_global, move_forward])
    #follow_or_move_sequence.add_children([follow_wall, move_forward_2])

    return root

# ADDED: è completo, ma una volta tra tutte quelle che ho provato si è impallato strano per un attimo tipo che ha rieseguito dei percorsi più volte senza senso, e inoltre nei vicoli ciechi da 2 caselle soltanto ogni tanto fa avanti e indietro tipo due volte invece che tornare direttamente indietro
def TremauxSubTree(name="Tremaux Algorithm"):
    # ADDED: Pure qua
    # CONCORDO
    # if(BB.get("algorithm_mode") != "tremaux"):
    #     return py_trees.common.Status.FAILURE
    
    root = Sequence(name, memory=True)

    choose_unvisited_path = ChooseUnvisitedPath()
    mark_path = MarkPath()
    move_forward = MoveForwardTremaux()
    backtrack_if_dead_end = BacktrackIfDeadEnd()
    
    root.add_children([choose_unvisited_path, mark_path, move_forward, backtrack_if_dead_end])
   
    return root


def build_tree():
    root = py_trees.composites.Sequence("Maze Solver", memory=True)
    
    pledge_branch_sequence = Sequence("Pledge Branch", memory=True)
    algorithm_selector_P = AlgorithmSelector("Is Pledge", "pledge")
    pledge_subtree = PledgeSubTree()
    
    tremaux_branch_sequence = Sequence("Tremaux Branch", memory=True)
    algorithm_selector_T = AlgorithmSelector("Is Tremaux", "tremaux")
    tremaux_subtree = TremauxSubTree()
    
    choose_algorithm = Selector("Choose Algorithm", memory=True)
    choose_algorithm.add_children([pledge_branch_sequence, tremaux_branch_sequence])
    pledge_branch_sequence.add_children([algorithm_selector_P, pledge_subtree])
    tremaux_branch_sequence.add_children([algorithm_selector_T, tremaux_subtree])
    
    exit_or_explore_selector = Selector("Exit or Explore", memory=True)
    
    root.add_children([exit_or_explore_selector])
    check_exit = CheckExit()
    exit_or_explore_selector.add_children([check_exit, choose_algorithm])

    return root

# ----------------------------
# Main
# ----------------------------
if __name__ == "__main__":
    root = build_tree()
    #py_trees.logging.level = py_trees.logging.Level.DEBUG
    tree = py_trees.trees.BehaviourTree(root)
    print("Behavior Tree Structure:")
    print(py_trees.display.unicode_tree(root, show_status=True))
    print()
    gui = MazeGUI(root)
    gui.run()
