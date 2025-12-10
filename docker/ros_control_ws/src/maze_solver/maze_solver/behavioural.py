#L'inserimento di randomizzazione all'interno della generazione risulta ideale per:
#Avere uno start non necessariamente al bordo del labirinto
#Settare la stazione di docking allo spawn e far tornare il robot alla stazione per la ricarica
#la generazione di maze (percorsi di dimensione 1) o di semplici mappe (implementazione attuale) con densità variabile
#ma tenendo a mente di non tralasciare la raggiungibilità dell'obiettivo

import py_trees
from py_trees.composites import Sequence, Selector
from py_trees.blackboard import Blackboard

from behaviours import *

# from helpers import *   # import locale da helpers.py 
# from config import *    # import locale da config.py
# from gui import *       # import locale da gui.py
# from behaviours import *
# from mapping import *

class BehaviouralTree:
    def __init__(self, goal_handle, is_paused, is_kidnapped, hazard, ir_sensors, lidar_scan):
        # ADDED: la blackboard si dovrebbe poter gestire mezza localmente in ogni Behaviour, sta pure nell'esempio del prof e nella documentazione con le register_key, anche se così a funzionare funziona
        BB = py_trees.blackboard.Client(name="Behavioural")
        
        BB.register_key(key="current_position", access=py_trees.common.Access.WRITE)
        BB.register_key(key="goal_position", access=py_trees.common.Access.READ)
        BB.register_key(key="paused", access=py_trees.common.Access.READ)
        BB.register_key(key="kidnapped", access=py_trees.common.Access.READ)
        BB.register_key(key="hazard", access=py_trees.common.Access.READ)
        BB.register_key(key="ir_sensors", access=py_trees.common.Access.READ)
        BB.register_key(key="lidar_scan", access=py_trees.common.Access.READ)
        BB.register_key(key="heading", access=py_trees.common.Access.WRITE)
        BB.register_key(key="reached_exit", access=py_trees.common.Access.WRITE)
        BB.register_key(key="last_action", access=py_trees.common.Access.WRITE)
        BB.register_key(key="visited", access=py_trees.common.Access.WRITE)
        BB.register_key(key="allow_visit_fallback", access=py_trees.common.Access.WRITE)
        BB.register_key(key="algorithm_mode", access=py_trees.common.Access.READ)
        BB.register_key(key="pledge_counter", access=py_trees.common.Access.WRITE)
        BB.register_key(key="heading_global", access=py_trees.common.Access.WRITE)
        BB.register_key(key="chosen_direction", access=py_trees.common.Access.WRITE)
        BB.register_key(key="map_size", access=py_trees.common.Access.READ)
        
        BB.current_position = goal_handle.start_position
        BB.goal_position = goal_handle.end_position
        BB.paused = is_paused
        BB.kidnapped = is_kidnapped
        BB.hazard = hazard
        BB.ir_sensors = ir_sensors
        BB.lidar_scan = lidar_scan
        #BB.set("maze_walls", set())            # blocked cells {(r,c)}
        BB.heading = 90                  # 0=N, 90=E, 180=S, 270=W SET ORIENTAMENTO INIZIALE
        BB.reached_exit = False
        BB.last_action = Idle
        BB.visited = {goal_handle.start_position}            # visited cells set  (Visita dello start)
        BB.allow_visit_fallback = False  # allows moving into visited if no unvisited options exist
        BB.chosen_direction = None

        #SCELTA ALGORITMO
        BB.algorithm_mode = goal_handle.algorithm  #algoritmo di default
        BB.pledge_counter = 0
        BB.heading_global = BB.get("heading") #la direzione che vuoi come riferimento, Pledge la richiede per il "tracking dei giri"
        BB.map_size = 20
        
        map = init_map(BB.get("map_size"))
        BB.set("map", map)
        
        self.build_tree()
        
    def init_map(bb, size):
        map = {}
        for r in range(size):
            for c in range(size):
                if r == 0 or r == size-1 or c == 0 or c == size-1:
                    map[(r, c)] = "wall"
                else:
                    map[(r, c)] = "unmapped"
        map[START] = "free"
        return map
    
    def PledgeSubTree(self, name="Pledge Algorithm"):
        root = Sequence(name, memory = True)
        update_deviation_counter = UpdateDeviationCounter("Update Counter")
        follow_wall = FollowWall("Follow Wall")
        move_forward = MoveForward("Move Forward")
        root.add_children([update_deviation_counter, follow_wall, move_forward])
        return root

    # ADDED: è completo, ma una volta tra tutte quelle che ho provato si è impallato strano per un attimo tipo che ha rieseguito dei percorsi più volte senza senso, e inoltre nei vicoli ciechi da 2 caselle soltanto ogni tanto fa avanti e indietro tipo due volte invece che tornare direttamente indietro
    def TremauxSubTree(self, name="Tremaux Algorithm"):
        root = Sequence(name, memory=True)

        choose_unvisited_path = ChooseUnvisitedPath()
        mark_path = MarkPath()
        move_forward = MoveForwardTremaux()
        backtrack_if_dead_end = BacktrackIfDeadEnd()
        
        root.add_children([choose_unvisited_path, mark_path, move_forward, backtrack_if_dead_end])
    
        return root

    def build_tree(self):
        root = py_trees.composites.Sequence("Maze Solver", memory=True)
        
        is_paused_sel = Selector("Is paused?", memory=True)
        is_paused = IsPaused()
        
        
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
        
        is_paused_sel.add_children[is_paused, exit_or_explore_selector]
        
        root.add_children([is_paused_sel])
        check_exit = CheckExit()
        exit_or_explore_selector.add_children([check_exit, choose_algorithm])

        self.tree = py_trees.trees.BehaviourTree(root)
        #return root
        
    def loop(self):
        while(True):
            self.tree.tick()

# ----------------------------
# Main
# ----------------------------
# if __name__ == "__main__":
    # root = build_tree()
    #py_trees.logging.level = py_trees.logging.Level.DEBUG
    #tree = py_trees.trees.BehaviourTree(root)
    # print("Behavior Tree Structure:")
    # print(py_trees.display.unicode_tree(root, show_status=True))
    # print()

    # crea_mappa_quadrata(BB, GRID_SIZE) 
    # stampa_mappa(BB, GRID_SIZE)
    

    # gui = MazeGUI(root)
    # gui.run()