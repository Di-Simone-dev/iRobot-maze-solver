#L'inserimento di randomizzazione all'interno della generazione risulta ideale per:
#Avere uno start non necessariamente al bordo del labirinto
#Settare la stazione di docking allo spawn e far tornare il robot alla stazione per la ricarica
#la generazione di maze (percorsi di dimensione 1) o di semplici mappe (implementazione attuale) con densità variabile
#ma tenendo a mente di non tralasciare la raggiungibilità dell'obiettivo

import py_trees
from py_trees.composites import Sequence, Selector
from py_trees.blackboard import Blackboard

from maze_solver.behaviours import *
from maze_solver import config

class BehaviouralTree:
    def __init__(self, goal_handle, is_paused, is_kidnapped, hazards, ir_sensors, lidar_scan, actuator_movement_action_client, clock, logger):
        # ADDED: la blackboard si dovrebbe poter gestire mezza localmente in ogni Behaviour, sta pure nell'esempio del prof e nella documentazione con le register_key, anche se così a funzionare funziona
        BB = py_trees.blackboard.Client(name="Behavioural")
        
        BB.register_key(key="current_position", access=py_trees.common.Access.WRITE)
        BB.register_key(key="goal_position", access=py_trees.common.Access.WRITE)
        BB.register_key(key="paused", access=py_trees.common.Access.WRITE)
        BB.register_key(key="kidnapped", access=py_trees.common.Access.WRITE)
        BB.register_key(key="hazards", access=py_trees.common.Access.WRITE)
        BB.register_key(key="ir_sensors", access=py_trees.common.Access.WRITE)
        BB.register_key(key="lidar_scan", access=py_trees.common.Access.WRITE)
        BB.register_key(key="heading", access=py_trees.common.Access.WRITE)
        BB.register_key(key="reached_exit", access=py_trees.common.Access.WRITE)
        BB.register_key(key="last_action", access=py_trees.common.Access.WRITE)
        BB.register_key(key="visited", access=py_trees.common.Access.WRITE)
        BB.register_key(key="allow_visit_fallback", access=py_trees.common.Access.WRITE)
        BB.register_key(key="algorithm_mode", access=py_trees.common.Access.WRITE)
        BB.register_key(key="pledge_counter", access=py_trees.common.Access.WRITE)
        BB.register_key(key="heading_global", access=py_trees.common.Access.WRITE)
        BB.register_key(key="chosen_direction", access=py_trees.common.Access.WRITE)
        BB.register_key(key="map", access=py_trees.common.Access.WRITE)
        BB.register_key(key="map_size", access=py_trees.common.Access.WRITE)
        BB.register_key(key="busy", access=py_trees.common.Access.WRITE)
        BB.register_key(key="actuator_movement_action_client", access=py_trees.common.Access.WRITE)
        BB.register_key(key="goal_handle", access=py_trees.common.Access.WRITE)
        BB.register_key(key="clock", access=py_trees.common.Access.WRITE)
        BB.register_key(key="logger", access=py_trees.common.Access.WRITE)
        
        BB.current_position = goal_handle.request.start_position
        BB.goal_position = goal_handle.request.end_position
        BB.paused = is_paused
        BB.kidnapped = is_kidnapped
        BB.hazards = hazards
        BB.ir_sensors = ir_sensors
        BB.lidar_scan = lidar_scan
        #BB.set("maze_walls", set())            # blocked cells {(r,c)}
        BB.heading = 90                  # 0=N, 90=E, 180=S, 270=W SET ORIENTAMENTO INIZIALE
        BB.reached_exit = False
        BB.last_action = "Idle"
        BB.visited = [goal_handle.request.start_position]            # visited cells set  (Visita dello start)
        BB.allow_visit_fallback = False  # allows moving into visited if no unvisited options exist
        BB.chosen_direction = None

        #SCELTA ALGORITMO
        BB.algorithm_mode = goal_handle.request.algorithm  #algoritmo di default
        BB.pledge_counter = 0
        BB.heading_global = BB.get("heading") #la direzione che vuoi come riferimento, Pledge la richiede per il "tracking dei giri"
        BB.map_size = 20
        BB.busy = False

        BB.actuator_movement_action_client = actuator_movement_action_client
        BB.goal_handle = goal_handle
        BB.clock = clock
        BB.logger = logger
        
        map = self.init_map(BB.get("map_size"))
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
        map[config.START] = "free"
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
        root = Sequence(name, memory=False)

        choose_unvisited_path = ChooseUnvisitedPath()
        mark_path = MarkPath()
        choose_direction = ChooseDirectionTremaux()
        move_forward = MoveForwardTremaux()
        #backtrack_if_dead_end = BacktrackIfDeadEnd()
        
        root.add_children([choose_unvisited_path, mark_path, choose_direction, move_forward])#, backtrack_if_dead_end])
    
        return root

    def build_tree(self):
        root = py_trees.composites.Sequence("Maze Solver", memory=False)
        
        is_paused_sel = Selector("Is paused?", memory=False)
        is_paused = IsPaused()
        
        kidnap_sel = Selector("Kidnap selector", memory=False)
        check_kidnap = CheckKidnap()
        
        hazards_sel = Selector("Hazard selector", memory=False)
        check_hazards = CheckHazards()
        
        busy_sel = Selector("Busy selector", memory=False)
        is_busy = IsBusy()
        
        pledge_branch_sequence = Sequence("Pledge Branch", memory=False)
        algorithm_selector_P = AlgorithmSelector("Is Pledge", "PLEDGE")
        pledge_subtree = self.PledgeSubTree()
        
        tremaux_branch_sequence = Sequence("Tremaux Branch", memory=False)
        algorithm_selector_T = AlgorithmSelector("Is Tremaux", "TREMAUX")
        tremaux_subtree = self.TremauxSubTree()
        
        choose_algorithm = Selector("Choose Algorithm", memory=False)
        choose_algorithm.add_children([pledge_branch_sequence, tremaux_branch_sequence])
        pledge_branch_sequence.add_children([algorithm_selector_P, pledge_subtree])
        tremaux_branch_sequence.add_children([algorithm_selector_T, tremaux_subtree])
        
        exit_or_explore_selector = Selector("Exit or Explore", memory=False)
        
        is_paused_sel.add_children([is_paused, kidnap_sel])
        kidnap_sel.add_children([check_kidnap, hazards_sel])
        hazards_sel.add_children([check_hazards, busy_sel])
        busy_sel.add_children([is_busy, exit_or_explore_selector])

        exec_sequence = Sequence("Exec Sequence", memory=False)
        check_exit = CheckExit()
        exit_or_explore_selector.add_children([check_exit, exec_sequence])



        lidar_node = LidarMap()
        feedback = Feedback()
        exec_sequence.add_children([lidar_node, choose_algorithm, feedback])

        root.add_children([is_paused_sel])
        self.tree = py_trees.trees.BehaviourTree(root)
        #return root
        
    def loop(self):
        while(True):
            self.tree.tick()
