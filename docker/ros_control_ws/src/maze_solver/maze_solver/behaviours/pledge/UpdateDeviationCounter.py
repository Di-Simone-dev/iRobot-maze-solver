import py_trees
from maze_solver.helpers import *
# from helpers_phys import *

#STEP 1 DEL PLEDGE
class UpdateDeviationCounter(py_trees.behaviour.Behaviour):
    def __init__(self, name="Update Counter"):
        super().__init__(name)
    
    def update(self):
        # counter = BB.get("pledge_counter") if BB.exists("pledge_counter") else 0
        # last_action = BB.get("last_action") if BB.exists("last_action") else ""
        
        # #if(BB.get("pose") == START):
        # step_mapping(BB.get("pose"),BB.get("heading"))
        # # Map actions to counter changes
        # action_deltas = {
        #     "Turn Left": -1,
        #     "Turn Right": 1,
        #     "Turn Back (2 times left)": -2,
        #     "Turn Back (2 times right)": 2
        # }
        
        # # Update counter based on last action
        # for action, delta in action_deltas.items():
        #     if action in last_action:
        #         counter += delta
        #         break
        
        # BB.set("pledge_counter", counter)
        # BB.set("pledge_counter_log", f"Pledge counter: {counter}")
        
        

        return py_trees.common.Status.SUCCESS