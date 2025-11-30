import py_trees
from helpers import *

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