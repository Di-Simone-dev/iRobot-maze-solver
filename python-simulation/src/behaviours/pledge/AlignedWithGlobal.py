import py_trees
from helpers import *

# Primo subtree del Pledge
# Controlla se l'allineamento globale (deciso all'inizio sta venendo seguito, altrimenti faila)
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