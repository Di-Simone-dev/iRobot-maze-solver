# ----------------------------
# Helpers
# ----------------------------
from config import *
import py_trees

BB = py_trees.blackboard.Blackboard()

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

if __name__ == "__main__":
    # codice che deve essere eseguito solo se il file è lanciato direttamente
    print("Questo è il file degli helper, non va eseguito")
