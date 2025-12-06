# ----------------------------
# Helpers
# ----------------------------
from config import *
from py_trees.blackboard import Blackboard

def crea_mappa_quadrata(bb, size):
    """
    Inizializza una mappa quadrata di dimensione 'size' x 'size'.
    - Bordi = "wall"
    - Interni = "unmapped"
    """
    mappa = {}
    for r in range(size):
        for c in range(size):
            if r == 0 or r == size-1 or c == 0 or c == size-1:
                mappa[(r, c)] = "wall"
            else:
                mappa[(r, c)] = "unmapped"
    mappa[START] = "free"
    bb.set("mappa", mappa)


def stampa_mappa(bb, size):
    """
    Stampa la mappa in forma tabellare per debug.
    """
    mappa = bb.get("mappa")
    for r in range(size):
        riga = []
        for c in range(size):
            val = mappa[(r, c)]
            if val == "wall":
                riga.append("#")   # simbolo muro
            elif val == "unmapped":
                riga.append("X")   # simbolo unmapped
            else:
                riga.append(".")
        print(" ".join(riga))
