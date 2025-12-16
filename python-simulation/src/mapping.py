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

"""
Sezione sperimentale per mappa incrementale
"""

def crea_mappa_incrementale(bb, size=3):
    #unica instruzione che realmente serve
    mappa = {}

    """
        for r in range(size):
        for c in range(size):
                mappa[(r, c)] = "unmapped"
    """

    #mappa[START] = "free"  #in teoria su (1,1)

    #da lasciare se si intende partire dall'angolo o commentare nel dubbio
    #mappa[(0, 1)] = "wall"
    #mappa[(1, 0)] = "wall"


    bb.set("mappa", mappa)


def stampa_mappa_incrementale(bb):
    mappa = bb.get("mappa")
    if not mappa:
        print("Mappa vuota")
        return
    
    righe = [coord[0] for coord in mappa.keys()]
    colonne = [coord[1] for coord in mappa.keys()]
    min_r, max_r = min(righe), max(righe)
    min_c, max_c = min(colonne), max(colonne)
    
    for r in range(min_r, max_r + 1):
        riga = []
        for c in range(min_c, max_c + 1):
            if (r, c) in mappa:
                val = mappa[(r, c)]
                riga.append("#" if val == "wall" else "X" if val == "unmapped" else ".")
            else:
                riga.append("?")
        print(" ".join(riga))