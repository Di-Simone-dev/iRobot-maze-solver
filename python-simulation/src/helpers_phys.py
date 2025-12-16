# ----------------------------
# Helpers
# ----------------------------
from config import *
from py_trees.blackboard import Blackboard
from helpers import *
from mapping import *


#UNICO METODO ATTUALMENTE IN USO
def step_mapping(cell, heading):# eseguito dopo movimenti per aggiornare la mappa
    r, c = cell

    # Direzioni relative
    direzioni = {
        0: {
            "DAVANTI": (-1, 0),
            "DAVANTI A DESTRA": (-1, 1),
            "DAVANTI A SINISTRA": (-1, -1),
        },
        90: {
            "DAVANTI": (0, 1),
            "DAVANTI A DESTRA": (1, 1),
            "DAVANTI A SINISTRA": (-1, 1),
        },
        180: {
            "DAVANTI": (1, 0),
            "DAVANTI A DESTRA": (1, -1),
            "DAVANTI A SINISTRA": (1, 1),
        },
        270: {
            "DAVANTI": (0, -1),
            "DAVANTI A DESTRA": (-1, -1),
            "DAVANTI A SINISTRA": (1, -1),
        },
    }

    mappa = BB.get("mappa")
    #print(heading)
    d = direzioni[heading]

    for nome, (dr, dc) in d.items():
        coord = (r + dr, c + dc)
        #if mappa[coord] == "unmapped":  versione vecchia
        if mappa.get(coord) is None or  mappa.get(coord) == "unmapped":
            print(f"La cella {coord} è {nome} ed è UNMAPPED")
            #SI PROCEDE CON IL MAPPING
            if(lidar(coord)):
                mappa[coord] = "wall"
            else:
                mappa[coord] = "free"
            print(f"La cella {coord} è {nome} ora è mappata come {mappa[coord]}")
        else:
            print(f"La cella {coord} è {nome} ma è già mappata come {mappa[coord]}")



    BB.set("mappa",mappa)
    #stampa_mappa(BB, GRID_SIZE)
    stampa_mappa_incrementale(BB)



def is_free_phys(cell):
    if(BB.get("mappa")[cell] == "unmapped"):# in questo caso va mappata con il lidar
        lidar_map_selective(cell) #qui, la cella diventa "wall" o "free"
    return BB.get("mappa")[cell] == "free"

def lidar(cell): #True se è un muro
    #QUI VANNO IMPLEMENTATE LE DINAMICHE DEL LIDAR
    return (cell in BB.get("maze_walls")) #al posto di questo


def lidar_map_selective(cell):
    d = "1" # distanza di output del lidar 
    pose = BB.get("pose")
    heading = BB.get("heading")
    relazione = relazione_cella(pose,heading,cell)
    match relazione:
        case "DAVANTI":
            print("La cella è davanti a te.")
        case "A DESTRA":
            print("La cella è alla tua destra.")
        case "A SINISTRA":
            print("La cella è alla tua sinistra.")
        case "DAVANTI A DESTRA":
            print("La cella è davanti a destra.")
        case "DAVANTI A SINISTRA":
            print("La cella è davanti a sinistra.")
        case _:
            print("La cella non rientra nelle relazioni definite.")

    return True

def relazione_cella(posizione, orientamento, target):
    """
    Determina la relazione spaziale di una cella rispetto alla posizione e orientamento.
    posizione: (r, c)
    orientamento: 'N', 'E', 'S', 'W'
    target: (rt, ct)
    """
    r, c = posizione
    rt, ct = target
    dr, dc = rt - r, ct - c

    # Direzioni base in funzione dell'orientamento
    direzioni = {
        "N": {"davanti": (-1, 0), "destra": (0, 1), "sinistra": (0, -1)},
        "E": {"davanti": (0, 1), "destra": (1, 0), "sinistra": (-1, 0)},
        "S": {"davanti": (1, 0), "destra": (0, -1), "sinistra": (0, 1)},
        "W": {"davanti": (0, -1), "destra": (-1, 0), "sinistra": (1, 0)},
    }

    d = direzioni[orientamento]

    if (dr, dc) == d["davanti"]:
        return "DAVANTI"
    elif (dr, dc) == d["destra"]:
        return "A DESTRA"
    elif (dr, dc) == d["sinistra"]:
        return "A SINISTRA"
    elif (dr, dc) == (d["davanti"][0] + d["destra"][0],
                      d["davanti"][1] + d["destra"][1]):
        return "DAVANTI A DESTRA"
    elif (dr, dc) == (d["davanti"][0] + d["sinistra"][0],
                      d["davanti"][1] + d["sinistra"][1]):
        return "DAVANTI A SINISTRA"
    else:
        return "ALTRO"


if __name__ == "__main__":
    # codice che deve essere eseguito solo se il file è lanciato direttamente
    print("Questo è il file degli helper, non va eseguito")
