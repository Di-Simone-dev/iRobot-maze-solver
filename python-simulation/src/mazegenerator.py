from config import *
from helpers import *
import random

# ----------------------------
# Maze generation, chiamato al init e al reset
# ----------------------------
def generate_walls():
    reachable = False
    while(not reachable):#piccolo trucchetto per ottenere un maze risolvibile dal robot
        start = START
        goal = (GRID_SIZE - 2, GRID_SIZE - 2)#hardcodato ma randomizzabile

        # Tutte le celle inizialmente muri
        walls = {(r, c) for r in range(GRID_SIZE) for c in range(GRID_SIZE)}
        walls.remove(start)
        walls.remove(goal)

        visited = set()
        stack = [start]

        # Direzioni (N, S, E, W)
        directions = [(-1, 0), (1, 0), (0, 1), (0, -1)]

        while stack:
            current = stack[-1]
            visited.add(current)

            # Trova vicini non visitati a distanza 2 (per mantenere corridoi larghi 1)
            neighbors = []
            for dr, dc in directions:
                nr, nc = current[0] + 2*dr, current[1] + 2*dc
                if 0 <= nr < GRID_SIZE and 0 <= nc < GRID_SIZE   and (nr, nc) not in visited:
                    neighbors.append((nr, nc))

            if neighbors:
                next_cell = random.choice(neighbors)
                # Rimuovi il muro tra current e next_cell
                wall = (current[0] + (next_cell[0] - current[0]) // 2,
                        current[1] + (next_cell[1] - current[1]) // 2)
                if wall in walls:
                    walls.remove(wall)
                if next_cell in walls:
                    walls.remove(next_cell)
                stack.append(next_cell)
            else:
                stack.pop()

        BB.set("maze_walls", walls)
        ##print(walls)
        #reachable = True
        reachable = is_reachable(START,GOAL,walls)
        ##print(reachable)

#Con questo sono sicuro che il maze sia risolvibile
def is_reachable(start, goal, walls):
    from collections import deque
    q = deque([start])
    visited = {start}
    directions = [(1,0),(-1,0),(0,1),(0,-1)]
    while q:
        r,c = q.popleft()
        if (r,c) == goal:
            return True
        for dr,dc in directions:
            nr,nc = r+dr, c+dc
            if 0 <= nr < GRID_SIZE and 0 <= nc < GRID_SIZE:
                if (nr,nc) not in walls and (nr,nc) not in visited:
                    visited.add((nr,nc))
                    q.append((nr,nc))
    return False
