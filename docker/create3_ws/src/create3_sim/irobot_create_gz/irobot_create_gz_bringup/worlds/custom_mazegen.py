import xml.etree.ElementTree as ET
from xml.dom import minidom
import random

# ----------------------------
# Configurazioni
# ----------------------------
GRID_SIZE = 11  # Numero di celle nel lato della griglia quadrata (DISPARI)
#CELL_SIZE = 15  # Dimensione celle in PIXEL
#UPDATE_PERIOD = 1  # refresh ogni 1ms

START = (1, 1)  # RANDOMIZZABILE idealmente
GOAL = (GRID_SIZE - 2, GRID_SIZE - 2)
# CONFIGURAZIONI DI GAZEBO

PLANE = '100 100'  # lati del piano

def prettify_xml(elem):
    """
    Formatta l'XML in modo leggibile con indentazione corretta
    """
    rough_string = ET.tostring(elem, encoding='unicode')
    reparsed = minidom.parseString(rough_string)
    pretty = reparsed.toprettyxml(indent='    ')
    # Rimuove la dichiarazione XML dalla prima riga
    lines = pretty.split('\n')
    if lines[0].startswith('<?xml'):
        lines = lines[1:]
    return '\n'.join(lines)

def add_obstacle_wall(obstacle_walls, x, y):
    """
    Aggiunge un muro alla lista degli ostacoli con nome incrementale.
    
    Args:
        obstacle_walls: lista degli ostacoli esistenti
        x: coordinata x
        y: coordinata y
    
    Returns:
        nome del muro aggiunto
    """
    wall_num = len(obstacle_walls) + 1
    wall_name = f'wall{wall_num}'
    pose = f'{x} {y} 0 0 0 0'
    size = '1 1 1'
    
    obstacle_walls.append((wall_name, pose, size))
    return wall_name

def create_complete_sdf_world(custom_obstacles=None):
    """
    Genera il file SDF completo come da documento fornito
    
    Args:
        custom_obstacles: lista opzionale di tuple (x, y) per ostacoli aggiuntivi
    """
    # Crea elemento root
    sdf = ET.Element('sdf', version='1.8')
    
    # Crea world
    world = ET.SubElement(sdf, 'world', name='custom')
    
    # Physics
    physics = ET.SubElement(world, 'physics', name='1ms', type='ignored')
    ET.SubElement(physics, 'max_step_size').text = '0.001'
    ET.SubElement(physics, 'real_time_factor').text = '1'
    ET.SubElement(physics, 'real_time_update_rate').text = '1000'
    
    # Plugins
    plugins = [
        ('gz::sim::systems::Physics', 'gz-sim-physics-system'),
        ('gz::sim::systems::UserCommands', 'gz-sim-user-commands-system'),
        ('gz::sim::systems::SceneBroadcaster', 'gz-sim-scene-broadcaster-system'),
        ('gz::sim::systems::Contact', 'gz-sim-contact-system')
    ]
    
    for plugin_name, filename in plugins:
        ET.SubElement(world, 'plugin', name=plugin_name, filename=filename)
    
    # Light (sole)
    light = ET.SubElement(world, 'light', name='sun', type='directional')
    ET.SubElement(light, 'cast_shadows').text = '1'
    ET.SubElement(light, 'pose').text = '0 0 10 0 -0 0'
    ET.SubElement(light, 'diffuse').text = '0.8 0.8 0.8 1'
    ET.SubElement(light, 'specular').text = '0.2 0.2 0.2 1'
    
    # Attenuation
    attenuation = ET.SubElement(light, 'attenuation')
    ET.SubElement(attenuation, 'range').text = '1000'
    ET.SubElement(attenuation, 'constant').text = '0.90000000000000002'
    ET.SubElement(attenuation, 'linear').text = '0.01'
    ET.SubElement(attenuation, 'quadratic').text = '0.001'
    
    ET.SubElement(light, 'direction').text = '-0.5 0.1 -0.9'
    
    # Spot
    spot = ET.SubElement(light, 'spot')
    ET.SubElement(spot, 'inner_angle').text = '0'
    ET.SubElement(spot, 'outer_angle').text = '0'
    ET.SubElement(spot, 'falloff').text = '0'
    
    # Gravity
    ET.SubElement(world, 'gravity').text = '0 0 -9.8'
    
    # Magnetic field
    ET.SubElement(world, 'magnetic_field').text = '6e-06 2.3e-05 -4.2e-05'
    
    # Atmosphere
    ET.SubElement(world, 'atmosphere', type='adiabatic')
    
    # Scene
    scene = ET.SubElement(world, 'scene')
    ET.SubElement(scene, 'ambient').text = '0.4 0.4 0.4 1'
    ET.SubElement(scene, 'background').text = '0.7 0.7 0.7 1'
    ET.SubElement(scene, 'shadows').text = '1'
    
    # Ground plane model
    ground = ET.SubElement(world, 'model', name='ground_plane')
    ET.SubElement(ground, 'static').text = '1'
    
    link = ET.SubElement(ground, 'link', name='link')
    
    # Collision
    collision = ET.SubElement(link, 'collision', name='collision')
    geometry = ET.SubElement(collision, 'geometry')
    plane = ET.SubElement(geometry, 'plane')
    ET.SubElement(plane, 'normal').text = '0 0 1'
    ET.SubElement(plane, 'size').text = PLANE
    
    surface = ET.SubElement(collision, 'surface')
    friction = ET.SubElement(surface, 'friction')
    ET.SubElement(friction, 'ode')
    ET.SubElement(surface, 'bounce')
    ET.SubElement(surface, 'contact')
    
    # Visual
    visual = ET.SubElement(link, 'visual', name='visual')
    geometry = ET.SubElement(visual, 'geometry')
    plane = ET.SubElement(geometry, 'plane')
    ET.SubElement(plane, 'normal').text = '0 0 1'
    ET.SubElement(plane, 'size').text = PLANE
    
    material = ET.SubElement(visual, 'material')
    ET.SubElement(material, 'ambient').text = '0.8 0.8 0.8 1'
    ET.SubElement(material, 'diffuse').text = '0.8 0.8 0.8 1'
    ET.SubElement(material, 'specular').text = '0.8 0.8 0.8 1'
    
    ET.SubElement(visual, 'plugin', name='__default__', filename='__default__')
    
    ET.SubElement(ground, 'plugin', name='__default__', filename='__default__')
    ET.SubElement(ground, 'pose').text = '0 0 0 0 -0 0'
    
    # Border model con 4 muri
    border = ET.SubElement(world, 'model', name='border')
    ET.SubElement(border, 'pose').text = '0 0 1 0 0 0'
    ET.SubElement(border, 'static').text = 'true'
    
    # Definizione dei bordi
    walls = [
        # ('wall1', '-10.005 0 0 0 0 0', '0.01 20 2'),
        # ('wall2', '10.005 0 0 0 0 0', '0.01 20 2'),
        # ('wall3', '0 10.005 0 0 0 0', '20 0.01 2'),
        # ('wall4', '0 -10.005 0 0 0 0', '20 0.01 2')
    ]
    
    for wall_name, wall_pose, wall_size in walls:
        wall_link = ET.SubElement(border, 'link', name=wall_name)
        ET.SubElement(wall_link, 'pose').text = wall_pose
        # RIMOSSO: ET.SubElement(wall_link, 'static').text = 'true'
        # Il tag static è già a livello di model
        
        # Collision
        collision = ET.SubElement(wall_link, 'collision', name=f'{wall_name}_collision')
        geometry = ET.SubElement(collision, 'geometry')
        box = ET.SubElement(geometry, 'box')
        ET.SubElement(box, 'size').text = wall_size
        
        # Visual
        visual = ET.SubElement(wall_link, 'visual', name=f'{wall_name}_visual')
        geometry = ET.SubElement(visual, 'geometry')
        box = ET.SubElement(geometry, 'box')
        ET.SubElement(box, 'size').text = wall_size
    
    # Obstacles model (diverso dal border)
    obstacles = ET.SubElement(world, 'model', name='obstacles')
    ET.SubElement(obstacles, 'pose').text = '0 0 0.5 0 0 0'
    ET.SubElement(obstacles, 'static').text = 'true'  # STATIC A LIVELLO MODEL
    
    # Definizione degli ostacoli base
    obstacle_walls = [
        # ('wall1', '-6.5 -9 0 0 0 0', '1 2 1'),
        # ('wall2', '-8.5 -6.5 0 0 0 0', '3 1 1')
    ]
    
    # Aggiungi ostacoli personalizzati se forniti
    if custom_obstacles:
        for x, y in custom_obstacles:
            add_obstacle_wall(obstacle_walls, x, y)
    
    for obs_name, obs_pose, obs_size in obstacle_walls:
        obs_link = ET.SubElement(obstacles, 'link', name=obs_name)
        ET.SubElement(obs_link, 'pose').text = obs_pose
        # RIMOSSO: ET.SubElement(obs_link, 'static').text = 'true'
        # Il tag static è già a livello di model 'obstacles'
        
        # Collision
        collision = ET.SubElement(obs_link, 'collision', name=f'{obs_name}_collision')
        geometry = ET.SubElement(collision, 'geometry')
        box = ET.SubElement(geometry, 'box')
        ET.SubElement(box, 'size').text = obs_size
        
        # Visual
        visual = ET.SubElement(obs_link, 'visual', name=f'{obs_name}_visual')
        geometry = ET.SubElement(visual, 'geometry')
        box = ET.SubElement(geometry, 'box')
        ET.SubElement(box, 'size').text = obs_size
    
    return sdf

def save_sdf(sdf_element, filename='custom.sdf'):
    """
    Salva il file SDF
    """
    xml_string = prettify_xml(sdf_element)
    
    with open(filename, 'w', encoding='utf-8') as f:
        f.write(xml_string)
    
    print(f"File SDF salvato: {filename}")


#QUI MI GENERO I MURI COME NELLA SIM PYTHON
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

        return walls
        #BB.set("maze_walls", walls)
        ##print(walls)
        #reachable = True
        #reachable = is_reachable(START,GOAL,walls)
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










# Genera il file completo
if __name__ == '__main__':
    # Esempio: aggiungi ostacoli personalizzati con il tuo algoritmo
    walls = generate_walls()
    
    sdf = create_complete_sdf_world(custom_obstacles=walls)
    save_sdf(sdf, 'custom.sdf')
    print("File SDF completo generato con successo!")
    print(f"Ostacoli totali generati: {len(walls)}")
