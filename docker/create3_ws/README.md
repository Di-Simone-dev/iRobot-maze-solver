# create3_ws

Workspace principale per i pacchetti relativi a iRobot Create3.

## Scopo
Contiene i sorgenti, le dipendenze e le configurazioni per la simulazione e i nodi specifici di Create3 (publisher/subscriber, bridge per simulatori, comportamenti ad alto livello).

## Struttura
- `src/` — sorgenti dei pacchetti ROS presenti nel workspace.
- `build/` — output della compilazione (`colcon build`).
- `install/` — artefatti installati dopo il build.

## Uso rapido
1. Compila o manualmente o attraverso l'estensione il container Docker attraverso i file .devcontainer e DockerFile


2. Apri una shell dentro il container/ambiente ROS o sorgi il setup del tuo ROS locale.

```bash
source /opt/ros/jazzy/setup.bash
cd home/ws
colcon build --symlink-install
source install/local_setup.bash
```

3. Avvia i nodi e il simulatore:

```bash
ros2 launch irobot_create_gz_bringup create3_gz.launch.py gazebo:=ignition world:=custom x:=-9.0 y:=-8.35 yaw:=-1.5707963267948966
```

## Note
- Per dettagli sui singoli pacchetti, esplora le cartelle sotto `src/`.
