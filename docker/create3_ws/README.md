# create3_ws

Workspace principale per i pacchetti relativi a iRobot Create3.

## Scopo
Contiene i sorgenti, le dipendenze e le configurazioni per la simulazione e i nodi specifici di Create3 (publisher/subscriber, bridge per simulatori, comportamenti ad alto livello).

## Struttura
- `src/` — sorgenti dei pacchetti ROS presenti nel workspace.
  - `irobot_create_common/` — descrizione fisica robot e controller necessari per la simulazione del comportamento.
  - `irobot_create_gz/` — contiene configurazioni e descrizioni di elementi necessari come bridge e mondi per poter effettuare la simulazione su Gazebo
- `build/` — output della compilazione (`colcon build`).
- `install/` — artefatti installati dopo il build.

## Installazione
0. Assicurarsi di aver installato e configurato docker e l'estensione di VSC Dev Container.
   
1. Estrarre il contenuto della cartella VSC_Workspace_Config.zip nella root di questo workspace e modificare i file per adattarsi con le specifiche della macchina in uso.

2. Crea il container mediante l'estenzione Dev Container con il comando
    ```bash
    >Dev Container: Reopen in Container
    ```
   
3. Terminata la creazione del container buildare il progetto attraverso i comandi
    ```bash
    source /opt/ros/jazzy/setup.bash
    cd home/ws
    colcon build --symlink-install
    source install/local_setup.bash
    ```

## Uso
0. Assicurarsi di aver avviato docker e di aver concesso diritti a docker per l'utilizzo della grafica per il simulatore di Gazebo
    ```bash
    xhost +local:docker
    ```
   
1. Avviare il container attraverso il comando:
    ```bash
    >Dev Container: Reopen in Container
    ```

2. Effettuare il source per utilizzare ros:
    ```bash
    source /opt/ros/jazzy/setup.bash
    cd home/ws
    source install/local_setup.bash
    ```
3. Avviare i nodi e il simulatore di Gazebo mediante launchfile
    ```bash
    ros2 launch irobot_create_gz_bringup create3_gz.launch.py gazebo:=ignition world:=custom x:=-9.0 y:=-8.35 yaw:=-1.5707963267948966
    ```

## Note
- Per dettagli sui singoli pacchetti, esplora le cartelle sotto `src/`.
