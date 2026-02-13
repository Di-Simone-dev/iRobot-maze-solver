# ros_control_ws

Workspace dedicato ai componenti di controllo (controller, planner e attuatori) usati dal progetto.

## Scopo
Contiene package relativi al controllo, manipolazione del movimento ed esecuzione degli algoritmi.

## Struttura
- `src/` — sorgenti dei pacchetti di controllo.
  - `actuator/` — Nodo attuatore responsabie della comunicazione con l'iRobot Create 3.
  - `bringup/` — Si occupa dell'avvio e della configurazione dei nodi attraverso apposito file di config e launchfile.
  - `custom_msg/` — Pacchetto contenente i messaggi personalizzati che è stato necessario creare.
  - `maze_solver/` — Nodo responsabile dell'esecuzione dell'algoritmo attraverso l'utilizzo di behavioural trees.
  - `planner/` — Nodo responsabie della comunicazione con l'esterno e della decodifica dei comandi in modo da identificare il nodo responsabile per la loro corretta esecuzione
  - `voice_node/` — Nodo responsabile della creazione di un webserver per la ricezione dei comandi vocali attraverso l'Ardruino Uno R4.
- `build/` — output di `colcon build`.
- `install/` — artefatti installati.

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
    colcon build
    source install/local_setup.bash
    ```

## Uso
0. Assicurarsi di aver avviato docker
   
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
3. Avviare i nodi mediante launchfile
    ```bash
    ros2 launch bringup bringup.launch.py
    ```


## Configurazione dei nodi
- Per configurare le impostazioni dei nodi è possibile modificare i file di configurazione.
- I file di configurazione dei nodi si trovano all'interno dei package sotto `src/bringup/config`.
- Dopo aver modificato i file di configurazione è necessario rieffettuare il build del progetto.
