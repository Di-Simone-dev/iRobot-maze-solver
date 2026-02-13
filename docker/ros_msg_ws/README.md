# ros_msg_ws

Workspace focalizzato sulla definizione e compilazione di messaggi, servizi e azioni custom usati nel progetto.

## Scopo
Separare la definizione dei tipi di messaggi (`.msg`), servizi (`.srv`) e azioni (`.action`) in un workspace dedicato per facilitare la generazione e la condivisione tra i package.

## Struttura
- `src/` — package che contengono file `.msg`, `.srv`, `.action`.
- `build/` — output della compilazione.
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

## Aggiornare o aggiungere nuovi messaggi:
- Aggiungi i file `.msg`/`.srv`/`.action` nella cartella `msg/`, `srv/`, `action/` del package.
- Aggiorna `package.xml` e `CMakeLists.txt` per includere le dipendenze di build necessarie (`rosidl_default_generators`, ecc.).
- Ricostruisci con `colcon build`.
