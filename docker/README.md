# Docker / Workspace ROS

Questa cartella contiene i workspace, gli artefatti di build e le utility per eseguire i pacchetti ROS relativi al progetto Create3.

**Nota:** come richiesto, il container/workspace `ros_ws` non è trattato in dettaglio qui.

## Panoramica della struttura
- `comandi/` — script e comandi utili per avviare o gestire container/workspace.
- `create3_ws/` — workspace principale per Create3 con sotto-cartelle comuni a build/install:
  - `build/` — output di compilazione (contiene sotto-cartelle per singoli pacchetti).
  - `install/` — artefatti installati dopo `colcon build`.
  - `src/` — sorgenti specifici per `create3_ws` (es. `create3_sim`).
- `irobot_create_msgs/` — pacchetto di messaggi/actions/srv usati dai nodi Create.
- `ros_control_ws/` — workspace per i componenti di controllo robot.
- `ros_msg_ws/` — workspace separato per generazione/compilazione messaggi ROS quando necessario.

## Come usare (linee guida generali)
- I workspace sono pensati per essere buildati all'interno di un ambiente ROS appropriato (container o macchina con tool ROS installati).
- Maggiori istruzioni sono presenti all'interno dei README presenti nei singoli workspace.

## Dove guardare per i pacchetti
- Per comprendere un singolo pacchetto ROS, esplora `docker/create3_ws/src` o le cartelle presenti in `install/` e `build/`.
- `irobot_create_msgs/` contiene definizioni di messaggi e servizi; utile come riferimento per l'interfaccia tra nodi.

## Logs e diagnosi
- La cartella `log/` contiene snapshot dei build precedenti. Se il build fallisce, confronta con i `log/` per individuare regressioni.
