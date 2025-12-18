# ros_msg_ws

Workspace focalizzato sulla definizione e compilazione di messaggi, servizi e azioni custom usati nel progetto.

## Scopo
Separare la definizione dei tipi di messaggi (`.msg`), servizi (`.srv`) e azioni (`.action`) in un workspace dedicato per facilitare la generazione e la condivisione tra i package.

## Struttura
- `src/` — package che contengono file `.msg`, `.srv`, `.action`.
- `build/` — output della compilazione.
- `install/` — artefatti installati.

## Uso rapido
1. Entrare nell'ambiente ROS (container o macchina con ROS):

```bash
source /opt/ros/jazzy/setup.bash
cd home/ws
colcon build
source install/local_setup.bash
```

2. Aggiornare o aggiungere nuovi messaggi:
- Aggiungi i file `.msg`/`.srv`/`.action` nella cartella `msg/`, `srv/`, `action/` del package.
- Aggiorna `package.xml` e `CMakeLists.txt` per includere le dipendenze di build necessarie (`rosidl_default_generators`, ecc.).
- Ricostruisci con `colcon build`.
