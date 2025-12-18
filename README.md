# iRobot Maze Solver

Progetto per la risoluzione di labirinti con iRobot Create3 e simulazione Python.
Contiene componenti ROS2 (workspace Docker) e una simulazione/GUI Python indipendente.

## Contenuto del repository
- [docker/](docker/) — workspace e risorse Docker per iRobot/Create3 e pacchetti ROS (es. `create3_ws`, `ros_control_ws`, `ros_msg_ws`). Vedi documentazione: [docker/README.md](docker/README.md)
- [python-simulation/](python-simulation/) — simulazione Python, GUI e comportamenti (file principali in `python-simulation/src`). Vedi documentazione: [python-simulation/README.md](python-simulation/README.md)
  - [python-simulation/requirements.txt](python-simulation/requirements.txt) — dipendenze Python.

## Scopo
- Fornire una piattaforma per sviluppare e testare algoritmi di esplorazione e risoluzione di labirinti per Create3.
- Offrire una simulazione Python che permette di provare i comportamenti senza avviare il container ROS.

## Requisiti (minimi)
- Python 3.8+ (consigliato 3.10/3.11)
- `virtualenv` o `venv` per isolare le dipendenze
- Se si vuole usare la parte ROS: Docker + strumenti ROS (la cartella `docker/` contiene i workspace pronti)
- Consigliata estensione Devcontainers di Visual Studio Code
