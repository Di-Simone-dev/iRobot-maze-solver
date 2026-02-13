# Docker / Workspace ROS

Questa cartella contiene i workspace e relativi container per eseguire la simulazione del iRobot Create 3


## Panoramica della struttura
- `comandi/` — script e comandi utili per avviare o gestire container/workspace.
- `create3_ws/` — workspace principale per Create3 responsabile dei comportamenti principali del Create3 e della simulazione in Gazebo.
- `ros_control_ws/` — workspace per i componenti di controllo del robot.
- `ros_msg_ws/` — workspace per creazione e build dei messaggi personalizzati nel progetto.

## Come usare (linee guida generali)
- I workspace sono pensati per essere buildati all'interno di un ambiente ROS appropriato (container o macchina con tool ROS installati).
- Maggiori istruzioni sono presenti all'interno dei README presenti nei singoli workspace.

## Avvio
0. Segure le istruzioni presenti nei workspace create3_ws e ros_control_ws.
   
1. Avviare entrambi i container dei precedentemente citati workspace seguendo le istruzioni presenti nei rispettivi readme.

2. Avviare i nodi mediante i launch file come indicato nei rispettivi readme.
   
3. Lanciare da terminale i comandi tesiderati per comandare il robot:
  ```bash
  ros2 topic pub --once /command custom_msg/msg/Command "{command: 'UNDOCK'}"
  ros2 topic pub --once /command custom_msg/msg/Command "{command: 'MODE A'}"
  ros2 topic pub --once /command custom_msg/msg/Command "{command: 'MODE B'}"
  ros2 topic pub --once /command custom_msg/msg/Command "{command: 'SOLVE'}"
  ```

## Logs e diagnosi
- La cartella `log/` contiene snapshot dei build precedenti. Se il build fallisce, confronta con i `log/` per individuare regressioni.
