# Python Simulation

Questo README descrive la simulazione Python inclusa nella cartella `python-simulation`.

## Scopo
La simulazione fornisce un ambiente locale per sviluppare, testare e visualizzare comportamenti di risoluzione di labirinti per iRobot Create3 senza necessità di avviare i container ROS.

## Requisiti
- Python 3.8+ (consigliato 3.10/3.11)
- Virtual environment: `venv` o `virtualenv`
- Dipendenze: contenute in `requirements.txt`

## Installazione veloce
Dal repository, entra nella cartella `python-simulation` e crea un ambiente virtuale:

```bash
cd python-simulation
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Avvio
- Avviare l'interfaccia grafica (GUI):

```bash
python3 -m src.gui
```

- Avviare il comportamento/headless (se disponibile):

```bash
python3 -m src.behavtree_robot
```

## Panoramica dei file principali (`python-simulation/src`)
- `__init__.py` — package init
- `behavtree_robot.py` — nodo/componente di esecuzione dei behaviour tree
- `gui.py` — interfaccia grafica per visualizzare la mappa e lo stato del robot
- `config.py` — impostazioni e parametri della simulazione
- `helpers.py` — funzioni di utilità generali
- `helpers_phys.py` — helper per la fisica/sensori della simulazione
- `mapping.py` — gestione/memorizzazione della mappa esplorata
- `mazegenerator.py` — generatori di labirinti per test
- `behaviours/` — directory contenente i comportamenti e le logiche
- `graphical_trees/` — alberi comportamentali visuali

## Configurazione
- Modifica i parametri in `config.py` per cambiare dimensioni del labirinto, velocità, o parametri del robot.

## Logging & Debug
- I messaggi di log vengono stampati su console; per debug aumentare il livello di logging nel codice (es. `logging.DEBUG`).

## Contribuire
- Apri issue per bug o miglioramenti.
- Per modifiche: crea una branch, testa localmente con la GUI e apri una pull request.

## Note
- Questo README descrive l'uso locale via Python; per integrazioni ROS o deployment in container, consulta la documentazione nella cartella `docker/`.
