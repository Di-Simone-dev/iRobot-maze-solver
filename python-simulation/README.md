# Python Simulation

Questo README descrive la simulazione Python inclusa nella cartella `python-simulation`.

## Scopo
La simulazione fornisce un ambiente locale per sviluppare, testare e visualizzare comportamenti di risoluzione di labirinti per iRobot Create3 senza necessità di avviare i container ROS.

## Requisiti
- Python 3.8+ (consigliato 3.10/3.11)
- Virtual environment: `venv` o `virtualenv`
- Dipendenze: contenute in `requirements.txt` e `linux.txt`

## Installazione veloce
Dal repository, entra nella cartella `python-simulation` e crea un ambiente virtuale:

```bash
cd python-simulation
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
sudo apt install python3-tk
```

## Avvio
- Avviare l'interfaccia grafica (GUI):

```bash
python src/behavtree_robot.py
```

## Panoramica dei file principali (`python-simulation/src`)
- `__init__.py` — package init
- `behavtree_robot.py` — nodo/componente centrale di esecuzione dei behaviour tree e di lancio della gui
- `gui.py` — classe grafica per visualizzare la mappa e lo stato del robot
- `config.py` — impostazioni e parametri della simulazione
- `helpers.py` — funzioni di utilità generali
- `helpers_phys.py` — helper per la fisica/sensori della simulazione
- `mapping.py` — gestione/memorizzazione della mappa esplorata
- `mazegenerator.py` — generatori di labirinti per test
- `behaviours/` — directory contenente i comportamenti e le logiche
- `graphical_trees/` — behaviour tree grafici generati dal codice

## Configurazione
- Modifica i parametri in `config.py` per cambiare dimensioni del labirinto, velocità, o parametri del robot.

## Logging & Debug
- I messaggi di log vengono stampati su console; per debug aumentare il livello di logging nel codice.
