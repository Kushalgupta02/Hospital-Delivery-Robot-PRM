# MediRoute: Hospital Delivery Robot with Probabilistic Roadmap (PRM)

What makes it different from a plain PRM demo:
1. Door-biased sampling: extra samples in doorways fix the narrow-passage weakness.
2. Roadmap refresh: samples are reused and edges re-validated when a dynamic obstacle appears or leaves.
3. Mission logic: emergency (priority) requests, nearest-first ordering, battery model with charging dock.
4. Two backends with one planner interface: own PRM + offline simulator, or CoppeliaSim + OMPL.
5. Experiments built in: samples vs success, door width vs success, multi-query speed-up.

## Install
    Python 3.9+      CoppeliaSim Edu 4.x (only for the coppelia backend)
    pip install -r requirements.txt

## Run
    python -m unittest discover tests            # self-test
    python main.py offline --requests Ward2 Ward4:E@20 Ward1 Ward3 --dynamic --battery 45
    python main.py experiments --trials 10       # CSV + PNG in results/

    # CoppeliaSim (start it first, scene open)
    python main.py scene                         # builds walls, furniture, delivery dummies
    # drag a Pioneer P3-DX into the scene, name it PioneerP3DX, save scenes/hospital.ttt
    python main.py coppelia --requests Ward2 Ward4 Ward1 --algo PRM
    # --algo own | PRM | PRMstar | RRTConnect | RRTstar

Request syntax: `Ward4`, `Ward4:E` (emergency), `Ward4:E@30` (emergency arriving at t=30 s).
