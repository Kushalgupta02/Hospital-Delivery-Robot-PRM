"""MediRoute entry point.
  python main.py offline --requests Ward2 Ward4:E@20 Ward1 Ward3 --dynamic --battery 45
  python main.py experiments
  python main.py scene          # build the scene in CoppeliaSim
  python main.py coppelia --requests Ward2 Ward4 Ward1 [--algo PRM|PRMstar|RRTConnect|RRTstar|own]
"""
import argparse, os
from config import *
from hospital_map import build_world, STOPS
from battery import BatteryModel
from logger import Logger
from mission import Mission
from planners import PRMPlanner
from scheduler import parse_request
from plots import plot_scene

os.makedirs('results', exist_ok=True)


def run_offline(a):
    plan_world, truth = build_world(PLAN_RADIUS, a.door), build_world(ROBOT_RADIUS, a.door)
    planner = PRMPlanner(plan_world, n=a.n, door_bias=not a.no_bias, seed=a.seed)
    print(f"roadmap: {len(planner.prm.G.nodes)} nodes, {planner.prm.G.number_of_edges()} edges, "
          f"built in {planner.build_time:.2f}s")
    plot_scene(plan_world, planner.prm, fname='results/roadmap.png', title='PRM roadmap')
    events = []
    if a.dynamic:    # a person steps into the corridor at t=15 s and leaves at t=60 s
        events = [(15.0, 'add', 'person', (-2.0, 0.0, -1.6, 0.4)), (60.0, 'remove', 'person', None)]
    from sim_offline import OfflineEnv
    env = OfflineEnv([plan_world, truth], STOPS['Pharmacy'], events=events)
    bat = BatteryModel(a.battery)
    log = Logger()
    Mission(planner, env, bat, log).run([parse_request(r) for r in a.requests])
    log.print_table(); log.save('results/delivery_log.csv')
    print(f"total time {env.time:.1f}s | collisions {env.collisions} | charges {bat.charges}")
    plot_scene(plan_world, planner.prm, trace=env.trace, fname='results/run.png',
               title='Robot trace over PRM roadmap')


def run_coppelia(a):
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient  # noqa
    from coppelia_backend import CoppeliaRobot, OMPLPlanner
    robot = CoppeliaRobot()
    if a.algo == 'own':
        planner = PRMPlanner(build_world(PLAN_RADIUS, a.door), n=a.n, seed=a.seed)
    else:
        planner = OMPLPlanner(robot, a.algo)
    log = Logger()
    try:
        Mission(planner, robot, BatteryModel(a.battery), log).run([parse_request(r) for r in a.requests])
    finally:
        robot.stop()
    log.print_table(); log.save('results/delivery_log_coppelia.csv')


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['offline', 'coppelia', 'experiments', 'scene'])
    ap.add_argument('--requests', nargs='+', default=['Ward2', 'Ward4', 'Ward1'])
    ap.add_argument('--n', type=int, default=N_SAMPLES)
    ap.add_argument('--door', type=float, default=DOOR_WIDTH)
    ap.add_argument('--battery', type=float, default=BATTERY_CAPACITY)
    ap.add_argument('--seed', type=int, default=None)
    ap.add_argument('--no-bias', action='store_true')
    ap.add_argument('--dynamic', action='store_true')
    ap.add_argument('--algo', default='PRM')
    ap.add_argument('--trials', type=int, default=10)
    a = ap.parse_args()
    if a.mode == 'offline':
        run_offline(a)
    elif a.mode == 'coppelia':
        run_coppelia(a)
    elif a.mode == 'scene':
        import build_scene; build_scene.main()
    else:
        import experiments; experiments.run_all(a.trials)
