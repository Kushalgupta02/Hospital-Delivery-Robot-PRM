"""CoppeliaSim backend: remote-API robot + OMPL planner.
Needs CoppeliaSim running (scene built with build_scene.py + a Pioneer P3-DX).
simOMPL signatures differ slightly between versions: check the simOMPL manual page if a call fails."""
import math, time
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from config import *
from controller import wheel_speeds
from prm import path_length

YAW_OFFSET = 0.0   # add a constant if your robot model drives sideways


class CoppeliaRobot:
    def __init__(self, robot='/PioneerP3DX', left='/PioneerP3DX/leftMotor',
                 right='/PioneerP3DX/rightMotor'):
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')
        self.client.setStepping(True)
        self.h = self.sim.getObject(robot)
        self.lm, self.rm = self.sim.getObject(left), self.sim.getObject(right)
        self.collisions = 0
        self.sim.startSimulation()

    @property
    def time(self):
        return self.sim.getSimulationTime()

    def pose(self):
        p = self.sim.getObjectPosition(self.h, -1)
        o = self.sim.getObjectOrientation(self.h, -1)
        return (p[0], p[1], o[2] + YAW_OFFSET)

    def xy(self):
        return self.pose()[:2]

    def set_cmd(self, v, w):
        vl, vr = wheel_speeds(v, w)
        self.sim.setJointTargetVelocity(self.lm, vl)
        self.sim.setJointTargetVelocity(self.rm, vr)

    def step(self):
        self.client.step()

    def wait(self, seconds):
        self.set_cmd(0, 0)
        for _ in range(int(seconds / DT)):
            self.step()

    def stop(self):
        self.sim.stopSimulation()


class OMPLPlanner:
    """Same interface as PRMPlanner, but planning is done by simOMPL inside CoppeliaSim."""

    def __init__(self, robot, algo='PRM', hospital='/Hospital', max_time=5.0, n_pts=150):
        self.r, self.sim = robot, robot.sim
        self.ompl = robot.client.require('simOMPL')
        self.algo, self.max_time, self.n_pts = algo, max_time, n_pts
        sim = self.sim
        shapes = sim.getObjectsInTree(sim.getObject(hospital), sim.object_shape_type)
        self.obst = sim.createCollection(0)
        for s in shapes:
            sim.addItemToCollection(self.obst, sim.handle_single, s, 0)
        self.parts = sim.createCollection(0)
        sim.addItemToCollection(self.parts, sim.handle_tree, robot.h, 0)

    def plan(self, start, goal):
        o = self.ompl
        task = o.createTask('hospital')
        o.setAlgorithm(task, getattr(o.Algorithm, self.algo))
        ss = o.createStateSpace('xy', o.StateSpaceType.position2d, self.r.h,
                                list(BOUNDS_LOW), list(BOUNDS_HIGH), 1)
        o.setStateSpace(task, [ss])
        o.setCollisionPairs(task, [self.parts, self.obst])
        o.setStartState(task, list(start))
        o.setGoalState(task, list(goal))
        o.setup(task)
        t0 = time.perf_counter()
        ok, flat = o.compute(task, self.max_time, -1, self.n_pts)
        dt = time.perf_counter() - t0
        o.destroyTask(task)
        if not ok:
            return None, {'plan_time': dt, 'length': 0.0}
        pts = [np.array([flat[i], flat[i + 1]]) for i in range(0, len(flat), 2)]
        return pts, {'plan_time': dt, 'length': path_length(pts)}

    def estimate(self, a, b):
        return math.dist(a, b)

    def refresh(self):
        pass

    def path_is_free(self, pts):
        return True       # hook for a proximity-sensor check (extension)


def compare_algorithms(robot, pairs, algos=('PRM', 'PRMstar', 'RRTConnect', 'RRTstar'), trials=10):
    from hospital_map import STOPS
    rows = []
    for a in algos:
        pl = OMPLPlanner(robot, a)
        for s, g in pairs:
            for _ in range(trials):
                p, info = pl.plan(STOPS[s], STOPS[g])
                rows.append({'algo': a, 'pair': f'{s}->{g}', 'ok': p is not None,
                             'plan_time': info['plan_time'], 'length': info['length']})
    return rows
