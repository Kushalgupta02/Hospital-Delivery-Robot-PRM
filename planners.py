"""Planner interface used by the mission:
   plan(start, goal) -> (waypoints | None, info) ; estimate(a, b) ; refresh() ; path_is_free(pts)"""
import time
import numpy as np
from prm import PRM, path_length, shortcut
from config import *


class PRMPlanner:
    def __init__(self, world, n=N_SAMPLES, k=K_NEIGHBOURS, door_bias=True, smooth=True, seed=None):
        self.world, self.smooth = world, smooth
        self.prm = PRM(world, n, k, door_bias, seed=seed)
        self._cache = {}

    @property
    def build_time(self):
        return self.prm.build_time

    def plan(self, start, goal):
        t0 = time.perf_counter()
        path = self.prm.query(start, goal)
        if path is not None and self.smooth:
            path = shortcut(path, self.world)
        dt = time.perf_counter() - t0
        return path, {'plan_time': dt, 'length': path_length(path) if path else 0.0}

    def estimate(self, a, b):
        key = (round(a[0], 1), round(a[1], 1), round(b[0], 1), round(b[1], 1))
        if key not in self._cache:
            p, info = self.plan(a, b)
            self._cache[key] = info['length'] if p else float('inf')
        return self._cache[key]

    def refresh(self):
        self.prm.refresh()
        self._cache.clear()

    def path_is_free(self, pts):
        return self.world.path_free(pts)
