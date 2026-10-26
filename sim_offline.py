"""Kinematic stand-in for CoppeliaSim so the whole pipeline runs without it."""
import math
from config import *


class OfflineEnv:
    def __init__(self, worlds, start, theta=0.0, events=()):
        self.worlds = worlds               # [planning world, truth world]
        self.truth = worlds[-1]
        self.x, self.y = start
        self.th = theta
        self.v = self.w = 0.0
        self.time = 0.0
        self.collisions, self._in_hit = 0, False
        self.trace = [(self.x, self.y)]
        self.events = sorted(events, key=lambda e: e[0])   # (t, 'add'|'remove', name, rect)

    def xy(self):
        return (self.x, self.y)

    def pose(self):
        return (self.x, self.y, self.th)

    def set_cmd(self, v, w):
        self.v, self.w = v, w

    def step(self):
        while self.events and self.events[0][0] <= self.time:
            _, kind, name, rect = self.events.pop(0)
            for wd in self.worlds:
                wd.add_dynamic(name, rect) if kind == 'add' else wd.remove_dynamic(name)
            print(f"[t={self.time:6.1f}s] event: {kind} obstacle '{name}'")
        self.th += self.w * DT
        self.x += self.v * math.cos(self.th) * DT
        self.y += self.v * math.sin(self.th) * DT
        self.time += DT
        hit = not self.truth.point_free((self.x, self.y))
        if hit and not self._in_hit:
            self.collisions += 1
        self._in_hit = hit
        self.trace.append((self.x, self.y))

    def wait(self, seconds):
        self.set_cmd(0, 0)
        for _ in range(int(seconds / DT)):
            self.step()
