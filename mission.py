"""Mission executive: queue -> battery check -> plan -> follow -> replan -> unload."""
import numpy as np
from config import *
from controller import WaypointFollower
from hospital_map import STOPS
from scheduler import DeliveryQueue


class Mission:
    def __init__(self, planner, robot, battery, logger, base='Pharmacy', dock='ChargingDock'):
        self.planner, self.robot, self.battery, self.log = planner, robot, battery, logger
        self.base, self.dock = base, dock
        self.leg = 0

    # ----- low level: follow one path; returns False if blocked -----
    def _follow(self, path):
        f = WaypointFollower(path)
        last, steps = np.array(self.robot.xy()), 0
        while True:
            v, w, done = f.command(self.robot.pose())
            if done:
                self.robot.set_cmd(0, 0)
                return True
            self.robot.set_cmd(v, w)
            self.robot.step()
            cur = np.array(self.robot.xy())
            self.battery.consume(float(np.linalg.norm(cur - last)), DT)
            last, steps = cur, steps + 1
            if steps % 5 == 0 and not self.planner.path_is_free(f.remaining(cur)):
                self.robot.set_cmd(0, 0)
                return False
            if steps > MAX_LEG_STEPS:
                self.robot.set_cmd(0, 0)
                return False

    # ----- one leg with replanning -----
    def travel(self, name):
        goal = STOPS[name]
        t0, c0 = self.robot.time, self.robot.collisions
        plan_t = length = 0.0
        replans, ok = 0, False
        for _ in range(MAX_REPLANS + 1):
            path, info = self.planner.plan(self.robot.xy(), goal)
            plan_t += info['plan_time']
            if path is None:                       # blocked: wait, then re-validate roadmap
                self.robot.wait(1.0)
                self.planner.refresh()
                replans += 1
                continue
            length += info['length']
            if self._follow(path):
                ok = True
                break
            replans += 1
            self.planner.refresh()                 # obstacle detected -> repair roadmap
        self.leg += 1
        self.log.add(leg=self.leg, target=name, status='OK' if ok else 'FAIL', plan_time_s=plan_t,
                     path_len_m=length, travel_time_s=self.robot.time - t0, replans=replans,
                     battery_after=self.battery.level, collisions=self.robot.collisions - c0)
        return ok

    def _recharge(self):
        print(f"[t={self.robot.time:6.1f}s] battery {self.battery.level:.0f}% -> going to dock")
        self.travel(self.dock)
        self.robot.wait(self.battery.charge_time())
        self.battery.full()

    def _needs_charge(self, ward):
        here = self.robot.xy()
        need = self.battery.cost(self.planner.estimate(here, STOPS[ward])
                                 + self.planner.estimate(STOPS[ward], STOPS[self.dock]))
        return self.battery.level - 1.2 * need < BATTERY_RESERVE

    def run(self, requests):
        q = DeliveryQueue(requests)
        while len(q):
            req = q.pop_next(self.robot.time, self.robot.xy(), STOPS, self.planner.estimate)
            if req is None:
                self.robot.wait(1.0)
                continue
            if self._needs_charge(req.ward):
                self._recharge()
            tag = ' (EMERGENCY)' if req.priority else ''
            print(f"[t={self.robot.time:6.1f}s] delivering to {req.ward}{tag}")
            if self.travel(req.ward):
                self.robot.wait(UNLOAD_TIME)
        print(f"[t={self.robot.time:6.1f}s] returning to {self.base}")
        self.travel(self.base)
