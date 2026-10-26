"""Waypoint follower (heading P-control). Shared by the offline sim and CoppeliaSim."""
import math
import numpy as np
from config import *


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def wheel_speeds(v, w):
    """Unicycle (v m/s, w rad/s) -> left/right wheel angular velocity (rad/s)."""
    return (v - w * AXLE / 2) / WHEEL_R, (v + w * AXLE / 2) / WHEEL_R


class WaypointFollower:
    def __init__(self, pts, v_max=V_MAX, k_w=K_W, tol=WAYPOINT_TOL):
        self.pts = [np.asarray(p, float) for p in pts]
        self.i, self.v_max, self.k_w, self.tol = 0, v_max, k_w, tol

    def command(self, pose):
        x, y, th = pose
        last = len(self.pts) - 1
        while self.i <= last:
            tol = 0.06 if self.i == last else self.tol
            if math.hypot(self.pts[self.i][0] - x, self.pts[self.i][1] - y) < tol:
                self.i += 1
            else:
                break
        if self.i > last:
            return 0.0, 0.0, True
        dx, dy = self.pts[self.i][0] - x, self.pts[self.i][1] - y
        err = wrap(math.atan2(dy, dx) - th)
        v = 0.0 if abs(err) > 1.0 else self.v_max * math.cos(err)
        if self.i == last:
            v = min(v, 0.8 * math.hypot(dx, dy) + 0.03)
        return v, self.k_w * err, False

    def remaining(self, xy):
        return [np.asarray(xy, float)] + self.pts[self.i:]
