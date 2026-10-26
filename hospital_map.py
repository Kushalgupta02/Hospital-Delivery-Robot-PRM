"""2D hospital floor plan: walls, furniture, doors, delivery points.
The same geometry is used by the own PRM, the offline simulator and build_scene.py
(which creates the matching 3D scene in CoppeliaSim)."""
import numpy as np
from config import *

UPPER_DOORS = [-3.5, 0.0, 3.5]      # door centres in the wall above the corridor
LOWER_DOORS = [-2.5, 1.5]           # door centres in the wall below the corridor
UP_Y, LOW_Y = (1.0, 1.2), (-1.2, -1.0)   # wall y-ranges (corridor is y in [-1, 1])

STOPS = {
    'Pharmacy':     (-3.5, 2.6),
    'Ward1':        (0.0, 2.6),
    'Ward2':        (3.5, 2.6),
    'Ward3':        (-2.5, -2.6),
    'Ward4':        (3.0, -2.6),
    'ChargingDock': (-4.4, 0.0),
}

FURNITURE = [
    (-4.9, 3.5, -3.4, 3.9),                            # pharmacy shelf
    (-1.8, 3.3, -0.8, 3.9), (0.8, 3.3, 1.8, 3.9),      # ward 1 beds
    (4.2, 1.5, 4.9, 2.5),                              # ward 2 bed
    (-4.9, -3.9, -3.9, -2.9),                          # ward 3 bed
    (3.9, -3.9, 4.9, -3.0), (0.3, -3.9, 1.0, -3.0),    # ward 4 beds
    (2.6, -0.9, 3.1, -0.4),                            # trolley parked in corridor
]


def _wall_with_gaps(x0, x1, y0, y1, centres, dw):
    rects, cur = [], x0
    for c in sorted(centres):
        rects.append((cur, y0, c - dw / 2, y1))
        cur = c + dw / 2
    rects.append((cur, y0, x1, y1))
    return rects


def wall_rects(door_w=DOOR_WIDTH):
    walls = _wall_with_gaps(-5, 5, *UP_Y, UPPER_DOORS, door_w)
    walls += _wall_with_gaps(-5, 5, *LOW_Y, LOWER_DOORS, door_w)
    walls += [(-2.1, 1.2, -1.9, 4.0), (1.9, 1.2, 2.1, 4.0), (-0.1, -4.0, 0.1, -1.2)]
    return walls


def door_points():
    uy, ly = sum(UP_Y) / 2, sum(LOW_Y) / 2
    return [(c, uy) for c in UPPER_DOORS] + [(c, ly) for c in LOWER_DOORS]


class World:
    """Axis-aligned rectangles inflated by the robot radius (robot = point)."""

    def __init__(self, radius=PLAN_RADIUS, door_w=DOOR_WIDTH):
        self.r, self.door_w = radius, door_w
        self.static = wall_rects(door_w) + FURNITURE
        self.dynamic = {}
        self.low, self.high = np.array(BOUNDS_LOW), np.array(BOUNDS_HIGH)
        self.door_points = door_points()
        self._refresh()

    def _refresh(self):
        rects = self.static + list(self.dynamic.values())
        a = np.array(rects, dtype=float)
        self._inf = a + np.array([-self.r, -self.r, self.r, self.r])

    def add_dynamic(self, name, rect):
        self.dynamic[name] = rect
        self._refresh()

    def remove_dynamic(self, name):
        self.dynamic.pop(name, None)
        self._refresh()

    def points_free(self, P):
        P = np.atleast_2d(P)
        x, y = P[:, 0:1], P[:, 1:2]
        r = self._inf
        hit = (x > r[:, 0]) & (x < r[:, 2]) & (y > r[:, 1]) & (y < r[:, 3])
        inb = np.all((P >= self.low + self.r) & (P <= self.high - self.r), axis=1)
        return inb & ~hit.any(axis=1)

    def point_free(self, p):
        return bool(self.points_free(np.asarray(p, float))[0])

    def edge_free(self, a, b):
        a, b = np.asarray(a, float), np.asarray(b, float)
        n = max(2, int(np.linalg.norm(b - a) / EDGE_STEP) + 1)
        t = np.linspace(0, 1, n)[:, None]
        return bool(self.points_free(a + (b - a) * t).all())

    def path_free(self, pts):
        return all(self.edge_free(pts[i], pts[i + 1]) for i in range(len(pts) - 1))


def build_world(radius=PLAN_RADIUS, door_w=DOOR_WIDTH):
    return World(radius, door_w)
