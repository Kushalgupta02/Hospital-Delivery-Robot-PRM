"""Own PRM with two additions: door-biased sampling and roadmap refresh (re-validation)."""
import time
import numpy as np
import networkx as nx
from scipy.spatial import cKDTree
from config import *


class PRM:
    def __init__(self, world, n=N_SAMPLES, k=K_NEIGHBOURS, door_bias=True,
                 door_samples=DOOR_SAMPLES, seed=None):
        self.w, self.n, self.k = world, n, k
        self.door_bias, self.door_samples = door_bias, door_samples
        self.rng = np.random.default_rng(seed)
        self.build_time = 0.0
        self.build()

    # ---------- learning phase ----------
    def _sample(self):
        pts = []
        while sum(len(p) for p in pts) < self.n:
            P = self.rng.uniform(self.w.low, self.w.high, (self.n, 2))
            pts.append(P[self.w.points_free(P)])
        pts = np.vstack(pts)[: self.n]
        if self.door_bias:
            hw = self.w.door_w / 2
            for cx, cy in self.w.door_points:
                D = np.column_stack([self.rng.uniform(cx - hw, cx + hw, self.door_samples),
                                     self.rng.uniform(cy - 0.6, cy + 0.6, self.door_samples)])
                D = np.vstack([D, [[cx, cy], [cx, cy - 0.5], [cx, cy + 0.5]]])
                pts = np.vstack([pts, D[self.w.points_free(D)]])
        return pts

    def build(self):
        t0 = time.perf_counter()
        self.pts = self._sample()
        self.tree = cKDTree(self.pts)
        self._connect()
        self.build_time = time.perf_counter() - t0

    def _connect(self):
        G = nx.Graph()
        free = self.w.points_free(self.pts)
        G.add_nodes_from(int(i) for i in np.nonzero(free)[0])
        _, idx = self.tree.query(self.pts, k=self.k + 1)
        for i in list(G.nodes):
            for j in idx[i][1:]:
                j = int(j)
                if j in G and not G.has_edge(i, j) and self.w.edge_free(self.pts[i], self.pts[j]):
                    G.add_edge(i, j, weight=float(np.linalg.norm(self.pts[i] - self.pts[j])))
        self.G = G

    def refresh(self):
        """Dynamic obstacle added/removed: keep the samples, re-validate nodes and edges."""
        self._connect()

    # ---------- query phase ----------
    def query(self, start, goal, k_link=25):
        start, goal = np.asarray(start, float), np.asarray(goal, float)
        if not (self.w.point_free(start) and self.w.point_free(goal)):
            return None
        H = self.G.copy()
        for name, p in (('s', start), ('g', goal)):
            _, idx = self.tree.query(p, k=k_link)
            linked = 0
            for j in idx:
                j = int(j)
                if j in self.G and self.w.edge_free(p, self.pts[j]):
                    H.add_edge(name, j, weight=float(np.linalg.norm(p - self.pts[j])))
                    linked += 1
                    if linked >= self.k:
                        break
        if self.w.edge_free(start, goal):
            H.add_edge('s', 'g', weight=float(np.linalg.norm(start - goal)))
        try:
            route = nx.shortest_path(H, 's', 'g', weight='weight')
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            return None
        return [start if v == 's' else goal if v == 'g' else self.pts[v] for v in route]


def path_length(path):
    return float(sum(np.linalg.norm(np.asarray(path[i + 1]) - np.asarray(path[i]))
                     for i in range(len(path) - 1)))


def shortcut(path, world):
    """Greedy visibility smoothing: jump to the farthest waypoint that is directly reachable."""
    out, i = [path[0]], 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not world.edge_free(path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out
