import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.collections import LineCollection
import numpy as np
from hospital_map import STOPS


def plot_scene(world, prm=None, path=None, trace=None, fname='results/scene.png', title=''):
    fig, ax = plt.subplots(figsize=(9, 7.2))
    for (x0, y0, x1, y1) in world.static:
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, color='#444'))
    for (x0, y0, x1, y1) in world.dynamic.values():
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, color='#d33'))
    if prm is not None:
        segs = [(prm.pts[a], prm.pts[b]) for a, b in prm.G.edges]
        ax.add_collection(LineCollection(segs, colors='#9bc', linewidths=0.4))
        ax.plot(prm.pts[:, 0], prm.pts[:, 1], '.', ms=2, color='#369')
    if trace is not None:
        t = np.array(trace)
        ax.plot(t[:, 0], t[:, 1], '-', color='#e80', lw=1.6, label='robot trace')
    if path is not None:
        p = np.array(path)
        ax.plot(p[:, 0], p[:, 1], 'g-o', lw=2, ms=4, label='planned path')
    for n, (x, y) in STOPS.items():
        ax.plot(x, y, 'r*', ms=11)
        ax.annotate(n, (x, y), textcoords='offset points', xytext=(5, 5), fontsize=8)
    ax.set_xlim(-5.2, 5.2); ax.set_ylim(-4.2, 4.2); ax.set_aspect('equal')
    ax.set_title(title or 'MediRoute: hospital floor')
    if trace is not None or path is not None:
        ax.legend(loc='lower left', fontsize=8)
    fig.tight_layout(); fig.savefig(fname, dpi=130); plt.close(fig)


def plot_curves(rows, xkey, ykey, groupkey, fname, xlabel, ylabel, title):
    fig, ax = plt.subplots(figsize=(6, 4))
    for g in sorted({r[groupkey] for r in rows}):
        pts = sorted((r[xkey], r[ykey]) for r in rows if r[groupkey] == g)
        ax.plot([p[0] for p in pts], [p[1] for p in pts], 'o-', label=str(g))
    ax.set_xlabel(xlabel); ax.set_ylabel(ylabel); ax.set_title(title); ax.grid(alpha=.3)
    ax.legend(); fig.tight_layout(); fig.savefig(fname, dpi=130); plt.close(fig)
