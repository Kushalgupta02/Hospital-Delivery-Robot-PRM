"""Offline experiments (own PRM). Results -> results/*.csv and *.png"""
import csv, time, itertools
import numpy as np
from config import *
from hospital_map import build_world, STOPS
from planners import PRMPlanner
from plots import plot_curves

PAIRS = [('Pharmacy', 'Ward1'), ('Pharmacy', 'Ward2'), ('Pharmacy', 'Ward3'),
         ('Pharmacy', 'Ward4'), ('Ward2', 'Ward3'), ('Ward4', 'ChargingDock')]


def _save(rows, name):
    with open(f'results/{name}.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)


def _trial(world, n, bias, seed):
    pl = PRMPlanner(world, n=n, door_bias=bias, seed=seed)
    ok, lens = 0, []
    for a, b in PAIRS:
        p, info = pl.plan(STOPS[a], STOPS[b])
        if p:
            ok += 1; lens.append(info['length'])
    return pl.build_time, ok / len(PAIRS), (np.mean(lens) if lens else np.nan)


def exp_samples(trials=10, Ns=(100, 200, 400, 800)):
    world, rows = build_world(), []
    for n, bias in itertools.product(Ns, (False, True)):
        res = [_trial(world, n, bias, s) for s in range(trials)]
        rows.append({'N': n, 'door_bias': bias, 'success_rate': np.mean([r[1] for r in res]),
                     'build_time_s': np.mean([r[0] for r in res]),
                     'mean_len_m': np.nanmean([r[2] for r in res])})
        print(rows[-1])
    _save(rows, 'exp_samples')
    plot_curves(rows, 'N', 'success_rate', 'door_bias', 'results/exp_samples.png',
                'samples N', 'success rate', 'Success vs roadmap size')


def exp_doors(trials=10, widths=(1.0, 0.8, 0.7, 0.65), n=300):
    rows = []
    for dw, bias in itertools.product(widths, (False, True)):
        world = build_world(door_w=dw)
        res = [_trial(world, n, bias, s) for s in range(trials)]
        rows.append({'door_w': dw, 'door_bias': bias, 'success_rate': np.mean([r[1] for r in res])})
        print(rows[-1])
    _save(rows, 'exp_doors')
    plot_curves(rows, 'door_w', 'success_rate', 'door_bias', 'results/exp_doors.png',
                'door width (m)', 'success rate', 'Narrow-passage problem')


def exp_multi_query(Q=60):
    world = build_world()
    rng = np.random.default_rng(1)
    names = list(STOPS)
    qs = [rng.choice(names, 2, replace=False) for _ in range(Q)]
    t0 = time.perf_counter()
    pl = PRMPlanner(world, seed=0)
    for a, b in qs:
        pl.plan(STOPS[a], STOPS[b])
    multi = time.perf_counter() - t0
    t0 = time.perf_counter()
    for i, (a, b) in enumerate(qs[:15]):
        PRMPlanner(world, seed=i).plan(STOPS[a], STOPS[b])
    single = (time.perf_counter() - t0) / 15 * Q
    rows = [{'queries': Q, 'build_once_total_s': multi, 'rebuild_each_total_s_est': single,
             'speedup': single / multi}]
    print(rows[0]); _save(rows, 'exp_multi_query')


def run_all(trials=10):
    exp_samples(trials); exp_doors(trials); exp_multi_query()
