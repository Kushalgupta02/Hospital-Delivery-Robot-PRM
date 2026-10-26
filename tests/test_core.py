import sys, os, unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from hospital_map import build_world, STOPS
from planners import PRMPlanner
from prm import path_length


class TestCore(unittest.TestCase):
    def test_stops_are_free(self):
        w = build_world()
        for n, p in STOPS.items():
            self.assertTrue(w.point_free(p), n)

    def test_wall_blocks_straight_line(self):
        self.assertFalse(build_world().edge_free(STOPS['Pharmacy'], STOPS['Ward3']))

    def test_path_found_and_valid(self):
        w = build_world()
        p, _ = PRMPlanner(w, seed=0).plan(STOPS['Pharmacy'], STOPS['Ward4'])
        self.assertIsNotNone(p)
        self.assertTrue(w.path_free(p))
        self.assertGreater(path_length(p), 8.0)

    def test_blocked_corridor_gives_no_path(self):
        w = build_world()
        pl = PRMPlanner(w, seed=0)
        w.add_dynamic('box', (-2.5, -1.0, 4.0, 1.0))   # seals the corridor
        pl.refresh()
        self.assertIsNone(pl.plan(STOPS['Pharmacy'], STOPS['Ward3'])[0])
        w.remove_dynamic('box')
        pl.refresh()
        self.assertIsNotNone(pl.plan(STOPS['Pharmacy'], STOPS['Ward3'])[0])


if __name__ == '__main__':
    unittest.main()
