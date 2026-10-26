import csv


class Logger:
    FIELDS = ['leg', 'target', 'status', 'plan_time_s', 'path_len_m', 'travel_time_s',
              'replans', 'battery_after', 'collisions']

    def __init__(self):
        self.rows = []

    def add(self, **kw):
        self.rows.append(kw)

    def save(self, path):
        with open(path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=self.FIELDS, extrasaction='ignore')
            w.writeheader()
            w.writerows(self.rows)

    def print_table(self):
        print(f"{'leg':>3} {'target':<13}{'status':<7}{'plan(s)':>8}{'len(m)':>8}{'time(s)':>9}"
              f"{'replan':>7}{'batt%':>7}{'coll':>5}")
        for r in self.rows:
            print(f"{r['leg']:>3} {r['target']:<13}{r['status']:<7}{r['plan_time_s']:>8.3f}"
                  f"{r['path_len_m']:>8.2f}{r['travel_time_s']:>9.1f}{r['replans']:>7}"
                  f"{r['battery_after']:>7.1f}{r['collisions']:>5}")
