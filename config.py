"""Central configuration for MediRoute. Change values here, not in the modules."""
# ---- floor (metres) ----
BOUNDS_LOW = (-5.0, -4.0)
BOUNDS_HIGH = (5.0, 4.0)
DOOR_WIDTH = 1.0
# ---- robot ----
ROBOT_RADIUS = 0.22          # real footprint radius (P3-DX is about 0.22 m)
SAFETY_MARGIN = 0.05         # extra clearance used only by the planner
PLAN_RADIUS = ROBOT_RADIUS + SAFETY_MARGIN
WHEEL_R = 0.0975             # P3-DX wheel radius
AXLE = 0.381                 # P3-DX axle length
# ---- PRM ----
N_SAMPLES = 400
K_NEIGHBOURS = 12
DOOR_SAMPLES = 12            # extra samples per door (door-biased sampling)
EDGE_STEP = 0.05             # collision-check resolution along an edge
# ---- controller ----
DT = 0.05                    # simulation step (CoppeliaSim default is 0.05 s)
V_MAX = 0.35
K_W = 2.5
WAYPOINT_TOL = 0.10
# ---- mission ----
UNLOAD_TIME = 2.0            # seconds
MAX_REPLANS = 8
MAX_LEG_STEPS = 8000
# ---- battery model ----
BATTERY_CAPACITY = 100.0
ENERGY_PER_M = 2.0
ENERGY_IDLE_PER_S = 0.02
BATTERY_RESERVE = 12.0
CHARGE_RATE = 5.0            # % per simulated second at the dock
