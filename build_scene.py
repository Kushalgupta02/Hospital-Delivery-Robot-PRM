"""Builds the hospital scene in a running CoppeliaSim from hospital_map.py.
Afterwards drag a Pioneer P3-DX (Model browser > robots > mobile) to the Pharmacy and
name it 'PioneerP3DX' (default). Save the scene as scenes/hospital.ttt."""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from hospital_map import wall_rects, FURNITURE, STOPS

WALL_H = 1.0


def make_box(sim, rect, h, alias, parent):
    x0, y0, x1, y1 = rect
    s = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [x1 - x0, y1 - y0, h], 0)
    sim.setObjectPosition(s, -1, [(x0 + x1) / 2, (y0 + y1) / 2, h / 2])
    sim.setObjectInt32Param(s, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(s, sim.shapeintparam_respondable, 1)
    try:
        sim.setObjectSpecialProperty(s, sim.objectspecialproperty_collidable)
    except Exception:
        pass                           # newer versions: set 'collidable' in object properties
    sim.setObjectAlias(s, alias)
    sim.setObjectParent(s, parent, True)
    return s


def main():
    sim = RemoteAPIClient().require('sim')
    sim.stopSimulation()
    root = sim.createDummy(0.1)
    sim.setObjectAlias(root, 'Hospital')
    for i, r in enumerate(wall_rects()):
        make_box(sim, r, WALL_H, f'wall_{i}', root)
    for i, r in enumerate(FURNITURE):
        make_box(sim, r, 0.6, f'furn_{i}', root)
    floor = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [10, 8, 0.02], 0)
    sim.setObjectPosition(floor, -1, [0, 0, -0.01])
    sim.setObjectInt32Param(floor, sim.shapeintparam_static, 1)
    sim.setObjectAlias(floor, 'floor')
    for name, (x, y) in STOPS.items():
        d = sim.createDummy(0.15)
        sim.setObjectPosition(d, -1, [x, y, 0.05])
        sim.setObjectAlias(d, name)
    print('Hospital scene created. Add the P3-DX robot and save the scene.')


if __name__ == '__main__':
    main()
