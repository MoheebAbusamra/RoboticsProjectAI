"""Integration checks for factory geometry, policies and moving obstacle routes."""
import itertools
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from factory_bot.planners import plan
from factory_bot.planning_grid import PlanningGrid

PKG = Path(__file__).resolve().parents[1]


def config(name):
    return yaml.safe_load((PKG / 'config' / (name + '.yaml')).read_text())


def grid():
    return PlanningGrid.from_files(PKG / 'maps/factory_map.yaml',
                                   PKG / 'config/planning.yaml',
                                   PKG / 'config/zones.yaml')


def test_every_station_pair_respects_hazard_and_destination_access():
    g = grid()
    stations = config('stations')['stations']
    for (a, p), (b, q) in itertools.permutations(stations.items(), 2):
        start, goal = (p['x'], p['y']), (q['x'], q['y'])
        allow = g.restricted_access(start, goal)
        result = plan(g, start, goal, 'astar', allow_restricted=allow)
        assert result.success, (a, b, result.reason)
        assert not any(g.forbidden[r, c] for r, c in result.path), (a, b)
        if a != 'QC_LAB' and b != 'QC_LAB':
            assert not allow
            assert not any(g.restricted[r, c] for r, c in result.path), (a, b)
        else:
            assert allow, (a, b)


def test_qc_robot_can_leave_and_take_another_job():
    g = grid()
    stations = config('stations')['stations']
    p, q = stations['QC_LAB'], stations['TOOL_CRIB']
    start, goal = (p['x'], p['y']), (q['x'], q['y'])
    assert g.restricted_access(start, goal)
    assert plan(g, start, goal, 'astar', allow_restricted=True).success


def test_slow_delivery_station_has_a_real_speed_limit():
    g = grid()
    p = config('stations')['stations']['ASSEMBLY_B']
    assert g.slow[g.world_to_cell(p['x'], p['y'])]
    zones = config('zones')['zones']
    assert zones['ASSEMBLY_SLOW']['max_speed'] == .15


def test_moving_obstacle_sweeps_clear_static_geometry():
    manifest = json.loads((PKG / 'models/industrial_factory/manifest.json').read_text())
    for name, spec in config('obstacles')['movers'].items():
        # Conservative axis-aligned envelopes for the existing straight routes.
        hx, hy = (1.25, .45) if name.startswith('forklift') else (.36, .36)
        a, b = np.array(spec['route'][0]), np.array(spec['route'][1])
        for t in np.linspace(0, 1, 80):
            x, y = a + t * (b - a)
            for c in manifest['collisions']:
                cx, cy, _ = c['xyz']
                sx, sy, _ = c['size']
                assert abs(x-cx) >= hx+sx/2 or abs(y-cy) >= hy+sy/2, (name, c['name'], x, y)


def test_visual_meshes_have_only_simple_collision_shapes_and_local_resources():
    world = ET.parse(PKG / 'worlds/factory.sdf').getroot()
    assert not world.findall('.//collision/geometry/mesh')
    for uri in world.findall('.//visual/geometry/mesh/uri'):
        assert uri.text.startswith('model://')
        model, relative = uri.text.removeprefix('model://').split('/', 1)
        assert model in ('industrial_factory', 'station_cargo')
        assert (PKG / 'models' / model / relative).is_file()
    manifest = json.loads((PKG / 'models/industrial_factory/manifest.json').read_text())
    # Premium visual budget increased with the requested fidelity upgrade.
    # The physics budget remains the original set of primitive collisions.
    assert sum(manifest['triangles'].values()) < 80000
    assert manifest['static_collision_shapes'] < 30


def test_mapping_sweep_has_robot_clearance():
    g = grid()
    points = [[1., 1.]] + config('mapping_route')['waypoints_gazebo']
    for a, b in zip(points, points[1:]):
        a, b = np.array(a), np.array(b)
        for t in np.linspace(0, 1, max(2, int(np.linalg.norm(b-a)/.025))):
            x, y = a+t*(b-a)-1.0  # Gazebo -> map frame
            assert g.passable(*g.world_to_cell(x, y), allow_restricted=True), (a, b, x, y)
