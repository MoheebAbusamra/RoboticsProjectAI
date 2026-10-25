"""Offline checks for visual transfers, suspended jobs and payload clearance."""
import copy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

from factory_bot.cargo_scene import CargoScene, job_key, stock_name, offset_pose

PKG = Path(__file__).resolve().parents[1]


def config():
    return yaml.safe_load((PKG/'config/cargo.yaml').read_text())


def job(state='ASSIGNED', **kwargs):
    return dict(id='J1', robot='robot1', pickup='TOOL_CRIB', dropoff='ASSEMBLY_A',
                created=1., state=state, **kwargs)


def test_same_prop_transfers_from_source_to_rotated_deck_to_destination():
    c = config(); scene = CargoScene(c); j = job()
    scene.update_jobs([j]); name = scene.bindings[job_key(j)]['model']
    assert scene.desired[name] == offset_pose(c['stations']['TOOL_CRIB']['pickup_pose'], -.065, 0.)
    assert scene.desired[stock_name('TOOL_CRIB',0)] == tuple(c['parked'])
    j['state'] = 'PICKED'; scene.update_jobs([j]); scene.update_robot_pose('robot1',(3,4,math.pi/2))
    x,y,z,yaw = scene.desired[name]
    assert abs(x-3.05)<1e-8 and abs(y-3.965)<1e-8 and z==.179
    assert scene.desired[stock_name('TOOL_CRIB',0)] != tuple(c['parked'])
    j['state'] = 'DELIVERED'; scene.update_jobs([j])
    assert not scene.carried and not scene.bindings
    assert name in scene.deposits.values()
    assert scene.desired[name][2] == c['stations']['ASSEMBLY_A']['dropoff_pose'][2]


def test_emergency_preserves_suspended_cargo_and_uses_other_slot():
    scene = CargoScene(config()); old = job('PICKED')
    scene.update_jobs([old]); original = copy.deepcopy(scene.bindings[job_key(old)])
    emergency = dict(id='E1',robot='robot1',pickup='ASSEMBLY_B',dropoff='QC_LAB',created=2.,state='PICKED')
    scene.update_jobs([old,emergency])
    assert scene.bindings[job_key(old)] == original
    assert len(scene.carried)==2
    assert scene.bindings[job_key(emergency)]['slot'] != original['slot']
    emergency['state']='DELIVERED';scene.update_jobs([old,emergency])
    assert scene.bindings[job_key(old)] == original and len(scene.carried)==1


def test_two_robots_can_reserve_the_same_station_without_overlapping_supply_units():
    scene=CargoScene(config()); a=job(); b=job();b.update(id='J2',robot='robot2')
    scene.update_jobs([a,b]); models=[v['model'] for v in scene.bindings.values()]
    assert scene.desired[models[0]] != scene.desired[models[1]]
    assert all(scene.desired[stock_name('TOOL_CRIB',i)]==tuple(config()['parked']) for i in range(2))


def test_failures_requeues_and_duplicates_do_not_leave_or_grow_payloads():
    scene=CargoScene(config()); j=job('PICKED');scene.update_jobs([j])
    initial=len(scene.desired);j['state']='FAILED';scene.update_jobs([j])
    assert not scene.bindings and not scene.carried
    for i in range(40):
        j=job('PICKED');j.update(id=str(i),created=float(i));scene.update_jobs([j,j])
        j['state']='DELIVERED';scene.update_jobs([j])
    assert len(scene.desired)==initial and len(scene.deposits)<=4
    scene.update_jobs([])
    assert not scene.bindings


def test_every_station_has_cargo_and_two_deck_slots_clear_lidar_and_shell():
    cfg=config(); stations=yaml.safe_load((PKG/'config/stations.yaml').read_text())['stations']
    assert set(cfg['stations'])==set(stations)
    manifest=json.loads((PKG/'models/station_cargo/manifest.json').read_text())
    for kind in cfg['types']:
        b=manifest['bounds_m'][kind]
        assert cfg['deck_z']+b['max'][2] < .23-.01
        for dx,dy in cfg['deck_slots']:
            assert -.1475 < dx+b['min'][0] < dx+b['max'][0] < .066
            assert -.11 < dy+b['min'][1] < dy+b['max'][1] < .11
        obj=PKG/'models/station_cargo/meshes'/f'{kind}.obj'
        assert sum(l.startswith('f ') for l in obj.read_text().splitlines())==manifest['triangles'][kind]


def test_cargo_adds_no_collision_and_all_predeclared_models_are_unique():
    root=ET.parse(PKG/'worlds/factory.sdf').getroot(); models=root.findall('world/model')
    cargo=[m for m in models if m.get('name','').startswith('cargo_')]
    assert len(cargo)==45 and len({m.get('name') for m in cargo})==45
    assert not any(m.findall('.//collision') for m in cargo)
    assert not any(m.get('name','').startswith('press_guard') for m in root.findall('.//collision'))
