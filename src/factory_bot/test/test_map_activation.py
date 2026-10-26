"""Offline transaction, queue, acknowledgement and process-ownership checks. No simulation."""
from concurrent.futures import Future
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
import yaml

from factory_bot.map_catalog import inspect_map, freeze_map, navigation_command, robot_options, check_robot_poses
from factory_bot.map_transition import MapTransition

PKG = Path(__file__).resolve().parents[1]


def completed(value):
    f = Future(); f.set_result(value); return f


@pytest.fixture
def map_file(tmp_path):
    pixels = np.full((32, 32), 254, dtype=np.uint8); pixels[5, 7] = 0; pixels[10, 12] = 128
    (tmp_path/'floor.pgm').write_bytes(b'P5\n32 32\n255\n'+pixels.tobytes())
    path = tmp_path/'saved map.yaml'
    path.write_text(yaml.safe_dump(dict(image='floor.pgm', resolution=.05, origin=[-1.3,-1.3,0.],
        negate=0, occupied_thresh=.65, free_thresh=.196, mode='trinary')))
    return path


def test_map_inspection_and_copy_keep_real_occupancy_and_ignore_later_source_edits(map_file, tmp_path):
    asset = inspect_map(map_file); assert asset.pixels[5,7] == 0 and asset.pixels[10,12] == 128
    (tmp_path/'floor.pgm').write_bytes(b'damaged after preview')
    frozen = freeze_map(asset, tmp_path/'copies')
    assert frozen.digest == asset.digest and inspect_map(frozen.path).digest == asset.digest
    assert frozen.source == str(map_file) and frozen.path != str(map_file)
    assert frozen.info()['known_percent'] < 100


@pytest.mark.parametrize('change', ['rotation', 'negative_resolution', 'bad_threshold', 'missing_image', 'mode', 'rgb'])
def test_incompatible_maps_are_rejected_before_activation(map_file, change):
    meta = yaml.safe_load(map_file.read_text())
    if change == 'rotation': meta['origin'][2] = .1
    elif change == 'negative_resolution': meta['resolution'] = -1
    elif change == 'bad_threshold': meta['free_thresh'] = .9
    elif change == 'missing_image': meta['image'] = 'absent.pgm'
    elif change == 'mode': meta['mode'] = 'scale'
    elif change == 'rgb': (map_file.parent/'floor.pgm').write_bytes(b'P6\n32 32\n255\n'+bytes(32*32*3))
    map_file.write_text(yaml.safe_dump(meta))
    with pytest.raises((ValueError, OSError)): inspect_map(map_file)


def test_command_passes_paths_poses_and_distinct_robot_settings_as_arguments_without_shell():
    poses = {'robot1':[3.,4.,.5], 'robot2':[6.,7.,0.]}
    profiles = {'robot1':{'algorithm':'bfs','controller':'pid'},
                'robot2':{'algorithm':'dijkstra','controller':'pure_pursuit'}}
    cmd = navigation_command('/tmp/a map;$(ignored).yaml', ['robot1','robot2'], poses=poses, profiles=profiles)
    assert cmd[:4] == ['ros2','launch','factory_bot','navigation_group.launch.py']
    assert 'map:=/tmp/a map;$(ignored).yaml' in cmd
    assert json.loads(next(v.split(':=',1)[1] for v in cmd if v.startswith('initial_poses:='))) == poses
    assert robot_options(json.dumps(profiles), ['robot1','robot2'], 'profiles') == profiles
    assert robot_options('{"robot1":[1,2,0]}', ['robot1'], 'poses')['robot1'] == [1.,2.,0.]
    with pytest.raises(ValueError): robot_options('{"other":[1,2,0]}', ['robot1'], 'poses')


class Asset:
    def __init__(self, path): self.path = path; self.source = path; self.digest = path
    def info(self): return {'path':self.path, 'source':self.source, 'fingerprint':self.digest}


class Backend:
    def __init__(self):
        self.calls=[]; self.allowed=True; self.is_held=True; self.is_stopped=True; self.is_released=True
        self.failure=''; self.pause_future=None; self.ready_flag=True
        self.saved={'poses':{'robot1':[3.,4.,0.], 'robot2':[7.,6.,1.]},
                    'profiles':{'robot1':{'algorithm':'bfs','controller':'pid'}}}
    def check_idle(self):
        if not self.allowed: raise ValueError('busy fleet')
    def freeze(self, asset):
        saved=Asset('copy:'+asset.path); saved.source=asset.source; return saved
    def pause(self, enabled):
        self.calls.append(('pause',enabled))
        if enabled and self.pause_future:return self.pause_future
        return completed(NS(success=True,message=json.dumps(self.saved) if enabled else 'resumed'))
    def hold(self, value):self.calls.append(('hold',value))
    def held(self):return self.is_held
    def released(self):return self.is_released
    def stop(self):self.calls.append(('stop',))
    def stopped(self):return self.is_stopped
    def load_fleet(self, path):self.calls.append(('load',path));return completed(NS(result=0))
    def start(self,path,snapshot):self.calls.append(('start',path,snapshot))
    def ready(self,path,since):
        if path == self.failure:raise RuntimeError('ICP could not initialize')
        return self.ready_flag, 'waiting for localization'


def machine():
    io=Backend();m=MapTransition(io,Asset('old.yaml'));m.phase='READY';return m,io


def advance(m, n=12, start=1):
    for t in range(start,start+n):m.tick(float(t))


def test_success_holds_before_restart_and_reports_new_map_only_after_every_ack():
    m,io=machine();m.request(Asset('new.yaml'),0,'request')
    m.tick(1);assert m.phase=='HOLDING' and m.active.path=='old.yaml'
    m.tick(2);assert io.calls[-1]==('stop',)
    m.tick(3);m.tick(4);assert m.phase=='VERIFYING' and m.active.path=='old.yaml'
    io.ready_flag=False;m.tick(5);assert ('hold',False) not in io.calls
    io.ready_flag=True;m.tick(6);assert m.phase=='RELEASING'
    io.is_released=False;m.tick(7);assert ('pause',False) not in io.calls
    io.is_released=True;m.tick(8);m.tick(9)
    assert m.phase=='READY' and m.active.path=='new.yaml' and m.status()['verified']
    assert ('start','new.yaml',io.saved) in io.calls
    assert io.calls.index(('hold',False)) < io.calls.index(('pause',False))


def test_busy_fleet_rejects_without_process_or_motion_changes():
    m,io=machine();io.allowed=False
    with pytest.raises(ValueError,match='busy'):m.request(Asset('new'),0)
    assert not io.calls and m.phase=='READY'


def test_denied_pause_preserves_old_running_group_and_late_ack_never_releases_uncertain_hold():
    m,io=machine();io.pause_future=completed(NS(success=False,message='new job arrived'))
    m.request(Asset('new'),0);m.tick(1)
    assert m.phase=='READY' and io.calls==[('pause',True)]
    m,io=machine();io.pause_future=Future();m.request(Asset('new'),0);m.tick(9)
    assert m.phase=='ERROR_HELD' and ('hold',True) in io.calls
    io.pause_future.set_result(NS(success=True,message=json.dumps(io.saved)));advance(m)
    assert m.phase=='ERROR_HELD' and ('pause',False) not in io.calls


def test_malformed_success_ack_keeps_dispatch_held_instead_of_claiming_ready():
    m,io=machine();io.pause_future=completed(NS(success=True,message='not JSON'))
    m.request(Asset('new'),0);m.tick(1)
    assert m.phase=='ERROR_HELD' and not m.status()['verified']


def test_new_map_failure_restores_previous_copy_with_original_poses_and_profiles():
    m,io=machine();io.failure='new.yaml';m.request(Asset('new.yaml'),0);advance(m)
    assert m.phase=='READY' and m.active.path=='copy:old.yaml'
    assert 'ICP' in m.error and m.rollback
    assert ('load','copy:old.yaml') in io.calls and ('start','copy:old.yaml',io.saved) in io.calls


def test_failed_new_and_rollback_localization_stay_held_and_timeout_is_bounded():
    m,io=machine();m.request(Asset('new'),0);advance(m,4);io.ready_flag=False
    m.tick(35);assert m.rollback and m.phase=='STOPPING'
    m.tick(36);m.tick(37);assert m.phase=='VERIFYING'
    m.tick(68);assert m.phase=='ERROR_HELD'
    assert ('pause',False) not in io.calls


def test_never_launches_replacement_before_old_processes_are_gone():
    m,io=machine();io.is_stopped=False;m.request(Asset('new'),0);advance(m,3)
    assert not any(c[0] in ('load','start') for c in io.calls)
    m.tick(11);m.tick(20);assert m.phase=='ERROR_HELD'


def test_uncertain_resume_does_not_restart_under_a_possibly_dispatched_job():
    m,io=machine();original=io.pause
    io.pause=lambda enabled:original(True) if enabled else Future()
    m.request(Asset('new'),0);advance(m,6)
    assert m.phase=='RESUMING'
    m.tick(15)
    assert m.phase=='ERROR_HELD' and m.active.path=='new'
    assert len([c for c in io.calls if c[0]=='stop'])==1
    assert ('load','copy:old.yaml') not in io.calls


def test_pose_and_profile_snapshot_is_refreshed_after_hold_acknowledgement():
    m,io=machine();captured={'poses':{'robot1':[3.01,4.,0.]},
        'profiles':{'robot1':{'algorithm':'dijkstra','controller':'pure_pursuit'}}}
    io.capture=lambda previous:captured
    m.request(Asset('new'),0);advance(m)
    assert ('start','new',captured) in io.calls


def test_repair_can_pass_hold_gate_when_the_owned_group_is_already_stopped():
    from factory_bot.map_coordinator import MapCoordinator
    snapshot={'poses':{'robot1':[1.,2.,0.]},'profiles':{}}
    h=NS(group=NS(process=object(),stopped=lambda:True),machine=NS(was_ready=False))
    assert MapCoordinator.held(h)
    assert MapCoordinator.capture(h,snapshot) is snapshot


def fleet_harness():
    from factory_bot.fleet_manager import FleetManager
    st={'pose':[2.,3.,0.],'state':'IDLE','stopped':True,'defaults':{'algorithm':'astar','controller':'pid'}}
    h=NS(robots={'robot1':{'state':'IDLE','job':None,'suspended':None,'status':dict(st)}},
         safe_snapshot=None,map_paused=False,status_at={'robot1':100.},jobs=[],counter=0,
         publish_state=lambda:None,stations={'TOOL_CRIB':{},'QC_LAB':{}},
         now=lambda:100.,get=lambda key:'TOOL_CRIB',
         get_logger=lambda:NS(warn=lambda *a,**k:None,info=lambda *a,**k:None,error=lambda *a,**k:None))
    h.allocate=lambda now:FleetManager.allocate(h,now)
    return h


def test_fleet_pause_retains_new_jobs_and_disables_all_dispatch_including_emergencies(monkeypatch):
    from factory_bot.fleet_manager import FleetManager
    monkeypatch.setattr('factory_bot.fleet_manager.time.monotonic',lambda:100.)
    h=fleet_harness();response=FleetManager.on_map_pause(h,NS(data=True),NS())
    assert response.success and h.map_paused
    FleetManager.on_emergency(h,NS(data='{"station":"QC_LAB"}'))
    assert len(h.jobs)==1 and h.jobs[0].state=='WAITING'
    FleetManager.step(h);assert h.jobs[0].state=='WAITING'
    h.status_at={};again=FleetManager.on_map_pause(h,NS(data=True),NS())
    assert again.success and json.loads(again.message)==json.loads(response.message)
    FleetManager.on_map_pause(h,NS(data=False),NS());assert not h.map_paused and len(h.jobs)==1


@pytest.mark.parametrize('reason',['job','moving','stale','aside'])
def test_fleet_rechecks_idleness_atomically_at_pause_service(monkeypatch,reason):
    from factory_bot.fleet_manager import FleetManager
    monkeypatch.setattr('factory_bot.fleet_manager.time.monotonic',lambda:100.)
    h=fleet_harness();rec=h.robots['robot1']
    if reason=='job':rec['job']=object()
    elif reason=='moving':rec['status']['stopped']=False
    elif reason=='stale':h.status_at['robot1']=0.
    else:rec['status']['aside']=True
    response=FleetManager.on_map_pause(h,NS(data=True),NS())
    assert not response.success and not h.map_paused


def test_navigator_hold_stops_existing_motion_and_defers_a_goal_until_release():
    from factory_bot.navigator import Navigator
    calls=[];h=NS(map_hold=False,pending_map_goal=None,stop=lambda:calls.append('stop'),
                 publish_status=lambda:calls.append('status'),on_goal=lambda msg:calls.append(msg.data))
    Navigator.on_map_hold(h,NS(data=True));Navigator.on_map_hold(h,NS(data=True))
    Navigator.on_goal(h,NS(data='STORES'));Navigator.loop(h)
    assert h.pending_map_goal=='STORES' and calls.count('status')==1 and calls.count('stop')==2
    Navigator.on_map_hold(h,NS(data=False));assert 'STORES' in calls and h.pending_map_goal is None


def test_readiness_rejects_old_instances_wrong_map_and_unaccepted_icp():
    from factory_bot.map_coordinator import MapCoordinator
    records={'/fleet/state':{'map_yaml':'new','map_paused':True},
             '/traffic/state':{'map_yaml':'new','instance_id':'traffic-new'}}
    for r in ['robot1','robot2']:
        records['/'+r+'/nav_status']={'map_yaml':'new','instance_id':r+'-new','map_hold':True,'stopped':True}
        records['/'+r+'/icp_status']={'map_yaml':'new','instance_id':r+'-icp-new','accepted':2,'ok':True}
    h=NS(group=NS(alive=lambda:True),robots=['robot1','robot2'],old_instances={},
         value=lambda topic,since=0:records[topic],get=lambda key:True,server_ready=lambda path:True)
    assert MapCoordinator.ready(h,'new',0)[0]
    records['/robot1/icp_status']['accepted']=0;assert not MapCoordinator.ready(h,'new',0)[0]
    records['/robot1/icp_status']['accepted']=2;h.old_instances['/robot1/nav_status']='robot1-new'
    assert not MapCoordinator.ready(h,'new',0)[0]
    h.old_instances={};records['/traffic/state']['map_yaml']='old'
    assert not MapCoordinator.ready(h,'new',0)[0]


def test_process_signals_only_retained_matching_identities(monkeypatch,tmp_path):
    from factory_bot.map_coordinator import NavigationProcess
    owner=NavigationProcess(tmp_path);owner.owned={123:'original',456:'reused',789:'gone'};calls=[]
    monkeypatch.setattr(owner,'identity',lambda pid:{123:'original',456:'different'}.get(pid))
    monkeypatch.setattr('factory_bot.map_coordinator.os.kill',lambda pid,sig:calls.append((pid,sig)))
    owner.signal_owned(15)
    assert calls==[(123,15)]


def test_map_preparation_checks_fingerprint_and_current_pose_coverage(map_file,tmp_path):
    from factory_bot.map_coordinator import MapCoordinator
    h=NS(share=PKG,freeze=lambda asset:freeze_map(asset,tmp_path/'copies'))
    asset=inspect_map(map_file)
    prepared,grid=MapCoordinator.prepare(h,str(map_file),asset.digest)
    assert prepared.digest==asset.digest
    check_robot_poses(grid,{'robot1':[-.3,-.3,0.]})
    with pytest.raises(ValueError,match='current pose'):
        check_robot_poses(grid,{'robot1':[8.,9.,0.]})
    with pytest.raises(ValueError,match='changed since preview'):
        MapCoordinator.prepare(h,str(map_file),'old fingerprint')


def test_fleet_load_changes_grid_only_while_paused(map_file):
    from factory_bot.fleet_manager import FleetManager
    from nav2_msgs.srv import LoadMap
    h=fleet_harness();h.map_share=str(PKG);h.map_yaml='old';h.grid=object()
    h.set_parameters=lambda values:[NS(successful=True)]
    rejected=FleetManager.on_map_load(h,NS(map_url=str(map_file)),NS())
    assert rejected.result!=LoadMap.Response.RESULT_SUCCESS and h.map_yaml=='old'
    h.map_paused=True
    accepted=FleetManager.on_map_load(h,NS(map_url=str(map_file)),NS())
    assert accepted.result==LoadMap.Response.RESULT_SUCCESS and h.map_yaml==str(map_file)
    assert h.grid.occupied.shape==(32,32)


def test_coordinator_ignores_stale_status_and_icp_status_expires_with_sensor_time(monkeypatch):
    from factory_bot.map_coordinator import MapCoordinator
    from factory_bot.icp_localizer import IcpLocalizer
    monkeypatch.setattr('factory_bot.map_coordinator.time.monotonic',lambda:100.)
    h=NS(data={'topic':({'map_yaml':'old'},90.)})
    assert not MapCoordinator.value(h,'topic')
    h.data['topic']=({'map_yaml':'new'},99.);assert MapCoordinator.value(h,'topic')['map_yaml']=='new'
    assert not MapCoordinator.value(h,'topic',99.5)
    messages=[];clock=[10.]
    h=NS(map_yaml='new',instance_id='new-node',accepted_since_start=2,rejected_in_row=0,last_scan_at=9.8,
         get_clock=lambda:NS(now=lambda:NS(nanoseconds=int(clock[0]*1e9))),
         status_pub=NS(publish=lambda m:messages.append(json.loads(m.data))))
    IcpLocalizer.publish_map_status(h);assert messages[-1]['ok']
    clock[0]=11.;IcpLocalizer.publish_map_status(h);assert not messages[-1]['ok']


def test_supervisor_construction_does_not_spawn_children_and_bad_requests_do_not_crash(monkeypatch,tmp_path):
    import factory_bot.map_coordinator as module
    from rclpy.node import Node
    from builtin_interfaces.msg import Time
    parameters={};messages=[];started=[]
    monkeypatch.setattr(Node,'__init__',lambda self,*a,**k:None)
    def declare(self,name,default):
        parameters[name]=str(tmp_path) if name=='state_dir' else default
    monkeypatch.setattr(Node,'declare_parameter',declare)
    monkeypatch.setattr(Node,'get_parameter',lambda self,name:NS(value=parameters[name]))
    monkeypatch.setattr(Node,'create_publisher',lambda *a,**k:NS(publish=lambda m:messages.append(m)))
    monkeypatch.setattr(Node,'create_subscription',lambda *a,**k:None)
    monkeypatch.setattr(Node,'create_client',lambda *a,**k:NS(service_is_ready=lambda:False))
    monkeypatch.setattr(Node,'create_timer',lambda *a,**k:None)
    monkeypatch.setattr(Node,'get_clock',lambda self:NS(now=lambda:NS(to_msg=lambda:Time(sec=1))))
    monkeypatch.setattr(Node,'get_logger',lambda self:NS(info=lambda *a:None,warn=lambda *a:None))
    monkeypatch.setattr(module,'get_package_share_directory',lambda name:str(PKG))
    monkeypatch.setattr(module,'AsyncParameterClient',lambda *a:NS(services_are_ready=lambda:False))
    class Group:
        def __init__(self,*a):pass
        def start(self,cmd):started.append(cmd)
        def alive(self):return True
        def track(self):pass
        def close(self):pass
    monkeypatch.setattr(module,'NavigationProcess',Group)
    coordinator=module.MapCoordinator()
    try:
        assert not started
        coordinator.on_request(NS(data='[]'))
        assert 'JSON object' in coordinator.request_error
        coordinator.receive('bad',NS(data='[]'));assert 'bad' not in coordinator.data
        coordinator.tick()
        assert len(started)==1 and started[0][3]=='navigation_group.launch.py'
        assert coordinator.machine.phase=='VERIFYING'
    finally:coordinator.close()
