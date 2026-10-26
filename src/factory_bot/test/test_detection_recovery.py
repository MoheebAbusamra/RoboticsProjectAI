"""Offline regression checks for the observed forklift / following-robot stall."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from factory_bot.normal_obstacle_policy import NormalObstaclePolicy
from factory_bot.motion_progress import MotionProgress
from factory_bot.live_layer import LiveLayer
from factory_bot.planning_grid import PlanningGrid
from factory_bot.traffic import RobotView, TrafficRules

PKG = Path(__file__).resolve().parents[1]


def grid():
    occ = np.zeros((160,160),dtype=bool)
    occ[0,:]=occ[-1,:]=occ[:,0]=occ[:,-1]=True
    return PlanningGrid(occ,np.zeros_like(occ),.05,(0,0))


def test_wait_requires_three_seconds_and_a_fresh_scan():
    p=NormalObstaclePolicy()
    assert p.decide(True,0,0)=='wait'
    assert p.decide(True,2.9,2.9)=='wait'
    assert p.decide(True,3.1,2.9)=='wait'
    assert p.decide(True,3.1,3.1)=='replan'


def test_a_successful_replan_never_grants_an_unlimited_forward_bypass():
    p=NormalObstaclePolicy();p.decide(True,0,0);p.rerouted(True,3)
    assert p.decide(True,3.1,3.1)=='detour'
    assert p.decide(True,5.9,5.9)=='detour'
    assert p.decide(True,6.1,6.1)=='wait'
    assert p.decide(True,9.2,9.2)=='replan'
    p.rerouted(False,9.2)
    assert p.decide(True,9.3,9.3)=='wait'


def test_clear_sector_allows_driving_after_detour_or_wait():
    p=NormalObstaclePolicy();p.decide(True,0,0)
    assert p.decide(False,2,2)=='wait'
    assert p.decide(False,3.1,3.1)=='drive'
    p.rerouted(True,4)
    assert p.decide(False,4.1,4.1)=='drive'


def test_no_progress_is_detected_even_when_the_controller_keeps_driving():
    w=MotionProgress();pose=(2.,3.,0.)
    assert w.observe(pose,0,.2,.1)=='ok'
    assert w.observe(pose,3.1,.2,.1)=='stalled'
    # Waiting/replanning without physical movement must not erase the episode.
    assert w.observe(pose,6.2,0,0)=='ok'
    assert w.observe(pose,9.3,.2,.1)=='stalled'
    assert w.observe(pose,33.2,.2,.1)=='failed'


def test_actual_motion_or_intentional_traffic_reset_prevents_false_stall():
    w=MotionProgress();w.observe((0,0,0),0,.2,0)
    assert w.observe((.1,0,0),3.1,.2,0)=='ok'
    assert w.observe((.1,0,0),6.2,.2,0)=='stalled'
    w.reset()
    assert w.observe((.1,0,0),50,0,0)=='ok'
    assert w.observe((.1,0,.2),51,0,.4)=='ok'


def test_low_lidar_points_use_the_correct_forward_offset():
    g=grid();layer=LiveLayer(g);pose=(1.,3.,0.)
    layer.update(pose,[.5],0,.01,.02,4,0,laser_x=.205)
    expected=g.world_to_cell(1.705,3.)
    incorrect=g.world_to_cell(1.60,3.)
    assert layer.mask(0)[expected] and not layer.mask(0)[incorrect]
    assert layer.front_unmapped_distance(pose,[.5],0,.01,.02,4,laser_x=.205)==.5


def test_close_following_robot_waits_even_if_its_cargo_has_higher_priority():
    rules=TrafficRules(grid())
    lead=RobotView('robot1',(4.65,4.,math.pi),True,[(4.65,4),(2.,4)],'TO_PICKUP','routine')
    rear=RobotView('robot2',(5.01,3.85,math.pi),True,[(5.01,3.85),(2.,4)],'TO_DROPOFF','routine')
    cmd=rules.update({'robot1':lead,'robot2':rear},0)
    assert cmd['robot2']['cmd']=='wait' and cmd['robot2']['because']=='robot1'
    assert cmd['robot1']['cmd']=='go'


def test_blocked_active_robot_is_eligible_to_make_room():
    rules=TrafficRules(grid())
    blocked=RobotView('robot1',(3.,3.,0.),True,[(3.,3.),(6.,3.)],blocked_s=3.)
    assert rules._stationary(blocked,{})


def test_forklift_one_box_encloses_every_visual_vertex_and_lidar_matches_it():
    root=ET.parse(PKG/'worlds/factory.sdf').getroot()
    link=root.find("world/model[@name='forklift_1']/link")
    collisions=link.findall('collision');assert len(collisions)==1
    c=collisions[0];proxy=link.find("visual[@name='lidar_outer_envelope']")
    center=np.array([float(v) for v in c.findtext('pose').split()[:3]])
    size=np.array([float(v) for v in c.findtext('geometry/box/size').split()])
    verts=np.array([[float(v) for v in line.split()[1:4]] for line in
                    (PKG/'models/industrial_factory/meshes/forklift.obj').read_text().splitlines()
                    if line.startswith('v ')])
    assert np.all(verts.min(axis=0)>=center-size/2)
    assert np.all(verts.max(axis=0)<=center+size/2)
    assert proxy.findtext('pose')==c.findtext('pose')
    assert proxy.findtext('geometry/box/size')==c.findtext('geometry/box/size')
    flag=int(proxy.findtext('visibility_flags'));assert flag==0x80000000
    assert flag & 0x8fffffff and not flag & 0x0fffffff


def test_safety_sensor_has_a_body_height_scan_ahead_of_its_own_geometry():
    root=ET.parse(PKG/'urdf/fleet_bot.urdf.xacro').getroot()
    origin=root.find("joint[@name='obstacle_laser_joint']/origin")
    x,y,z=[float(v) for v in origin.get('xyz').split()]
    assert x>.20 and y==0 and .11+z==.12
    sensor=ET.parse(PKG/'urdf/fleet_bot.gazebo.xacro').getroot().find(".//sensor[@name='obstacle_lidar_${ns}']")
    assert sensor.findtext('lidar/scan/horizontal/samples')=='91'
    assert int(sensor.findtext('lidar/visibility_mask')) & 0x80000000


def navigator_harness(now=3.1):
    """Exercise real navigator methods without creating a ROS node or physics world."""
    from types import SimpleNamespace
    from factory_bot.navigator import Navigator

    class Harness:
        loop=Navigator.loop
        normal_obstacle_mode=Navigator.normal_obstacle_mode
        handle_normal_obstacle=Navigator.handle_normal_obstacle

        def __init__(self):
            self.robot='robot1';self.time=now;self.state='DRIVING';self.aside=None;self.loc_ok=True
            self.goal={'blocked_since':0.,'blocked_count':1,'dynamic_wait':True,
                       'min_front':math.inf,'traffic_wait_s':0.}
            self.front_min=self.front_dynamic_min=.60;self.front_scan_at=now
            self.normal_obstacle=NormalObstaclePolicy();self.normal_obstacle.rerouted(True,3.)
            self.motion_progress=MotionProgress();self.last_command=(0.,0.)
            from factory_bot.backup_recovery import BackupRecovery
            self.backup=BackupRecovery();self.odom_at=-math.inf
            self.odom_pose=None
            self.detour_active=False;self.commands=[]
            self.follower=SimpleNamespace(step=lambda *a,**kw:(.25,.40,False))

        def get(self,name):
            return {'scan_timeout':.6,'dynamic_stop_distance':.8,'blocked_timeout':30.,
                    'normal_obstacle_wait':3.,'stop_distance':.35}[name]

        def now(self):return self.time
        blocked_expired=Navigator.blocked_expired
        def pose(self):return (2.,3.,0.)
        def traffic_wait(self):return False
        def speed_limit(self,pose):return None
        def get_logger(self):return SimpleNamespace(info=lambda *a,**k:None,warn=lambda *a,**k:None)
        def drive(self,v,w):self.commands.append((v,w));self.last_command=(v,w)
        def stop(self):self.drive(0.,0.)
        def finish(self,state,reason=''):self.state=state;self.reason=reason;self.stop()

    return Harness()


def test_real_control_loop_keeps_close_range_stop_during_a_guarded_detour():
    nav=navigator_harness();nav.front_min=nav.front_dynamic_min=.30;nav.loop()
    assert nav.commands[-1]==(0.,.40)
    assert nav.state=='BLOCKED' and nav.goal['blocked_since']==0.
    nav.time=31.1;nav.front_scan_at=31.1;nav.loop()
    assert nav.state=='FAILED' and nav.commands[-1]==(0.,0.)


def test_real_control_loop_stops_if_either_scan_stream_is_stale():
    nav=navigator_harness();nav.front_scan_at=nav.time-1.;nav.loop()
    assert nav.commands[-1]==(0.,0.)


def test_detour_can_follow_a_safe_straight_prefix_before_the_path_bends():
    nav=navigator_harness();nav.loop()
    assert nav.commands[-1]==(.25,.40) and nav.detour_active
    assert nav.goal['blocked_since']==0.
    nav.time=6.1;nav.front_scan_at=6.1;nav.last_command=(0.,0.);nav.loop()
    assert nav.commands[-1]==(0.,0.) and nav.state=='BLOCKED'
