"""Offline reverse recovery, swept-space, traffic and real navigator integration tests."""
import math
from pathlib import Path
from types import SimpleNamespace as NS
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from factory_bot.backup_recovery import (BackupRecovery,reverse_path,rear_map_clear,
                                         rear_scan_clear,peer_blocks_reverse)
from factory_bot.live_layer import LiveLayer
from factory_bot.planning_grid import PlanningGrid
from factory_bot.traffic import RobotView,TrafficRules

PKG=Path(__file__).resolve().parents[1]
P=(4.,4.,0.);GOAL=(8.,4.)


def grid(zones=None):
    occ=np.zeros((160,200),dtype=bool);occ[[0,-1],:]=True;occ[:,[0,-1]]=True
    return PlanningGrid(occ,np.zeros_like(occ),.05,(0,0),zones=zones)


def request():
    recovery=BackupRecovery()
    assert [recovery.note_replan(P,P,t,0.) for t in [3.,9.,15.]]==['retry','retry','request']
    recovery.request(P,P,GOAL,15.,'request-1')
    return recovery


def test_three_attempts_trigger_and_rotation_or_jitter_is_not_progress():
    b=BackupRecovery();b.observe_forward(P,P,GOAL,.2)
    for i in range(2):assert b.note_replan(P,P,i)=='retry'
    b.observe_forward((4.02,4.,1.),(4.02,4.,1.),GOAL,0.)
    assert b.note_replan(P,P,3)=='request'


def test_both_map_pose_and_wheel_odometry_must_show_real_forward_progress():
    b=BackupRecovery();b.observe_forward(P,P,GOAL,.2);b.note_replan(P,P,0)
    b.observe_forward((4.1,4.,0.),P,GOAL,.2);assert b.attempts==1
    b.observe_forward((4.1,4.,0.),(4.1,4.,0.),GOAL,.2)
    assert b.attempts==0 and b.deadline is None


def test_waiting_never_reverses_without_clearance_and_matching_permission():
    b=request()
    assert b.step(P,P,15.1,True,False)=='wait'
    assert b.step(P,P,15.2,False,True,'rear blocked')=='wait'
    assert b.step(P,P,15.3,True,True)=='reverse'


def test_reverse_completes_by_measured_distance_and_cannot_rearm_by_returning_to_blockage():
    b=request();assert b.step(P,P,16.,True,True)=='reverse'
    end=(3.70,4.,0.)
    assert b.step(end,end,22.,True,True)=='complete'
    assert b.used
    b.observe_forward(P,P,GOAL,.2);assert b.used  # no net progress beyond the original blockage
    assert [b.note_replan(P,P,t) for t in [23,26,29]]==['retry','retry','exhausted']
    good=(4.21,4.,0.);b.observe_forward(good,good,GOAL,.2)
    assert not b.used and b.attempts==0


def test_backup_toward_a_goal_does_not_itself_reset_the_allowance():
    b=BackupRecovery();b.note_replan(P,P,0);b.request(P,P,(2.,4.),0,'toward')
    b.step(P,P,.1,True,True);end=(3.7,4.,0.);b.step(end,end,6.1,True,True)
    b.observe_forward((3.69,4.,math.pi),(3.69,4.,math.pi),(2.,4.),.2)
    assert b.used


@pytest.mark.parametrize('failure',['rear','timeout','stall','slip','lateral','heading','clock','nan','overrun'])
def test_backup_stops_for_each_failure(failure):
    b=request();b.step(P,P,16.,True,True)
    pose=odom=P;now=16.1;clear=True
    if failure=='rear':clear=False
    elif failure=='timeout':now=24.1
    elif failure=='stall':now=19.1
    elif failure=='slip':odom=(3.85,4.,0.);now=19.1
    elif failure=='lateral':pose=(3.95,4.06,0.)
    elif failure=='heading':pose=(3.95,4.,.2)
    elif failure=='clock':now=0.
    elif failure=='nan':pose=(math.nan,4.,0.)
    elif failure=='overrun':pose=odom=(3.6,4.,0.)
    assert b.step(pose,odom,now,clear,True,'rear blocked')=='failed'


def test_lost_grant_stops_and_pending_wait_and_total_episode_are_bounded():
    b=request();b.step(P,P,16.,True,True)
    assert b.step(P,P,16.2,True,False)=='wait'
    assert request().step(P,P,25.1,True,False)=='failed'
    assert b.expired(38.1)
    # Only intentional forward-mode traffic holds pause the ordinary episode deadline.
    b=BackupRecovery();b.note_replan(P,P,0,0);b.pause(2);b.resume(12)
    assert not b.expired(39) and b.expired(41)


def scan(values):
    return rear_scan_clear(values,-math.pi/2,math.pi/180,.02,4.,.3)


def test_rear_scan_checks_side_of_robot_corridor_and_accepts_valid_no_returns():
    values=[math.inf]*181;assert scan(values)[0]
    values[90]=.25;assert not scan(values)[0]
    values=[math.inf]*181;values[135]=.25  # rear side corner, outside narrow centre cone
    assert not scan(values)[0]
    values=[math.inf]*181;values[90]=1.;assert scan(values)[0]


def test_rear_returns_cover_worker_cylinder_beyond_visible_feet_without_double_inflating_walls():
    # Visible feet lie outside the robot's swept width, but the worker's wider
    # cylinder overlaps it. Unmapped return padding must catch that contact.
    values=[math.inf]*181;values[165]=.45
    assert not scan(values)[0]
    # A mapped wall 40 cm to the side is actually clear of the robot footprint.
    values=[math.inf]*181;values[0]=.4;g=grid()
    row,col=g.world_to_cell(3.795,4.4);g.occupied[row,col]=True
    assert rear_scan_clear(values,-math.pi/2,math.pi/180,.02,4,.3,grid=g,pose=P)[0]


@pytest.mark.parametrize('bad',[0.,math.nan,-math.inf,.01])
def test_unknown_or_too_close_rear_measurements_are_not_clear(bad):
    values=[math.inf]*181;values[100]=bad;assert not scan(values)[0]
    assert not rear_scan_clear([math.inf]*91,-math.pi/4,math.pi/180,.02,4,.3)[0]


def test_static_rear_check_allows_retreat_from_a_front_contact_but_blocks_rear_and_unknown():
    g=grid();r,c=g.world_to_cell(4.20,4.);g.occupied[r,c]=True
    assert rear_map_clear(g,P,.3)[0]
    r,c=g.world_to_cell(3.6,4.19);g.occupied[r,c]=True
    assert not rear_map_clear(g,P,.3)[0]
    g.occupied[r,c]=False;g.unknown[r,c]=True
    assert not rear_map_clear(g,P,.3)[0]
    assert not rear_map_clear(g,(.4,4.,0.),.3)[0]


def test_reverse_corridor_respects_rotated_geometry_and_policy_access():
    zone=dict(rule='forbidden',x_min=3.4,x_max=3.65,y_min=3.7,y_max=4.3)
    assert not rear_map_clear(grid([zone]),P,.3)[0]
    zone['rule']='authorized_only';g=grid([zone])
    assert not rear_map_clear(g,P,.3,False)[0]
    assert rear_map_clear(g,P,.3,True)[0]
    g=grid();r,c=g.world_to_cell(4.,3.55);g.occupied[r,c]=True
    assert not rear_map_clear(g,(4.,4.,math.pi/2),.3)[0]


def test_rear_sensor_points_enter_live_layer_behind_the_correct_robot_offset():
    g=grid();layer=LiveLayer(g)
    layer.update(P,[.5],math.pi,.01,.02,4,0,laser_x=-.205)
    assert layer.mask(0)[g.world_to_cell(3.295,4.)]
    assert not layer.mask(0)[g.world_to_cell(4.295,4.)]


@pytest.mark.parametrize('stale', ['scan', 'pose', 'none'])
def test_rear_callback_does_not_refresh_old_measurements_in_live_layer(stale):
    from factory_bot.navigator import Navigator
    calls=[]
    msg=NS(header=NS(stamp=NS(sec=9 if stale=='scan' else 10,nanosec=0),
                     frame_id='robot1/rear_obstacle_laser_link'),
           ranges=[.5],angle_min=0.,angle_increment=.01,range_min=.02,range_max=4.)
    nav=NS(now=lambda:10.,robot='robot1',pose=lambda:P,pose_at=9. if stale=='pose' else 10.,
           get=lambda name:.6,live=NS(update=lambda *a,**kw:calls.append((a,kw))))
    Navigator.on_rear_scan(nav,msg)
    assert bool(calls)==(stale=='none')


def recovering(name='robot1',pose=P,identifier='a',phase='waiting',**kw):
    data={'id':identifier,'phase':phase,'path':reverse_path(pose,.3),'distance_m':.3,'travelled_m':0.}
    return RobotView(name,pose,True,data['path'],'TO_DROPOFF','routine',recovery=data,stopped=True,**kw)


def acknowledge(views,cmd):
    for name,v in views.items():
        if 'backup_hold' in cmd[name]:v.traffic_hold=cmd[name]['backup_hold'];v.stopped=True


def test_traffic_requires_stopped_acknowledgement_and_uses_unique_request_id():
    rules=TrafficRules(grid());owner=recovering()
    peer=RobotView('robot2',(4.5,4.,math.pi),True,[(4.5,4),(3.,4)],stopped=False)
    views={'robot1':owner,'robot2':peer};cmd=rules.update(views,0)
    assert not cmd['robot1']['backup_granted'] and cmd['robot2']['cmd']=='wait'
    acknowledge(views,cmd);cmd=rules.update(views,.25)
    assert cmd['robot1']['backup_granted'] and cmd['robot1']['backup_id']=='a'
    # Old acknowledgement is invalid for the next recovery request.
    owner.recovery['id']='b';cmd=rules.update(views,.5)
    assert not cmd['robot1']['backup_granted']


def test_peer_in_rear_space_blocks_backup_but_a_front_peer_can_be_escaped():
    assert peer_blocks_reverse(P,.3,(3.5,4.))
    assert not peer_blocks_reverse(P,.3,(4.4,4.))
    rules=TrafficRules(grid());views={'robot1':recovering(),
        'robot2':RobotView('robot2',(3.5,4.,0.),False,[],stopped=True)}
    cmd=rules.update(views,0);acknowledge(views,cmd)
    assert not rules.update(views,.25)['robot1']['backup_granted']


def test_simultaneous_recoveries_are_serialized_and_active_backup_is_not_preempted():
    rules=TrafficRules(grid());views={'robot1':recovering(),
        'robot2':recovering('robot2',(7.,4.,0.),'b')}
    cmd=rules.update(views,0);acknowledge(views,cmd);cmd=rules.update(views,.25)
    assert cmd['robot1']['backup_granted'] and not cmd['robot2'].get('backup_granted')
    views['robot1'].recovery['phase']='backing';views['robot1'].stopped=False
    views['robot2'].job_priority='emergency'
    assert rules.update(views,.5)['robot1']['backup_granted']
    views['robot1'].recovery=None
    cmd=rules.update(views,.75)
    assert rules.backup_owner[0]=='robot2' and cmd['robot2']['backup_granted']


def test_stale_peer_or_owner_never_receives_a_grant_and_far_traffic_can_continue():
    rules=TrafficRules(grid());views={'robot1':recovering(),
        'robot2':RobotView('robot2',(8.,6.,0.),True,[(8.,6.),(9.,6.)],stopped=False)}
    assert rules.update(views,0)['robot1']['backup_granted']
    assert rules.cmd['robot2']['cmd']=='go'
    views['robot2'].fresh=False;assert not rules.update(views,.25)['robot1']['backup_granted']
    views['robot2'].fresh=True;views['robot1'].fresh=False
    assert not rules.update(views,.5)['robot1']['backup_granted']


def test_rear_sensor_placement_bridge_and_mapping_sensor_are_preserved():
    robot=ET.parse(PKG/'urdf/fleet_bot.urdf.xacro').getroot()
    origin=robot.find("joint[@name='rear_obstacle_laser_joint']/origin")
    assert [float(v) for v in origin.get('xyz').split()]==[-.205,0.,.010]
    assert float(origin.get('rpy').split()[2])==pytest.approx(math.pi)
    root=ET.parse(PKG/'urdf/fleet_bot.gazebo.xacro').getroot()
    rear=root.find(".//sensor[@name='rear_obstacle_lidar_${ns}']")
    assert rear.findtext('lidar/scan/horizontal/samples')=='181'
    assert rear.findtext('lidar/visibility_mask')=='2415919103'
    primary=root.find(".//sensor[@name='lidar_${ns}']")
    assert primary.findtext('lidar/scan/horizontal/samples')=='360'
    assert '/rear_obstacle_scan@sensor_msgs/msg/LaserScan' in (PKG/'launch/fleet_sim.launch.py').read_text()


def navigator():
    """Real navigator methods with fake publishers/clock; never constructs a ROS node."""
    from factory_bot.navigator import Navigator
    class Harness:
        replan=Navigator.replan
        completed_replan=Navigator.completed_replan
        run_backup=Navigator.run_backup
        normal_obstacle_mode=Navigator.normal_obstacle_mode
        blocked_expired=Navigator.blocked_expired
        start_goal=Navigator.start_goal
        parse_goal=Navigator.parse_goal
        def __init__(self):
            self.time=3.;self.robot='robot1';self.state='BLOCKED';self.aside=None;self.loc_ok=True
            self.grid=grid();self.live=LiveLayer(self.grid);self.backup=BackupRecovery()
            self.position=P;self.odom_pose=P;self.odom_at=self.time;self.pose_at=self.time
            self.traffic={};self.traffic_at=self.time
            self.rear_scan=NS(ranges=[math.inf]*181,angle_min=-math.pi/2,angle_increment=math.pi/180,
                              range_min=.02,range_max=4.,header=NS(frame_id='robot1/rear_obstacle_laser_link'))
            self.rear_scan_at=self.front_scan_at=self.time
            self.goal={'x':8.,'y':4.,'algorithm':'astar','controller':'pure_pursuit','priority':'routine',
                       'allow':False,'replan_ms':0.,'replans':0,'all_paths':[], 'blocked_since':0.}
            self.original_goal=self.goal;self.commands=[];self.plans=[];self.last_command=(0.,0.)
            self.follower=NS(set_path=lambda path:None);self.ctrl_settings={};self.stations={};self.seq=1
            from factory_bot.motion_progress import MotionProgress
            self.motion_progress=MotionProgress()
            self.truth=(999.,999.,0.)  # metrics-only pose must have no influence
        def get(self,name):
            return {'scan_timeout':.6,'backup_traffic_lease':.75,'normal_obstacle_wait':3.,
                    'retry_period':2.,'blocked_timeout':30.,'algorithm':'astar','controller':'pure_pursuit'}[name]
        def now(self):return self.time
        def pose(self):return self.position
        def publish_status(self):pass
        def publish_plan(self,path):self.plans.append(path)
        def get_logger(self):return NS(info=lambda *a,**k:None,warn=lambda *a,**k:None,error=lambda *a,**k:None)
        def drive(self,v,w):self.commands.append((v,w));self.last_command=(v,w)
        def stop(self):self.drive(0.,0.)
        def finish(self,state,reason=''):self.state=state;self.reason=reason;self.stop()
        def sensors_now(self):self.odom_at=self.pose_at=self.rear_scan_at=self.front_scan_at=self.time
        def authorize(self):
            self.traffic={'cmd':'go','backup_granted':True,'backup_id':self.backup.identifier}
            self.traffic_at=self.time
    return Harness()


def fake_plan(success):
    return NS(success=success,time_ms=1.,length_m=4.,path=[(80,80),(80,160)],expanded=2,reason='blocked')


@pytest.mark.parametrize('success',[True,False])
def test_real_replan_hook_counts_successes_and_failures_and_keeps_the_delivery(monkeypatch,success):
    import factory_bot.navigator as module
    monkeypatch.setattr(module,'plan',lambda *a,**k:fake_plan(success))
    nav=navigator()
    for t in [3.,9.,15.]:nav.time=t;nav.replan('test reroute')
    assert nav.state=='RECOVERING' and nav.backup.used and nav.backup.attempts==3
    assert nav.goal is nav.original_goal
    assert nav.plans[-1][-1]==pytest.approx((3.7,4.))
    assert nav.commands[-1]==(0.,0.)


def begin_navigation_backup():
    nav=navigator();nav.time=15.;nav.sensors_now()
    for t in [3.,9.,15.]:nav.backup.note_replan(P,P,t,0.)
    nav.backup.request(P,P,GOAL,15.,'lease');nav.state='RECOVERING'
    return nav


def test_real_reverse_command_requires_current_matching_traffic_lease():
    nav=begin_navigation_backup();nav.run_backup(P,nav.time)
    assert nav.commands[-1]==(0.,0.)
    nav.authorize();nav.traffic['backup_id']='old-request';nav.run_backup(P,nav.time)
    assert nav.commands[-1]==(0.,0.)
    nav.authorize();nav.run_backup(P,nav.time)
    assert nav.commands[-1]==(-.05,0.)
    nav.time+=.1;nav.sensors_now();nav.traffic_at=nav.time-.8
    nav.run_backup(P,nav.time);assert nav.commands[-1]==(0.,0.)


@pytest.mark.parametrize('source',['rear','front','odom','tf','localization','frame','obstacle'])
def test_real_backup_stops_for_invalid_inputs(monkeypatch,source):
    nav=begin_navigation_backup();nav.authorize();nav.run_backup(P,nav.time)
    nav.time+=.1;nav.sensors_now();nav.authorize()
    if source=='rear':nav.rear_scan_at=0.
    elif source=='front':nav.front_scan_at=0.
    elif source=='odom':nav.odom_at=0.
    elif source=='tf':nav.pose_at=0.
    elif source=='localization':nav.loc_ok=False
    elif source=='frame':nav.rear_scan.header.frame_id='wrong-frame'
    elif source=='obstacle':nav.rear_scan.ranges[90]=.15
    nav.run_backup(P,nav.time)
    assert nav.state=='FAILED' and nav.commands[-1]==(0.,0.)


def test_real_completion_replans_from_retreat_position_with_original_options(monkeypatch):
    import factory_bot.navigator as module
    calls=[]
    def planner(g,start,goal,algorithm,**kw):
        calls.append((start,goal,algorithm));return fake_plan(True)
    monkeypatch.setattr(module,'plan',planner)
    nav=begin_navigation_backup();nav.authorize();nav.run_backup(P,nav.time)
    nav.position=nav.odom_pose=(3.7,4.,0.);nav.time=21.;nav.sensors_now();nav.authorize()
    nav.run_backup(nav.position,nav.time)
    assert nav.commands[-1]==(0.,0.) and nav.state=='DRIVING'
    assert calls[-1]==((3.7,4.),GOAL,'astar')
    assert nav.goal is nav.original_goal and nav.goal['controller']=='pure_pursuit'
    assert nav.backup.used


def test_emergency_goal_cancels_reverse_before_planning_and_preserves_options(monkeypatch):
    import factory_bot.navigator as module
    nav=begin_navigation_backup();nav.authorize();nav.run_backup(P,nav.time)
    def planner(*a,**kw):
        assert nav.last_command==(0.,0.)
        return fake_plan(True)
    monkeypatch.setattr(module,'plan',planner)
    nav.start_goal('{"goal":"8 4", "priority":"emergency", "algorithm":"bfs", "controller":"pid"}')
    assert not nav.backup.active and not nav.backup.used
    assert nav.goal['priority']=='emergency' and nav.goal['algorithm']=='bfs' and nav.goal['controller']=='pid'


def test_real_nav_status_reports_stationary_acknowledgement_only_with_fresh_stopped_odometry():
    import json
    from factory_bot.navigator import Navigator
    nav=begin_navigation_backup();messages=[]
    nav.odom_velocity=(0.,0.);nav.traffic={'cmd':'wait','backup_hold':'robot2:id'}
    nav.status_pub=NS(publish=lambda msg:messages.append(json.loads(msg.data)))
    Navigator.publish_status(nav)
    assert messages[-1]['stopped'] and messages[-1]['traffic_hold']=='robot2:id'
    nav.odom_velocity=(-.05,0.);Navigator.publish_status(nav)
    assert not messages[-1]['stopped']
    nav.odom_velocity=(0.,0.);nav.odom_at=0.;Navigator.publish_status(nav)
    assert not messages[-1]['stopped']


def test_traffic_manager_uses_recovery_route_and_marks_old_status_stale():
    from factory_bot.traffic_manager import TrafficManager
    now=10.
    h=NS(status={'robot1':{'pose':P,'state':'RECOVERING','stopped':True,
        'recovery':{'id':'x','phase':'waiting','path':reverse_path(P,.3)}}},
        received={'robot1':9.9},blocked_since={'robot1':None},fleet={},
        path={'robot1':[(4.,4.),(8.,4.)]},get=lambda name:.6)
    view=TrafficManager.view(h,'robot1',now)
    assert view.fresh and view.active and view.route[-1]==pytest.approx((3.7,4.))
    assert not TrafficManager.view(h,'robot1',11.).fresh
