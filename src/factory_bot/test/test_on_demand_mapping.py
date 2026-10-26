"""Measured-map math, bounded lifecycle and adapters, without starting ROS or simulation."""
from concurrent.futures import Future
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
import yaml

from factory_bot.mapping_data import MappingData, compose, inverse
from factory_bot.mapping_session import MappingSession
from factory_bot.map_catalog import inspect_map

PKG=Path(__file__).resolve().parents[1]
SCAN=([2.]*72,-math.pi,2*math.pi/72,.12,12.)


def test_se2_rebases_wheel_odom_without_resetting_existing_robot():
    data=MappingData('ogm',(3.,4.,math.pi/2))
    assert data.local_odom((10.,8.,math.pi/2))==pytest.approx((0,0,0))
    assert compose(data.anchor,data.local_odom((10.,9.,math.pi/2)))==pytest.approx((3.,5.,math.pi/2))
    assert compose((2.,3.,.7),inverse((2.,3.,.7)))==pytest.approx((0,0,0))


@pytest.mark.parametrize('mode',['slam','ogm','slam_ogm','compare'])
def test_measured_maps_export_as_real_previewable_files_with_unknown_space(mode,tmp_path):
    data=MappingData(mode,(0.,0.,0.))
    for i in range(12):
        stamp=(i+1)*1_000_000_000
        data.add_scan(stamp,SCAN,(0,0,0),(0,0,0)); data.add_slam_pose(stamp,(0,0,0))
    grid=np.full((60,60),-1,dtype=np.int8);grid[10:40,10:40]=0;grid[15,15]=100
    data.add_slam_grid(grid,.05,(-1.,-1.,0.))
    report=data.save(tmp_path,'test')
    assert report['success'] and report['scans']==12
    names={v['kind'] for v in report['maps']}
    assert report['primary_kind'] in names
    if mode=='slam_ogm':assert {'slam','ogm_odom','ogm_slam','ogm_truth'}==names
    for output in report['maps']:
        asset=inspect_map(output['path']); assert asset.pixels.shape==(292,332)
        assert set(np.unique(asset.pixels))=={0,205,254}
        assert asset.meta['origin']==[-1.3,-1.3,0.]
        assert output['reference_only']==(output['kind']=='ogm_truth')
    assert json.loads((tmp_path/'session.json').read_text())['hybrid_note'].startswith('OGM uses online')


def test_slam_export_rotates_and_translates_grid_preserving_unknown():
    data=MappingData('slam',(2.,1.,math.pi/2),resolution=1.,origin=(0.,0.),width=5.,height=5.)
    data.add_slam_grid(np.array([[100,0],[-1,100]],dtype=np.int8),1.,(0.,0.,0.))
    assert data.slam[1,1]==100 and data.slam[2,1]==0
    assert data.slam[1,0]==-1 and data.slam[2,0]==100
    assert data.slam[0,4]==-1


def test_hybrid_pairs_only_recent_scans_and_never_reuses_one_scan():
    data=MappingData('slam_ogm',(0.,0.,0.))
    data.add_scan(1_000_000_000,SCAN,(0,0,0))
    assert not data.add_slam_pose(500_000_000,(0,0,0))
    assert data.add_slam_pose(1_000_000_000,(0,0,0))
    assert not data.add_slam_pose(1_010_000_000,(0,0,0))
    assert data.pose_pairs==1


def test_no_measurements_is_failure_and_comparison_does_not_treat_unknown_as_free(tmp_path):
    data=MappingData('ogm',(0.,0.,0.)); assert not data.save(tmp_path,'empty')['success']
    row=MappingData.compare({'a':np.array([[0,100,-1]]),'b':np.array([[0,0,100]])})[0]
    assert row['common_cells']==2 and row['agreement_percent']==50. and row['occupied_iou']==0.


def future(value=None):
    f=Future();f.set_result(value or NS(success=True,message='{}'));return f


class IO:
    waypoints=['1 2','3 4']
    def __init__(self):
        self.calls=[];self.arrival=None;self.engine={'ready':True};self.is_parked=True;self.is_restored=True
        self.pause_future=future();self.hold_ack=True;self.is_alive=True;self.stop_ack=True
    def call(self,*args):self.calls.append(args)
    def check_start(self):self.call('check')
    def new_session(self,mode):return 'session','/tmp/session'
    def world_snapshot(self):return {'enabled':True,'places':{'pallet_1':'parked'}}
    def pause(self,value):self.call('pause',value);return self.pause_future if value else future()
    def role(self,robot):self.call('role',robot)
    def park_world(self):self.call('park')
    def parked(self):return self.is_parked
    def start_engine(self,*args):self.call('start',*args)
    def engine_status(self,identifier):return self.engine
    def engine_alive(self):return self.is_alive
    def go(self,goal):self.call('go',goal);self.arrival=None
    def goal_result(self):return self.arrival
    def hold(self,value):self.call('hold',value)
    def held(self):return self.hold_ack
    def cancel_goal(self):self.call('cancel');return future()
    def finish_engine(self):self.call('finish');return future()
    def stop_engine(self):self.call('stop')
    def engine_stopped(self):return self.stop_ack
    def restore_world(self,snapshot):self.call('restore',snapshot)
    def restored(self,snapshot):return self.is_restored
    def released(self):return True


def survey():
    io=IO();session=MappingSession(io);session.request('slam_ogm',0.)
    for _ in range(3):session.tick(1.)
    assert session.phase=='SURVEYING';return io,session


def cleanup(io,s,save=True):
    for _ in range(3):s.tick(2.)
    if save:
        assert s.phase=='SAVING'
        io.engine={'phase':'SAVED','maps':[{'path':'/tmp/new.yaml'}],'comparison':[{'a':'slam','b':'ogm'}]}
        s.tick(2.)
    s.tick(2.);assert s.phase=='RETURNING_HOME'
    io.arrival='ARRIVED';s.tick(3.);assert s.phase=='RESTORING'
    s.tick(3.);s.tick(3.);s.tick(3.);assert s.phase=='DONE'


def test_full_survey_saves_then_returns_home_restores_scene_and_only_then_resumes_queue():
    io,s=survey();io.arrival='ARRIVED';s.tick(1.);assert s.index==1
    io.arrival='ARRIVED';s.tick(1.);assert s.phase=='STOP_ROBOT'
    cleanup(io,s)
    assert s.maps==[{'path':'/tmp/new.yaml'}] and s.comparison
    assert io.calls.index(('go','HOME_1'))<next(i for i,v in enumerate(io.calls) if v[0]=='restore')<io.calls.index(('pause',False))


def test_finish_partial_survey_and_cancel_discard_both_restore_scene():
    io,s=survey();s.finish();s.tick(1.);cleanup(io,s);assert s.index==0
    io,s=survey();s.cancel(1.);cleanup(io,s,False)
    assert s.cancelled and ('finish',) not in io.calls and not s.maps


def test_parking_requires_ack_before_starting_mapper_and_pause_denial_changes_nothing():
    io=IO();io.is_parked=False;s=MappingSession(io);s.request('ogm',0.);s.tick(1.);s.tick(1.)
    assert s.phase=='PARKING' and not any(v[0]=='start' for v in io.calls)
    io=IO();io.pause_future=future(NS(success=False,message='busy'));s=MappingSession(io)
    s.request('ogm',0.);s.tick(1.);assert s.phase=='DONE' and s.error=='busy' and ('park',) not in io.calls


def test_failure_cancels_only_survey_and_failed_cleanup_keeps_dispatch_held():
    io,s=survey();io.arrival='FAILED';s.tick(1.);assert s.phase=='STOP_ROBOT' and s.error
    cleanup(io,s,False)
    io,s=survey();s.cancel(1.);io.hold_ack=False;s.tick(100.)
    assert s.phase=='ERROR_HELD' and ('pause',False) not in io.calls
    s.cancel(101.);assert s.phase=='STOP_ROBOT'


def test_uncertain_resume_never_retries_by_cancelling_newly_assigned_mission():
    io=IO();s=MappingSession(io);s.phase='RESUMING';s.future=Future();s.deadline=1.
    s.tick(2.);assert s.phase=='ERROR_RESUME';before=list(io.calls)
    s.cancel(3.);s.tick(4.);assert io.calls==before


def test_existing_survey_route_is_reachable_on_current_static_grid():
    from factory_bot.planners import plan
    from factory_bot.planning_grid import PlanningGrid
    grid=PlanningGrid.from_files(str(PKG/'maps/factory_map.yaml'),str(PKG/'config/planning.yaml'),str(PKG/'config/zones.yaml'))
    points=yaml.safe_load((PKG/'config/mapping_route.yaml').read_text())['waypoints_gazebo']
    previous=(0.,0.)
    for x,y in points+[[1.,1.]]:
        target=(x-1.,y-1.)
        result=plan(grid,previous,target,'astar',allow_restricted=grid.restricted_access(previous,target))
        assert result.success,(target,result.reason)
        previous=target


def test_mapping_launch_is_isolated_and_uses_no_simulator_or_global_tf():
    import importlib.util
    spec=importlib.util.spec_from_file_location('generation',PKG/'launch/mapping_generation.launch.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    from launch import LaunchContext
    context=LaunchContext();context.launch_configurations.update(mode='slam_ogm',session_id='test',session_dir='/tmp/session',anchor='[0.,0.,0.]'.replace('0.','0.0'))
    actions=module.launch_setup(context)
    assert len(actions)==4
    source=(PKG/'launch/mapping_generation.launch.py').read_text()
    assert "('/tf','/mapping/tf')" in source and "('/tf_static','/mapping/tf_static')" in source
    assert 'gz_sim' not in source and 'robot_state_publisher' not in source
    assert "('/map','/mapping/map')" in source and "('/map_metadata','/mapping/map_metadata')" in source
    config=yaml.safe_load((PKG/'config/slam_on_demand.yaml').read_text())
    assert config['/mapping/slam_toolbox']['ros__parameters']['map_name']=='/mapping/map'


def test_primary_scan_is_timestamp_matched_and_reframed_without_mutating_source():
    from collections import deque
    from factory_bot.mapping_engine import MappingEngine
    from sensor_msgs.msg import LaserScan
    scans=[];data=MappingData('slam',(0,0,0))
    h=NS(seen=0,finishing=None,phase='STARTING',every=1,odom=deque([(1_000_000_000,(0,0,0))]),truth=deque(),
        skipped=0,data=data,scan_pub=NS(publish=lambda m:scans.append(m)))
    msg=LaserScan();msg.header.stamp.sec=1;msg.header.frame_id='robot1/rear_laser_link';msg.ranges=[2.]*72
    msg.angle_increment=.1;msg.range_min=.1;msg.range_max=12.
    MappingEngine.on_scan(h,msg);assert h.skipped==1 and not scans
    msg.header.frame_id='robot1/laser_link';MappingEngine.on_scan(h,msg)
    assert scans[0].header.frame_id=='mapping_laser' and msg.header.frame_id=='robot1/laser_link' and data.scans==1
    msg.header.stamp.sec=5;MappingEngine.on_scan(h,msg);assert h.skipped==2 and data.scans==1


def test_mover_settled_waits_for_every_inflight_pose_even_if_newest_reply_arrives_first():
    from factory_bot.obstacle_mover import ObstacleMover
    futures=[];messages=[]
    def call(req):
        f=Future();futures.append(f);return f
    h=NS(placements={},inflight={},pose_errors={},places={'pallet_1':'parked'},active=False,
        client=NS(service_is_ready=lambda:True,call_async=call),state_pub=NS(publish=lambda m:messages.append(json.loads(m.data))))
    h.publish_state=lambda:ObstacleMover.publish_state(h)
    ObstacleMover.set_pose(h,'forklift_1',2.,3.,0.);ObstacleMover.set_pose(h,'forklift_1',10.,10.,0.)
    futures[1].set_result(NS(success=True));assert not messages[-1]['settled'] and messages[-1]['pending']==['forklift_1']
    futures[0].set_result(NS(success=True));assert messages[-1]['settled']


def test_mapping_io_requires_matching_goal_ack_and_fresh_mapping_identifier():
    from factory_bot.mapping_controls import MappingControls
    values={'/robot1/nav_status':{'goal':{'seq':7,'name':'STORES'},'state':'ARRIVED'},
            '/mapping/session_status':{'id':'previous','ready':True}}
    io=NS(owner=NS(value=lambda topic:values.get(topic,{})),goal_seq=7,goal_name='HOME_1')
    assert MappingControls.goal_result(io) is None
    values['/robot1/nav_status'].update(goal={'seq':8,'name':'HOME_1'})
    assert MappingControls.goal_result(io)=='ARRIVED'
    assert not MappingControls.engine_status(io,'new')


def test_survey_progress_advances_within_first_leg_and_handles_replan_without_fake_motion():
    from factory_bot.mapping_controls import MappingControls
    from nav_msgs.msg import Path as RosPath
    from geometry_msgs.msg import PoseStamped
    status={'goal':{'seq':2,'name':'2 0'},'pose':[1.,0.,0.],'state':'DRIVING'}
    io=NS(owner=NS(value=lambda topic:status),goal_seq=1,goal_name='2 0',goal_stamp=100_000_000_000,
        route=[(0,0),(1,0),(2,0)],route_length=2.,route_base=0.,fraction=0.)
    assert MappingControls.goal_fraction(io)==pytest.approx(.5)
    path=RosPath();path.header.stamp.sec=99
    MappingControls.on_plan(io,path);assert io.route_length==2.
    path.header.stamp.sec=101
    for x,y in [(1.,0.),(1.,1.),(2.,1.)]:
        p=PoseStamped();p.pose.position.x=x;p.pose.position.y=y;path.poses.append(p)
    MappingControls.on_plan(io,path);assert MappingControls.goal_fraction(io)==pytest.approx(.5)
    status['pose']=[1.,1.,0.];assert MappingControls.goal_fraction(io)==pytest.approx(.75)
    status['state']='ARRIVED';assert MappingControls.goal_fraction(io)==1.


def test_progress_accepts_small_sim_clock_skew_but_rejects_older_out_of_order_routes():
    from factory_bot.mapping_controls import MappingControls
    from nav_msgs.msg import Path as RosPath
    io=NS(goal_name='2 0',goal_stamp=100_000_000_000,fraction=0.,route=[],route_length=0.)
    p=RosPath();p.header.stamp.sec=99;p.header.stamp.nanosec=950_000_000
    MappingControls.on_plan(io,p);assert io.latest_route_stamp==99_950_000_000
    p.header.stamp.nanosec=900_000_000
    MappingControls.on_plan(io,p);assert io.latest_route_stamp==99_950_000_000


def test_hybrid_preview_is_visible_before_second_slam_pose_without_changing_primary_export(tmp_path):
    data=MappingData('slam_ogm',(0.,0.,0.));data.add_scan(1_000_000_000,SCAN,(0,0,0))
    assert data.preview()[1]=='ogm_odom' and data.preview()[0] is not None
    data.add_slam_grid(np.array([[0,100],[-1,0]],dtype=np.int8),.05,(0,0,0))
    assert data.preview()[1]=='slam'
    assert data.save(tmp_path,'partial')['primary_kind']=='ogm_slam'


def test_visualization_relay_keeps_factory_frame_and_rviz_reads_the_measured_topics():
    from factory_bot.map_coordinator import MapCoordinator
    from nav_msgs.msg import OccupancyGrid,Path as RosPath
    grids=[];paths=[];h=NS(mapping_preview_pub=NS(publish=grids.append),mapping_path_pub=NS(publish=paths.append))
    msg=OccupancyGrid();msg.header.frame_id='mapping_map';msg.info.width=2;msg.info.height=2;msg.data=[0,0,100,-1]
    MapCoordinator.on_mapping_grid(h,msg);assert not grids
    msg.header.frame_id='map';MapCoordinator.on_mapping_grid(h,msg);assert grids==[msg]
    path=RosPath();path.header.frame_id='map';MapCoordinator.on_mapping_path(h,path);assert paths==[path]
    cfg=yaml.safe_load((PKG/'rviz/mapping.rviz').read_text())['Visualization Manager']
    assert cfg['Global Options']['Fixed Frame']=='map'
    topics={d.get('Topic',{}).get('Value') for d in cfg['Displays']}
    assert {'/mapping/preview','/mapping/preview_path','/robot1/scan','/robot1/plan'}<=topics
    assert '/map' not in topics
