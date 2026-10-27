"""Full-footprint route exclusion and acknowledged placement, without Gazebo."""
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace as NS
import pytest
from factory_bot.pallet_placement import PlacementPolicy,rectangle,polygon_distance
from factory_bot.pallet_control import PalletControl
PKG=Path(__file__).resolve().parents[1]


@pytest.fixture
def policy():return PlacementPolicy.from_share(PKG)


def test_actual_collision_sizes_world_conversion_and_snap(policy):
    assert policy.bounds==pytest.approx((-.525,.525,-.5,.5))
    assert policy.routes['forklift_1']==[(1.4,6.),(13.,6.)]
    assert policy.routes['worker_1']==[(8.5,.5),(8.5,8.8)]
    assert policy.routes['worker_2'][0]==pytest.approx((.2,11.5))
    assert policy.routes['worker_2'][1]==pytest.approx((4.2,11.5))
    assert policy.snap(3.13,4.12)==(3.25,4.)


@pytest.mark.parametrize('xy,name',[((6.,6.),'forklift_1'),((8.5,2.),'worker_1'),((3.5,11.5),'worker_2')])
def test_full_routes_protected_without_current_actor_positions(policy,xy,name):
    allowed,reason,point=policy.validate(*xy)
    assert not allowed and name in reason


def test_outer_footprint_cannot_clip_route_from_off_route_centre(policy):
    allowed,reason,_=policy.validate(5.,5.25)
    assert not allowed and 'forklift_1' in reason
    assert policy.validate(3.5,4.)[0]


@pytest.mark.parametrize('xy',[(.0,-.75),(1.5,2.5),(16.,4.),(-1.,4.)])
def test_static_collisions_and_factory_boundary(policy,xy):
    assert not policy.validate(*xy)[0]


def test_robot_guard_and_bad_coordinates(policy):
    assert policy.validate(3.5,4.)[0]
    allowed,reason,_=policy.validate(3.5,4.,{'robot2':(4.,4.)})
    assert not allowed and 'robot2' in reason
    for x in (None,float('nan'),float('inf'),True,'no'):
        with pytest.raises(ValueError):policy.snap(x,3.)
    assert polygon_distance(rectangle(0,0,(-1,1,-1,1)),rectangle(2,0,(-1,1,-1,1)))==0.


def controller(policy):
    futures=[];sent=[];clock=[0.]
    def send(x,y,z):
        f=Future();futures.append(f);sent.append((x,y,z));return f
    c=PalletControl(policy,{'parked':[19.,8.],'z':0.,'drop_points':{'main_aisle':[8.,7.]}},send,lambda:None,lambda:clock[0])
    return c,futures,sent,clock


def test_place_move_remove_confirm_after_success_and_ids_are_idempotent(policy):
    c,futures,sent,clock=controller(policy)
    c.request({'id':'one','action':'place','x':3.5,'y':4.},robots={})
    assert sent==[(4.5,5.,0.)] and c.status()['pending'] and c.place=='parked'
    futures[-1].set_result(NS(success=True));assert c.place=='grid' and c.pose==[3.5,4.]
    c.request({'id':'one','action':'place','x':3.5,'y':4.},robots={});assert len(sent)==1
    c.request({'id':'two','action':'place','x':4.,'y':4.},robots={})
    futures[-1].set_result(NS(success=True));assert c.pose==[4.,4.]
    c.request({'id':'three','action':'remove'})
    assert sent[-1]==(19.,8.,0.) and c.place=='grid'
    futures[-1].set_result(NS(success=True));assert c.place=='parked' and c.pose==[18.,7.]


def test_pending_failure_timeout_and_mapping_lock_never_publish_unsafe_target(policy):
    c,futures,sent,clock=controller(policy)
    c.request({'id':'route','action':'place','x':6.,'y':6.},robots={});assert not sent
    c.request({'id':'map','action':'place','x':3.5,'y':4.},robots={},blocked='mapping active');assert not sent
    c.request({'id':'missing','action':'place','x':3.5,'y':4.},robots=None);assert not sent
    c.request({'id':'one','action':'place','x':3.5,'y':4.},robots={})
    c.request({'id':'two','action':'remove'});assert len(sent)==1 and c.status()['pending']
    clock[0]=9.;assert next(v for v in c.status()['requests'] if v['id']=='one')['phase']=='UNCERTAIN'
    assert c.status()['pending']
    futures[-1].set_result(NS(success=False));assert not c.status()['pending'] and c.place=='parked'
    assert next(v for v in c.status()['requests'] if v['id']=='one')['phase']=='FAILED'


def test_legacy_grid_restore_and_unsafe_named_drop(policy):
    c,futures,sent,clock=controller(policy)
    c.legacy('main_aisle',robots={});assert not sent
    c.legacy('grid',robots={},x=3.5,y=4.)
    futures[-1].set_result(NS(success=True));assert c.place=='grid' and c.pose==[3.5,4.]
    c.legacy('parked');futures[-1].set_result(NS(success=True));assert c.place=='parked'


def test_mapping_restores_and_verifies_selected_grid_location():
    from factory_bot.mapping_controls import MappingControls
    import json
    events=[];state={'enabled':True,'places':{'pallet_1':'grid'},'pallet':{'pose':[3.5,4.]},'pending':[]}
    snapshot=dict(state)
    owner=NS(value=lambda topic:state if topic=='/fleet/obstacles_state' else {},robots=['robot1','robot2'],
        machine=NS(active=NS(path='map')),ready=lambda *a:(True,''))
    io=NS(owner=owner,events=NS(publish=lambda m:events.append(json.loads(m.data))))
    MappingControls.restore_world(io,snapshot)
    assert events[0]=={'model':'pallet_1','place':'grid','x':3.5,'y':4.}
    assert MappingControls.restored(io,snapshot)
    state['pallet']={'pose':[4.,4.]};assert not MappingControls.restored(io,snapshot)


def test_mapping_waits_for_pallet_lock_and_captures_the_completed_placement():
    from factory_bot.mapping_session import MappingSession
    from test_on_demand_mapping import IO
    io=IO();ready=[False];world={'places':{'pallet_1':'parked'}}
    io.world_ready=lambda:ready[0];io.world_snapshot=lambda:dict(world)
    session=MappingSession(io);session.request('ogm',0.);session.tick(1.)
    assert session.phase=='PAUSING' and ('park',) not in io.calls
    world.update(places={'pallet_1':'grid'},pallet={'pose':[3.5,4.]});ready[0]=True;session.tick(2.)
    assert session.phase=='PARKING' and session.world['pallet']['pose']==[3.5,4.]


@pytest.mark.parametrize('name',['HOME_1','HOME_2','STORES','TOOL_CRIB','QC_LAB','ASSEMBLY_A','ASSEMBLY_B'])
def test_stations_are_protected_even_when_no_robot_is_docked(policy,name):
    import yaml
    station=yaml.safe_load((PKG/'config/stations.yaml').read_text())['stations'][name]
    allowed,reason,_=policy.validate(station['x'],station['y'],robots={})
    assert not allowed and name in reason
    c,futures,sent,clock=controller(policy)
    c.request({'id':name,'action':'place','x':station['x'],'y':station['y']},robots={})
    assert not sent and c.status()['requests'][-1]['phase']=='REJECTED'


def test_pallet_edge_and_clearance_cannot_clip_a_station_while_its_centre_is_outside(policy):
    # HOME_2 is at x=1.6. Centre x=2.5 is off the station, but the pallet edge
    # reaches x=1.975: only 0.075 m from its 0.6 m docking envelope.
    allowed,reason,_=policy.validate(2.5,0.,robots={})
    assert not allowed and 'HOME_2' in reason
    assert policy.validate(2.75,0.,robots={})[0]
