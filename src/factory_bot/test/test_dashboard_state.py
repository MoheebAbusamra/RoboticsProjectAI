"""Image stride, coordinate frames, task validation and bounded UI data."""
import math

import numpy as np
import pytest

from factory_bot.dashboard_state import (DashboardState, age, delivery_request, image_rgb,
                                         project_map, real_time_factor, route_remaining)


def test_camera_padding_and_bgr_order():
    data=bytes([30,20,10,60,50,40,99,99,90,80,70,120,110,100,99,99])
    result=image_rgb(2,2,8,'bgr8',data)
    np.testing.assert_array_equal(result,[[[10,20,30],[40,50,60]],[[70,80,90],[100,110,120]]])


def test_camera_alpha_mono_and_truncation():
    np.testing.assert_array_equal(image_rgb(1,1,4,'rgba8',bytes([1,2,3,80])),[[[1,2,3]]])
    np.testing.assert_array_equal(image_rgb(1,1,1,'mono8',bytes([70])),[[[70,70,70]]])
    with pytest.raises(ValueError):image_rgb(2,2,6,'rgb8',bytes(4))
    with pytest.raises(ValueError):image_rgb(1,1,1,'yuv422',bytes(4))


def test_map_projection_uses_the_world_offset_once():
    p={'origin':[.1,.8],'x':[.15,.8],'y':[.1,.75],'world_offset':[1,1]}
    np.testing.assert_allclose(project_map(-1,-1,p),[.1,.8])
    np.testing.assert_allclose(project_map(2,3,p),[.25,.60])


def test_task_requests_reject_invalid_stations_and_equal_endpoints():
    s={'STORES':{},'QC_LAB':{}}
    assert delivery_request('STORES','QC_LAB','urgent',s)['priority']=='urgent'
    for a,b,priority in [('STORES','STORES','routine'),('X','STORES','routine'),('STORES','QC_LAB','emergency')]:
        with pytest.raises(ValueError):delivery_request(a,b,priority,s)


def test_stale_data_remains_last_known_and_is_not_marked_fresh():
    state=DashboardState();state.update('/fleet/state',{'robots':{}},now=10)
    s=state.snapshot()
    assert age(s,'/fleet/state',now=14)==4
    assert math.isinf(age(s,'/traffic/state',now=14))
    assert s['values']['/fleet/state']=={'robots':{}}


def test_events_and_rate_samples_are_bounded():
    state=DashboardState()
    for i in range(1000):
        state.event('system',str(i),now=i)
        state.update('/odom',{'v':0},now=i)
    assert len(state.snapshot()['events'])==120
    assert len(state.snapshot()['rates']['/odom'])==240


def test_sim_reset_and_real_time_factor():
    state=DashboardState();state.clock(10,now=20);state.clock(12,now=21)
    assert real_time_factor(state.snapshot())==2
    state.clock(0,now=22)
    assert real_time_factor(state.snapshot()) is None
    assert state.snapshot()['events'][0]['message']=='Simulation clock reset'


def test_route_remaining_includes_distance_from_the_robot_to_route():
    assert route_remaining([(0,0),(1,0),(2,0)],(.8,0,0))==pytest.approx(1.2)
    assert route_remaining([],None) is None
