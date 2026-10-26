"""Dashboard parameter changes: confirmation, partial failures and bounded waits."""
from concurrent.futures import Future
from types import SimpleNamespace as NS

import pytest

from factory_bot.dashboard_state import DashboardState
from factory_bot.dashboard_navigation import (NavigationSettingsClient, SETTINGS_TOPIC, TARGETS,
    navigation_request, settings_consensus)


def done(value):
    f=Future();f.set_result(value);return f


class Client:
    def __init__(self, keys):
        self.values={k:'astar' if k=='algorithm' else 'pure_pursuit' for k in keys}
        self.available=True;self.reject=False;self.hang=False;self.writes=[];self.lie=False

    def services_are_ready(self):return self.available

    def get_parameters(self, keys):
        return done(NS(values=[NS(type=4,string_value=self.values[k]) for k in keys]))

    def set_parameters_atomically(self, values):
        self.writes.append(values)
        if self.hang:return Future()
        if not self.reject and not self.lie:self.values.update(values)
        return done(NS(result=NS(successful=not self.reject,reason='rejected for test' if self.reject else '')))


def setup():
    state=DashboardState();clock=[10.]
    clients={n:Client(keys) for n,keys in TARGETS.items()}
    worker=NavigationSettingsClient(state,clients,lambda name,value:(name,value),lambda:clock[0])
    worker.tick();worker.tick()
    return state,worker,clients,clock


def test_only_supported_existing_parameter_values_are_accepted():
    for a in ['astar','dijkstra','bfs']:
        for c in ['pid','pure_pursuit']:assert navigation_request(a,c)=={'algorithm':a,'controller':c}
    for a,c in [('A*','pid'),('astar','stanley'),(None,'pid')]:
        with pytest.raises(ValueError):navigation_request(a,c)


def test_success_requires_acknowledgement_and_read_back_from_every_node():
    state,w,clients,_=setup()
    w.apply({'algorithm':'bfs','controller':'pid'});w.tick()
    assert state.snapshot()['values'][SETTINGS_TOPIC]['applying']
    w.tick()  # set acknowledgements; read-back is still outstanding
    assert not any(e['kind']=='settings' for e in state.snapshot()['events'])
    w.tick()
    actual=state.snapshot()['values'][SETTINGS_TOPIC]
    assert settings_consensus(actual['nodes'])=={'algorithm':'bfs','controller':'pid'}
    assert not actual['applying'] and not actual['error']
    assert any(e['kind']=='settings' for e in state.snapshot()['events'])
    assert clients['/fleet_manager'].writes==[[('algorithm','bfs')]]
    assert clients['/robot1/navigator'].writes==[[('algorithm','bfs'),('controller','pid')]]


@pytest.mark.parametrize('algorithm',['astar','dijkstra','bfs'])
@pytest.mark.parametrize('controller',['pid','pure_pursuit'])
def test_all_six_supported_profiles_reach_existing_parameters(algorithm,controller):
    state,w,_,_=setup();w.apply({'algorithm':algorithm,'controller':controller})
    for _ in range(3):w.tick()
    actual=state.snapshot()['values'][SETTINGS_TOPIC]
    assert settings_consensus(actual['nodes'])=={'algorithm':algorithm,'controller':controller}
    assert not actual['error']


def test_missing_service_prevents_all_writes():
    _,w,clients,_=setup();clients['/robot2/navigator'].available=False
    with pytest.raises(ValueError):w.apply({'algorithm':'dijkstra','controller':'pid'})
    assert not any(c.writes for c in clients.values())


def test_partial_rejection_is_visible_with_actual_values_and_no_success_event():
    state,w,clients,_=setup();clients['/robot2/navigator'].reject=True
    w.apply({'algorithm':'dijkstra','controller':'pid'})
    for _ in range(3):w.tick()
    actual=state.snapshot()['values'][SETTINGS_TOPIC]
    assert 'rejected' in actual['error']
    assert actual['nodes']['/fleet_manager']['algorithm']=='dijkstra'
    assert actual['nodes']['/robot2/navigator']['algorithm']=='astar'
    assert settings_consensus(actual['nodes'])=={'algorithm':None,'controller':None}
    assert not any(e['kind']=='settings' for e in state.snapshot()['events'])


def test_success_reply_without_changed_values_does_not_claim_applied():
    state,w,clients,_=setup();clients['/robot1/navigator'].lie=True
    w.apply({'algorithm':'bfs','controller':'pid'})
    for _ in range(3):w.tick()
    assert 'Read-back differs' in state.snapshot()['values'][SETTINGS_TOPIC]['error']
    assert not any(e['kind']=='settings' for e in state.snapshot()['events'])


def test_timeout_is_finite_and_never_repeats_the_write():
    state,w,clients,clock=setup();clients['/robot1/navigator'].hang=True
    w.apply({'algorithm':'bfs','controller':'pid'});w.tick();clock[0]+=6;w.tick()
    actual=state.snapshot()['values'][SETTINGS_TOPIC]
    assert not actual['applying'] and 'timed out' in actual['error']
    clock[0]+=2;w.tick();w.tick()
    assert all(len(c.writes)==1 for c in clients.values())
