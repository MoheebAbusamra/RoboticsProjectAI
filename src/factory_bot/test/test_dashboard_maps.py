"""Native map-panel interactions against fake telemetry/transport, never ROS or Gazebo."""
import os
from pathlib import Path

os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import numpy as np
import pytest
import yaml
from PyQt5.QtWidgets import QApplication, QFileDialog
from PyQt5.QtGui import QColor

from factory_bot.dashboard import Dashboard, preview
from factory_bot.dashboard_state import DashboardState
from factory_bot.map_catalog import inspect_map


class Bridge:
    def __init__(self):self.requests=[]
    def request(self,kind,payload):self.requests.append((kind,payload))
    def set_camera(self,robot):pass
    def stop(self):pass
    def join(self,timeout=None):pass


@pytest.fixture
def panel(tmp_path):
    app=QApplication.instance() or QApplication([])
    share=Path(__file__).resolve().parents[1];state=DashboardState();preview(state,share);state.preview=False
    bridge=Bridge();window=Dashboard(share,state,bridge);window.timer.stop()
    generated=str((share/'maps/factory_map.yaml').resolve());active=inspect_map(generated).info()
    status={'phase':'READY','busy':False,'verified':True,'active':active,'generated':generated,
            'can_activate':True,'recent':[],'message':'Ready','error':''}
    state.update('/maps/status',status);window.refresh()
    pixels=np.full((32,32),254,dtype=np.uint8);pixels[5,7]=0;pixels[10,12]=128
    (tmp_path/'candidate.pgm').write_bytes(b'P5\n32 32\n255\n'+pixels.tobytes())
    path=tmp_path/'candidate.yaml';path.write_text(yaml.safe_dump(dict(image='candidate.pgm',resolution=.05,
        origin=[-1.3,-1.3,0],occupied_thresh=.65,free_thresh=.196,negate=0)))
    yield app,window,bridge,status,path
    window.close();app.processEvents()


def test_browse_previews_actual_map_without_applying_and_preserves_active_label(panel,monkeypatch):
    app,w,bridge,status,path=panel;p=w.maps_panel
    assert not p.use_button.isEnabled()  # generated map is already active
    monkeypatch.setattr(QFileDialog,'getOpenFileName',lambda *a,**k:(str(path),'YAML'))
    p.browse()
    assert p.use_button.isEnabled() and not bridge.requests
    assert status['active']['source'] in p.active.text() and str(path) in p.selected.text()
    assert '32 × 32' in p.stats.text()
    assert QColor(p.view.image.pixel(7,5)).getRgb()[:3]==(231,214,181)
    assert QColor(p.view.image.pixel(12,10)).getRgb()[:3]==(87,62,71)
    assert QColor(p.view.image.pixel(0,0)).getRgb()[:3]==(42,26,34)


def test_activation_waits_for_backend_confirmation_and_never_labels_selection_as_active(panel):
    app,w,bridge,status,path=panel;p=w.maps_panel;p.choose(str(path));p.use_button.click()
    kind,request=bridge.requests[-1]
    assert kind=='map_activate' and request['path']==str(path) and request['fingerprint']==p.asset.digest
    assert not p.use_button.isEnabled() and str(path) not in p.active.text()
    progress=dict(status,phase='VERIFYING',busy=True,verified=False,received_request_id=request['id'],
                  message='waiting for accepted ICP matches')
    w.state.update('/maps/status',progress);w.refresh()
    assert not p.use_button.isEnabled() and not w.demo_button.isEnabled()
    confirmed=dict(status,active=p.asset.info(),received_request_id=request['id'])
    w.state.update('/maps/status',confirmed);w.refresh()
    assert str(path) in p.active.text() and p.restore_button.isEnabled() and not p.use_button.isEnabled()


def test_busy_offline_and_preview_disable_activation_and_restore(panel):
    app,w,bridge,status,path=panel;p=w.maps_panel;p.choose(str(path))
    w.state.update('/maps/status',dict(status,can_activate=False,idle_reason='robot1 is busy'));w.refresh()
    assert not p.use_button.isEnabled() and 'busy' in p.note.text()
    w.state.update('/maps/status',status);w.state.preview=True;w.refresh()
    assert not p.use_button.isEnabled() and not p.restore_button.isEnabled()
    w.state.preview=False;w.state.seen['/maps/status']=-100;w.refresh()
    assert not p.use_button.isEnabled() and 'offline' in p.note.text().lower() and not bridge.requests


def test_restore_is_explicit_and_invalid_preview_clears_old_image(panel):
    app,w,bridge,status,path=panel;p=w.maps_panel;p.choose(str(path))
    w.state.update('/maps/status',dict(status,active=p.asset.info()));w.refresh();p.restore_button.click()
    assert bridge.requests[-1][1]['action']=='restore'
    p.choose(str(path.parent/'missing.yaml'))
    assert p.asset is None and p.view.image is None and not p.use_button.isEnabled()


def test_an_identical_map_copy_can_be_activated_as_a_different_saved_source(panel):
    app,w,bridge,status,path=panel;p=w.maps_panel
    original=Path(status['generated']);meta=yaml.safe_load(original.read_text())
    meta['image']=str(original.parent/meta['image'])
    path.write_text(yaml.safe_dump(meta));p.choose(str(path))
    assert p.asset.digest==status['active']['fingerprint']
    assert p.use_button.isEnabled()


def test_map_activation_waits_for_navigation_settings_to_finish(panel):
    from factory_bot.dashboard_navigation import SETTINGS_TOPIC
    app,w,bridge,status,path=panel;p=w.maps_panel;p.choose(str(path))
    w.state.update(SETTINGS_TOPIC,{'applying':True});w.refresh()
    assert not p.use_button.isEnabled() and 'defaults' in p.note.text()
