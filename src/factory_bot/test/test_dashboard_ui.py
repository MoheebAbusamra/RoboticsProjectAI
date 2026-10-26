"""Offscreen interaction checks. Uses a fake transport and never starts ROS or Gazebo."""
import os
from pathlib import Path

os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import pytest
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QComboBox, QPushButton

from factory_bot.dashboard import Dashboard, BRAND, preview
from factory_bot.dashboard_state import DashboardState


class Bridge:
    def __init__(self):self.requests=[];self.cameras=[]
    def request(self,kind,payload):self.requests.append((kind,payload))
    def set_camera(self,robot):self.cameras.append(robot)
    def stop(self):pass
    def join(self,timeout=None):pass


@pytest.fixture
def window():
    app=QApplication.instance() or QApplication([])
    share=Path(os.environ.get('FACTORY_DASHBOARD_ASSETS',Path(__file__).resolve().parents[1]))
    state=DashboardState();preview(state,share);state.preview=False
    bridge=Bridge();w=Dashboard(share,state,bridge);w.timer.stop()
    yield app,w,bridge
    w.close();app.processEvents()


def test_rebrand_and_existing_dispatch_camera_queue_are_preserved(window):
    app,w,bridge=window
    assert w.windowTitle()==BRAND
    w.submit_inline()
    assert bridge.requests[-1]==('delivery',{'pickup':'STORES','dropoff':'ASSEMBLY_A','priority':'routine'})
    w.select_robot('robot2');assert bridge.cameras[-1]=='robot2'
    w.set_page(1);assert bridge.cameras[-1] is None
    w.set_page(0);assert bridge.cameras[-1]=='robot2'
    assert w.queue_table.rowCount()==1
    assert w.active_table.item(0,6).text()=='A*'
    assert w.active_table.item(0,7).text()=='Pure pursuit'


def test_settings_dialog_sends_parameters_and_does_not_replace_active_routes(window):
    app,w,bridge=window;checked=[]
    def choose():
        d=QApplication.activeModalWidget()
        a=d.findChild(QComboBox,'navigation_algorithm');c=d.findChild(QComboBox,'navigation_controller')
        a.setCurrentIndex(a.findData('dijkstra'));c.setCurrentIndex(c.findData('pid'))
        apply=d.findChild(QPushButton,'apply_navigation');checked.append(apply.isEnabled());apply.click()
        d.accept()
    QTimer.singleShot(30,choose);w.navigation_dialog()
    assert checked==[True]
    assert bridge.requests==[('navigation_settings',{'algorithm':'dijkstra','controller':'pid'})]
    assert w.active_table.item(0,6).text()=='A*'  # current route telemetry stays authoritative


def test_preview_disables_parameter_application(window):
    app,w,bridge=window;w.state.preview=True;enabled=[]
    def inspect():
        d=QApplication.activeModalWidget();enabled.append(d.findChild(QPushButton,'apply_navigation').isEnabled());d.accept()
    QTimer.singleShot(30,inspect);w.navigation_dialog()
    assert enabled==[False] and not bridge.requests
