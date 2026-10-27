"""Native Qt pallet controls with fake telemetry, without ROS or a simulator."""
import os
import time
from pathlib import Path
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import Qt,QPoint
from PyQt5.QtTest import QTest
from factory_bot.dashboard import Dashboard,preview
from factory_bot.dashboard_state import DashboardState
from test_dashboard_maps import Bridge


def test_selection_commands_ack_removal_and_busy_gates():
    app=QApplication.instance() or QApplication([]);share=Path(__file__).resolve().parents[1]
    state=DashboardState();preview(state,share);state.preview=False;bridge=Bridge()
    w=Dashboard(share,state,bridge);w.timer.stop();p=w.pallet_panel
    obstacle={'pallet':{'place':'parked','pose':[18.,7.],'pending':False,'robots_ready':True,'requests':[]}}
    state.update('/maps/status',{'busy':False});state.update('/fleet/state',{'map_paused':False})
    state.update('/fleet/obstacles_state',obstacle);w.set_page(3);w.show();app.processEvents()
    try:
        p.select_xy(0.,0.);assert not p.place_button.isEnabled() and 'HOME_1' in p.selection.text()
        p.select_xy(5.,5.25);assert not p.place_button.isEnabled() and 'forklift' in p.selection.text()
        p.select_xy(3.5,4.);assert p.place_button.isEnabled() and not p.remove_button.isEnabled()
        rect,scale=p.view.projection();ox,oy=p.policy.origin
        point=QPoint(round(rect.left()+(4.-ox)*scale),round(rect.bottom()-(4.-oy)*scale))
        QTest.mouseClick(p.view,Qt.LeftButton,pos=point);assert p.x.value()==4. and p.y.value()==4.
        p.place_button.click();kind,request=bridge.requests[-1]
        assert kind=='pallet' and request['action']=='place' and request['x']==4. and not p.place_button.isEnabled()
        obstacle['pallet'].update(place='grid',pose=[4.,4.],requests=[{'id':request['id'],'phase':'APPLIED','message':'placed'}])
        state.update('/fleet/obstacles_state',obstacle);w.refresh()
        assert p.place_button.text()=='Move pallet' and p.remove_button.isEnabled()
        p.remove_button.click();assert bridge.requests[-1][1]['action']=='remove'
        state.update('/maps/status',{'busy':True});w.refresh();assert not p.place_button.isEnabled() and not p.remove_button.isEnabled()
        p.pending=None;state.update('/maps/status',{'busy':False});state.seen['/fleet/obstacles_state']=time.monotonic()-10;w.refresh()
        assert not p.place_button.isEnabled() and 'offline' in p.note.text().lower()
        state.preview=True;w.refresh();assert not p.remove_button.isEnabled()
    finally:w.close();app.processEvents()
