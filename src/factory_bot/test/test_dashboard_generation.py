"""Native Qt mapping controls against fake telemetry; no nodes or processes."""
import os
import time
from pathlib import Path
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from PyQt5.QtWidgets import QApplication

from factory_bot.dashboard import Dashboard, preview
from factory_bot.dashboard_state import DashboardState
from factory_bot.dashboard_generation import GenerationDialog
from test_dashboard_maps import Bridge


def test_generation_controls_progress_outputs_and_explicit_preview():
    app=QApplication.instance() or QApplication([]);share=Path(__file__).resolve().parents[1]
    state=DashboardState();preview(state,share);state.preview=False;bridge=Bridge()
    w=Dashboard(share,state,bridge);w.timer.stop()
    status={'phase':'READY','can_activate':True,'can_generate':True,'busy':False,'mapping':{}}
    state.update('/maps/status',status);d=GenerationDialog(w.maps_panel);d.timer.stop()
    try:
        assert d.mode.count()==3 and d.start.isEnabled() and not d.finish.isEnabled()
        d.mode.setCurrentIndex(2);d.start.click();assert bridge.requests[-1][1]['mode']=='slam_ogm' and bridge.requests[-1][1]['rviz'] is True
        request=bridge.requests[-1][1]
        session={'id':'test','busy':True,'phase':'SURVEYING','mode':'slam_ogm','waypoint':2,'total':14,
            'started_wall':time.monotonic()-1,'survey_fraction':.25,'engine':{'scans':80,'slam_poses':21,'hybrid_pairs':21,'pose':[1.,2.,0.]}}
        status.update(phase='MAPPING/SURVEYING',busy=True,can_activate=False,can_generate=False,mapping=session,
            received_request_id=request['id'])
        state.update('/maps/status',status);state.update('/mapping/live_map',{'width':3,'height':2,
            'origin':[-1.3,-1.3,0.],'resolution':.05,'values':[-1,0,100,0,100,-1]});d.refresh();w.refresh()
        assert d.finish.isEnabled() and d.cancel.isEnabled() and not d.start.isEnabled()
        assert d.progress.value()==250 and d.view.image.width()==3 and 'Scans 80' in d.stats.text()
        assert d.view.robots['robot1']==[1.,2.,0.]
        d.rviz.click();assert bridge.requests[-1][1]['action']=='mapping_visualize'
        status['received_request_id']=bridge.requests[-1][1]['id'];state.update('/maps/status',status);d.refresh()
        d.finish.click();assert bridge.requests[-1][1]['action']=='finish_mapping'
        session.update(phase='DONE',busy=False,maps=[{'label':'SLAM + OGM','path':str(share/'maps/factory_map.yaml')}],
            comparison=[{'a':'slam','b':'ogm_slam','common_cells':123,'agreement_percent':98.,'occupied_iou':.9}])
        status.update(busy=False,can_activate=True,can_generate=True);state.update('/maps/status',status);d.refresh()
        assert d.preview.isEnabled() and '98.0%' in d.comparison.text()
        before=len(bridge.requests);d.preview.click();assert len(bridge.requests)==before
        assert w.maps_panel.asset is not None
        session.update(phase='STARTING_MAPPERS',busy=True,engine={'ready_reason':'Waiting for SLAM map on /mapping/map'})
        state.update('/maps/status',status);d.refresh();assert d.progress.maximum()==0 and '/mapping/map' in d.note.text()
        session.update(phase='DONE',busy=False,error='starting mappers timed out: waiting for map')
        state.update('/maps/status',status);d.refresh();assert 'Survey failed' in d.note.text()
    finally:d.close();w.close();app.processEvents()


def test_dashboard_font_is_fixed_width_and_antialiased():
    from PyQt5.QtGui import QFontMetrics,QFont
    from factory_bot.dashboard import configure_mono_font,mono_font
    app=QApplication.instance() or QApplication([]);configure_mono_font()
    font=mono_font(14);metrics=QFontMetrics(font)
    assert metrics.horizontalAdvance('iiii')==metrics.horizontalAdvance('WWWW')
    assert font.styleStrategy() != QFont.NoAntialias and font.family()!='Pixelify Sans'
