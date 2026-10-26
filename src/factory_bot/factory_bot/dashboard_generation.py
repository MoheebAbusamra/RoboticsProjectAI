"""On-demand survey controls. Sensor maps are previewed before explicit activation."""
from types import SimpleNamespace
import time
import uuid

import numpy as np
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QLabel, QComboBox,
                            QPushButton, QProgressBar, QCheckBox)

from factory_bot.dashboard_maps import OccupancyView
from factory_bot.dashboard_navigation import SETTINGS_TOPIC


class GenerationDialog(QDialog):
    def __init__(self, panel):
        super().__init__(panel); self.panel=panel; self.pending=None; self.error=''; self.preview_stamp=None
        self.setWindowTitle('son3a feh masr | Mapping studio'); self.resize(1080,760)
        layout=QVBoxLayout(self)
        title=QLabel('MAPPING STUDIO / MEASURE THE FACTORY'); title.setObjectName('sectionTitle'); layout.addWidget(title)
        hint=QLabel('Start with both robots idle at HOME. Robot 1 surveys using the current navigation map. '
            'Robot 2 waits; workers, forklift and pallet are parked. Saved outputs require explicit activation. '
            'This is a guided survey, not autonomous exploration of an unknown factory.')
        hint.setWordWrap(True); layout.addWidget(hint)
        row=QHBoxLayout(); self.mode=QComboBox(); self.mode.setAccessibleName('Mapping method')
        for label,key in [('SLAM / scan matching + loop closure','slam'),('OGM / lidar + wheel odometry','ogm'),
                          ('SLAM + OGM / combined map + comparison','slam_ogm')]:self.mode.addItem(label,key)
        row.addWidget(self.mode,1)
        self.start=QPushButton('Start survey'); self.start.setObjectName('primary'); self.start.clicked.connect(lambda:self.submit('generate'))
        self.finish=QPushButton('Finish & save now'); self.finish.clicked.connect(lambda:self.submit('finish_mapping'))
        self.cancel=QPushButton('Cancel / return home'); self.cancel.clicked.connect(lambda:self.submit('cancel_mapping'))
        for button in (self.start,self.finish,self.cancel):row.addWidget(button)
        layout.addLayout(row)
        row=QHBoxLayout();self.auto_rviz=QCheckBox('Open RViz when the survey starts');self.auto_rviz.setChecked(True)
        row.addWidget(self.auto_rviz);row.addStretch()
        self.rviz=QPushButton('Open RViz');self.rviz.clicked.connect(lambda:self.submit('mapping_visualize'));row.addWidget(self.rviz)
        layout.addLayout(row)
        self.progress=QProgressBar(); layout.addWidget(self.progress)
        self.note=QLabel(''); self.note.setWordWrap(True); self.note.setTextFormat(Qt.PlainText); layout.addWidget(self.note)
        self.view=OccupancyView(panel.view.stations,panel.view.zones); layout.addWidget(self.view,1)
        self.stats=QLabel('LIVE MEASUREMENTS / waiting for a survey'); self.stats.setWordWrap(True); layout.addWidget(self.stats)
        self.comparison=QLabel(''); self.comparison.setWordWrap(True); layout.addWidget(self.comparison)
        row=QHBoxLayout(); self.results=QComboBox(); self.results.setAccessibleName('Generated map outputs'); row.addWidget(self.results,1)
        self.preview=QPushButton('Preview saved result'); self.preview.clicked.connect(self.choose_result); row.addWidget(self.preview)
        close=QPushButton('Close'); close.clicked.connect(self.close); row.addWidget(close); layout.addLayout(row)
        self.timer=QTimer(self); self.timer.timeout.connect(self.refresh); self.timer.start(250); self.refresh()

    def submit(self,action):
        request={'id':uuid.uuid4().hex,'action':action}
        if action=='generate':request.update(mode=self.mode.currentData(),rviz=self.auto_rviz.isChecked())
        try:self.panel.send('map_activate',request); self.pending=(request['id'],time.monotonic()); self.error=''
        except ValueError as error:self.error=str(error)
        self.refresh()

    def refresh(self):
        snap=self.panel.state.snapshot(); now=time.monotonic(); status=snap['values'].get('/maps/status',{})
        session=status.get('mapping',{}); engine=session.get('engine',{}); busy=session.get('busy',False)
        fresh=now-snap['seen'].get('/maps/status',-float('inf'))<2.5
        if self.pending:
            identifier,submitted=self.pending
            if status.get('received_request_id')==identifier:self.pending=None
            elif now-submitted>10:self.error='No coordinator acknowledgement'; self.pending=None
        enabled=bool(self.panel.live and not snap['preview'] and fresh and not self.pending)
        settings_busy=snap['values'].get(SETTINGS_TOPIC,{}).get('applying')
        self.start.setEnabled(enabled and status.get('can_generate',False) and not settings_busy)
        self.mode.setEnabled(not busy)
        self.auto_rviz.setEnabled(not busy);self.rviz.setEnabled(enabled)
        self.finish.setEnabled(enabled and session.get('phase')=='SURVEYING')
        self.cancel.setEnabled(enabled and busy and session.get('phase') not in ('RESUMING','RELEASING','ERROR_RESUME'))
        self.cancel.setText('Retry cleanup / return home' if session.get('phase')=='ERROR_HELD' else 'Cancel / return home')
        phase=session.get('phase','IDLE')
        if not session.get('id'):
            self.progress.setRange(0,1);self.progress.setValue(0);self.progress.setFormat('Ready to start a survey')
        elif busy and phase in ('PAUSING','PARKING','STARTING_MAPPERS'):
            self.progress.setRange(0,0);self.progress.setFormat('Preparing sensors and mapper')
        else:
            fraction=session.get('survey_fraction',session.get('waypoint',0)/max(1,session.get('total',1)))
            complete=phase=='DONE' and not session.get('error') and not session.get('cancelled')
            self.progress.setRange(0,1000);self.progress.setValue(1000 if complete else round(fraction*1000))
            self.progress.setFormat('Survey route: %p% / '+str(session.get('waypoint',0))+' of '+str(session.get('total',0))+' waypoints')
        failure=session.get('error') or engine.get('error')
        error=self.error or status.get('request_error') or failure or session.get('visualizer_error')
        text='Design preview / controls disabled' if snap['preview'] else 'Coordinator offline' if not fresh else (
            session.get('message','Ready') if busy or session.get('id') else status.get('generation_reason','Ready to survey'))
        if error:text+='\n'+error
        if busy and engine.get('ready_reason'):text+='\n'+engine['ready_reason']
        if phase=='DONE':
            text='Survey failed' if failure else 'Survey cancelled / scene restored' if session.get('cancelled') else 'Maps saved / scene restored'
            if error:text+='\n'+error
        if self.pending:text+='\nRequest sent; awaiting acknowledgement'
        self.note.setText(text); self.note.setStyleSheet('color: '+('#efa39b' if error else '#c4b19d'))
        self.stats.setText(f"Scans {engine.get('scans',0)} / SLAM maps {engine.get('slam_maps',0)} / SLAM poses {engine.get('slam_poses',0)} / "
            f"Combined pairs {engine.get('hybrid_pairs',0)} / Skipped scans {engine.get('skipped',0)}\n"
            f"Preview: {engine.get('preview_kind') or 'waiting for measurements'}. Cream occupied / burgundy free / mauve unknown. "
            'Simulator-reference output is labelled separately.')
        self.comparison.setText('\n'.join(f"{v['a']} vs {v['b']}: {v['common_cells']} common cells / "
            f"agreement {v['agreement_percent']}% / occupied IoU {v['occupied_iou']}"
            for v in session.get('comparison',[]) if v['common_cells']))
        if session.get('id')!=getattr(self,'display_session',None):
            self.display_session=session.get('id'); self.results.clear(); self.view.image=None; self.preview_stamp=None; self.view.trajectory=[];self.view.robots={}; self.view.update()
        stamp=snap['seen'].get('/mapping/live_map',0.)
        grid=snap['values'].get('/mapping/live_map')
        if grid and stamp>=session.get('started_wall',float('inf')) and stamp!=self.preview_stamp:
            self.preview_stamp=stamp; values=np.asarray(grid['values'],dtype=np.int16).reshape(grid['height'],grid['width'])
            pixels=np.full(values.shape,205,dtype=np.uint8); pixels[(values>=0)&(values<=19)]=254; pixels[values>=65]=0
            self.view.set_asset(SimpleNamespace(pixels=pixels[::-1].copy(),meta={'origin':grid['origin'],
                'resolution':grid['resolution'],'negate':0,'occupied_thresh':.65,'free_thresh':.196}))
        if engine.get('pose'):self.view.robots={'robot1':engine['pose']}
        if snap['seen'].get('/mapping/trajectory',0)>=session.get('started_wall',float('inf')):
            self.view.trajectory=snap['values'].get('/mapping/trajectory',[])
        self.view.update()
        paths=[self.results.itemData(i) for i in range(self.results.count())]
        for result in session.get('maps',[]):
            if result['path'] not in paths:
                label=result['label']+(' / SIMULATOR REFERENCE' if result.get('reference_only') else '')
                self.results.addItem(label,result['path']); paths.append(result['path'])
        self.preview.setEnabled(bool(self.results.count()) and not busy)

    def choose_result(self):
        path=self.results.currentData()
        if path:self.panel.choose(path); self.close()

    def closeEvent(self,event):
        # Closing this view leaves the coordinator-owned session visible through Maps.
        self.timer.stop(); super().closeEvent(event)
