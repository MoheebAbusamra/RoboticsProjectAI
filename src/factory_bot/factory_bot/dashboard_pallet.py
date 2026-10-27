"""Grid placement of the existing pallet, with simulator-confirmed status."""
import math
import time
import uuid

from PyQt5.QtCore import Qt,QRectF,QPointF,pyqtSignal
from PyQt5.QtGui import QColor,QPainter,QPen,QPolygonF
from PyQt5.QtWidgets import QWidget,QFrame,QLabel,QPushButton,QDoubleSpinBox,QVBoxLayout,QHBoxLayout,QFormLayout

from factory_bot.dashboard_maps import OccupancyView
from factory_bot.pallet_placement import PlacementPolicy,rectangle


class PalletGrid(OccupancyView):
    selected=pyqtSignal(float,float)
    def __init__(self,policy,stations,zones):
        super().__init__(stations,zones);self.policy=policy;self.set_asset(policy.asset)
        self.target=(3.5,4.);self.valid=False;self.pallet=None;self.setCursor(Qt.CrossCursor)

    def projection(self):
        width,height=self.asset.pixels.shape[1],self.asset.pixels.shape[0]
        scale=min((self.width()-48)/width,(self.height()-48)/height)
        return QRectF((self.width()-width*scale)/2,(self.height()-height*scale)/2,width*scale,height*scale),scale/self.policy.res

    def mousePressEvent(self,event):
        if event.button()!=Qt.LeftButton:return
        rect,scale=self.projection()
        if rect.contains(event.pos()):
            x=self.policy.origin[0]+(event.x()-rect.left())/scale
            y=self.policy.origin[1]+(rect.bottom()-event.y())/scale
            self.selected.emit(*self.policy.snap(x,y))

    def paintEvent(self,event):
        super().paintEvent(event)
        p=QPainter(self);p.setRenderHint(QPainter.Antialiasing);rect,scale=self.projection();p.setClipRect(rect)
        ox,oy=self.policy.origin
        def point(x,y):return QPointF(rect.left()+(x-ox)*scale,rect.bottom()-(y-oy)*scale)
        p.setPen(QPen(QColor(196,177,157,38),1))
        for x in range(math.ceil(ox/self.policy.step),math.floor((ox+rect.width()/scale)/self.policy.step)+1):
            xx=point(x*self.policy.step,0).x();p.drawLine(QPointF(xx,rect.top()),QPointF(xx,rect.bottom()))
        for y in range(math.ceil(oy/self.policy.step),math.floor((oy+rect.height()/scale)/self.policy.step)+1):
            yy=point(0,y*self.policy.step).y();p.drawLine(QPointF(rect.left(),yy),QPointF(rect.right(),yy))
        for name,poly in self.policy.station_areas:
            p.setPen(QPen(QColor('#a2bde4'),1,Qt.DashLine));p.setBrush(QColor(120,165,225,48))
            p.drawPolygon(QPolygonF([point(*v) for v in poly]))
        for name,poly in self.policy.corridors:
            p.setPen(QPen(QColor('#d78d8d'),1));p.setBrush(QColor(215,105,105,38))
            p.drawPolygon(QPolygonF([point(*v) for v in poly]))
        for name,route in self.policy.routes.items():
            p.setPen(QPen(QColor('#efa39b'),1,Qt.DashLine));p.setBrush(Qt.NoBrush)
            p.drawPolyline(QPolygonF([point(*v) for v in route]));p.drawText(point(*route[-1])+QPointF(5,-7),name)
        def pallet(where,color,fill,style):
            p.setPen(QPen(QColor(color),2,style));p.setBrush(QColor(fill))
            p.drawPolygon(QPolygonF([point(*v) for v in rectangle(*where,self.policy.bounds)]))
        if self.pallet and self.pallet.get('place')!='parked':pallet(self.pallet['pose'],'#94cdbd','#456a60',Qt.SolidLine)
        pallet(self.target,'#dfb582' if self.valid else '#efa39b','#50402e' if self.valid else '#63373c',Qt.DashLine)
        p.setPen(QPen(QColor('#f3e8d2'),1));centre=point(*self.target)
        p.drawLine(centre+QPointF(-5,0),centre+QPointF(5,0));p.drawLine(centre+QPointF(0,-5),centre+QPointF(0,5))


class PalletPanel(QWidget):
    def __init__(self,share,state,send,stations,zones,live):
        super().__init__();self.state=state;self.send=send;self.live=live
        self.policy=PlacementPolicy.from_share(share);self.pending=None;self.error=''
        box=QVBoxLayout(self);box.setContentsMargins(0,0,0,0);box.setSpacing(12)
        hint=QLabel('PALLET / DYNAMIC OBSTACLE\nClick a grid point to place or move the existing pallet. Keep the whole pallet clear of red actor corridors and blue station docking areas.')
        hint.setWordWrap(True);box.addWidget(hint)
        row=QHBoxLayout();left=QFrame();left.setObjectName('card');col=QVBoxLayout(left)
        self.view=PalletGrid(self.policy,stations,zones);self.view.selected.connect(self.select_xy);col.addWidget(self.view,1)
        legend=QLabel('0.25 m grid  /  green: confirmed pallet  /  dashed: selected position  /  red: actor routes  /  blue: station docking areas')
        legend.setWordWrap(True);col.addWidget(legend);row.addWidget(left,3)
        right=QFrame();right.setObjectName('card');right.setMaximumWidth(370);right.setMinimumWidth(280);controls=QVBoxLayout(right)
        controls.addWidget(QLabel('PALLET CONTROL'))
        self.current=QLabel('Waiting for obstacle-controller status');self.current.setWordWrap(True);self.current.setTextFormat(Qt.PlainText);controls.addWidget(self.current)
        form=QFormLayout();self.x=QDoubleSpinBox();self.y=QDoubleSpinBox()
        for widget,title,origin,size in ((self.x,'X / metres',self.policy.origin[0],self.policy.width),(self.y,'Y / metres',self.policy.origin[1],self.policy.height)):
            widget.setRange(origin,origin+size*self.policy.res);widget.setSingleStep(self.policy.step);widget.setDecimals(2)
            widget.setKeyboardTracking(False);widget.setAccessibleName('Pallet '+title);widget.valueChanged.connect(self.changed);form.addRow(title,widget)
        controls.addLayout(form)
        detail=QLabel('Coordinates are the pallet centre in the factory map frame. The pallet stays aligned with the grid.\n\nIts full 1.05 × 1.00 m collision footprint is checked, not just the selected point.')
        detail.setWordWrap(True);controls.addWidget(detail)
        self.selection=QLabel('');self.selection.setWordWrap(True);self.selection.setTextFormat(Qt.PlainText);controls.addWidget(self.selection)
        self.place_button=QPushButton('Place pallet');self.place_button.setObjectName('primary');self.place_button.clicked.connect(lambda:self.submit('place'));controls.addWidget(self.place_button)
        self.remove_button=QPushButton('Remove / park outside');self.remove_button.clicked.connect(lambda:self.submit('remove'));controls.addWidget(self.remove_button)
        self.note=QLabel('');self.note.setWordWrap(True);self.note.setTextFormat(Qt.PlainText);controls.addWidget(self.note)
        note=QLabel('Changes apply after Gazebo acknowledges the pose. Pallet controls are locked during mapping and map switching. The pallet is detected by the existing lidar and rerouting logic.')
        note.setWordWrap(True);controls.addWidget(note);controls.addStretch();row.addWidget(right,1);box.addLayout(row,1)
        self.select_xy(3.5,4.)

    def select_xy(self,x,y):
        x,y=self.policy.snap(x,y)
        for widget,value in ((self.x,x),(self.y,y)):
            widget.blockSignals(True);widget.setValue(value);widget.blockSignals(False)
        self.error='';self.refresh(self.state.snapshot())

    def changed(self):self.select_xy(self.x.value(),self.y.value())

    def submit(self,action):
        if not self.live or self.state.preview:return
        request={'id':uuid.uuid4().hex,'action':action}
        if action=='place':request.update(x=self.x.value(),y=self.y.value())
        self.pending=(request['id'],time.monotonic());self.error='';self.send('pallet',request);self.refresh(self.state.snapshot())

    def refresh(self,snapshot):
        now=time.monotonic();values=snapshot['values'];state=values.get('/fleet/obstacles_state',{});pallet=state.get('pallet',{})
        fresh=now-snapshot['seen'].get('/fleet/obstacles_state',-math.inf)<2.5
        if self.pending:
            identifier,started=self.pending;response=next((v for v in pallet.get('requests',[]) if v['id']==identifier),None)
            if response:
                self.error=response['message'] if response['phase'] in ('REJECTED','FAILED','UNCERTAIN') else ''
                if response['phase'] in ('APPLIED','REJECTED','FAILED'):self.pending=None
            elif now-started>10:self.error='No pallet acknowledgement; check the connection. Showing the last confirmed position.';self.pending=None
        robots={r:st['pose'] for r in ('robot1','robot2') if (st:=values.get('/'+r+'/nav_status',{})).get('pose') and now-snapshot['seen'].get('/'+r+'/nav_status',-math.inf)<2.5}
        valid,reason,target=self.policy.validate(self.x.value(),self.y.value(),robots)
        self.view.target=target;self.view.valid=valid;self.view.pallet=pallet or None;self.view.robots=robots;self.view.update()
        self.selection.setText(f'Selected ({target[0]:.2f}, {target[1]:.2f}) m\n'+reason)
        self.selection.setStyleSheet('color: '+('#94cdbd' if valid else '#efa39b'))
        placed=bool(pallet and pallet.get('place')!='parked')
        self.current.setText('Pallet inside factory\n'+str(tuple(pallet['pose']))+' m' if placed else 'Pallet parked outside' if pallet else 'Waiting for pallet status / restart the complete demo')
        busy=values.get('/maps/status',{}).get('busy') or values.get('/fleet/state',{}).get('map_paused')
        ready=bool(self.live and not snapshot['preview'] and fresh and pallet and not pallet.get('pending') and not pallet.get('blocked') and not busy and not self.pending)
        self.place_button.setText('Move pallet' if placed else 'Place pallet');self.place_button.setEnabled(ready and valid and pallet.get('robots_ready',False))
        self.remove_button.setEnabled(ready and placed)
        if snapshot['preview']:message='Design preview / live pallet controls disabled'
        elif not fresh:message='Obstacle controller offline / waiting for fresh telemetry'
        elif busy or pallet.get('blocked'):message='Pallet controls are locked during mapping or map switching'
        elif self.error:message=self.error
        elif self.pending or pallet.get('pending'):message='Waiting for Gazebo placement acknowledgement'
        elif not pallet.get('robots_ready'):message='Waiting for fresh positions from both robots'
        else:message='Select a valid grid point, then place or move the pallet.'
        self.note.setText(message);self.note.setStyleSheet('color: '+('#efa39b' if self.error else '#c4b19d'))
