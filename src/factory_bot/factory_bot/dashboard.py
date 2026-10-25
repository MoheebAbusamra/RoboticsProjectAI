"""Northline Control: native Python/Qt fleet operations dashboard.

Live: ros2 run factory_bot dashboard
Design preview (no ROS commands): python3 -m factory_bot.dashboard --preview
"""
import argparse
import json
import math
from pathlib import Path
import signal
import sys
import time

import yaml
from PyQt5.QtCore import Qt, QRectF, QPointF, QTimer, QEvent, pyqtSignal
from PyQt5.QtGui import (QColor, QFont, QPalette, QImage, QPainter, QPainterPath, QPen,
                         QPixmap, QPolygonF, QRadialGradient, QLinearGradient)
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QLabel, QPushButton,
                             QFrame, QVBoxLayout, QHBoxLayout, QGridLayout, QComboBox,
                             QStackedWidget, QScrollArea, QDialog, QDialogButtonBox,
                             QTableWidget, QTableWidgetItem, QHeaderView, QAbstractItemView,
                             QSizePolicy, QFileDialog)

from factory_bot.dashboard_state import (DashboardState, ROBOT_NAMES, age, topic_hz,
                                         real_time_factor, project_map, route_remaining,
                                         image_rgb, delivery_request)

BG='#0b1110'; PANEL='#131b18'; BORDER='#2b3830'; TEXT='#edf1e8'; MUTED='#a0afa4'
GREEN='#b9e788'; R1='#7cdbb1'; R2='#efbb75'; RED='#ed8c7c'
COLORS={'robot1':R1,'robot2':R2}

STYLE=f'''
QMainWindow, QDialog {{ background: {BG}; color: {TEXT}; }}
QWidget {{ color: {TEXT}; }}
QFrame#card {{ background: {PANEL}; border: 1px solid {BORDER}; border-radius: 13px; }}
QLabel {{ background: transparent; border: none; }}
QPushButton {{ background: #1c2820; border: 1px solid #344436; border-radius: 8px; padding: 9px 13px; color: {TEXT}; }}
QPushButton:hover {{ background: #2c3c2d; border-color: #66845b; }}
QPushButton:pressed {{ background: #394e34; }}
QPushButton:focus, QComboBox:focus {{ border: 2px solid {GREEN}; }}
QPushButton:disabled {{ color: #68776b; background: #18211b; border-color: #27342b; }}
QPushButton#primary {{ background: {GREEN}; color: #152111; border-color: {GREEN}; font-weight: 600; }}
QPushButton#primary:hover {{ background: #c9f19e; }}
QPushButton#primary:disabled {{ background: #344331; border-color: #344331; color: #85927e; }}
QPushButton#danger {{ background: #31201b; color: #f5b1a4; border-color: #63372c; }}
QPushButton#danger:hover {{ background: #4b2a22; }}
QPushButton#nav {{ border-color: transparent; background: transparent; color: {MUTED}; border-radius: 18px; }}
QPushButton#nav:checked {{ color: {TEXT}; background: #263429; border: 1px solid #45533d; }}
QComboBox {{ background: #0e1711; border: 1px solid #384637; border-radius: 7px; padding: 9px 10px; min-height: 18px; }}
QComboBox::drop-down {{ width: 25px; border: none; }}
QComboBox QAbstractItemView {{ background: #17231a; color: {TEXT}; selection-background-color: #3c5434; border: 1px solid #627559; }}
QScrollArea {{ background: transparent; border: none; }}
QScrollBar:vertical {{ background: #101811; width: 8px; }}
QScrollBar::handle:vertical {{ background: #42543b; border-radius: 4px; min-height: 28px; }}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0; }}
QTableWidget {{ background: #111a14; alternate-background-color: #17221a; gridline-color: #28362a; border: none; selection-background-color: #34492f; }}
QHeaderView::section {{ background: #1c291e; color: {MUTED}; border: none; padding: 12px 8px; font-size: 11px; }}
QTableWidget::item {{ padding: 10px; border-bottom: 1px solid #243226; }}
QDialogButtonBox {{ background: transparent; }}
'''


def text_label(text='', size=13, color=TEXT, bold=False, mono=False):
    w=QLabel(text)
    f=QFont('DejaVu Sans Mono' if mono else 'DejaVu Sans')
    f.setPixelSize(max(11,size));f.setBold(bold);w.setFont(f)
    w.setStyleSheet('color: '+color+';')
    return w


def set_text(widget, text):
    text=str(text)
    if widget.text()!=text:widget.setText(text)


def button(text, callback=None, primary=False, danger=False):
    w=QPushButton(text)
    if primary:w.setObjectName('primary')
    if danger:w.setObjectName('danger')
    if callback:w.clicked.connect(callback)
    w.setCursor(Qt.PointingHandCursor)
    return w


def clock_text(t):
    if t is None:return '--:--:--'
    t=max(0,int(t));return f'{t//3600:02d}:{t//60%60:02d}:{t%60:02d}'


def friendly(s):return str(s or '—').replace('_',' ').title()


class Card(QFrame):
    def __init__(self, title=None, note=None):
        super().__init__();self.setObjectName('card')
        self.box=QVBoxLayout(self);self.box.setContentsMargins(16,14,16,14);self.box.setSpacing(12)
        if title:
            row=QHBoxLayout();row.addWidget(text_label(title,11,MUTED,True));row.addStretch()
            if note:row.addWidget(text_label(note,11,MUTED,False,True))
            self.box.addLayout(row)


class Logo(QWidget):
    def __init__(self):super().__init__();self.setFixedSize(31,31)
    def paintEvent(self,event):
        p=QPainter(self);p.setRenderHint(QPainter.Antialiasing)
        p.setPen(QPen(QColor(GREEN),3))
        p.drawLine(4,26,4,6);p.drawLine(4,6,25,26);p.drawLine(25,26,25,6)
        p.setPen(QPen(QColor('#e9f1e5'),2));p.drawLine(12,6,26,6)


class Metric(Card):
    def __init__(self,title,suffix):
        super().__init__();self.box.setSpacing(5);self.box.setContentsMargins(14,11,14,11)
        self.box.addWidget(text_label(title,11,MUTED))
        row=QHBoxLayout();self.value=text_label('—',27,TEXT,False,True)
        row.addWidget(self.value);row.addStretch();row.addWidget(text_label(suffix,10,MUTED))
        self.box.addLayout(row)


class FactoryView(QWidget):
    selected=pyqtSignal(str)
    def __init__(self,share,stations,zones):
        super().__init__();self.setMinimumHeight(300);self.setMouseTracking(True)
        self.setSizePolicy(QSizePolicy.Expanding,QSizePolicy.Expanding)
        self.pix=QPixmap(str(share/'models/dashboard/factory_overview.png'))
        self.proj=json.loads((share/'models/dashboard/projection.json').read_text())
        self.stations,self.zones=stations,zones
        self.snapshot={'values':{},'seen':{}};self.robot='robot1';self.mode='model';self.zoom=1.;self.pan=QPointF(0,0)
        self.drag=None;self.robot_hits={};self.labels=True;self.routes=True
        self.setToolTip('Click a robot to inspect it. Drag to pan; wheel to zoom. Double-click to reset.')

    def set_snapshot(self,snapshot,robot):self.snapshot=snapshot;self.robot=robot;self.update()
    def reset(self):self.zoom=1.;self.pan=QPointF(0,0);self.update()
    def wheelEvent(self,e):
        self.zoom=max(.75,min(2.4,self.zoom*(1.12 if e.angleDelta().y()>0 else 1/1.12)));self.update()
    def mousePressEvent(self,e):
        if e.button()==Qt.LeftButton:
            for name,pos in self.robot_hits.items():
                if math.hypot(pos.x()-e.x(),pos.y()-e.y())<22:self.selected.emit(name);return
            self.drag=e.pos();self.setCursor(Qt.ClosedHandCursor)
    def mouseMoveEvent(self,e):
        if self.drag is not None:self.pan+=e.pos()-self.drag;self.drag=e.pos();self.update()
    def mouseReleaseEvent(self,e):self.drag=None;self.setCursor(Qt.ArrowCursor)
    def mouseDoubleClickEvent(self,e):self.reset()

    def paintEvent(self,e):
        p=QPainter(self);p.setRenderHint(QPainter.Antialiasing);p.setRenderHint(QPainter.SmoothPixmapTransform)
        r=QRectF(self.rect());p.fillRect(r,QColor('#101a13'))
        grad=QRadialGradient(r.center(),r.width()*.7);grad.setColorAt(0,QColor('#253728'));grad.setColorAt(1,QColor('#101a13'))
        p.fillRect(r,grad)
        p.setPen(QPen(QColor(112,141,106,8),1))
        for x in range(0,self.width(),32):p.drawLine(x,0,x,self.height())
        for y in range(0,self.height(),32):p.drawLine(0,y,self.width(),y)
        if self.mode=='model':
            iw,ih=self.proj['size'];x0,y0,x1,y1=self.proj.get('content_bounds',[0,0,1,1])
            scale=min((self.width()-38)/(iw*(x1-x0)),(self.height()-34)/(ih*(y1-y0)))*self.zoom
            w,h=iw*scale,ih*scale
            atlas=QRectF(self.width()/2-(x0+x1)*w/2+self.pan.x(),self.height()/2-(y0+y1)*h/2+self.pan.y(),w,h)
            def point(x,y):
                u,v=project_map(x,y,self.proj);return QPointF(atlas.x()+u*w,atlas.y()+v*h)
            p.drawPixmap(atlas,self.pix,QRectF(self.pix.rect()))
        else:
            scale=min((self.width()-100)/16,(self.height()-60)/14)*self.zoom
            cx,cy=self.width()/2+self.pan.x(),self.height()/2+self.pan.y()
            def point(x,y):return QPointF(cx+(x-7)*scale,cy-(y-6)*scale)
            corners=[point(-1,-1),point(15,-1),point(15,13),point(-1,13)]
            p.setBrush(QColor('#233127'));p.setPen(QPen(QColor('#66815a'),1));p.drawPolygon(QPolygonF(corners))
            # Obstacles come from the unchanged collision manifest, not from a fabricated floor plan.
            for c in self.collisions:
                x,y,z=c['xyz'];sx,sy,h=c.get('size',(0,0,0))
                if z-h/2<=.23<=z+h/2:
                    x-=1;y-=1
                    pts=[point(x+dx*sx/2,y+dy*sy/2) for dx,dy in [(-1,-1),(1,-1),(1,1),(-1,1)]]
                    p.setBrush(QColor('#526357'));p.setPen(QPen(QColor('#91a28c'),.6));p.drawPolygon(QPolygonF(pts))
            for z in self.zones.values():
                color={'forbidden':RED,'slow':R2,'authorized_only':R1}.get(z['rule'],GREEN)
                fill=QColor(color);fill.setAlpha(34);p.setBrush(fill);p.setPen(QPen(QColor(color),1,Qt.DashLine))
                pts=[point(x,y) for x,y in [(z['x_min'],z['y_min']),(z['x_max'],z['y_min']),
                                           (z['x_max'],z['y_max']),(z['x_min'],z['y_max'])]]
                p.drawPolygon(QPolygonF(pts))
        values=self.snapshot['values'];fleet=values.get('/fleet/state',{})
        if self.routes:
            for robot in ROBOT_NAMES:
                path=values.get(f'/{robot}/plan',[])
                nav=values.get(f'/{robot}/nav_status',{})
                if not path or nav.get('state') not in ('DRIVING','BLOCKED','ALIGNING'):continue
                pose=nav.get('pose');start=0
                if pose:start=min(range(len(path)),key=lambda i:math.dist(path[i],pose[:2]))
                route=QPainterPath(point(*path[start]))
                for xy in path[start+1:]:route.lineTo(point(*xy))
                glow=QColor(COLORS[robot]);glow.setAlpha(40);p.setPen(QPen(glow,10));p.drawPath(route)
                pen=QPen(QColor(COLORS[robot]),2.3,Qt.DashLine);p.setPen(pen);p.drawPath(route)
        if self.labels:
            f=QFont('DejaVu Sans');f.setPixelSize(10);f.setBold(True);p.setFont(f)
            for name,s in self.stations.items():
                if name.startswith('HOME'):continue
                pos=point(s['x'],s['y']);caption=name.replace('_',' ')
                textwidth=p.fontMetrics().horizontalAdvance(caption)
                near_robot=any(rec.get('pose') and math.dist(rec['pose'][:2],(s['x'],s['y']))<.75
                               for rec in fleet.get('robots',{}).values())
                chip=QRectF(pos.x()-textwidth/2-7,pos.y()-(46 if near_robot else 23),textwidth+14,21)
                if near_robot:
                    p.setPen(QPen(QColor('#78876a'),.7));p.drawLine(QPointF(chip.center().x(),chip.bottom()),pos)
                p.setPen(QPen(QColor('#78876a'),.6));p.setBrush(QColor(17,28,19,220));p.drawRoundedRect(chip,6,6)
                p.setPen(QColor('#e1e8d7'));p.drawText(chip,Qt.AlignCenter,caption)
                p.setPen(Qt.NoPen);p.setBrush(QColor('#d7e4c2'));p.drawEllipse(pos,2.5,2.5)
        self.robot_hits={}
        for robot in ROBOT_NAMES:
            nav=values.get(f'/{robot}/nav_status',{});pose=nav.get('pose')
            if not pose:pose=fleet.get('robots',{}).get(robot,{}).get('pose')
            if not pose:continue
            pos=point(*pose[:2]);self.robot_hits[robot]=pos
            stale=age(self.snapshot,f'/{robot}/nav_status')>3
            color=QColor('#839184' if stale else COLORS[robot])
            halo=QColor(color);halo.setAlpha(35);p.setPen(Qt.NoPen);p.setBrush(halo);p.drawEllipse(pos,24,24)
            p.setPen(QPen(color,2));p.setBrush(QColor('#101c14'));p.drawEllipse(pos,12,12)
            if robot==self.robot:p.setPen(QPen(color,1,Qt.DashLine));p.setBrush(Qt.NoBrush);p.drawEllipse(pos,19,19)
            toward=point(pose[0]+.6*math.cos(pose[2]),pose[1]+.6*math.sin(pose[2]))
            delta=toward-pos;length=max(.001,math.hypot(delta.x(),delta.y()));end=pos+delta*(27/length)
            p.setPen(QPen(color,2));p.drawLine(pos,end)
            p.setPen(color);f=QFont('DejaVu Sans');f.setPixelSize(10);f.setBold(True);p.setFont(f)
            p.drawText(QRectF(pos.x()-12,pos.y()-9,24,18),Qt.AlignCenter,'R'+robot[-1])
        p.setPen(QColor('#b1bda5'));f=QFont('DejaVu Sans Mono');f.setPixelSize(10);p.setFont(f)
        p.drawText(12,self.height()-12,'N ↑    16 × 14 m    /    ICP MAP FRAME')
        if not fleet:
            p.setPen(QColor('#c4ceb8'));p.drawText(QRectF(20,12,self.width()-40,24),Qt.AlignRight,'WAITING FOR LIVE POSES')
        p.end()


class CameraView(QWidget):
    def __init__(self):
        super().__init__();self.setMinimumHeight(175);self.setSizePolicy(QSizePolicy.Expanding,QSizePolicy.Expanding)
        self.image=QImage();self.note='Waiting for camera';self.live=False
        self.setAccessibleName('Selected robot first-person camera')
    def paintEvent(self,e):
        p=QPainter(self);p.setRenderHint(QPainter.SmoothPixmapTransform);p.setRenderHint(QPainter.Antialiasing)
        r=QRectF(self.rect());p.fillRect(r,QColor('#080f0c'))
        if not self.image.isNull():
            size=self.image.size();scale=min(r.width()/size.width(),r.height()/size.height())
            target=QRectF((r.width()-size.width()*scale)/2,(r.height()-size.height()*scale)/2,size.width()*scale,size.height()*scale)
            p.drawImage(target,self.image)
            if not self.live:p.fillRect(r,QColor(3,10,6,115))
            pen=QPen(QColor(215,234,214,85),1);p.setPen(pen)
            x,y=r.center().x(),r.center().y();p.drawLine(QPointF(x-10,y),QPointF(x+10,y));p.drawLine(QPointF(x,y-10),QPointF(x,y+10))
        else:
            p.setPen(QPen(QColor('#3e5542'),1));p.drawRoundedRect(r.adjusted(18,18,-18,-18),8,8)
            p.setPen(QColor('#a9b9a7'));p.drawText(r,Qt.AlignCenter,'NO CAMERA SIGNAL\nStart the simulation with cameras enabled')
        strip=QRectF(0,r.height()-30,r.width(),30);p.fillRect(strip,QColor(7,15,10,210))
        p.setPen(QColor(GREEN if self.live else R2));f=QFont('DejaVu Sans Mono');f.setPixelSize(10);p.setFont(f)
        p.drawText(strip.adjusted(10,0,-10,0),Qt.AlignVCenter,self.note)
        p.end()


class SpeedChart(QWidget):
    def __init__(self):super().__init__();self.setMinimumHeight(150);self.history={}
    def paintEvent(self,e):
        p=QPainter(self);p.setRenderHint(QPainter.Antialiasing)
        r=QRectF(42,12,max(1,self.width()-60),max(1,self.height()-38))
        p.setPen(QPen(QColor('#334534'),1));f=QFont('DejaVu Sans Mono');f.setPixelSize(10);p.setFont(f)
        for value in [0,.15,.3,.45]:
            y=r.bottom()-value/.5*r.height();p.drawLine(QPointF(r.left(),y),QPointF(r.right(),y))
            p.setPen(QColor(MUTED));p.drawText(0,int(y+4),f'{value:.2f}');p.setPen(QPen(QColor('#334534'),1))
        now=time.monotonic()
        for robot,h in self.history.items():
            path=QPainterPath();active=False
            for t,v in h:
                if now-t>120:continue
                if v is None:active=False;continue
                pt=QPointF(r.right()-(now-t)/120*r.width(),r.bottom()-abs(v)/.5*r.height())
                if active:path.lineTo(pt)
                else:path.moveTo(pt);active=True
            p.setPen(QPen(QColor(COLORS[robot]),2));p.drawPath(path)
        p.setPen(QColor(MUTED));p.drawText(int(r.left()),self.height()-3,'−120 s')
        p.drawText(int(r.right()-25),self.height()-3,'now');p.end()


class Dashboard(QMainWindow):
    def __init__(self,share,state,bridge=None):
        super().__init__();self.share,self.state,self.bridge=share,state,bridge
        self.stations=yaml.safe_load((share/'config/stations.yaml').read_text())['stations']
        self.zones=yaml.safe_load((share/'config/zones.yaml').read_text())['zones']
        self.current_robot='robot1';self.camera_key=None;self.camera_error=None;self.last_tables=None;self.last_events=None
        self.snapshot=state.snapshot();self.actions=[]
        self.setWindowTitle('Northline Control · Factory fleet');self.setMinimumSize(1000,680)
        self.setStyleSheet(STYLE)
        root=QWidget();self.setCentralWidget(root);outer=QVBoxLayout(root);outer.setContentsMargins(22,18,22,12);outer.setSpacing(14)
        top=QHBoxLayout();top.setSpacing(12);top.addWidget(Logo())
        brand=QVBoxLayout();brand.setSpacing(1);brand.addWidget(text_label('NORTHLINE',16,TEXT,True));brand.addWidget(text_label('FLEET OPERATIONS',9,MUTED))
        top.addLayout(brand);top.addStretch()
        self.nav=[]
        for i,title in enumerate(['Overview','Mission queue','Diagnostics']):
            b=button(title,lambda checked=False,i=i:self.set_page(i));b.setObjectName('nav');b.setCheckable(True);self.nav.append(b);top.addWidget(b)
        top.addStretch();self.connection=text_label('CONNECTING',10,R2,False,True);top.addWidget(self.connection)
        self.simclock=text_label('--:--:-- SIM',11,MUTED,False,True);top.addWidget(self.simclock)
        outer.addLayout(top)
        line=QFrame();line.setFrameShape(QFrame.HLine);line.setStyleSheet('color: #2b372b;');outer.addWidget(line)
        heading=QHBoxLayout();h=QVBoxLayout();h.setSpacing(3)
        h.addWidget(text_label('Factory operations',29,TEXT));h.addWidget(text_label('Two robots. One coordinated floor.',12,MUTED));heading.addLayout(h);heading.addStretch()
        self.demo_button=button('Run demo jobs',lambda:self.send('demo',{}));self.actions.append(self.demo_button);heading.addWidget(self.demo_button)
        new=button('+  New mission',self.mission_dialog,True);self.actions.append(new);heading.addWidget(new);outer.addLayout(heading)
        self.banner=text_label('',12,R2);self.banner.setWordWrap(True);outer.addWidget(self.banner)
        self.pages=QStackedWidget();outer.addWidget(self.pages,1)
        self.pages.addWidget(self.operations_page());self.pages.addWidget(self.missions_page());self.pages.addWidget(self.diagnostics_page())
        bottom=QHBoxLayout();bottom.addWidget(text_label('NORTHLINE / FACTORY 01',9,MUTED,False,True));bottom.addStretch()
        bottom.addWidget(text_label('STATIC MODEL + LIVE ROS TELEMETRY',9,MUTED,False,True));outer.addLayout(bottom)
        self.set_page(0)
        self.timer=QTimer(self);self.timer.timeout.connect(self.refresh);self.timer.start(100)
        screen=QApplication.primaryScreen().availableGeometry();self.resize(min(1480,screen.width()-30),min(950,screen.height()-35))
        self.refresh()

    def scrolling(self,w):
        s=QScrollArea();s.setWidgetResizable(True);s.setWidget(w)
        s.viewport().setAutoFillBackground(False);w.setAutoFillBackground(False)
        return s

    def set_page(self,i):
        self.pages.setCurrentIndex(i)
        for j,b in enumerate(self.nav):b.setChecked(i==j)
        if self.bridge:self.bridge.set_camera(self.current_robot if i==0 else None)

    def operations_page(self):
        page=QWidget();layout=QHBoxLayout(page);layout.setContentsMargins(0,0,0,0);layout.setSpacing(14)
        left=QWidget();leftbox=QVBoxLayout(left);leftbox.setContentsMargins(0,0,0,0);leftbox.setSpacing(12)
        stage=Card();stage.box.setContentsMargins(0,0,0,12);stage.box.setSpacing(0)
        header=QHBoxLayout();header.setContentsMargins(16,12,12,8)
        header.addWidget(text_label('FACTORY 01',11,TEXT,True));header.addStretch()
        self.modebox=QComboBox();self.modebox.addItems(['Factory model','Floor plan']);self.modebox.setAccessibleName('Factory view mode')
        self.modebox.currentIndexChanged.connect(self.change_mode);header.addWidget(self.modebox)
        reset=button('Reset view',lambda:self.map.reset());header.addWidget(reset);stage.box.addLayout(header)
        self.map=FactoryView(self.share,self.stations,self.zones)
        self.map.collisions=json.loads((self.share/'models/industrial_factory/manifest.json').read_text())['collisions']
        self.map.selected.connect(self.select_robot);stage.box.addWidget(self.map,1)
        legend=QHBoxLayout();legend.setContentsMargins(16,7,16,0)
        for title,c in [('●  ROBOT 01',R1),('●  ROBOT 02',R2),('▰  RESTRICTED',RED),('▰  SLOW ZONE',R2)]:legend.addWidget(text_label(title,9,c,False,True))
        legend.addStretch();stage.box.addLayout(legend);leftbox.addWidget(stage,1)
        metrics=QHBoxLayout();metrics.setSpacing(10);self.metrics={}
        for key,title,suffix in [('delivered','DELIVERED','JOBS'),('queue','IN QUEUE','JOBS'),('moving','ON THE MOVE','/ 2'),('waits','TRAFFIC WAITS','EVENTS')]:
            m=Metric(title,suffix);self.metrics[key]=m;metrics.addWidget(m)
        leftbox.addLayout(metrics)
        robotrow=QHBoxLayout();robotrow.setSpacing(12);self.robot_cards={}
        for robot in ROBOT_NAMES:
            card=Card();card.box.setContentsMargins(11,9,11,9);row=QHBoxLayout();row.setSpacing(6)
            im=QLabel();pix=QPixmap(str(self.share/f'models/dashboard/robot_{"blue" if robot=="robot1" else "amber"}.png'))
            im.setPixmap(pix.scaled(122,88,Qt.KeepAspectRatio,Qt.SmoothTransformation));im.setFixedSize(122,88);row.addWidget(im)
            col=QVBoxLayout();col.setSpacing(5);col.addWidget(text_label('ROBOT 0'+robot[-1],12,COLORS[robot],True))
            status=text_label('Waiting for telemetry',11,MUTED);goal=text_label('No active mission',11,MUTED);col.addWidget(status);col.addWidget(goal)
            inspect=button('Inspect →',lambda checked=False,r=robot:self.select_robot(r));inspect.setStyleSheet('padding: 4px 6px; border: none; background: transparent; text-align: left;')
            col.addWidget(inspect);row.addLayout(col,1);card.box.addLayout(row);robotrow.addWidget(card)
            self.robot_cards[robot]=(card,status,goal)
        leftbox.addLayout(robotrow)
        events=Card('ACTIVITY','LATEST EVENTS');events.box.setSpacing(6);self.activity=[]
        for _ in range(3):
            lab=text_label('—',11,MUTED);lab.setWordWrap(True);events.box.addWidget(lab);self.activity.append(lab)
        leftbox.addWidget(events)
        right=QWidget();right.setMinimumWidth(320);right.setMaximumWidth(430)
        rightbox=QVBoxLayout(right);rightbox.setContentsMargins(0,0,0,0);rightbox.setSpacing(12)
        camcard=Card();camcard.box.setContentsMargins(12,12,12,10);camcard.box.setSpacing(8)
        crow=QHBoxLayout();crow.addWidget(text_label('FIRST-PERSON VIEW',11,MUTED,True));crow.addStretch()
        self.robotbox=QComboBox();self.robotbox.addItems(['Robot 01','Robot 02']);self.robotbox.setAccessibleName('Camera robot')
        self.robotbox.currentIndexChanged.connect(lambda i:self.select_robot(ROBOT_NAMES[i]));crow.addWidget(self.robotbox);camcard.box.addLayout(crow)
        self.camera=CameraView();camcard.box.addWidget(self.camera,1)
        controls=QHBoxLayout();controls.addWidget(text_label('FRONT RGB · 80° FOV',9,MUTED,False,True));controls.addStretch()
        controls.addWidget(button('Expand',self.expand_camera));camcard.box.addLayout(controls);rightbox.addWidget(camcard,1)
        telemetry=Card('ROBOT TELEMETRY');grid=QGridLayout();grid.setHorizontalSpacing(18);grid.setVerticalSpacing(9);self.telemetry={}
        for i,(key,title) in enumerate([('speed','SPEED'),('route','ROUTE LEFT'),('localization','LOCALIZATION'),('clearance','NEAREST RETURN'),('position','MAP POSITION'),('heading','HEADING')]):
            col=QVBoxLayout();col.setSpacing(3);col.addWidget(text_label(title,9,MUTED));v=text_label('—',15,TEXT,False,True);col.addWidget(v);self.telemetry[key]=v;grid.addLayout(col,i//2,i%2)
        telemetry.box.addLayout(grid);self.traffic_line=text_label('Traffic · waiting for coordinator',11,R2);self.traffic_line.setWordWrap(True);telemetry.box.addWidget(self.traffic_line);rightbox.addWidget(telemetry)
        dispatch=Card('DISPATCH A MISSION');form=QGridLayout();form.setSpacing(8)
        self.pickup=QComboBox();self.dropoff=QComboBox();self.priority=QComboBox()
        stations=[s for s in self.stations if not s.startswith('HOME')]
        for combo in [self.pickup,self.dropoff]:combo.addItems(stations)
        self.pickup.setCurrentText('STORES');self.dropoff.setCurrentText('ASSEMBLY_A');self.priority.addItems(['Routine','Urgent'])
        for col,(name,widget) in enumerate([('Pickup',self.pickup),('Destination',self.dropoff)]):
            form.addWidget(text_label(name,11,MUTED),0,col);form.addWidget(widget,1,col);widget.setAccessibleName(name)
        dispatch.box.addLayout(form)
        actionrow=QHBoxLayout();actionrow.setSpacing(8);self.priority.setAccessibleName('Priority');actionrow.addWidget(self.priority)
        send=button('Send mission  →',self.submit_inline,True);self.actions.append(send);actionrow.addWidget(send,1);dispatch.box.addLayout(actionrow)
        self.form_note=text_label('Robot assignment is automatic.',10,MUTED);self.form_note.setWordWrap(True);dispatch.box.addWidget(self.form_note)
        emergency=button('Emergency kit delivery',self.emergency_dialog,danger=True);self.actions.append(emergency);dispatch.box.addWidget(emergency)
        rightbox.addWidget(dispatch);layout.addWidget(left,3);layout.addWidget(right,1)
        return self.scrolling(page)

    def table(self,headers):
        w=QTableWidget(0,len(headers));w.setHorizontalHeaderLabels(headers);w.verticalHeader().hide()
        w.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch);w.setAlternatingRowColors(True)
        w.setEditTriggers(QAbstractItemView.NoEditTriggers);w.setSelectionBehavior(QAbstractItemView.SelectRows)
        w.setMinimumHeight(180);w.setAccessibleName(' / '.join(headers));return w

    def missions_page(self):
        page=QWidget();box=QVBoxLayout(page);box.setContentsMargins(0,0,0,0);box.setSpacing(14)
        active=Card('ACTIVE MISSIONS','LIVE FLEET ASSIGNMENTS');self.active_table=self.table(['ROBOT','JOB','PRIORITY','STATE','DESTINATION','NAVIGATION']);active.box.addWidget(self.active_table);box.addWidget(active)
        waiting=Card('MISSION QUEUE','FLEET SCHEDULER');self.queue_table=self.table(['JOB','PRIORITY','PICKUP','DESTINATION','WAITING']);waiting.box.addWidget(self.queue_table);box.addWidget(waiting,1)
        row=QHBoxLayout();row.addWidget(text_label('Delivery records remain in results/jobs.csv. The queue reflects the running fleet session.',12,MUTED));row.addStretch()
        b=button('New mission',self.mission_dialog,True);self.actions.append(b);row.addWidget(b);box.addLayout(row)
        return self.scrolling(page)

    def diagnostics_page(self):
        page=QWidget();box=QVBoxLayout(page);box.setContentsMargins(0,0,0,0);box.setSpacing(12)
        row=QHBoxLayout();self.diagnostics={}
        for key,title,suffix in [('rtf','REAL-TIME FACTOR','×'),('cpu','VM CPU LOAD','%'),('ram','VM MEMORY','GiB'),('failed','FAILED JOBS','JOBS')]:
            m=Metric(title,suffix);self.diagnostics[key]=m;row.addWidget(m)
        box.addLayout(row)
        chart=Card('MEASURED ROBOT SPEED','ODOMETRY · m/s · LAST 120 STEADY SECONDS');self.chart=SpeedChart();chart.box.addWidget(self.chart);box.addWidget(chart)
        topics=Card('STREAM HEALTH','STEADY CLOCK AGE / RECEIVE RATE');self.topic_table=self.table(['ROS TOPIC','STATUS','AGE','RATE']);topics.box.addWidget(self.topic_table);box.addWidget(topics,1)
        logcard=Card('LOCALIZATION');self.icp_log=text_label('Waiting for localization logs.',12,MUTED);self.icp_log.setWordWrap(True);logcard.box.addWidget(self.icp_log)
        logcard.box.addWidget(text_label('Pose source: ICP + wheel odometry. Battery and production sensors are not simulated.',11,MUTED));box.addWidget(logcard)
        events=Card('EVENT JOURNAL');self.event_table=self.table(['SIM TIME','TYPE','EVENT']);self.event_table.horizontalHeader().setSectionResizeMode(2,QHeaderView.Stretch);events.box.addWidget(self.event_table);box.addWidget(events)
        return self.scrolling(page)

    def change_mode(self,i):
        if hasattr(self,'map'):self.map.mode='model' if i==0 else 'plan';self.map.reset()

    def select_robot(self,robot):
        if robot not in ROBOT_NAMES:return
        self.current_robot=robot;self.camera_key=None;self.camera_error=None
        if self.bridge:self.bridge.set_camera(robot if self.pages.currentIndex()==0 else None)
        if self.robotbox.currentIndex()!=ROBOT_NAMES.index(robot):self.robotbox.setCurrentIndex(ROBOT_NAMES.index(robot))
        self.refresh()

    def send(self,kind,payload):
        if self.bridge is None:
            self.state.event('error','Preview mode · live controls are disabled');return
        try:self.bridge.request(kind,payload)
        except ValueError as e:self.state.event('error',str(e))

    def submit_inline(self):
        try:
            payload=delivery_request(self.pickup.currentText(),self.dropoff.currentText(),self.priority.currentText().lower(),self.stations)
        except ValueError as e:set_text(self.form_note,str(e));return
        self.send('delivery',payload);set_text(self.form_note,'Request queued locally · awaiting fleet confirmation.')

    def mission_dialog(self):
        dialog=QDialog(self);dialog.setWindowTitle('Dispatch a mission');dialog.setMinimumWidth(420)
        box=QVBoxLayout(dialog);box.setContentsMargins(24,24,24,24);box.setSpacing(14)
        box.addWidget(text_label('New delivery mission',23,TEXT));box.addWidget(text_label('The fleet manager selects the robot.',12,MUTED))
        controls={}
        for key,title in [('pickup','Pickup station'),('dropoff','Destination'),('priority','Priority')]:
            box.addWidget(text_label(title,12,MUTED));w=QComboBox();w.setAccessibleName(title)
            w.addItems(['Routine','Urgent'] if key=='priority' else [s for s in self.stations if not s.startswith('HOME')]);box.addWidget(w);controls[key]=w
        controls['pickup'].setCurrentText(self.pickup.currentText());controls['dropoff'].setCurrentText(self.dropoff.currentText())
        note=text_label('',12,RED);note.setWordWrap(True);box.addWidget(note)
        row=QHBoxLayout();row.addWidget(button('Cancel',dialog.reject));send=button('Send mission',primary=True);row.addWidget(send);box.addLayout(row)
        def submit():
            try:p=delivery_request(controls['pickup'].currentText(),controls['dropoff'].currentText(),controls['priority'].currentText().lower(),self.stations)
            except ValueError as e:set_text(note,str(e));return
            self.send('delivery',p);dialog.accept()
        send.clicked.connect(submit);dialog.exec_()

    def emergency_dialog(self):
        d=QDialog(self);d.setWindowTitle('Emergency kit delivery');d.setMinimumWidth(410)
        box=QVBoxLayout(d);box.setContentsMargins(24,24,24,24);box.setSpacing(14)
        box.addWidget(text_label('Dispatch the emergency kit',22,TEXT));note=text_label('Uses the existing emergency preemption policy.\nChoose the station requiring the kit.',12,MUTED);note.setWordWrap(True);box.addWidget(note)
        combo=QComboBox();combo.addItems([s for s in self.stations if not s.startswith('HOME')]);combo.setCurrentText('ASSEMBLY_A');combo.setAccessibleName('Emergency destination');box.addWidget(combo)
        row=QHBoxLayout();row.addWidget(button('Cancel',d.reject));row.addWidget(button('Dispatch kit',lambda:(self.send('emergency',{'station':combo.currentText()}),d.accept()),danger=True));box.addLayout(row);d.exec_()

    def expand_camera(self):
        d=QDialog(self);d.setWindowTitle('First-person view · '+self.current_robot);d.resize(1000,650)
        box=QVBoxLayout(d);view=CameraView();box.addWidget(view)
        timer=QTimer(d)
        def refresh():view.image=self.camera.image;view.note=self.current_robot.upper()+' / '+self.camera.note;view.live=self.camera.live;view.update()
        timer.timeout.connect(refresh);timer.start(100);refresh();d.exec_()

    def fill_table(self,table,rows):
        table.setRowCount(len(rows))
        for i,row in enumerate(rows):
            for j,value in enumerate(row):
                item=QTableWidgetItem(str(value));item.setToolTip(str(value));table.setItem(i,j,item)
            table.setRowHeight(i,44)

    def refresh(self):
        if not hasattr(self,'camera'):return
        s=self.state.snapshot();self.snapshot=s;v=s['values'];now=time.monotonic()
        fresh=age(s,'/fleet/state',now)<3 and not s['error'] and not s['preview']
        for a in self.actions:a.setEnabled(bool(fresh and self.bridge))
        connected=fresh
        self.connection.setStyleSheet('color: '+(GREEN if connected else R2)+';')
        set_text(self.connection,'●  ROS LIVE' if connected else ('DESIGN PREVIEW' if s['preview'] else '○  OFFLINE / STALE'))
        paused=age(s,'/clock',now)>2
        set_text(self.simclock,clock_text(s['sim'])+(' PAUSED' if paused and s['sim'] is not None else ' SIM'))
        message=''
        if s['preview']:message='DESIGN PREVIEW · Sample telemetry for layout review. Live task controls are disabled.'
        elif s['error']:message='ROS connection error · '+s['error']
        elif not fresh:message='Waiting for fresh fleet telemetry · start the simulation and fleet manager. Task controls are disabled.'
        elif paused:message='Simulation clock is not advancing. Showing last received values.'
        self.banner.setVisible(bool(message));set_text(self.banner,message)
        fleet=v.get('/fleet/state',{});traffic=v.get('/traffic/state',{})
        numbers={'delivered':fleet.get('delivered','—'),'queue':len(fleet['queue']) if 'queue' in fleet else '—',
                 'moving':sum(v.get(f'/{r}/nav_status',{}).get('state') in ('DRIVING','ALIGNING') and v.get('/traffic/state',{}).get('robots',{}).get(r,{}).get('cmd','go')!='wait' for r in ROBOT_NAMES) if fresh or s['preview'] else '—',
                 'waits':sum(rec.get('waits',0) for rec in traffic.get('robots',{}).values()) if traffic else '—'}
        for key,value in numbers.items():set_text(self.metrics[key].value,value)
        self.map.set_snapshot(s,self.current_robot)
        for robot,(card,status,goal) in self.robot_cards.items():
            f=fleet.get('robots',{}).get(robot,{});nav=v.get(f'/{robot}/nav_status',{})
            stale=age(s,f'/{robot}/nav_status',now)>3
            set_text(status,('Stale · ' if stale and nav else '')+friendly(f.get('state') or nav.get('state') or 'No telemetry'))
            set_text(goal,(f.get('job') or 'No mission')+(' → '+str(f['target']).replace('_',' ') if f.get('target') else ''))
            selected=robot==self.current_robot
            border=COLORS[robot] if selected else BORDER
            card.setStyleSheet('QFrame#card { background: '+PANEL+'; border: 1px solid '+border+'; border-radius: 13px; }')
        r=self.current_robot;nav=v.get(f'/{r}/nav_status',{});pose=nav.get('pose');odom=v.get(f'/{r}/odom',{})
        scan=v.get(f'/{r}/scan',{});loc=v.get(f'/{r}/localization_ok');locfresh=age(s,f'/{r}/localization_ok',now)<3
        remaining=route_remaining(v.get(f'/{r}/plan',[]),pose) if nav.get('state') in ('DRIVING','BLOCKED','ALIGNING') else None
        telemetry={'speed':f"{abs(odom['v']):.2f} m/s" if odom and age(s,f'/{r}/odom',now)<3 else '—',
                   'route':f'{remaining:.1f} m' if remaining is not None else '—',
                   'localization':('ICP LOCKED' if loc else 'LOST') if locfresh else 'NO UPDATE',
                   'clearance':f"{scan['nearest']:.2f} m" if scan.get('nearest') is not None and age(s,f'/{r}/scan',now)<3 else '—',
                   'position':f'{pose[0]:.2f}, {pose[1]:.2f}' if pose else '—',
                   'heading':f'{math.degrees(pose[2]):.0f}°' if pose else '—'}
        for key,value in telemetry.items():set_text(self.telemetry[key],value)
        self.telemetry['localization'].setStyleSheet('color: '+(GREEN if loc and locfresh else R2)+';')
        c=traffic.get('robots',{}).get(r,{})
        tline='Traffic · '+str(c.get('cmd','No coordinator')).upper()
        if c.get('because'):tline+=' / '+c['because'].upper()
        if c.get('rule'):tline+='\n'+c['rule']
        if age(s,'/traffic/state',now)>3:tline+=' · stale'
        set_text(self.traffic_line,tline)
        topic=f'/{r}/camera/image_raw';frame=v.get(topic);stamp=s['seen'].get(topic)
        if self.camera_key!=(r,stamp):
            self.camera_key=(r,stamp)
            if frame:
                try:
                    rgb=image_rgb(frame['width'],frame['height'],frame['step'],frame['encoding'],frame['data'])
                    self.camera.image=QImage(rgb.data,frame['width'],frame['height'],frame['width']*3,QImage.Format_RGB888).copy()
                    self.camera_error=None
                except (ValueError,KeyError) as e:self.camera.image=QImage();self.camera_error=str(e)
            else:self.camera.image=QImage()
        frameage=age(s,topic,now);self.camera.live=frameage<2 and frame is not None and self.camera_error is None
        self.camera.note=self.camera_error or (f"{'LIVE' if self.camera.live else 'STALE'}  {frame['width']}×{frame['height']}  /  {topic_hz(s,topic,now):.1f} Hz  /  {frameage:.1f}s ago" if frame else 'NO SIGNAL / '+r.upper())
        self.camera.update()
        events=s['events']
        for i,w in enumerate(self.activity):
            if i<len(events):
                ev=events[i];set_text(w,clock_text(ev['sim'])+'  '+ev['message']);w.setStyleSheet('color: '+(R2 if ev['kind'] in ('error','warning') else MUTED)+';')
            else:set_text(w,'Waiting for fleet activity.' if i==0 else '')
        # Tables only change at 1 Hz; camera and overlay can refresh at 10 Hz.
        tick=int(now)
        if self.last_tables!=tick:
            self.last_tables=tick
            rows=[]
            for robot,f in fleet.get('robots',{}).items():rows.append([robot.upper(),f.get('job') or '—',friendly(f.get('priority')),friendly(f.get('state')),friendly(f.get('target')),friendly(f.get('nav'))])
            self.fill_table(self.active_table,rows)
            self.fill_table(self.queue_table,[[j['id'],friendly(j['priority']),friendly(j['pickup']),friendly(j['dropoff']),f"{j['waiting_s']:.0f} s"] for j in fleet.get('queue',[])])
            expected=['/fleet/state','/traffic/state']+[f'/{r}/{t}' for r in ROBOT_NAMES for t in ['nav_status','odom','scan','localization_ok','camera/image_raw']]
            self.fill_table(self.topic_table,[[t,('STANDBY' if t.endswith('/camera/image_raw') and t!=f"/{s.get('active_camera')}/camera/image_raw" else ('LIVE' if age(s,t,now)<3 else 'NO DATA / STALE')),f'{age(s,t,now):.1f}s' if math.isfinite(age(s,t,now)) else '—',f'{topic_hz(s,t,now):.1f} Hz'] for t in expected])
            self.fill_table(self.event_table,[[clock_text(ev['sim']),ev['kind'].upper(),ev['message']] for ev in events])
            logs=[value['message'] for key,value in v.items() if key.startswith('/diagnostics/')]
            set_text(self.icp_log,'\n'.join(logs) if logs else 'Waiting for localization logs.')
            resources=v.get('/system/resources',{});rtf=real_time_factor(s)
            for key,value in {'rtf':f'{rtf:.2f}' if rtf is not None and not paused else '—',
                              'cpu':f"{resources['cpu']:.0f}" if resources.get('cpu') is not None else '—',
                              'ram':f"{resources['ram_used_gb']:.1f} / {resources['ram_total_gb']:.1f}" if resources else '—',
                              'failed':fleet.get('failed','—')}.items():set_text(self.diagnostics[key].value,value)
        self.chart.history=s['speed_history']
        if self.pages.currentIndex()==2:self.chart.update()

    def closeEvent(self,e):
        self.timer.stop()
        if self.bridge:self.bridge.stop();self.bridge.join(timeout=3)
        e.accept()

    def changeEvent(self,e):
        super().changeEvent(e)
        if e.type()==QEvent.WindowStateChange and getattr(self,'bridge',None) and hasattr(self,'pages'):
            self.bridge.set_camera(self.current_robot if not self.isMinimized() and self.pages.currentIndex()==0 else None)


def preview(state,share):
    """Explicitly labeled design fixture. Never publishes ROS messages."""
    state.preview=True;now=time.monotonic();state.clock(138.2,now)
    robots={'robot1':{'state':'TO_DROPOFF','job':'J003','priority':'urgent','target':'QC_LAB','pose':[3.3,7.8,1.0],'nav':'DRIVING'},
            'robot2':{'state':'TO_PICKUP','job':'J004','priority':'routine','target':'TOOL_CRIB','pose':[10.2,6.,3.1],'nav':'DRIVING'}}
    state.update('/fleet/state',{'robots':robots,'queue':[{'id':'J005','priority':'routine','pickup':'STORES','dropoff':'ASSEMBLY_A','waiting_s':12}],
                                 'delivered':8,'failed':0},now)
    state.update('/traffic/state',{'robots':{'robot1':{'cmd':'go','waits':2},'robot2':{'cmd':'wait','because':'robot1','rule':'give way','waits':1}}},now)
    for r in ROBOT_NAMES:
        state.update(f'/{r}/nav_status',{'pose':robots[r]['pose'],'state':'DRIVING'},now)
        state.update(f'/{r}/localization_ok',True,now)
        state.update(f'/{r}/odom',{'v':.23 if r=='robot1' else 0,'w':.1},now)
        state.update(f'/{r}/scan',{'nearest':1.23,'beams':360,'max_range':12},now)
    state.update('/robot1/plan',[(3.3,7.8),(4.7,9.),(7,10.7)],now)
    state.update('/robot2/plan',[(10.2,6.),(7.,6.),(3.7,6.),(.6,4.5)],now)
    state.event('system','Design preview · controls disabled',now)


def main(args=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preview',action='store_true')
    parser.add_argument('--assets',type=Path)
    parser.add_argument('--capture',type=Path,help='Save a dashboard screenshot after capture-after seconds')
    parser.add_argument('--capture-after',type=float,default=8)
    parser.add_argument('--duration',type=float,help='Close after this many steady seconds (for validation)')
    parser.add_argument('--size',help='Optional initial window size, e.g. 1440x960')
    options,ros_args=parser.parse_known_args(args)
    if options.assets:share=options.assets.resolve()
    else:
        try:
            from ament_index_python.packages import get_package_share_directory
            share=Path(get_package_share_directory('factory_bot'))
        except (ImportError,LookupError):share=Path(__file__).resolve().parents[1]
    app=QApplication([sys.argv[0]]);app.setApplicationName('Northline Control')
    font=QFont('DejaVu Sans');font.setPixelSize(13);app.setFont(font)
    palette=QPalette()
    for role,color in [(QPalette.Window,BG),(QPalette.Base,BG),(QPalette.AlternateBase,PANEL),
                       (QPalette.WindowText,TEXT),(QPalette.Text,TEXT),(QPalette.Button,PANEL),
                       (QPalette.ButtonText,TEXT),(QPalette.Highlight,'#465c37'),(QPalette.HighlightedText,TEXT)]:
        palette.setColor(role,QColor(color))
    app.setPalette(palette)
    state=DashboardState();bridge=None
    if options.preview:preview(state,share)
    else:
        from factory_bot.dashboard_ros import RosBridge
        stations=yaml.safe_load((share/'config/stations.yaml').read_text())['stations']
        bridge=RosBridge(state,stations,ros_args);bridge.start()
    window=Dashboard(share,state,bridge)
    if options.size:
        try:w,h=map(int,options.size.lower().split('x'));window.resize(max(1000,w),max(680,h))
        except ValueError:parser.error('--size requires WIDTHxHEIGHT')
    window.show()
    signal.signal(signal.SIGINT,lambda *_:window.close())
    signal.signal(signal.SIGTERM,lambda *_:window.close())
    if options.capture:
        def capture():
            options.capture.parent.mkdir(parents=True,exist_ok=True);window.grab().save(str(options.capture))
        QTimer.singleShot(max(100,int(options.capture_after*1000)),capture)
    if options.duration:QTimer.singleShot(int(options.duration*1000),window.close)
    result=app.exec_()
    if bridge:bridge.stop();bridge.join(timeout=3)
    return result


if __name__=='__main__':raise SystemExit(main())
