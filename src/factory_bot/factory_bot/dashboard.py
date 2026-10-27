"""son3a feh masr | Made in Egypt: native Python/Qt fleet operations dashboard.

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
from PyQt5.QtGui import (QColor, QFont, QFontDatabase, QPalette, QImage, QPainter, QPainterPath, QPen,
                         QPixmap, QPolygonF, QRadialGradient, QLinearGradient)
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QLabel, QPushButton,
                             QFrame, QVBoxLayout, QHBoxLayout, QGridLayout, QComboBox,
                             QStackedWidget, QScrollArea, QDialog, QDialogButtonBox,
                             QTableWidget, QTableWidgetItem, QHeaderView, QAbstractItemView,
                             QSizePolicy, QFileDialog)

from factory_bot.dashboard_state import (DashboardState, ROBOT_NAMES, age, topic_hz,
                                         real_time_factor, project_map, route_remaining,
                                         image_rgb, delivery_request)
from factory_bot.dashboard_navigation import (PLANNERS, CONTROLLERS, SETTINGS_TOPIC, TARGETS,
                                              display_option, navigation_request, settings_consensus)
from factory_bot.dashboard_maps import MapsPanel
from factory_bot.dashboard_pallet import PalletPanel

BRAND='son3a feh masr | Made in Egypt'
BG='#1b1116'; PANEL='#2a1a22'; BORDER='#60424e'; TEXT='#f3e8d2'; MUTED='#c4b19d'
ACCENT='#e7d6b5'; ON_ACCENT='#291820'; GREEN='#b2caa6'; R1='#94cdbd'; R2='#dfb582'; RED='#efa39b'
SCREEN='#170f14'; SCREEN_EDGE='#674650'; SCREEN_TEXT='#f4e9d4'; SCREEN_MUTED='#cdbda7'
MONO_FAMILY='DejaVu Sans Mono'
COLORS={'robot1':R1,'robot2':R2}
MAP_COLORS={'robot1':'#9dd9c5','robot2':'#efbf82'}

STYLE=f'''
QMainWindow, QDialog {{ background: {BG}; color: {TEXT}; }}
QWidget {{ color: {TEXT}; }}
QFrame#card {{ background: {PANEL}; border: 1px solid {BORDER}; border-radius: 6px; }}
QFrame#screenCard {{ background: {SCREEN}; border: 1px solid {SCREEN_EDGE}; border-radius: 6px; }}
QLabel {{ background: transparent; border: none; }}
QPushButton {{ background: {PANEL}; border: 1px solid {BORDER}; border-radius: 5px; padding: 7px 13px; color: {TEXT}; min-height: 20px; }}
QPushButton:hover {{ background: #3b2732; border-color: {TEXT}; }}
QPushButton:pressed {{ background: #4a303d; }}
QPushButton:focus, QComboBox:focus {{ border-color: {ACCENT}; background: #3b2833; }}
QPushButton:disabled {{ color: #aa9296; background: #35242e; border-color: #513844; }}
QPushButton#primary {{ background: {ACCENT}; color: {ON_ACCENT}; border-color: {ACCENT}; font-weight: 600; }}
QPushButton#primary:hover {{ background: #fff0d4; }}
QPushButton#primary:pressed {{ background: #c8b38f; }}
QPushButton#primary:disabled {{ background: #624c53; border-color: #624c53; color: #cbb6a8; }}
QPushButton#danger {{ background: #3b222c; color: {RED}; border-color: #a36769; }}
QPushButton#danger:hover {{ background: #522d3a; }}
QPushButton#nav {{ border-color: transparent; background: transparent; color: {MUTED}; border-radius: 5px; }}
QPushButton#nav:checked {{ color: {ON_ACCENT}; background: {ACCENT}; border-color: {ACCENT}; }}
QPushButton#nav:focus {{ border-color: {TEXT}; }}
QComboBox {{ background: #21141b; border: 1px solid {BORDER}; border-radius: 4px; padding: 6px 10px; min-height: 20px; }}
QComboBox::drop-down {{ width: 25px; border: none; }}
QComboBox QAbstractItemView {{ background: {PANEL}; color: {TEXT}; selection-background-color: {ACCENT}; selection-color: {ON_ACCENT}; border: 1px solid {BORDER}; }}
QFrame#screenCard QComboBox {{ background: #40252d; color: {SCREEN_TEXT}; border-color: #836369; }}
QFrame#screenCard QPushButton {{ background: #40252d; color: {SCREEN_TEXT}; border-color: #836369; }}
QScrollArea {{ background: transparent; border: none; }}
QScrollBar:vertical {{ background: #21151c; width: 8px; }}
QScrollBar::handle:vertical {{ background: #80616b; border-radius: 3px; min-height: 28px; }}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0; }}
QTableWidget {{ background: {PANEL}; alternate-background-color: #32212c; gridline-color: #4d3441; border: none; selection-background-color: {ACCENT}; selection-color: {ON_ACCENT}; }}
QHeaderView::section {{ background: #3c2834; color: {TEXT}; border: none; padding: 12px 8px; font-size: 14px; font-weight: 600; }}
QTableWidget::item {{ padding: 9px; border-bottom: 1px solid #4d3441; }}
QDialogButtonBox {{ background: transparent; }}
QToolTip {{ background: {SCREEN}; color: {TEXT}; border: 1px solid {BORDER}; padding: 6px; }}
'''


def configure_mono_font(share=None):
    """Use the desktop's readable monospace face without loading the pixel font."""
    global MONO_FAMILY
    families=QFontDatabase().families()
    MONO_FAMILY=next((name for name in ('DejaVu Sans Mono','Liberation Mono','Noto Sans Mono')
                      if name in families),QFontDatabase.systemFont(QFontDatabase.FixedFont).family())


def mono_font(size=14,bold=False):
    font=QFont(MONO_FAMILY)
    font.setPixelSize(max(12,round(size)))
    font.setStyleHint(QFont.Monospace);font.setFixedPitch(True)
    font.setWeight(QFont.DemiBold if bold else QFont.Normal)
    font.setStyleStrategy(QFont.PreferAntialias);font.setHintingPreference(QFont.PreferDefaultHinting)
    return font


def text_label(text='', size=13, color=TEXT, bold=False, mono=False):
    w=QLabel(text)
    w.setFont(mono_font(size,bold))
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
            row=QHBoxLayout();row.addWidget(text_label(title,11,MUTED,True,True));row.addStretch()
            if note:row.addWidget(text_label(note,11,MUTED,False,True))
            self.box.addLayout(row)


class GridCanvas(QWidget):
    def paintEvent(self,event):
        p=QPainter(self);p.fillRect(self.rect(),QColor(BG));p.setPen(QPen(QColor('#30202a'),.6))
        for x in range(0,self.width(),28):p.drawLine(x,0,x,self.height())
        for y in range(0,self.height(),28):p.drawLine(0,y,self.width(),y)
        p.end()


class Logo(QWidget):
    def __init__(self):super().__init__();self.setFixedSize(42,42);self.setAccessibleName('son3a feh masr')
    def paintEvent(self,event):
        p=QPainter(self);p.setPen(QPen(QColor(TEXT),1));p.setBrush(QColor(PANEL));p.drawRect(1,1,39,39)
        pattern=['11111','10000','11111','00001','11111']
        for y,row in enumerate(pattern):
            for x,value in enumerate(row):
                if value=='1':p.fillRect(9+x*5,9+y*5,4,4,QColor(RED if y==0 else ACCENT))
        p.end()


class Metric(Card):
    def __init__(self,title,suffix):
        super().__init__();self.box.setSpacing(5);self.box.setContentsMargins(14,11,14,11)
        self.box.addWidget(text_label(title,11,MUTED,True,True))
        row=QHBoxLayout();self.value=text_label('—',30,ACCENT,True,True)
        row.addWidget(self.value);row.addStretch();row.addWidget(text_label(suffix,10,MUTED))
        self.box.addLayout(row)


class FactoryView(QWidget):
    selected=pyqtSignal(str)
    def __init__(self,share,stations,zones):
        super().__init__();self.setMinimumHeight(280);self.setMouseTracking(True)
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
        r=QRectF(self.rect());p.fillRect(r,QColor(SCREEN))
        grad=QRadialGradient(r.center(),r.width()*.7);grad.setColorAt(0,QColor('#392530'));grad.setColorAt(1,QColor(SCREEN))
        p.fillRect(r,grad)
        p.setPen(QPen(QColor(210,184,158,14),1))
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
            p.setBrush(QColor('#4a3536'));p.setPen(QPen(QColor('#c1a58b'),1));p.drawPolygon(QPolygonF(corners))
            # Obstacles come from the unchanged collision manifest, not from a fabricated floor plan.
            for c in self.collisions:
                x,y,z=c['xyz'];sx,sy,h=c.get('size',(0,0,0))
                if z-h/2<=.23<=z+h/2:
                    x-=1;y-=1
                    pts=[point(x+dx*sx/2,y+dy*sy/2) for dx,dy in [(-1,-1),(1,-1),(1,1),(-1,1)]]
                    p.setBrush(QColor('#857466'));p.setPen(QPen(QColor('#d1baa1'),.6));p.drawPolygon(QPolygonF(pts))
            for z in self.zones.values():
                color={'forbidden':'#ef9e8b','slow':'#efbf82','authorized_only':'#9dd9c5'}.get(z['rule'],SCREEN_TEXT)
                fill=QColor(color);fill.setAlpha(34);p.setBrush(fill);p.setPen(QPen(QColor(color),1,Qt.DashLine))
                pts=[point(x,y) for x,y in [(z['x_min'],z['y_min']),(z['x_max'],z['y_min']),
                                           (z['x_max'],z['y_max']),(z['x_min'],z['y_max'])]]
                p.drawPolygon(QPolygonF(pts))
        values=self.snapshot['values'];fleet=values.get('/fleet/state',{})
        if self.routes:
            for robot in ROBOT_NAMES:
                path=values.get(f'/{robot}/plan',[])
                nav=values.get(f'/{robot}/nav_status',{})
                if not path or nav.get('state') not in ('DRIVING','BLOCKED','ALIGNING','RECOVERING'):continue
                pose=nav.get('pose');start=0
                if pose:start=min(range(len(path)),key=lambda i:math.dist(path[i],pose[:2]))
                route=QPainterPath(point(*path[start]))
                for xy in path[start+1:]:route.lineTo(point(*xy))
                glow=QColor(MAP_COLORS[robot]);glow.setAlpha(40);p.setPen(QPen(glow,10));p.drawPath(route)
                pen=QPen(QColor(MAP_COLORS[robot]),2.3,Qt.DashLine);p.setPen(pen);p.drawPath(route)
        if self.labels:
            p.setFont(mono_font(10,True))
            for name,s in self.stations.items():
                if name.startswith('HOME'):continue
                pos=point(s['x'],s['y']);caption=name.replace('_',' ')
                textwidth=p.fontMetrics().horizontalAdvance(caption)
                near_robot=any(rec.get('pose') and math.dist(rec['pose'][:2],(s['x'],s['y']))<.75
                               for rec in fleet.get('robots',{}).values())
                chip=QRectF(pos.x()-textwidth/2-7,pos.y()-(46 if near_robot else 23),textwidth+14,21)
                if near_robot:
                    p.setPen(QPen(QColor('#a68b77'),.7));p.drawLine(QPointF(chip.center().x(),chip.bottom()),pos)
                p.setPen(QPen(QColor('#a68b77'),.6));p.setBrush(QColor(43,23,29,235));p.drawRoundedRect(chip,3,3)
                p.setPen(QColor(SCREEN_TEXT));p.drawText(chip,Qt.AlignCenter,caption)
                p.setPen(Qt.NoPen);p.setBrush(QColor(SCREEN_TEXT));p.drawEllipse(pos,2.5,2.5)
        self.robot_hits={}
        for robot in ROBOT_NAMES:
            nav=values.get(f'/{robot}/nav_status',{});pose=nav.get('pose')
            if not pose:pose=fleet.get('robots',{}).get(robot,{}).get('pose')
            if not pose:continue
            pos=point(*pose[:2]);self.robot_hits[robot]=pos
            stale=age(self.snapshot,f'/{robot}/nav_status')>3
            color=QColor('#ac9b8c' if stale else MAP_COLORS[robot])
            halo=QColor(color);halo.setAlpha(35);p.setPen(Qt.NoPen);p.setBrush(halo);p.drawEllipse(pos,24,24)
            p.setPen(QPen(color,2));p.setBrush(QColor(SCREEN));p.drawEllipse(pos,12,12)
            if robot==self.robot:p.setPen(QPen(color,1,Qt.DashLine));p.setBrush(Qt.NoBrush);p.drawEllipse(pos,19,19)
            toward=point(pose[0]+.6*math.cos(pose[2]),pose[1]+.6*math.sin(pose[2]))
            delta=toward-pos;length=max(.001,math.hypot(delta.x(),delta.y()));end=pos+delta*(27/length)
            p.setPen(QPen(color,2));p.drawLine(pos,end)
            p.setPen(color);p.setFont(mono_font(10,True))
            p.drawText(QRectF(pos.x()-12,pos.y()-9,24,18),Qt.AlignCenter,'R'+robot[-1])
        p.setPen(QColor(SCREEN_MUTED));p.setFont(mono_font(10))
        p.drawText(12,self.height()-12,'N ↑    16 × 14 m    /    ICP MAP FRAME')
        if not fleet:
            p.setPen(QColor(SCREEN_TEXT));p.drawText(QRectF(20,12,self.width()-40,24),Qt.AlignRight,'WAITING FOR LIVE POSES')
        p.end()


class CameraView(QWidget):
    def __init__(self):
        super().__init__();self.setMinimumHeight(155);self.setSizePolicy(QSizePolicy.Expanding,QSizePolicy.Expanding)
        self.image=QImage();self.note='Waiting for camera';self.live=False
        self.setAccessibleName('Selected robot first-person camera')
    def paintEvent(self,e):
        p=QPainter(self);p.setRenderHint(QPainter.SmoothPixmapTransform);p.setRenderHint(QPainter.Antialiasing)
        r=QRectF(self.rect());p.fillRect(r,QColor('#211217'))
        if not self.image.isNull():
            size=self.image.size();scale=min(r.width()/size.width(),r.height()/size.height())
            target=QRectF((r.width()-size.width()*scale)/2,(r.height()-size.height()*scale)/2,size.width()*scale,size.height()*scale)
            p.drawImage(target,self.image)
            if not self.live:p.fillRect(r,QColor(24,10,15,115))
            pen=QPen(QColor(244,233,212,85),1);p.setPen(pen)
            x,y=r.center().x(),r.center().y();p.drawLine(QPointF(x-10,y),QPointF(x+10,y));p.drawLine(QPointF(x,y-10),QPointF(x,y+10))
        else:
            p.setPen(QPen(QColor(SCREEN_EDGE),1));p.drawRoundedRect(r.adjusted(18,18,-18,-18),4,4)
            p.setPen(QColor(SCREEN_MUTED));p.drawText(r.adjusted(20,18,-20,-42),Qt.AlignCenter|Qt.TextWordWrap,'NO CAMERA SIGNAL\nEnable cameras in the simulation')
        strip=QRectF(0,r.height()-30,r.width(),30);p.fillRect(strip,QColor(33,18,23,225))
        p.setPen(QColor('#b7d5b4' if self.live else '#efbf82'));p.setFont(mono_font(10))
        p.drawText(strip.adjusted(10,0,-10,0),Qt.AlignVCenter,self.note)
        p.end()


class SpeedChart(QWidget):
    def __init__(self):super().__init__();self.setMinimumHeight(150);self.history={}
    def paintEvent(self,e):
        p=QPainter(self);p.setRenderHint(QPainter.Antialiasing)
        r=QRectF(42,12,max(1,self.width()-60),max(1,self.height()-38))
        p.setPen(QPen(QColor('#4d3441'),1));p.setFont(mono_font(10))
        for value in [0,.15,.3,.45]:
            y=r.bottom()-value/.5*r.height();p.drawLine(QPointF(r.left(),y),QPointF(r.right(),y))
            p.setPen(QColor(MUTED));p.drawText(0,int(y+4),f'{value:.2f}');p.setPen(QPen(QColor('#4d3441'),1))
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
        configure_mono_font(share);QApplication.instance().setFont(mono_font(14))
        self.stations=yaml.safe_load((share/'config/stations.yaml').read_text())['stations']
        self.zones=yaml.safe_load((share/'config/zones.yaml').read_text())['zones']
        self.current_robot='robot1';self.camera_key=None;self.camera_error=None;self.last_tables=None;self.last_events=None
        self.snapshot=state.snapshot();self.actions=[]
        self.setWindowTitle(BRAND);self.setMinimumSize(1000,680)
        self.setStyleSheet(STYLE)
        root=GridCanvas();self.setCentralWidget(root);outer=QVBoxLayout(root);outer.setContentsMargins(20,16,20,12);outer.setSpacing(12)
        top=QHBoxLayout();top.setSpacing(12);top.addWidget(Logo())
        brand=QVBoxLayout();brand.setSpacing(2);brand.addWidget(text_label('son3a feh masr',20,ACCENT,True));brand.addWidget(text_label('Made in Egypt / FLEET OPERATIONS',10,MUTED,False,True))
        top.addLayout(brand);top.addStretch()
        self.nav=[]
        for i,title in enumerate(['Overview','Mission queue','Maps','Obstacles','Diagnostics']):
            b=button(title,lambda checked=False,i=i:self.set_page(i));b.setObjectName('nav');b.setCheckable(True);b.setShortcut('Alt+'+str(i+1));self.nav.append(b);top.addWidget(b)
        top.addStretch();self.connection=text_label('CONNECTING',10,R2,False,True);top.addWidget(self.connection)
        self.simclock=text_label('--:--:-- SIM',11,MUTED,False,True);top.addWidget(self.simclock)
        outer.addLayout(top)
        line=QFrame();line.setFrameShape(QFrame.HLine);line.setStyleSheet('color: '+BORDER+';');outer.addWidget(line)
        heading=QHBoxLayout();h=QVBoxLayout();h.setSpacing(3)
        h.addWidget(text_label('Factory control',28,TEXT,True));h.addWidget(text_label('Dispatch & monitor / Factory 01',12,MUTED,False,True));heading.addLayout(h);heading.addStretch()
        self.navigation_button=button('Navigation settings',self.navigation_dialog)
        self.navigation_button.setShortcut('Ctrl+,');self.navigation_button.setToolTip('Choose routing and controller defaults for new route legs. Ctrl+,')
        heading.addWidget(self.navigation_button)
        self.demo_button=button('Run demo jobs',lambda:self.send('demo',{}));self.actions.append(self.demo_button);heading.addWidget(self.demo_button)
        new=button('+  New mission',self.mission_dialog,True);new.setShortcut('Ctrl+N');self.actions.append(new);heading.addWidget(new);outer.addLayout(heading)
        self.banner=text_label('',12,R2);self.banner.setWordWrap(True);outer.addWidget(self.banner)
        self.pages=QStackedWidget();outer.addWidget(self.pages,1)
        self.pages.addWidget(self.operations_page());self.pages.addWidget(self.missions_page())
        self.maps_panel=MapsPanel(share,state,self.send,self.stations,self.zones,bool(bridge))
        self.pages.addWidget(self.maps_panel)
        self.pallet_panel=PalletPanel(share,state,self.send,self.stations,self.zones,bool(bridge))
        self.pages.addWidget(self.pallet_panel);self.pages.addWidget(self.diagnostics_page())
        bottom=QHBoxLayout();bottom.addWidget(text_label('son3a feh masr | Made in Egypt',10,MUTED,False,True));bottom.addStretch()
        self.map_badge=text_label('MAP / AWAITING COORDINATOR',9,MUTED,False,True)
        bottom.addWidget(self.map_badge);outer.addLayout(bottom)
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
        legend=QHBoxLayout();legend.setContentsMargins(16,7,16,0);legend.setSpacing(18)
        for title,c in [('●  ROBOT 01',R1),('●  ROBOT 02',R2),('▰  RESTRICTED',RED),('▰  SLOW ZONE',R2)]:legend.addWidget(text_label(title,9,c,False,True))
        legend.addStretch();stage.box.addLayout(legend);leftbox.addWidget(stage,1)
        metrics=QHBoxLayout();metrics.setSpacing(10);self.metrics={}
        for key,title,suffix in [('delivered','DELIVERED','JOBS'),('queue','IN QUEUE','JOBS'),('moving','ON THE MOVE','/ 2'),('waits','TRAFFIC WAITS','EVENTS')]:
            m=Metric(title,suffix);self.metrics[key]=m;metrics.addWidget(m)
        leftbox.addLayout(metrics)
        robotrow=QHBoxLayout();robotrow.setSpacing(12);self.robot_cards={}
        for robot in ROBOT_NAMES:
            card=Card();card.box.setContentsMargins(11,8,11,8);row=QHBoxLayout();row.setSpacing(6)
            im=QLabel();pix=QPixmap(str(self.share/f'models/dashboard/robot_{"blue" if robot=="robot1" else "amber"}.png'))
            im.setPixmap(pix.scaled(122,88,Qt.KeepAspectRatio,Qt.SmoothTransformation));im.setFixedSize(122,88);row.addWidget(im)
            col=QVBoxLayout();col.setSpacing(3);col.addWidget(text_label('ROBOT 0'+robot[-1],12,COLORS[robot],True))
            status=text_label('Waiting for telemetry',11,MUTED);goal=text_label('No active mission',11,MUTED);col.addWidget(status);col.addWidget(goal)
            inspect=button('Inspect →',lambda checked=False,r=robot:self.select_robot(r));inspect.setStyleSheet('padding: 4px 6px; border: none; background: transparent; text-align: left;')
            col.addWidget(inspect);row.addLayout(col,1);card.box.addLayout(row);robotrow.addWidget(card)
            self.robot_cards[robot]=(card,status,goal)
        leftbox.addLayout(robotrow)
        events=Card('ACTIVITY','LATEST EVENTS');events.box.setContentsMargins(14,10,14,10);events.box.setSpacing(4);self.activity=[]
        for _ in range(3):
            lab=text_label('—',11,MUTED);lab.setWordWrap(True);events.box.addWidget(lab);self.activity.append(lab)
        leftbox.addWidget(events)
        right=QWidget();right.setMinimumWidth(320);right.setMaximumWidth(430)
        rightbox=QVBoxLayout(right);rightbox.setContentsMargins(0,0,0,0);rightbox.setSpacing(12)
        camcard=Card();camcard.setObjectName('screenCard');camcard.box.setContentsMargins(12,10,12,10);camcard.box.setSpacing(6)
        crow=QHBoxLayout();crow.addWidget(text_label('FIRST-PERSON VIEW',11,SCREEN_TEXT,True,True));crow.addStretch()
        self.robotbox=QComboBox();self.robotbox.addItems(['Robot 01','Robot 02']);self.robotbox.setAccessibleName('Camera robot')
        self.robotbox.currentIndexChanged.connect(lambda i:self.select_robot(ROBOT_NAMES[i]));crow.addWidget(self.robotbox);camcard.box.addLayout(crow)
        self.camera=CameraView();camcard.box.addWidget(self.camera,1)
        controls=QHBoxLayout();controls.addWidget(text_label('FRONT RGB · 80° FOV',9,SCREEN_MUTED,False,True));controls.addStretch()
        controls.addWidget(button('Expand',self.expand_camera));camcard.box.addLayout(controls);rightbox.addWidget(camcard,1)
        telemetry=Card('ROBOT TELEMETRY');telemetry.box.setContentsMargins(12,8,12,8);telemetry.box.setSpacing(5)
        grid=QGridLayout();grid.setHorizontalSpacing(18);grid.setVerticalSpacing(3);self.telemetry={}
        for i,(key,title) in enumerate([('speed','SPEED'),('route','ROUTE LEFT'),('localization','LOCALIZATION'),('clearance','NEAREST RETURN'),('position','MAP POSITION'),('heading','HEADING')]):
            col=QVBoxLayout();col.setSpacing(1);col.addWidget(text_label(title,9,MUTED));v=text_label('—',15,TEXT,False,True);col.addWidget(v);self.telemetry[key]=v;grid.addLayout(col,i//2,i%2)
        telemetry.box.addLayout(grid);self.traffic_line=text_label('Traffic · waiting for coordinator',11,R2);self.traffic_line.setWordWrap(True);telemetry.box.addWidget(self.traffic_line)
        self.active_navigation=text_label('Active route · no navigation goal',11,MUTED);self.active_navigation.setWordWrap(True);telemetry.box.addWidget(self.active_navigation);rightbox.addWidget(telemetry)
        dispatch=Card('DISPATCH A MISSION');dispatch.box.setContentsMargins(12,8,12,8);dispatch.box.setSpacing(6)
        form=QGridLayout();form.setSpacing(6)
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
        active=Card('ACTIVE MISSIONS','LIVE FLEET ASSIGNMENTS');self.active_table=self.table(['ROBOT','JOB','PRIORITY','STATE','DESTINATION','NAVIGATION','ROUTING','CONTROLLER']);active.box.addWidget(self.active_table);box.addWidget(active)
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

    def navigation_dialog(self):
        d=QDialog(self);d.setWindowTitle('Navigation settings / '+BRAND);d.setMinimumWidth(570)
        box=QVBoxLayout(d);box.setContentsMargins(24,24,24,24);box.setSpacing(16)
        box.addWidget(text_label('NAVIGATION / SESSION DEFAULTS',11,MUTED,True,True))
        box.addWidget(text_label('Choose how the robots move',24,ACCENT,True))
        hint=text_label('Applies to both robots when a new route leg starts. Active routes keep their current settings. Defaults reset when the fleet is restarted.',12,MUTED)
        hint.setWordWrap(True);box.addWidget(hint)
        initial=self.state.snapshot()['values'].get(SETTINGS_TOPIC,{})
        common=settings_consensus(initial.get('nodes',{}))
        controls={};row=QHBoxLayout();row.setSpacing(16)
        for key,title,options in [('algorithm','Routing algorithm',PLANNERS),('controller','Path controller',CONTROLLERS)]:
            col=QVBoxLayout();col.setSpacing(8);col.addWidget(text_label(title,12,TEXT,True))
            combo=QComboBox();combo.setAccessibleName(title);combo.setObjectName('navigation_'+key)
            for label,value in options:combo.addItem(label,value)
            value=common.get(key)
            if value is not None:combo.setCurrentIndex(combo.findData(value))
            elif initial.get('nodes'):
                combo.insertItem(0,'Mixed / choose one',None);combo.setCurrentIndex(0)
            controls[key]=combo;col.addWidget(combo);row.addLayout(col,1)
        box.addLayout(row)
        detail=text_label('A*: heuristic search  /  Dijkstra: path cost  /  BFS: grid steps\nPID: heading correction  /  Pure pursuit: path lookahead',11,MUTED,False,True)
        detail.setWordWrap(True);box.addWidget(detail)
        current=Card('SAMPLE DEFAULTS' if self.state.preview else 'READ FROM ROS');current_values=text_label('',12,TEXT,False,True)
        current_values.setWordWrap(True);current_values.setMinimumHeight(current_values.fontMetrics().lineSpacing()*3+8)
        current.box.addWidget(current_values);box.addWidget(current)
        note=text_label('',12,MUTED);note.setWordWrap(True);note.setAccessibleName('Navigation settings status');box.addWidget(note)
        buttons=QHBoxLayout();buttons.addWidget(button('Close',d.reject));buttons.addStretch()
        apply=button('Apply defaults',primary=True);apply.setObjectName('apply_navigation');buttons.addWidget(apply);box.addLayout(buttons)
        # Keep the common primary styling while providing a stable test/accessibility name.
        apply.setStyleSheet('QPushButton { background: '+ACCENT+'; color: '+ON_ACCENT+'; border-color: '+ACCENT+'; } QPushButton:disabled { background: #624c53; color: #cbb6a8; border-color: #624c53; }')
        submission=[None];form_error=[''];last_submit=[None]
        def refresh():
            snapshot=self.state.snapshot();data=snapshot['values'].get(SETTINGS_TOPIC,{})
            if submission[0] is not None and (snapshot['seen'].get(SETTINGS_TOPIC,0)>submission[0] or time.monotonic()-submission[0]>6):submission[0]=None
            pending=bool(data.get('applying') or submission[0] is not None)
            maps=snapshot['values'].get('/maps/status',{})
            held=snapshot['values'].get('/fleet/state',{}).get('map_paused',False)
            available=bool(self.bridge and not snapshot['preview'] and not maps.get('busy') and not held and data.get('ready') and age(snapshot,SETTINGS_TOPIC)<6)
            apply.setEnabled(available and not pending)
            for combo in controls.values():combo.setEnabled(not pending)
            lines=[]
            for target,keys in TARGETS.items():
                values=data.get('nodes',{}).get(target,{})
                labels=[display_option(values.get(k),PLANNERS if k=='algorithm' else CONTROLLERS) for k in keys]
                lines.append(target+'  /  '+' + '.join(labels))
            set_text(current_values,'\n'.join(lines))
            confirmed=False
            if last_submit[0]:
                request,stamp=last_submit[0]
                confirmed=settings_consensus(data.get('nodes',{}))==request and any(
                    event['kind']=='settings' and event['when']>=stamp for event in snapshot['events'])
            if form_error[0]:message=form_error[0]
            elif snapshot['preview']:message='Design preview / ROS parameter changes are disabled.'
            elif pending:message='Applying and verifying defaults…'
            elif data.get('error'):message=data['error']
            elif available and confirmed:message='Applied and verified. New route legs will use your selection.'
            elif available:message='Current defaults read from all three nodes. Choose options, then apply.'
            else:message='Waiting for parameter services on the fleet manager and both navigators.'
            set_text(note,message);note.setStyleSheet('color: '+(RED if form_error[0] or data.get('error') else GREEN if confirmed and available else MUTED)+';')
        def submit():
            try:
                payload=navigation_request(controls['algorithm'].currentData(),controls['controller'].currentData())
                if self.bridge is None:raise ValueError('Live ROS connection required.')
                self.bridge.request('navigation_settings',payload)
                form_error[0]='';submission[0]=time.monotonic();last_submit[0]=(payload,submission[0]);refresh()
            except ValueError as e:form_error[0]=str(e);refresh()
        def clear_error(*_):form_error[0]='';last_submit[0]=None;refresh()
        for combo in controls.values():combo.currentIndexChanged.connect(clear_error)
        apply.clicked.connect(submit);timer=QTimer(d);timer.timeout.connect(refresh);timer.start(250)
        refresh();d.exec_();timer.stop()

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
        settings=v.get(SETTINGS_TOPIC,{})
        maps=v.get('/maps/status',{});map_busy=maps.get('busy') or v.get('/fleet/state',{}).get('map_paused',False)
        for a in self.actions:a.setEnabled(bool(fresh and self.bridge and not settings.get('applying') and not map_busy))
        self.navigation_button.setEnabled(not bool(map_busy))
        self.maps_panel.refresh(s)
        self.pallet_panel.refresh(s)
        active_map=maps.get('active',{})
        set_text(self.map_badge,('MAP / '+active_map.get('name','UNKNOWN')) if maps.get('verified') and age(s,'/maps/status',now)<2.5 else 'MAP / '+maps.get('phase','UNCONFIRMED'))
        self.map_badge.setToolTip(active_map.get('source','Start the complete demo to verify the active map'))
        common=settings_consensus(settings.get('nodes',{}))
        if settings.get('applying'):profile='Navigation · applying…'
        elif settings.get('ready') and age(s,SETTINGS_TOPIC,now)<6:
            profile='Navigation · '+(display_option(common['algorithm'],PLANNERS) if common['algorithm'] else 'Mixed')
            profile+=' / '+(display_option(common['controller'],CONTROLLERS) if common['controller'] else 'Mixed')
        else:profile='Navigation settings'
        set_text(self.navigation_button,profile)
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
                 'moving':sum((v.get(f'/{r}/nav_status',{}).get('state') in ('DRIVING','ALIGNING') or
                    (v.get(f'/{r}/nav_status',{}).get('state')=='RECOVERING' and v.get(f'/{r}/nav_status',{}).get('recovery',{}).get('phase')=='backing'))
                    and v.get('/traffic/state',{}).get('robots',{}).get(r,{}).get('cmd','go')!='wait' for r in ROBOT_NAMES) if fresh or s['preview'] else '—',
                 'waits':sum(rec.get('waits',0) for rec in traffic.get('robots',{}).values()) if traffic else '—'}
        for key,value in numbers.items():set_text(self.metrics[key].value,value)
        self.map.set_snapshot(s,self.current_robot)
        for robot,(card,status,goal) in self.robot_cards.items():
            f=fleet.get('robots',{}).get(robot,{});nav=v.get(f'/{robot}/nav_status',{})
            stale=age(s,f'/{robot}/nav_status',now)>3
            state_text=('Recovering · '+friendly(nav.get('recovery',{}).get('phase')) if nav.get('state')=='RECOVERING' else
                        friendly(f.get('state') or nav.get('state') or 'No telemetry'))
            set_text(status,('Stale · ' if stale and nav else '')+state_text)
            set_text(goal,(f.get('job') or 'No mission')+(' → '+str(f['target']).replace('_',' ') if f.get('target') else ''))
            selected=robot==self.current_robot
            border=COLORS[robot] if selected else BORDER
            card.setStyleSheet('QFrame#card { background: '+PANEL+'; border: 1px solid '+border+'; border-radius: 6px; }')
        r=self.current_robot;nav=v.get(f'/{r}/nav_status',{});pose=nav.get('pose');odom=v.get(f'/{r}/odom',{})
        active_goal=nav.get('goal') or {}
        set_text(self.active_navigation,'Active route · '+display_option(active_goal.get('algorithm'),PLANNERS)+' / '+
                 display_option(active_goal.get('controller'),CONTROLLERS) if active_goal else 'Active route · no navigation goal')
        scan=v.get(f'/{r}/scan',{});loc=v.get(f'/{r}/localization_ok');locfresh=age(s,f'/{r}/localization_ok',now)<3
        remaining=route_remaining(v.get(f'/{r}/plan',[]),pose) if nav.get('state') in ('DRIVING','BLOCKED','ALIGNING','RECOVERING') else None
        telemetry={'speed':f"{odom['v']:.2f} m/s" if odom and age(s,f'/{r}/odom',now)<3 else '—',
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
        if nav.get('state')=='RECOVERING':
            recovery=nav.get('recovery',{})
            set_text(self.active_navigation,f"Recovery · {friendly(recovery.get('phase'))} / {recovery.get('travelled_m',0):.2f} m\n{recovery.get('reason','')}")
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
            for robot,f in fleet.get('robots',{}).items():
                goal=v.get(f'/{robot}/nav_status',{}).get('goal') or {}
                rows.append([robot.upper(),f.get('job') or '—',friendly(f.get('priority')),friendly(f.get('state')),friendly(f.get('target')),friendly(f.get('nav')),
                             display_option(goal.get('algorithm'),PLANNERS) if goal else '—',display_option(goal.get('controller'),CONTROLLERS) if goal else '—'])
            self.fill_table(self.active_table,rows)
            self.fill_table(self.queue_table,[[j['id'],friendly(j['priority']),friendly(j['pickup']),friendly(j['dropoff']),f"{j['waiting_s']:.0f} s"] for j in fleet.get('queue',[])])
            expected=['/fleet/state','/traffic/state']+[f'/{r}/{t}' for r in ROBOT_NAMES for t in ['nav_status','odom','scan','rear_obstacle_scan','localization_ok','camera/image_raw']]
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
        state.update(f'/{r}/nav_status',{'pose':robots[r]['pose'],'state':'DRIVING','goal':{'algorithm':'astar','controller':'pure_pursuit'}},now)
        state.update(f'/{r}/localization_ok',True,now)
        state.update(f'/{r}/odom',{'v':.23 if r=='robot1' else 0,'w':.1},now)
        state.update(f'/{r}/scan',{'nearest':1.23,'beams':360,'max_range':12},now)
    state.update('/robot1/plan',[(3.3,7.8),(4.7,9.),(7,10.7)],now)
    state.update('/robot2/plan',[(10.2,6.),(7.,6.),(3.7,6.),(.6,4.5)],now)
    state.update(SETTINGS_TOPIC,{'nodes':{name:{key:'astar' if key=='algorithm' else 'pure_pursuit' for key in keys} for name,keys in TARGETS.items()},'ready':True,'applying':False,'error':''},now)
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
    app=QApplication([sys.argv[0]]);app.setApplicationName(BRAND)
    configure_mono_font(share);app.setFont(mono_font(14))
    palette=QPalette()
    for role,color in [(QPalette.Window,BG),(QPalette.Base,BG),(QPalette.AlternateBase,PANEL),
                       (QPalette.WindowText,TEXT),(QPalette.Text,TEXT),(QPalette.Button,PANEL),
                       (QPalette.ButtonText,TEXT),(QPalette.Highlight,ACCENT),(QPalette.HighlightedText,ON_ACCENT)]:
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
