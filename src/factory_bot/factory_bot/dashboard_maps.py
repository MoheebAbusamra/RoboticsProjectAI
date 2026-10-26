"""Native saved-map panel. Preview is local; activation is acknowledged by the coordinator."""
from pathlib import Path
import math
import time
import uuid

import numpy as np
from PyQt5.QtCore import Qt, QRectF, QPointF
from PyQt5.QtGui import QColor, QImage, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import (QWidget, QFrame, QLabel, QPushButton, QComboBox, QFileDialog,
                            QVBoxLayout, QHBoxLayout)

from factory_bot.map_catalog import inspect_map
from factory_bot.dashboard_navigation import SETTINGS_TOPIC


class OccupancyView(QWidget):
    def __init__(self, stations, zones):
        super().__init__(); self.setMinimumSize(420, 330)
        self.asset = None; self.image = None; self.stations = stations; self.zones = zones; self.robots = {}
        self.trajectory = []

    def set_asset(self, asset):
        self.asset = asset
        p = asset.pixels.astype(float)/255
        if not asset.meta['negate']: p = 1-p
        rgb = np.full((*p.shape, 3), (87, 62, 71), dtype=np.uint8)
        rgb[p < asset.meta['free_thresh']] = (42, 26, 34)
        rgb[p > asset.meta['occupied_thresh']] = (231, 214, 181)
        self.image = QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.shape[1]*3, QImage.Format_RGB888).copy()
        self.update()

    def paintEvent(self, event):
        p = QPainter(self); p.fillRect(self.rect(), QColor('#170f14'))
        if self.image is None: return
        w, h = self.image.width(), self.image.height()
        scale = min((self.width()-48)/w, (self.height()-48)/h)
        rect = QRectF((self.width()-w*scale)/2, (self.height()-h*scale)/2, w*scale, h*scale)
        p.drawImage(rect, self.image)
        ox, oy, _ = self.asset.meta['origin']; res = self.asset.meta['resolution']
        def point(x, y): return QPointF(rect.left()+(x-ox)/res*scale, rect.bottom()-(y-oy)/res*scale)
        p.setClipRect(rect); p.setRenderHint(QPainter.Antialiasing)
        if self.trajectory:
            p.setPen(QPen(QColor('#94cdbd'),2));p.drawPolyline(QPolygonF([point(*v) for v in self.trajectory]))
        for z in self.zones.values():
            color = {'forbidden': '#efa39b', 'slow': '#dfb582', 'authorized_only': '#94cdbd'}[z['rule']]
            p.setPen(QPen(QColor(color), 1, Qt.DashLine)); p.setBrush(Qt.NoBrush)
            p.drawRect(QRectF(point(z['x_min'], z['y_max']), point(z['x_max'], z['y_min'])))
        for name, station in self.stations.items():
            pos = point(station['x'], station['y']); p.setPen(QColor('#c4b19d'))
            if not rect.contains(pos): continue
            p.setClipping(False); p.drawEllipse(pos, 3, 3)
            offset = 15 if name == 'HOME_1' else 29 if name == 'HOME_2' else -5
            p.drawText(pos+QPointF(6, offset), name.replace('_', ' '))
        p.setClipRect(rect)
        for name, pose in self.robots.items():
            pos = point(*pose[:2]); p.setPen(Qt.NoPen)
            p.setBrush(QColor('#94cdbd' if name == 'robot1' else '#dfb582')); p.drawEllipse(pos, 6, 6)
            if len(pose)>2:
                p.setPen(QPen(QColor('#f3e8d2'),2));p.drawLine(pos,point(pose[0]+.35*math.cos(pose[2]),pose[1]+.35*math.sin(pose[2])))


class MapsPanel(QWidget):
    def __init__(self, share, state, send, stations, zones, live):
        super().__init__(); self.share = share; self.state = state; self.send = send; self.live = live
        self.asset = None; self.error = ''; self.last_recent = None; self.pending = None; self.displayed_active = None
        box = QVBoxLayout(self); box.setContentsMargins(0, 0, 0, 0); box.setSpacing(12)
        self.active = QLabel('Active map / waiting for coordinator'); self.active.setWordWrap(True)
        self.active.setTextFormat(Qt.PlainText)
        self.active.setAccessibleName('Active map'); box.addWidget(self.active)
        row = QHBoxLayout(); row.setSpacing(14)
        left = QFrame(); left.setObjectName('card'); col = QVBoxLayout(left)
        col.addWidget(QLabel('OCCUPANCY PREVIEW / SELECTED FILE'))
        self.view = OccupancyView(stations, zones); col.addWidget(self.view, 1)
        col.addWidget(QLabel('Cream: occupied     Burgundy: free     Mauve: unknown'))
        col.addWidget(QLabel('Dashed areas: existing hazard, slow and authorized-zone policies'))
        self.stats = QLabel('Choose a map to inspect'); self.stats.setWordWrap(True); col.addWidget(self.stats)
        row.addWidget(left, 3)
        right = QFrame(); right.setObjectName('card'); right.setMinimumWidth(285); right.setMaximumWidth(400)
        controls = QVBoxLayout(right); controls.setSpacing(12)
        controls.addWidget(QLabel('SAVED MAPS'))
        self.generate_button = QPushButton('Generate map / Mapping studio')
        self.generate_button.clicked.connect(self.open_generation); controls.addWidget(self.generate_button)
        self.recent = QComboBox(); self.recent.setAccessibleName('Saved maps')
        self.recent.activated.connect(self.choose_recent); controls.addWidget(self.recent)
        self.browse_button = QPushButton('Browse map YAML…'); self.browse_button.clicked.connect(self.browse)
        controls.addWidget(self.browse_button)
        self.selected = QLabel(''); self.selected.setWordWrap(True); self.selected.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.selected.setTextFormat(Qt.PlainText)
        controls.addWidget(self.selected)
        self.use_button = QPushButton('Use this map'); self.use_button.setObjectName('primary')
        self.use_button.setAccessibleName('Use this map'); self.use_button.clicked.connect(self.activate)
        controls.addWidget(self.use_button)
        self.restore_button = QPushButton('Restore generated map'); self.restore_button.clicked.connect(self.restore)
        controls.addWidget(self.restore_button)
        self.note = QLabel(''); self.note.setWordWrap(True); self.note.setAccessibleName('Map activation status')
        self.note.setTextFormat(Qt.PlainText)
        controls.addWidget(self.note)
        hint = QLabel('Switching requires both robots idle and stopped.\n\nThe queue stays in memory. Navigation reloads together; deliveries resume after localization is verified.\n\nMaps must use the factory frame: Gazebo world minus (1, 1).')
        hint.setWordWrap(True); hint.setStyleSheet('color: #c4b19d;'); controls.addWidget(hint)
        controls.addStretch(); row.addWidget(right, 1); box.addLayout(row, 1)
        self.choose(str((share/'maps/factory_map.yaml').resolve()))

    def choose(self, path):
        try:
            asset = inspect_map(path); self.view.set_asset(asset); self.asset = asset; self.error = ''
            info = asset.info(); self.selected.setText('Selected file\n'+asset.source)
            self.stats.setText(f"{info['width']} × {info['height']} cells  /  {info['resolution']:.3f} m per cell\n"
                f"Known space {info['known_percent']:.1f}%  /  Origin {info['origin'][:2]}")
        except (OSError, ValueError, TypeError, KeyError) as error:
            self.asset = None; self.view.asset = None; self.view.image = None; self.view.update()
            self.stats.setText('Preview unavailable'); self.error = str(error); self.selected.setText('Could not load '+str(path))
        self.refresh(self.state.snapshot())

    def open_generation(self):
        from factory_bot.dashboard_generation import GenerationDialog
        self.generation_dialog = GenerationDialog(self); self.generation_dialog.show()

    def browse(self):
        path, _ = QFileDialog.getOpenFileName(self, 'Choose a saved occupancy map',
            str(Path(self.asset.source).parent if self.asset else Path.home()), 'ROS map YAML (*.yaml *.yml)')
        if path: self.choose(path)

    def choose_recent(self, index):
        path = self.recent.itemData(index)
        if path: self.choose(path)

    def submit(self, action):
        if not self.live or self.state.preview: return
        request = {'action': action, 'id': uuid.uuid4().hex}
        if action == 'activate':
            if self.asset is None: return
            request.update(path=self.asset.source, fingerprint=self.asset.digest)
        self.pending = (request['id'], time.monotonic()); self.error = ''
        self.send('map_activate', request); self.refresh(self.state.snapshot())

    def activate(self): self.submit('activate')
    def restore(self): self.submit('restore')

    def refresh(self, snapshot):
        now = time.monotonic(); status = snapshot['values'].get('/maps/status', {})
        fresh = now-snapshot['seen'].get('/maps/status', -float('inf')) < 2.5
        if self.pending:
            identifier, submitted = self.pending
            acknowledged = identifier in (status.get('received_request_id'), status.get('request_id'))
            if acknowledged and not status.get('busy'): self.pending = None
            elif now-submitted > 10 and not acknowledged:
                self.error = 'No activation acknowledgement; check the coordinator connection'; self.pending = None
        active = status.get('active', {})
        confirmed = status.get('verified') and fresh
        source = active.get('source', 'Awaiting map coordinator')
        label = 'Generated reference' if source == status.get('generated') else 'Saved map'
        self.active.setText(('ACTIVE / ' if confirmed else 'LAST CONFIRMED / ' if active else '')+label+'\n'+source+
            ('\nLoaded copy: '+active['path'] if active.get('path') and active['path'] != source else ''))
        self.active.setStyleSheet('color: '+('#b2caa6' if confirmed else '#dfb582')+';')
        paths = list(dict.fromkeys([status.get('generated')]+status.get('recent', [])))
        paths = [p for p in paths if p]
        if paths != self.last_recent:
            self.last_recent = paths; self.recent.clear()
            for path in paths:
                self.recent.addItem(('Generated reference' if path == status.get('generated') else Path(path).stem), path)
                self.recent.setItemData(self.recent.count()-1, path, Qt.ToolTipRole)
        settings_busy = snapshot['values'].get(SETTINGS_TOPIC, {}).get('applying')
        enabled = bool(self.live and not snapshot['preview'] and fresh and status.get('can_activate') and not self.pending and not settings_busy)
        same = bool(self.asset and self.asset.digest == active.get('fingerprint')
                    and self.asset.source == active.get('source') and confirmed)
        self.use_button.setEnabled(enabled and self.asset is not None and not same)
        self.restore_button.setEnabled(enabled and (not confirmed or source != status.get('generated')))
        if snapshot['preview']: message = 'Design preview / map activation is disabled.'
        elif self.error: message = self.error
        elif not fresh: message = 'Coordinator offline. Start a fresh complete demo to enable map activation.'
        elif self.pending: message = status.get('message') if status.get('busy') else 'Request sent; awaiting acknowledgement…'
        elif status.get('request_error'): message = status['request_error']
        elif status.get('busy'): message = status.get('message', 'Activating map…')
        elif status.get('error'): message = status.get('message', '')+'\n'+status['error']
        elif settings_busy: message = 'Wait for navigation defaults to finish applying.'
        elif not status.get('can_activate'): message = status.get('idle_reason', 'Wait for the fleet to become idle')
        elif same: message = 'Previewed map content is already active.'
        else: message = 'Inspect the preview, then activate. The generated map remains available.'
        self.note.setText(message)
        self.note.setStyleSheet('color: '+('#efa39b' if self.error or status.get('request_error') or status.get('error') else '#c4b19d')+';')
        self.view.robots = {r: v['pose'] for r in ('robot1', 'robot2')
            if (v := snapshot['values'].get('/'+r+'/nav_status', {})).get('pose')
            and now-snapshot['seen'].get('/'+r+'/nav_status', -float('inf')) < 2.5}
        self.view.update()
