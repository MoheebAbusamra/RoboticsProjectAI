"""Saved-map inspection and immutable activation copies. No ROS or Qt dependency."""
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

from factory_bot.map_source import resolve_map_yaml


@dataclass
class SavedMap:
    path: str
    source: str
    meta: dict
    pixels: np.ndarray
    digest: str
    image_bytes: bytes

    def info(self):
        h, w = self.pixels.shape
        p = self.pixels.astype(float) / 255
        if not self.meta.get('negate', 0): p = 1-p
        known = (p > self.meta['occupied_thresh']) | (p < self.meta['free_thresh'])
        return {'path': self.path, 'source': self.source, 'name': Path(self.source).stem,
                'fingerprint': self.digest, 'width': w, 'height': h,
                'resolution': self.meta['resolution'], 'origin': self.meta['origin'],
                'known_percent': round(float(known.mean()*100), 1)}


def inspect_map(value):
    path = Path(resolve_map_yaml(value))
    if path.stat().st_size > 65536: raise ValueError('Map YAML is too large')
    meta = yaml.safe_load(path.read_text())
    if not isinstance(meta, dict): raise ValueError('Map YAML must contain map metadata')
    try:
        res = float(meta['resolution']); origin = [float(v) for v in meta['origin']]
        image = Path(meta['image']).expanduser()
        if not image.is_absolute(): image = path.parent/image
        image = image.resolve(strict=True)
        occupied = float(meta.get('occupied_thresh', .65)); free = float(meta.get('free_thresh', .196))
        negate = int(meta.get('negate', 0))
    except (KeyError, TypeError, OSError, ValueError) as error:
        raise ValueError('Invalid map metadata or missing image: '+str(error)) from error
    if (len(origin) != 3 or not all(math.isfinite(v) for v in [res, *origin, occupied, free])
        or res <= 0 or abs(origin[2]) > 1e-6 or not 0 <= free < occupied <= 1 or negate not in (0, 1)):
        raise ValueError('Use an axis-aligned map with finite origin/resolution and valid thresholds')
    if meta.get('mode', 'trinary') != 'trinary': raise ValueError('Current navigation supports trinary maps')
    if image.suffix.lower() != '.pgm' or image.stat().st_size > 16*1024*1024:
        raise ValueError('Choose a PGM map image smaller than 16 MiB')
    with Image.open(image) as stream:
        if stream.format != 'PPM' or stream.mode != 'L' or stream.width*stream.height > 2_000_000 or min(stream.size) < 2:
            raise ValueError('Use an 8-bit grayscale PGM with supported dimensions')
        pixels = np.asarray(stream, dtype=np.uint8).copy()
    normalized = dict(image='map.pgm', mode='trinary', resolution=res, origin=origin,
                      negate=negate, occupied_thresh=occupied, free_thresh=free)
    image_bytes = b'P5\n%d %d\n255\n' % (pixels.shape[1], pixels.shape[0]) + pixels.tobytes()
    digest = hashlib.sha256(json.dumps(normalized, sort_keys=True).encode()+image_bytes).hexdigest()
    return SavedMap(str(path), str(path), normalized, pixels, digest, image_bytes)


def freeze_map(asset, directory):
    """Copy exactly the inspected image into a content-addressed directory."""
    folder = Path(directory)/asset.digest; folder.mkdir(parents=True, exist_ok=True)
    image = folder/'map.pgm'; target = folder/'map.yaml'
    # Atomic replacements prevent a partial copy from becoming a launch input.
    temp = folder/'map.pgm.tmp'; temp.write_bytes(asset.image_bytes); temp.replace(image)
    temp = folder/'map.yaml.tmp'; temp.write_text(yaml.safe_dump(asset.meta)); temp.replace(target)
    frozen = inspect_map(target)
    if frozen.digest != asset.digest: raise ValueError('Saved map copy failed verification')
    frozen.source = asset.source
    return frozen


def navigation_command(path, robots, algorithm='astar', controller='pure_pursuit', traffic=True,
                       poses=None, profiles=None):
    """Argument vector only: no shell, command interpolation or process execution."""
    return ['ros2', 'launch', 'factory_bot', 'navigation_group.launch.py', 'map:='+str(path),
            'robots:='+','.join(robots), 'algorithm:='+algorithm, 'controller:='+controller,
            'traffic:='+str(bool(traffic)).lower(), 'initial_poses:='+json.dumps(poses or {}),
            'profiles:='+json.dumps(profiles or {})]


def robot_options(raw, robots, kind):
    values = json.loads(raw)
    if not isinstance(values, dict) or set(values)-set(robots): raise ValueError('Unknown robot options')
    for value in values.values():
        if kind == 'poses':
            if not isinstance(value, list) or len(value) != 3 or not all(math.isfinite(float(v)) for v in value):
                raise ValueError('Initial poses require finite x, y, yaw')
            value[:] = [float(v) for v in value]
        elif (not isinstance(value, dict) or set(value) != {'algorithm', 'controller'} or
              value['algorithm'] not in ('astar', 'dijkstra', 'bfs') or value['controller'] not in ('pid', 'pure_pursuit')):
            raise ValueError('Invalid robot navigation defaults')
    return values


def check_robot_poses(grid, poses):
    for name, pose in poses.items():
        if (not pose or len(pose) != 3 or not all(math.isfinite(v) for v in pose) or
            not grid.passable(*grid.world_to_cell(*pose[:2]), allow_restricted=True)):
            raise ValueError('Selected map blocks or does not cover the current pose of '+name)
