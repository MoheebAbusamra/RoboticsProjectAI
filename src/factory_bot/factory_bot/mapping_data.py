"""Measured map datasets, frame conversion and export. No ROS/Qt dependency."""
from collections import OrderedDict
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml

from factory_bot.ogm import OccupancyGridMapper

MODES = ('slam', 'ogm', 'slam_ogm', 'compare')


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0]+c*b[0]-s*b[1], a[1]+s*b[0]+c*b[1],
            math.atan2(math.sin(a[2]+b[2]), math.cos(a[2]+b[2])))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c*a[0]-s*a[1], s*a[0]-c*a[1], -a[2])


class MappingData:
    def __init__(self, mode, anchor, resolution=.05, origin=(-1.3,-1.3), width=16.6, height=14.6):
        if mode not in MODES: raise ValueError('Unknown mapping mode')
        if len(anchor) != 3 or not all(math.isfinite(v) for v in anchor): raise ValueError('Invalid survey anchor')
        self.mode = mode; self.anchor = tuple(anchor); self.res = resolution; self.origin = tuple(origin)
        self.width = round(width/resolution); self.height = round(height/resolution)
        self.odom_origin = None; self.scans = 0; self.pose_pairs = 0; self.truth_pairs = 0
        self.slam = None; self.slam_maps = 0; self.slam_poses = 0; self.last_hybrid_stamp = -1
        self.cache = OrderedDict()
        make = lambda: OccupancyGridMapper(width_m=width,height_m=height,resolution=resolution,
            origin=origin,max_range=12.,sensor_offset=.10)
        self.odom = make() if mode != 'slam' else None
        self.hybrid = make() if mode == 'slam_ogm' else None
        self.truth = make() if mode in ('slam_ogm','compare') else None

    @property
    def uses_slam(self): return self.mode != 'ogm'

    def local_odom(self, pose):
        if self.odom_origin is None: self.odom_origin = tuple(pose)
        return compose(inverse(self.odom_origin), pose)

    def add_scan(self, stamp, scan, odom, truth=None):
        """scan=(ranges, angle_min, increment, range_min, range_max); stamp is nanoseconds."""
        if not all(math.isfinite(v) for v in [*odom, *scan[1:]]): return
        local = self.local_odom(odom); canonical = compose(self.anchor, local)
        self.scans += 1
        if self.odom: self.odom.update(*canonical, *scan)
        if self.truth is not None and truth is not None:
            self.truth.update(*truth, *scan); self.truth_pairs += 1
        self.cache[stamp] = scan
        while len(self.cache) > 400: self.cache.popitem(last=False)

    def add_slam_pose(self, stamp, pose):
        self.slam_poses += 1
        if self.hybrid is None or not self.cache or stamp <= self.last_hybrid_stamp: return False
        nearest = min(self.cache, key=lambda t:abs(t-stamp))
        if abs(nearest-stamp) > 100_000_000: return False
        self.hybrid.update(*compose(self.anchor,pose), *self.cache[nearest])
        for key in list(self.cache):
            if key<=nearest:del self.cache[key]
        self.last_hybrid_stamp = stamp; self.pose_pairs += 1
        return True

    def add_slam_grid(self, values, resolution, origin):
        """Resample the measured SLAM grid into the canonical factory frame, never SDF geometry."""
        if (values.ndim != 2 or values.size > 2_000_000 or not math.isfinite(resolution) or resolution <= 0
                or len(origin)!=3 or not all(math.isfinite(v) for v in origin)):
            raise ValueError('Invalid SLAM occupancy grid')
        xs = self.origin[0]+(np.arange(self.width)+.5)*self.res
        ys = self.origin[1]+(np.arange(self.height)+.5)*self.res
        xx,yy = np.meshgrid(xs,ys)
        def undo(x,y,pose):
            c,s = math.cos(pose[2]),math.sin(pose[2]);x=x-pose[0];y=y-pose[1]
            return c*x+s*y,-s*x+c*y
        lx,ly = undo(xx,yy,self.anchor); gx,gy = undo(lx,ly,origin)
        cols = np.floor(gx/resolution).astype(int); rows = np.floor(gy/resolution).astype(int)
        inside=(rows>=0)&(rows<values.shape[0])&(cols>=0)&(cols<values.shape[1])
        result=np.full((self.height,self.width),-1,dtype=np.int8)
        result[inside]=values[rows[inside],cols[inside]]
        self.slam=result;self.slam_maps+=1

    def datasets(self):
        result={}
        if self.slam is not None: result['slam']=self.slam
        if self.odom is not None: result['ogm_odom']=self.odom.to_occupancy()
        if self.hybrid is not None and self.pose_pairs>=2: result['ogm_slam']=self.hybrid.to_occupancy()
        if self.truth is not None and self.truth_pairs>=2: result['ogm_truth']=self.truth.to_occupancy()
        return result

    def preview(self):
        data=self.datasets()
        key = 'slam' if self.mode in ('slam','compare') else 'ogm_odom' if self.mode=='ogm' else 'ogm_slam'
        if key=='ogm_slam' and key not in data:key='slam' if 'slam' in data else 'ogm_odom'
        return data.get(key), key

    def save(self, directory, session_id):
        folder=Path(directory);folder.mkdir(parents=True,exist_ok=True)
        datasets=self.datasets();outputs=[]
        labels={'slam':'SLAM Toolbox','ogm_odom':'OGM / wheel odometry',
                'ogm_slam':'SLAM + OGM / online SLAM poses','ogm_truth':'OGM / simulator reference'}
        for name, values in datasets.items():
            if not np.any(values>=65) or not np.any((values>=0)&(values<=19)): continue
            pixels=np.full(values.shape,205,dtype=np.uint8)
            pixels[(values>=0)&(values<=19)]=254;pixels[values>=65]=0
            content=b'P5\n%d %d\n255\n' % (self.width,self.height)+pixels[::-1].tobytes()
            image=folder/(name+'.pgm');temporary=image.with_suffix('.pgm.tmp')
            temporary.write_bytes(content);temporary.replace(image)
            meta=dict(image=image.name,mode='trinary',resolution=self.res,origin=[*self.origin,0.],
                negate=0,occupied_thresh=.65,free_thresh=.196)
            path=folder/(name+'.yaml');temporary=path.with_suffix('.yaml.tmp')
            temporary.write_text(yaml.safe_dump(meta));temporary.replace(path)
            outputs.append({'path':str(path.resolve()),'kind':name,'label':labels[name],
                'reference_only':name=='ogm_truth','known_percent':round(float((pixels!=205).mean()*100),1),
                'image_sha256':hashlib.sha256(content).hexdigest()})
        required='slam' if self.mode in ('slam','compare') else 'ogm_odom' if self.mode=='ogm' else 'ogm_slam'
        success=self.scans>=10 and required in [p['kind'] for p in outputs]
        report={'session_id':session_id,'mode':self.mode,'success':success,'maps':outputs,
            'scans':self.scans,'slam_pose_messages':self.slam_poses,'hybrid_pairs':self.pose_pairs,
            'truth_pairs':self.truth_pairs,'anchor':self.anchor,'odom_origin':self.odom_origin,
            'frame':'map = Gazebo world - (1, 1)','primary_kind':required,
            'method':'Measured lidar. Initial coordinate anchor from existing ICP; no SDF rasterization.',
            'hybrid_note':'OGM uses online SLAM poses, not a re-optimized historical trajectory.',
            'comparison':self.compare(datasets),
            'error':'' if success else 'Not enough valid measurements for the requested primary map'}
        temporary=folder/'session.json.tmp';temporary.write_text(json.dumps(report,indent=2)+'\n')
        temporary.replace(folder/'session.json')
        return report

    @staticmethod
    def compare(datasets):
        """Compare occupancy only where both maps classify cells; unknown is never free."""
        result=[];names=list(datasets)
        for i,a in enumerate(names):
            for b in names[i+1:]:
                left,right=datasets[a],datasets[b]
                classified=lambda v:(v>=65)|((v>=0)&(v<=19))
                common=classified(left)&classified(right);count=int(common.sum())
                occupied_a=left>=65;occupied_b=right>=65
                union=int(((occupied_a|occupied_b)&common).sum())
                result.append({'a':a,'b':b,'common_cells':count,
                    'agreement_percent':round(float(((occupied_a==occupied_b)&common).sum())/count*100,2) if count else None,
                    'occupied_iou':round(float(((occupied_a&occupied_b)&common).sum())/union,4) if union else None})
        return result
