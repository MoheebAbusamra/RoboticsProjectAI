"""Pallet grid and collision checks shared by the dashboard and simulator adapter."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from factory_bot.map_catalog import inspect_map


def rectangle(x, y, bounds):
    a,b,c,d=bounds
    return [(x+a,y+c),(x+b,y+c),(x+b,y+d),(x+a,y+d)]


def hull(points):
    points=sorted(set(points))
    def cross(o,a,b):return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    lower=[];upper=[]
    for p in points:
        while len(lower)>=2 and cross(lower[-2],lower[-1],p)<=0:lower.pop()
        lower.append(p)
    for p in reversed(points):
        while len(upper)>=2 and cross(upper[-2],upper[-1],p)<=0:upper.pop()
        upper.append(p)
    return lower[:-1]+upper[:-1]


def point_segment(p,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1];length=dx*dx+dy*dy
    t=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/length)) if length else 0.
    return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)


def polygon_distance(a,b):
    """Distance between convex polygons; overlap or touching returns zero."""
    separate=False
    for polygon in (a,b):
        for p,q in zip(polygon,polygon[1:]+polygon[:1]):
            nx,ny=-(q[1]-p[1]),q[0]-p[0]
            pa=[nx*x+ny*y for x,y in a];pb=[nx*x+ny*y for x,y in b]
            if max(pa)<min(pb) or max(pb)<min(pa):separate=True
    if not separate:return 0.
    return min(point_segment(p,u,v) for first,second in ((a,b),(b,a)) for p in first
               for u,v in zip(second,second[1:]+second[:1]))


def primitive_bounds(model):
    """Conservative planar bounds of the existing SDF primitive collisions, in model frame."""
    points=[]
    def pose(element):
        values=[float(v) for v in (element.findtext('pose') or '0 0 0 0 0 0').split()]
        if len(values)!=6 or abs(values[3])+abs(values[4])>1e-6:raise ValueError('Unsupported tilted collision')
        return values[0],values[1],values[5]
    def transform(vertices,where):
        x,y,yaw=where;c,s=math.cos(yaw),math.sin(yaw)
        return [(x+c*a-s*b,y+s*a+c*b) for a,b in vertices]
    for link in model.findall('link'):
        for collision in link.findall('collision'):
            geom=collision.find('geometry')
            if geom.find('box') is not None:
                x,y,_=[float(v) for v in geom.findtext('box/size').split()];bounds=(-x/2,x/2,-y/2,y/2)
            elif geom.find('cylinder') is not None:
                r=float(geom.findtext('cylinder/radius'));bounds=(-r,r,-r,r)
            elif geom.find('sphere') is not None:
                r=float(geom.findtext('sphere/radius'));bounds=(-r,r,-r,r)
            else:raise ValueError('Pallet placement requires primitive mover collision envelopes')
            points.extend(transform(transform(rectangle(0,0,bounds),pose(collision)),pose(link)))
    if not points:raise ValueError('Missing collision envelope for '+str(model.get('name')))
    return (min(p[0] for p in points),max(p[0] for p in points),min(p[1] for p in points),max(p[1] for p in points))


class PlacementPolicy:
    def __init__(self,asset,config,bounds,offset=(1.,1.),step=.25,route_margin=.15,wall_margin=.05,
                 stations=None,station_half_size=.30,station_margin=.15):
        self.asset=asset;self.offset=offset;self.step=step;self.route_margin=route_margin;self.wall_margin=wall_margin
        self.bounds=bounds['pallet_1'];self.res=asset.meta['resolution'];self.origin=asset.meta['origin'][:2]
        self.height,self.width=asset.pixels.shape
        probability=asset.pixels.astype(float)/255
        if not asset.meta['negate']:probability=1-probability
        self.blocked=(probability>=asset.meta['free_thresh'])[::-1].copy()
        self.station_margin=station_margin
        self.station_areas=[(name,rectangle(float(s['x']),float(s['y']),
            (-station_half_size,station_half_size,-station_half_size,station_half_size)))
            for name,s in (stations or {}).items()]
        self.corridors=[];self.routes={}
        for name,spec in config['movers'].items():
            path=[(float(x)-offset[0],float(y)-offset[1]) for x,y in spec['route']];self.routes[name]=path
            if len(path)==1:path=path*2
            for a,b in zip(path,path[1:]):
                yaw=math.atan2(b[1]-a[1],b[0]-a[0])
                for heading in (yaw,yaw+math.pi):
                    c,s=math.cos(heading),math.sin(heading)
                    vertices=[(c*x-s*y,s*x+c*y) for x,y in rectangle(0,0,bounds[name])]
                    swept=hull([(p[0]+x,p[1]+y) for p in (a,b) for x,y in vertices])
                    self.corridors.append((name,swept))

    @classmethod
    def from_share(cls,share,config=None):
        share=Path(share);config=config or yaml.safe_load((share/'config/obstacles.yaml').read_text())
        names=set(config['movers'])|{'pallet_1'};world=ET.parse(share/'worlds/factory.sdf').getroot()
        bounds={m.get('name'):primitive_bounds(m) for m in world.findall('./world/model') if m.get('name') in names}
        if set(bounds)!=names:raise ValueError('Missing factory mover collision models')
        stations=yaml.safe_load((share/'config/stations.yaml').read_text())['stations']
        return cls(inspect_map(str(share/'maps/factory_map.yaml')),config,bounds,stations=stations)

    def snap(self,x,y):
        try:
            if isinstance(x,bool) or isinstance(y,bool):raise ValueError()
            x,y=float(x),float(y)
            if not math.isfinite(x) or not math.isfinite(y):raise ValueError()
            return tuple(round(math.floor(v/self.step+.5)*self.step,6) for v in (x,y))
        except (TypeError,ValueError,OverflowError):raise ValueError('Choose finite X and Y coordinates')

    def validate(self,x,y,robots=None):
        x,y=self.snap(x,y);polygon=rectangle(x,y,self.bounds);margin=self.wall_margin
        lo_x=min(p[0] for p in polygon)-margin;hi_x=max(p[0] for p in polygon)+margin
        lo_y=min(p[1] for p in polygon)-margin;hi_y=max(p[1] for p in polygon)+margin
        ox,oy=self.origin
        if lo_x<ox or lo_y<oy or hi_x>=ox+self.width*self.res or hi_y>=oy+self.height*self.res:
            return False,'The whole pallet must stay inside the factory floor',(x,y)
        for name,area in self.station_areas:
            if polygon_distance(polygon,area)<=self.station_margin+1e-8:
                return False,'Keep the '+name+' station docking area clear',(x,y)
        r0=int(math.floor((lo_y-oy)/self.res));r1=int(math.floor((hi_y-oy)/self.res))+1
        c0=int(math.floor((lo_x-ox)/self.res));c1=int(math.floor((hi_x-ox)/self.res))+1
        if self.blocked[r0:r1,c0:c1].any():return False,'Pallet overlaps a wall, machine or unknown floor',(x,y)
        for name,swept in self.corridors:
            if polygon_distance(polygon,swept)<=self.route_margin+1e-8:
                return False,'Keep clear of the full '+name+' route',(x,y)
        for name,pose in (robots or {}).items():
            # A conservative 0.6 m robot envelope and another 0.15 m placement gap.
            if polygon_distance(polygon,rectangle(pose[0],pose[1],(-.30,.30,-.30,.30)))<=.15:
                return False,'Pallet is too close to '+name,(x,y)
        return True,'Valid grid position',(x,y)
