"""Bounded reverse recovery and rear swept-space checks. No ROS dependency."""
import math

import numpy as np


def reverse_path(pose, distance, step=.025):
    count=max(1,math.ceil(distance/step))
    return [(pose[0]-distance*i/count*math.cos(pose[2]),
             pose[1]-distance*i/count*math.sin(pose[2])) for i in range(count+1)]


def peer_blocks_reverse(pose, distance, peer_xy, radius=.30):
    """Disc versus newly swept rear rectangle; a robot touching the front may be escaped."""
    dx,dy=peer_xy[0]-pose[0],peer_xy[1]-pose[1];c,s=math.cos(pose[2]),math.sin(pose[2])
    u,v=dx*c+dy*s,-dx*s+dy*c
    gap_x=max(-.19-distance-.08-u,0.,u+.19)
    gap_y=max(abs(v)-.235,0.)
    return math.hypot(gap_x,gap_y)<=radius


def rear_scan_clear(ranges, angle_min, angle_increment, range_min, range_max,
                    distance, half_length=.19, half_width=.195, margin=.04,
                    laser_x=-.205, stop_buffer=.08, grid=None, pose=None, obstacle_padding=.28):
    """Check a full-width rear corridor, not just the centre beam.

    Angles are in the rear sensor frame (+X points backwards). +inf is a
    legitimate no-return; missing coverage, NaN/zero/too-close returns are unknown.
    """
    values=np.asarray(ranges,dtype=float)
    if not all(math.isfinite(v) for v in [angle_min,angle_increment,range_min,range_max,distance]):
        return False,'rear scan metadata invalid'
    if len(values)<31 or angle_increment<=0 or range_max<distance+stop_buffer+.3:
        return False,'rear scan coverage unavailable'
    end=angle_min+angle_increment*(len(values)-1)
    if angle_min>-.5*math.pi+.02 or end<.5*math.pi-.02:
        return False,'rear scan does not cover the corridor'
    if np.any(np.isnan(values)|np.isneginf(values)|(values<=range_min)):
        return False,'rear scan invalid or obstacle inside minimum range'
    angles=angle_min+angle_increment*np.arange(len(values))
    hit=np.isfinite(values)&(values<range_max)
    r,a=values[hit],angles[hit]
    x=laser_x-r*np.cos(a);y=-r*np.sin(a)
    mapped=np.zeros(len(r),dtype=bool)
    if grid is not None and pose is not None and len(r):
        c,s=math.cos(pose[2]),math.sin(pose[2])
        wx=pose[0]+c*x-s*y;wy=pose[1]+s*x+c*y
        cols=np.floor((wx-grid.origin[0])/grid.res).astype(int)
        rows=np.floor((wy-grid.origin[1])/grid.res).astype(int)
        for dr in [-1,0,1]:
            for dc in [-1,0,1]:
                rr,cc=rows+dr,cols+dc
                inside=(rr>=0)&(rr<grid.height)&(cc>=0)&(cc<grid.width)
                mapped[inside]|=grid.occupied[rr[inside],cc[inside]]
    # A worker's physics cylinder is wider than its visible feet at this scan
    # height. Pad unmapped returns to cover that known envelope discrepancy.
    # Precisely mapped walls use the physical swept rectangle instead, avoiding
    # double inflation that would prevent retreat along a narrow aisle.
    padding=np.where(mapped,0.,obstacle_padding)
    far=-half_length-distance-stop_buffer;near=-half_length
    gap_x=np.maximum(np.maximum(far-x,x-near),0.)
    gap_y=np.maximum(np.abs(y)-(half_width+margin),0.)
    blocked=np.hypot(gap_x,gap_y)<=padding
    return (False,'rear obstacle in reverse corridor') if blocked.any() else (True,'rear clear')


def rear_map_clear(grid, pose, distance, allow_restricted=False,
                   half_length=.19, half_width=.195, margin=.04, stop_buffer=.08):
    """Check newly swept rear space against raw map cells and policy zones.

    The existing front footprint is deliberately excluded: an obstacle already
    touching the front must not forbid retreating away from it. Circular planner
    inflation is not a physical collision and cannot be used as an escape veto.
    Cell overlap uses the separating axes of the swept rectangle and map cells.
    """
    c,s=math.cos(pose[2]),math.sin(pose[2])
    near=-half_length;far=near-distance-stop_buffer
    centre=(near+far)/2;depth=(near-far)/2;width=half_width+margin
    cx,cy=pose[0]+centre*c,pose[1]+centre*s
    ex,ey=depth*abs(c)+width*abs(s),depth*abs(s)+width*abs(c)
    if (cx-ex<grid.origin[0] or cy-ey<grid.origin[1] or
        cx+ex>=grid.origin[0]+grid.width*grid.res or
        cy+ey>=grid.origin[1]+grid.height*grid.res):
        return False,'reverse corridor leaves the known map'
    r0,k0=grid.world_to_cell(cx-ex,cy-ey);r1,k1=grid.world_to_cell(cx+ex,cy+ey)
    half_cell=grid.res/2;projection=half_cell*(abs(c)+abs(s))
    for row in range(max(0,r0-1),min(grid.height,r1+2)):
        for col in range(max(0,k0-1),min(grid.width,k1+2)):
            x,y=grid.cell_to_world(row,col);dx,dy=x-cx,y-cy
            if abs(dx)>ex+half_cell or abs(dy)>ey+half_cell:continue
            if abs(dx*c+dy*s)>depth+projection or abs(-dx*s+dy*c)>width+projection:continue
            if grid.occupied[row,col] or grid.unknown[row,col]:
                return False,'static obstacle or unknown rear space'
            for zone in grid.zones:
                denied=zone['rule']=='forbidden' or (zone['rule']=='authorized_only' and not allow_restricted)
                if denied and (zone['x_min']-half_cell<=x<=zone['x_max']+half_cell and
                               zone['y_min']-half_cell<=y<=zone['y_max']+half_cell):
                    return False,'reverse corridor enters a restricted policy zone'
    return True,'map clear'


class BackupRecovery:
    def __init__(self, reroutes=3, progress=.05, distance=.30, speed=.05,
                 timeout=8., wait_timeout=10., blocked_timeout=30., rearm_progress=.20):
        if (reroutes<1 or not .05<=distance<=.5 or not .01<=speed<=.1 or
            not 1<=timeout<=15 or not 1<=wait_timeout<=10 or progress<=0):
            raise ValueError('Invalid bounded backup recovery settings')
        self.limit=int(reroutes);self.progress=progress;self.distance=distance;self.speed=speed
        self.timeout=timeout;self.wait_timeout=wait_timeout
        self.blocked_timeout=blocked_timeout;self.rearm_progress=rearm_progress
        self.reset()

    def reset(self):
        self.phase='idle';self.attempts=0;self.used=False;self.anchor=None;self.odom_anchor=None
        self.deadline=None;self.identifier=None;self.path=[];self.reason=''
        self.started=None;self.requested_at=None;self.start_pose=None;self.start_odom=None
        self.rearm_goal_distance=None;self.travelled=0.;self.progress_at=None;self.progress_m=0.
        self.paused_at=None
        self.post_pose=self.post_odom=None

    @property
    def active(self):return self.phase in ('waiting','backing')

    def observe_forward(self, pose, odom, goal, v):
        if self.active or odom is None:return
        if self.anchor is None:self.anchor=tuple(pose);self.odom_anchor=tuple(odom)
        if v<=.02:return  # rotation, waiting and backup are not forward progress
        moved=math.dist(pose[:2],self.anchor[:2])>=self.progress
        wheel_moved=math.dist(odom[:2],self.odom_anchor[:2])>=self.progress
        if moved and wheel_moved:
            self.anchor=tuple(pose);self.odom_anchor=tuple(odom);self.attempts=0
            if not self.used:self.deadline=None
        if (self.used and self.rearm_goal_distance is not None and
            self.post_pose is not None and self.post_odom is not None and
            math.dist(pose[:2],goal)<=self.rearm_goal_distance-self.rearm_progress and
            math.dist(pose[:2],self.start_pose[:2])>=self.rearm_progress and
            math.dist(odom[:2],self.start_odom[:2])>=self.rearm_progress and
            math.dist(pose[:2],self.post_pose[:2])>=self.rearm_progress and
            math.dist(odom[:2],self.post_odom[:2])>=self.rearm_progress):
            self.reset();self.anchor=tuple(pose);self.odom_anchor=tuple(odom)

    def note_replan(self, pose, odom, now, blocked_since=None):
        if self.anchor is None:
            self.anchor=tuple(pose);self.odom_anchor=tuple(odom) if odom else None
        if self.deadline is None:
            self.deadline=(now if blocked_since is None else blocked_since)+self.blocked_timeout
        self.attempts+=1
        if self.attempts<self.limit:return 'retry'
        return 'exhausted' if self.used else 'request'

    def request(self, pose, odom, goal, now, identifier):
        self.used=True;self.phase='waiting';self.identifier=identifier;self.requested_at=now
        self.start_pose=tuple(pose);self.start_odom=tuple(odom) if odom else None
        self.path=reverse_path(pose,self.distance);self.travelled=0.
        self.rearm_goal_distance=math.dist(pose[:2],goal)
        self.deadline=(self.deadline if self.deadline is not None else now+self.blocked_timeout)+self.timeout
        self.reason='waiting for rear clearance and traffic permission'

    def expired(self, now):return self.deadline is not None and now>self.deadline

    def pause(self, now):
        if self.paused_at is None:self.paused_at=now

    def resume(self, now):
        if self.paused_at is not None and self.deadline is not None:
            self.deadline+=max(0.,now-self.paused_at)
        self.paused_at=None

    def fail(self, reason):
        self.phase='failed';self.reason=reason
        return 'failed'

    def step(self, pose, odom, now, clear, granted, reason=''):
        if not self.active:return self.phase
        if not all(math.isfinite(v) for v in pose):return self.fail('invalid localization pose')
        if odom is not None and not all(math.isfinite(v) for v in odom):return self.fail('invalid odometry pose')
        if now<self.requested_at or self.expired(now):return self.fail('backup episode deadline expired or clock reset')
        if self.phase=='waiting':
            if now-self.requested_at>=self.wait_timeout:return self.fail('backup permission/clearance timed out: '+self.reason)
            self.reason=reason or 'waiting for traffic permission'
            if not clear or not granted or odom is None:return 'wait'
            self.phase='backing';self.started=now;self.start_pose=tuple(pose);self.start_odom=tuple(odom)
            self.progress_at=now;self.progress_m=0.;self.reason='backing up'
        if not clear:return self.fail('backup stopped: '+reason)
        if now-self.started>=self.timeout:return self.fail('backup motion timed out')
        if not granted:
            self.reason='traffic permission lost; stopped'
            return 'wait'
        if odom is None:return self.fail('backup odometry unavailable')
        def displacement(current,start):
            dx,dy=current[0]-start[0],current[1]-start[1];c,s=math.cos(start[2]),math.sin(start[2])
            return -(dx*c+dy*s),abs(-dx*s+dy*c),abs(math.remainder(current[2]-start[2],2*math.pi))
        measured=displacement(pose,self.start_pose);wheel=displacement(odom,self.start_odom)
        if any(x[0]<-.04 or x[1]>.04 or x[2]>.12 for x in [measured,wheel]):
            return self.fail('backup deviated from its reserved straight corridor')
        if max(measured[0],wheel[0])>self.distance+.04:return self.fail('backup distance limit exceeded')
        self.travelled=max(0.,min(measured[0],wheel[0]))
        if self.travelled>=self.distance-.01:
            self.phase='complete';self.reason='backup complete';self.attempts=0
            self.post_pose=tuple(pose);self.post_odom=tuple(odom)
            self.anchor=tuple(pose);self.odom_anchor=tuple(odom)
            return 'complete'
        if self.travelled-self.progress_m>=.02:self.progress_m=self.travelled;self.progress_at=now
        if now-self.progress_at>=3.:return self.fail('reverse commanded without measured position progress')
        self.reason='backing up';return 'reverse'

    def status(self):
        return {'id':self.identifier,'phase':self.phase,'reroutes_without_progress':self.attempts,
                'used':self.used,'distance_m':self.distance,'travelled_m':round(self.travelled,3),
                'path':self.path,'reason':self.reason}
