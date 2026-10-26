"""Live sensor-to-map adapter using isolated TF topics and measured inputs only."""
from collections import deque
import copy
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path as RosPath
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from factory_bot.mapping_data import MappingData, compose


def stamp_ns(stamp): return stamp.sec*1_000_000_000+stamp.nanosec
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class MappingEngine(Node):
    def __init__(self):
        super().__init__('mapping_engine')
        for name,value in {'mode':'slam','session_id':'','session_dir':'','anchor':[0.,0.,0.],
                           'every_n_scans':2}.items():self.declare_parameter(name,value)
        get=lambda key:self.get_parameter(key).value
        self.identifier=get('session_id');self.directory=Path(get('session_dir'))
        if not self.identifier or not str(get('session_dir')):raise ValueError('A mapping session is required')
        self.data=MappingData(get('mode'),get('anchor'));self.every=max(1,int(get('every_n_scans')))
        self.odom=deque(maxlen=200);self.truth=deque(maxlen=100);self.seen=0
        self.phase='STARTING';self.error='';self.finishing=None;self.report=None;self.last_map_at=-math.inf
        self.last_scan_at=-math.inf;self.next_preview=0.;self.skipped=0
        self.latest_pose=None;self.trajectory=deque(maxlen=2000);self.preview_kind=None
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.tf_pub=self.create_publisher(TFMessage,'/mapping/tf',10)
        self.static_pub=self.create_publisher(TFMessage,'/mapping/tf_static',latched)
        self.scan_pub=self.create_publisher(LaserScan,'/mapping/scan',qos_profile_sensor_data)
        self.status_pub=self.create_publisher(String,'/mapping/session_status',latched)
        self.preview_pub=self.create_publisher(OccupancyGrid,'/mapping/live_map',latched)
        self.path_pub=self.create_publisher(RosPath,'/mapping/trajectory',latched)
        self.create_subscription(Odometry,'/robot1/odom',self.on_odom,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/robot1/ground_truth',self.on_truth,qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/robot1/scan',self.on_scan,qos_profile_sensor_data)
        if self.data.uses_slam:
            self.create_subscription(OccupancyGrid,'/mapping/map',self.on_map,latched)
            self.create_subscription(PoseWithCovarianceStamped,'/mapping/pose',self.on_pose,10)
        self.create_service(Trigger,'~/finish',self.finish)
        t=TransformStamped();t.header.frame_id='mapping_base';t.child_frame_id='mapping_laser'
        t.transform.translation.x=.10;t.transform.translation.z=.23;t.transform.rotation.w=1.
        self.static_pub.publish(TFMessage(transforms=[t]))
        self.create_timer(.2,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_odom(self,msg):
        p=msg.pose.pose.position;pose=(p.x,p.y,yaw(msg.pose.pose.orientation));stamp=stamp_ns(msg.header.stamp)
        if not all(math.isfinite(v) for v in pose):return
        self.odom.append((stamp,pose));local=self.data.local_odom(pose)
        t=TransformStamped();t.header.stamp=msg.header.stamp;t.header.frame_id='mapping_odom';t.child_frame_id='mapping_base'
        t.transform.translation.x=local[0];t.transform.translation.y=local[1]
        t.transform.rotation.z=math.sin(local[2]/2);t.transform.rotation.w=math.cos(local[2]/2)
        self.tf_pub.publish(TFMessage(transforms=[t]))

    def on_truth(self,msg):
        p=msg.pose.pose.position
        self.truth.append((stamp_ns(msg.header.stamp),(p.x-1.,p.y-1.,yaw(msg.pose.pose.orientation))))

    def on_scan(self,msg):
        self.seen+=1
        if self.finishing is not None or self.phase in ('SAVED','FAILED') or self.seen%self.every or not self.odom:return
        if msg.header.frame_id.lstrip('/')!='robot1/laser_link':self.skipped+=1;return
        stamp=stamp_ns(msg.header.stamp);best=min(self.odom,key=lambda p:abs(p[0]-stamp))
        if abs(best[0]-stamp)>100_000_000:self.skipped+=1;return
        truth=None
        if self.truth:
            candidate=min(self.truth,key=lambda p:abs(p[0]-stamp))
            if abs(candidate[0]-stamp)<=100_000_000:truth=candidate[1]
        scan=(list(msg.ranges),msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max)
        if len(msg.ranges)<20 or not all(math.isfinite(v) for v in scan[1:]):self.skipped+=1;return
        self.data.add_scan(stamp,scan,best[1],truth);self.last_scan_at=time.monotonic()
        if not self.data.uses_slam:self.add_pose(compose(self.data.anchor,self.data.local_odom(best[1])),msg.header.stamp)
        if self.data.uses_slam:
            output=copy.deepcopy(msg);output.header.frame_id='mapping_laser';self.scan_pub.publish(output)

    def on_pose(self,msg):
        if msg.header.frame_id!='mapping_map':return
        p=msg.pose.pose.position;pose=(p.x,p.y,yaw(msg.pose.pose.orientation))
        if all(math.isfinite(v) for v in pose):
            self.data.add_slam_pose(stamp_ns(msg.header.stamp),pose)
            self.add_pose(compose(self.data.anchor,pose),msg.header.stamp)

    def add_pose(self,pose,stamp):
        self.latest_pose=list(pose)
        if self.trajectory:
            previous=self.trajectory[-1].pose.position
            if math.hypot(previous.x-pose[0],previous.y-pose[1])<.02:return
        msg=PoseStamped();msg.header.frame_id='map';msg.header.stamp=stamp
        msg.pose.position.x=pose[0];msg.pose.position.y=pose[1]
        msg.pose.orientation.z=math.sin(pose[2]/2);msg.pose.orientation.w=math.cos(pose[2]/2)
        self.trajectory.append(msg)

    def on_map(self,msg):
        if msg.header.frame_id!='mapping_map' or not msg.info.width or not msg.info.height:return
        if len(msg.data)!=msg.info.width*msg.info.height or len(msg.data)>2_000_000:return
        p=msg.info.origin.position
        try:
            self.data.add_slam_grid(np.asarray(msg.data,dtype=np.int8).reshape(msg.info.height,msg.info.width),
                msg.info.resolution,(p.x,p.y,yaw(msg.info.origin.orientation)))
            self.last_map_at=time.monotonic()
        except ValueError as error:self.error=str(error)

    def finish(self,request,response):
        if self.finishing is None:self.finishing=time.monotonic();self.phase='FINISHING'
        response.success=True;response.message=self.identifier
        return response

    def tick(self):
        now=time.monotonic()
        ready=(self.data.scans>0 and (not self.data.uses_slam or
               (self.data.slam is not None and self.data.slam_poses>0)))
        reason=('Waiting for /robot1/scan' if not self.seen else 'Waiting for timestamp-matched primary lidar and odometry' if not self.data.scans
                else 'Waiting for SLAM map on /mapping/map' if self.data.uses_slam and self.data.slam is None
                else 'Waiting for SLAM pose on /mapping/pose' if self.data.uses_slam and not self.data.slam_poses else 'Measured map ready')
        if self.finishing is None:self.phase='READY' if ready else 'STARTING'
        elif self.report is None:
            # Stop accepting scans, then allow the asynchronous matcher/map publisher to drain.
            drained=not self.data.uses_slam or self.last_map_at>=self.finishing
            if now-self.finishing>=5. and drained:
                try:
                    self.report=self.data.save(self.directory,self.identifier)
                    self.phase='SAVED' if self.report['success'] else 'FAILED';self.error=self.report['error']
                except Exception as error:self.phase='FAILED';self.error=str(error);self.report={'maps':[]}
            elif now-self.finishing>30.:
                self.phase='FAILED';self.error='No final SLAM map arrived after finishing';self.report={'maps':[]}
        if now>=self.next_preview:
            self.next_preview=now+1.
            values,kind=self.data.preview()
            self.preview_kind=kind
            if values is not None:
                msg=OccupancyGrid();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='map'
                msg.info.resolution=self.data.res;msg.info.width=self.data.width;msg.info.height=self.data.height
                msg.info.origin.position.x=self.data.origin[0];msg.info.origin.position.y=self.data.origin[1]
                msg.info.origin.orientation.w=1.;msg.data=values.reshape(-1).tolist();self.preview_pub.publish(msg)
            path=RosPath();path.header.frame_id='map';path.header.stamp=self.get_clock().now().to_msg()
            path.poses=list(self.trajectory);self.path_pub.publish(path)
        self.status_pub.publish(String(data=json.dumps({'id':self.identifier,'phase':self.phase,'ready':ready,
            'scans':self.data.scans,'slam_poses':self.data.slam_poses,'hybrid_pairs':self.data.pose_pairs,
            'slam_maps':self.data.slam_maps,'input_scans':self.seen,'ready_reason':reason,
            'pose':self.latest_pose,'preview_kind':self.preview_kind,
            'skipped':self.skipped,'error':self.error,'maps':(self.report or {}).get('maps',[]),
            'comparison':(self.report or {}).get('comparison',[])})))


def main(args=None):
    rclpy.init(args=args);node=MappingEngine()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
