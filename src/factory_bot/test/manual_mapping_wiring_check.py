"""Isolated ROS wiring probe: synthetic stationary sensor data, no Gazebo or commands."""
import argparse,json,math,os,signal,subprocess,time
from collections import Counter
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry,OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from std_srvs.srv import Trigger

p=argparse.ArgumentParser();p.add_argument('--mode',default='slam');p.add_argument('--finish',action='store_true');p.add_argument('--move',action='store_true');p.add_argument('--label',default='check');p.add_argument('--seconds',type=float,default=9.);p.add_argument('--expect-saved',action='store_true')
args=p.parse_args();folder=Path('work/mapping_wiring_checks/probe-'+args.mode+'-'+args.label);folder.mkdir(exist_ok=True,parents=True)
assert os.environ.get('ROS_DOMAIN_ID')=='93','Probe is restricted to isolated domain 93'
rclpy.init();node=rclpy.create_node('mapping_wiring_probe');latest={};counts={};headers={}
qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
for topic in ('/mapping/session_status',):
    node.create_subscription(String,topic,lambda m,t=topic:latest.__setitem__(t,json.loads(m.data)),qos)
def grid(msg,topic):
    counts[topic]=counts.get(topic,0)+1;headers[topic]={'frame':msg.header.frame_id}
    if isinstance(msg,OccupancyGrid):headers[topic].update(values=dict(Counter(msg.data)),width=msg.info.width,height=msg.info.height,
        origin=[msg.info.origin.position.x,msg.info.origin.position.y],resolution=msg.info.resolution)
for topic in ('/map','/mapping/map','/mapping/live_map'):
    node.create_subscription(OccupancyGrid,topic,lambda m,t=topic:grid(m,t),qos)
node.create_subscription(PoseWithCovarianceStamped,'/mapping/pose',lambda m:grid(m,'/mapping/pose'),10)
clock=node.create_publisher(Clock,'/clock',10);odom=node.create_publisher(Odometry,'/robot1/odom',qos_profile_sensor_data)
scan=node.create_publisher(LaserScan,'/robot1/scan',qos_profile_sensor_data);truth=node.create_publisher(Odometry,'/robot1/ground_truth',qos_profile_sensor_data)
finish=node.create_client(Trigger,'/mapping/engine/finish');requested=False
limit=time.monotonic()+1.5
while time.monotonic()<limit:rclpy.spin_once(node,timeout_sec=.1)
others=[name for name in node.get_node_names() if name!='mapping_wiring_probe']
if others:
    node.destroy_node();rclpy.shutdown();raise RuntimeError('Domain 93 must be unused: '+str(others))
log=(folder/'nodes.log').open('w')
proc=subprocess.Popen(['ros2','launch','factory_bot','mapping_generation.launch.py','mode:='+args.mode,
    'session_id:=probe_'+args.label,'session_dir:='+str(folder.resolve()),'anchor:=[0.0,0.0,0.0]'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
start=time.monotonic();next_publish=start
try:
    while time.monotonic()-start<args.seconds:
        now=time.monotonic()
        if now>=next_publish:
            next_publish=now+.05;t=100.+now-start
            stamp=Clock();stamp.clock.sec=int(t);stamp.clock.nanosec=int((t-int(t))*1e9);clock.publish(stamp)
            x=min(2.,.4*max(0.,now-start-2.)) if args.move else 0.
            o=Odometry();o.header.stamp=stamp.clock;o.header.frame_id='robot1/odom';o.child_frame_id='robot1/base_footprint';o.pose.pose.orientation.w=1.;o.pose.pose.position.x=x;odom.publish(o)
            o.pose.pose.position.x=1.+x;o.pose.pose.position.y=1.;truth.publish(o)
            m=LaserScan();m.header.stamp=stamp.clock;m.header.frame_id='robot1/laser_link'
            m.angle_min=-math.pi;m.angle_increment=2*math.pi/360;m.angle_max=m.angle_min+359*m.angle_increment
            m.range_min=.12;m.range_max=12.;m.scan_time=.1
            m.ranges=[]
            for i in range(360):
                c=math.cos(m.angle_min+i*m.angle_increment);s=math.sin(m.angle_min+i*m.angle_increment)
                tx=((4.-x-.1) if c>0 else (-4.-x-.1))/c if abs(c)>1e-8 else math.inf
                ty=(4. if s>0 else -4.)/s if abs(s)>1e-8 else math.inf
                distance=min(tx,ty)
                for cx,cy,radius in ((2.,2.,.35),(3.,.7,.25)):
                    vx,vy=cx-x-.1,cy;b=vx*c+vy*s;disc=b*b-(vx*vx+vy*vy-radius*radius)
                    if disc>=0 and b>0:distance=min(distance,b-math.sqrt(disc))
                m.ranges.append(distance)
            scan.publish(m)
        rclpy.spin_once(node,timeout_sec=.01)
        status=latest.get('/mapping/session_status',{})
        if args.finish and not requested and now-start>7 and status.get('ready') and finish.service_is_ready():
            finish.call_async(Trigger.Request());requested=True
        if requested and status.get('phase') in ('SAVED','FAILED'):break
    report={'mode':args.mode,'counts':counts,'headers':headers,'status':latest,'finished_requested':requested,
        'publishers':{t:[i.node_namespace+'/'+i.node_name for i in node.get_publishers_info_by_topic(t)] for t in ('/map','/mapping/map')}}
    (folder/'probe.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
    if args.expect_saved:
        assert latest['/mapping/session_status']['phase']=='SAVED',report
        assert not report['publishers']['/map'],report
        if args.mode=='slam_ogm':assert len(latest['/mapping/session_status']['maps'])==4,report
finally:
    os.killpg(proc.pid,signal.SIGINT)
    try:proc.wait(timeout=8.)
    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=3.)
    log.close();node.destroy_node();rclpy.shutdown()
