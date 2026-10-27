"""ROS I/O for the mapping session. Navigation and activation remain separate owners."""
import json
import math
from pathlib import Path
import time
import uuid

import yaml
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav_msgs.msg import Path as RosPath
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


class MappingControls:
    def __init__(self,owner,process_factory):
        self.owner=owner;self.group=process_factory(owner.directory/'sessions')
        self.visualizer=process_factory(owner.directory/'sessions')
        self.auto_visualize=True;self.visualizer_error=''
        (owner.directory/'sessions').mkdir(exist_ok=True)
        cfg=yaml.safe_load((owner.share/'config/mapping_route.yaml').read_text())
        self.waypoints=[f'{x-1.:.6f} {y-1.:.6f}' for x,y in cfg['waypoints_gazebo']]
        self.goal_name=None;self.goal_seq=0;self.session_id='';self.started=0.
        self.route=[];self.route_length=0.;self.route_base=0.;self.fraction=0.;self.goal_stamp=0
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.role_pub=owner.create_publisher(String,'/mapping/active_robot',latched)
        self.events=owner.create_publisher(String,'/fleet/obstacle_event',10)
        self.go_pub=owner.create_publisher(String,'/robot1/goto',10)
        self.cancel_client=owner.create_client(Trigger,'/robot1/navigator/cancel_for_mapping')
        self.finish_client=owner.create_client(Trigger,'/mapping/engine/finish')
        owner.create_subscription(RosPath,'/robot1/plan',self.on_plan,latched)
        self.role('')

    def check_start(self):
        o=self.owner
        if set(o.robots)!={'robot1','robot2'}:raise ValueError('The guided survey requires the two-robot factory demo')
        if o.machine.phase!='READY' or o.preparing:raise ValueError('Wait for verified navigation before mapping')
        o.check_idle()
        for r in o.robots:
            pose=o.value('/'+r+'/nav_status').get('pose')
            home=(0.,0.) if r=='robot1' else (1.6,0.)
            if not pose or math.dist(pose[:2],home)>.30:
                raise ValueError('Wait for automatic return to HOME_1 / HOME_2 before starting a survey')
        state=o.value('/fleet/obstacles_state')
        if not state or state.get('errors'):raise ValueError('Waiting for healthy obstacle-controller status')
        if state.get('pallet',{}).get('pending'):raise ValueError('Wait for the pallet placement acknowledgement before mapping')

    def new_session(self,mode):
        if mode not in ('slam','ogm','slam_ogm','compare'):raise ValueError('Unknown mapping mode')
        self.session_id='s_'+uuid.uuid4().hex[:12]
        folder=self.owner.directory/'sessions'/self.session_id;folder.mkdir()
        self.started=time.monotonic();return self.session_id,str(folder)

    def world_snapshot(self):return self.owner.value('/fleet/obstacles_state')
    def world_ready(self):
        state=self.owner.value('/fleet/obstacles_state');pallet=state.get('pallet',{})
        return bool(pallet.get('blocked') and not pallet.get('pending') and 'pallet_1' not in state.get('pending',[]))
    def pause(self,value):return self.owner.pause(value)
    def hold(self,value):self.owner.hold(value)
    def held(self):return self.owner.held()
    def released(self):return self.owner.released()

    def role(self,robot):self.role_pub.publish(String(data=robot))

    def park_world(self):
        for model in ('movers','pallet_1'):
            self.events.publish(String(data=json.dumps({'model':model,'place':'off' if model=='movers' else 'parked'})))

    def parked(self):
        o=self.owner;state=o.value('/fleet/obstacles_state')
        peers=all(o.value('/'+r+'/nav_status').get('survey_hold') for r in o.robots if r!='robot1')
        return bool(state and not state.get('enabled') and state.get('settled') and
                    state.get('places',{}).get('pallet_1')=='parked' and peers)

    def start_engine(self,mode,identifier,directory):
        anchor=self.owner.value('/robot1/nav_status')['pose']
        self.owner.clear_mapping_view()
        self.group.start(['ros2','launch','factory_bot','mapping_generation.launch.py',
            'mode:='+mode,'session_id:='+identifier,'session_dir:='+directory,'anchor:='+json.dumps(anchor)])
        if self.auto_visualize:
            try:self.open_visualizer()
            except Exception as error:self.visualizer_error=str(error)

    def open_visualizer(self):
        if self.visualizer.alive():return
        self.visualizer.start(['ros2','run','rviz2','rviz2','-d',str(self.owner.share/'rviz/mapping.rviz'),
            '--ros-args','-p','use_sim_time:=true'])
        self.visualizer_error=''

    def visualizer_status(self):
        process=getattr(self.visualizer,'process',None)
        if process is not None and process.poll() not in (None,0):
            self.visualizer_error='RViz exited with an error; inspect '+str(self.visualizer.log.name)
        return {'visualizer_open':self.visualizer.alive(),'visualizer_error':self.visualizer_error}

    def engine_alive(self):return self.group.alive()
    def engine_status(self,identifier):
        status=self.owner.value('/mapping/session_status')
        return status if status.get('id')==identifier else {}
    def stop_engine(self):self.group.stop()
    def engine_stopped(self):return self.group.stopped()

    def go(self,target):
        st=self.owner.value('/robot1/nav_status');self.goal_seq=st.get('goal',{}).get('seq',0);self.goal_name=target
        self.route=[];self.route_length=0.;self.route_base=0.;self.fraction=0.
        self.goal_stamp=self.owner.get_clock().now().nanoseconds
        self.latest_route_stamp=-1
        self.go_pub.publish(String(data=json.dumps({'goal':target,'priority':'routine','speed':.20})))

    def on_plan(self,msg):
        stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        # /clock callbacks can be a few samples apart between nodes. This affects display only.
        if not self.goal_name or stamp<self.goal_stamp-500_000_000 or stamp<getattr(self,'latest_route_stamp',-1):return
        self.latest_route_stamp=stamp
        self.route_base=self.fraction;self.route=[(p.pose.position.x,p.pose.position.y) for p in msg.poses]
        self.route_length=sum(math.dist(a,b) for a,b in zip(self.route,self.route[1:]))

    def goal_fraction(self):
        from factory_bot.dashboard_state import route_remaining
        st=self.owner.value('/robot1/nav_status');goal=st.get('goal',{})
        if goal.get('seq',0)<=self.goal_seq or goal.get('name')!=self.goal_name:return 0.
        if st.get('state')=='ARRIVED':return 1.
        remaining=route_remaining(self.route,st.get('pose'))
        if remaining is not None and self.route_length>0:
            done=max(0.,min(.99,1.-remaining/self.route_length))
            self.fraction=max(self.fraction,min(.99,self.route_base+(1-self.route_base)*done))
        return self.fraction

    def goal_result(self):
        st=self.owner.value('/robot1/nav_status');goal=st.get('goal',{})
        if goal.get('seq',0)<=self.goal_seq or goal.get('name')!=self.goal_name:return None
        return st.get('state') if st.get('state') in ('ARRIVED','FAILED') else None

    def cancel_goal(self):
        if not self.cancel_client.service_is_ready():raise RuntimeError('Survey stop service unavailable')
        return self.cancel_client.call_async(Trigger.Request())

    def finish_engine(self):
        if not self.finish_client.service_is_ready():raise RuntimeError('Mapping save service unavailable')
        return self.finish_client.call_async(Trigger.Request())

    def restore_world(self,snapshot):
        for model,place in snapshot.get('places',{}).items():
            event={'model':model,'place':place}
            if model=='pallet_1' and place=='grid':
                pose=snapshot['pallet']['pose'];event.update(x=pose[0],y=pose[1])
            self.events.publish(String(data=json.dumps(event)))
        self.events.publish(String(data=json.dumps({'model':'movers','place':'on' if snapshot.get('enabled') else 'off'})))

    def restored(self,snapshot):
        o=self.owner;state=o.value('/fleet/obstacles_state')
        if not state or state.get('errors') or state.get('enabled')!=snapshot.get('enabled'):return False
        if set(state.get('pending',[])) & set(snapshot.get('places',{})):return False
        if not snapshot.get('enabled') and not state.get('settled'):return False
        if state.get('places')!=snapshot.get('places'):return False
        if snapshot.get('places',{}).get('pallet_1')=='grid':
            actual=state.get('pallet',{}).get('pose');expected=snapshot['pallet']['pose']
            if not actual or math.dist(actual,expected)>.01:return False
        if any(o.value('/'+r+'/nav_status').get('survey_owner') for r in o.robots):return False
        return o.ready(o.machine.active.path,0.)[0]
