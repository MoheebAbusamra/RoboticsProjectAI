"""On-demand measured mapping only: no Gazebo, robot spawning, global TF or clock publisher."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node,LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from factory_bot.map_catalog import robot_options


def launch_setup(context,*args,**kwargs):
    import json
    get=lambda key:LaunchConfiguration(key).perform(context)
    mode=get('mode')
    if mode not in ('slam','ogm','slam_ogm','compare'):raise ValueError('Unknown mapping mode')
    anchor=robot_options(json.dumps({'robot1':json.loads(get('anchor'))}),['robot1'],'poses')['robot1']
    engine=Node(package='factory_bot',executable='mapping_engine',name='engine',namespace='mapping',
        output='screen',parameters=[{'use_sim_time':True,'mode':mode,'anchor':anchor,
            'session_id':get('session_id'),'session_dir':get('session_dir')}])
    if mode=='ogm':return [engine]
    share=get_package_share_directory('factory_bot')
    slam=LifecycleNode(package='slam_toolbox',executable='async_slam_toolbox_node',name='slam_toolbox',
        namespace='mapping',output='screen',parameters=[os.path.join(share,'config','slam_on_demand.yaml')],
        remappings=[('/tf','/mapping/tf'),('/tf_static','/mapping/tf_static'),
                    ('/map','/mapping/map'),('/map_metadata','/mapping/map_metadata'),('pose','/mapping/pose')])
    activate=RegisterEventHandler(OnStateTransition(target_lifecycle_node=slam,start_state='configuring',goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(slam),transition_id=Transition.TRANSITION_ACTIVATE))]))
    configure=EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(slam),transition_id=Transition.TRANSITION_CONFIGURE))
    return [engine,slam,activate,configure]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in
        {'mode':'slam','session_id':'','session_dir':'','anchor':'[0.0,0.0,0.0]'}.items()]
        +[OpaqueFunction(function=launch_setup)])
