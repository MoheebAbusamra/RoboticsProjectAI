"""Offline checks of actual launch wiring and map loading; never start ROS nodes."""
import importlib
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node as LaunchNode
from launch_ros.utilities import evaluate_parameters

from factory_bot.map_source import resolve_map_yaml
from factory_bot.map_catalog import navigation_command

PKG = Path(__file__).resolve().parents[1]
MAP_COMPONENTS = {'map_server', 'icp_localizer', 'navigator', 'fleet_manager', 'traffic_manager'}


@pytest.fixture
def selected_map(tmp_path):
    folder = tmp_path / 'selected map'
    folder.mkdir()
    pixels = np.full((32, 32), 254, dtype=np.uint8)
    pixels[2, 3] = 0
    (folder / 'scan.pgm').write_bytes(b'P5\n32 32\n255\n' + pixels[::-1].tobytes())
    path = folder / 'factory_map.yaml'
    path.write_text(yaml.safe_dump({'image': 'scan.pgm', 'resolution': .05,
        'origin': [-1.3, -1.3, 0.], 'negate': 0, 'occupied_thresh': .65, 'free_thresh': .196}))
    return path


def text(value, context):
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


def launch_nodes(filename, overrides=None):
    """Expand our real includes/functions as descriptions only, skipping Gazebo/RViz execution."""
    context = LaunchContext()
    context.launch_configurations.update(overrides or {})
    path = PKG / 'launch' / filename
    spec = importlib.util.spec_from_file_location('map_test_' + path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # Load includes from this source package (also works before installation).
    module.get_package_share_directory = lambda name: str(PKG)
    description = module.generate_launch_description()
    result = []

    def visit(actions):
        for action in actions:
            if isinstance(action, DeclareLaunchArgument):
                if action.name not in context.launch_configurations:
                    context.launch_configurations[action.name] = text(action.default_value, context)
            elif isinstance(action, OpaqueFunction):
                visit(action.execute(context) or [])
            elif isinstance(action, TimerAction):
                visit(action.actions)
            elif isinstance(action, IncludeLaunchDescription):
                # This loads a description, never executes its actions. The public
                # location property becomes a resolved filename after this call.
                action.launch_description_source.get_launch_description(context)
                child = Path(action.launch_description_source.location).name
                if child not in {'fleet.launch.py', 'navigation.launch.py', 'localization.launch.py', 'navigation_group.launch.py'}:
                    continue
                args = dict(context.launch_configurations)
                args.update({text(k, context): text(v, context) for k, v in action.launch_arguments})
                result.extend(launch_nodes(child, args))
            elif isinstance(action, LaunchNode):
                executable = text(action.node_executable, context)
                if executable in MAP_COMPONENTS or executable == 'map_coordinator':
                    # Inspect the normalized parameter dictionaries without executing Node actions.
                    params = {}
                    for item in evaluate_parameters(context, action._Node__parameters):
                        params.update(item)
                    if executable == 'map_coordinator':
                        command = navigation_command(params['map_yaml'], params['robots'], params['algorithm'],
                                                     params['controller'], params['traffic'])
                        args = dict(context.launch_configurations)
                        args.update(dict(item.split(':=', 1) for item in command[4:]))
                        result.extend(launch_nodes(command[3], args))
                    else:
                        result.append((executable, params))

    visit(description.entities)
    return result


@pytest.mark.parametrize('entry,count', [('demo.launch.py', 7), ('fleet.launch.py', 7),
                                        ('navigation.launch.py', 5), ('localization.launch.py', 3)])
def test_selected_map_reaches_every_consumer_in_each_entry_point(selected_map, entry, count):
    nodes = launch_nodes(entry, {'map': str(selected_map), 'algorithm': 'bfs', 'controller': 'pid',
                                'allocation': 'optimal'})
    assert len(nodes) == count
    for executable, params in nodes:
        key = 'yaml_filename' if executable == 'map_server' else 'map_yaml'
        assert params[key] == str(selected_map.resolve())
        if executable == 'navigator':
            assert params['algorithm'] == 'bfs' and params['controller'] == 'pid'
        elif executable == 'fleet_manager':
            assert params['algorithm'] == 'bfs' and params['allocation'] == 'optimal'
    if entry in ('demo.launch.py', 'fleet.launch.py'):
        assert {p['robot'] for e, p in nodes if e == 'navigator'} == {'robot1', 'robot2'}
        assert {e for e, p in nodes} == MAP_COMPONENTS


def test_default_map_and_traffic_disabled_remain_compatible():
    nodes = launch_nodes('demo.launch.py', {'traffic': 'false'})
    expected = str((PKG / 'maps/factory_map.yaml').resolve())
    assert len(nodes) == 6
    assert not any(e == 'traffic_manager' for e, p in nodes)
    assert all(p.get('map_yaml', p.get('yaml_filename')) == expected for e, p in nodes)


def test_restart_group_preserves_actual_robot_poses_and_individual_navigation_defaults(selected_map):
    import json
    poses = {'robot1': [3, 4, .2], 'robot2': [8, 6, 1.]}
    profiles = {'robot1': {'algorithm': 'bfs', 'controller': 'pid'},
                'robot2': {'algorithm': 'dijkstra', 'controller': 'pure_pursuit'}}
    nodes = launch_nodes('navigation_group.launch.py', {'map': str(selected_map),
        'initial_poses': json.dumps(poses), 'profiles': json.dumps(profiles)})
    assert len(nodes) == 6  # map server, two ICPs, two navigators, traffic; fleet remains outside
    for executable, params in nodes:
        if executable == 'icp_localizer':
            assert tuple(params['initial_pose']) == tuple(poses[params['robot']])
        elif executable == 'navigator':
            assert params['algorithm'] == profiles[params['robot']]['algorithm']
            assert params['controller'] == profiles[params['robot']]['controller']


def test_launch_expands_one_selection_consistently_and_never_silently_falls_back(selected_map, monkeypatch):
    monkeypatch.setenv('SELECTED_FACTORY_MAP', str(selected_map))
    nodes = launch_nodes('fleet.launch.py', {'map': '$SELECTED_FACTORY_MAP'})
    assert all(p.get('map_yaml', p.get('yaml_filename')) == str(selected_map) for e, p in nodes)
    with pytest.raises(FileNotFoundError, match='Selected map YAML'):
        launch_nodes('fleet.launch.py', {'map': str(selected_map.parent / 'missing.yaml')})


def test_map_path_supports_home_relative_paths_and_symlinks(selected_map, tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    monkeypatch.chdir(tmp_path)
    alias = tmp_path / 'alias.yaml'
    alias.symlink_to(selected_map)
    assert resolve_map_yaml('~/alias.yaml') == str(selected_map)
    assert resolve_map_yaml('alias.yaml') == str(selected_map)
    with pytest.raises(ValueError, match='empty'):
        resolve_map_yaml('   ')
    with pytest.raises(FileNotFoundError):
        resolve_map_yaml(tmp_path)


@pytest.mark.parametrize('module_name,class_name', [('fleet_manager', 'FleetManager'),
    ('traffic_manager', 'TrafficManager'), ('navigator', 'Navigator'), ('icp_localizer', 'IcpLocalizer')])
def test_real_constructors_read_the_selected_grid_not_the_generated_map(selected_map, monkeypatch,
                                                                       module_name, class_name):
    """Intercept at map loading, before any subscription, publisher or timer is created."""
    from rclpy.node import Node
    from factory_bot.planning_grid import PlanningGrid
    module = importlib.import_module('factory_bot.' + module_name)
    monkeypatch.setenv('SELECTED_FACTORY_MAP', str(selected_map))
    overrides = {'map_yaml': '$SELECTED_FACTORY_MAP'}
    values = {}
    monkeypatch.setattr(Node, '__init__', lambda self, *a, **kw: None)
    def declare(self, name, default):
        values[name] = overrides.get(name, default)
        return SimpleNamespace(value=values[name])
    monkeypatch.setattr(Node, 'declare_parameter', declare)
    monkeypatch.setattr(Node, 'get_parameter', lambda self, name: SimpleNamespace(value=values[name]))
    loaded = []
    class MapLoaded(Exception):
        pass
    if module_name == 'icp_localizer':
        original = module.load_map
        def load(path):
            occupied, res, origin = original(path)
            loaded.append((path, occupied))
            raise MapLoaded
        monkeypatch.setattr(module, 'load_map', load)
    else:
        original = PlanningGrid.from_files
        def load(path, *args):
            grid = original(path, *args)
            loaded.append((path, grid.occupied))
            raise MapLoaded
        monkeypatch.setattr(PlanningGrid, 'from_files', staticmethod(load))
    with pytest.raises(MapLoaded):
        getattr(module, class_name)()
    path, occupied = loaded[0]
    assert path == str(selected_map)
    assert occupied.shape == (32, 32) and occupied.sum() == 1 and occupied[2, 3]
