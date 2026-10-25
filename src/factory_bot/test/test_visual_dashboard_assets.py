"""Visual detail must not replace simple contacts or obstruct the laser plane."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET


PKG=Path(__file__).resolve().parents[1]


def test_premium_runtime_triangle_budget_includes_both_robots_and_workers():
    factory=json.loads((PKG/'models/industrial_factory/manifest.json').read_text())['triangles']
    robot=json.loads((PKG/'models/robot_premium/manifest.json').read_text())['triangles']
    runtime=sum(factory.values())+factory['worker']+robot['robot_blue']+robot['robot_amber']+4*robot['wheel']+2*robot['lidar']
    assert runtime<100000
    for name,count in factory.items():
        assert count==sum(line.startswith('f ') for line in (PKG/f'models/industrial_factory/meshes/{name}.obj').open())


def test_robot_collision_primitives_and_scan_plane_remain_separate_from_detail():
    robot=ET.parse(PKG/'urdf/fleet_bot.urdf.xacro').getroot()
    assert not robot.findall('.//collision/geometry/mesh')
    base=robot.find("link[@name='base_link']/collision/geometry/box")
    assert base.get('size')=='${base_length} ${base_width} ${base_height}'
    for name in ['body_blue','body_amber']:
        z=[float(l.split()[3]) for l in (PKG/f'models/robot_premium/meshes/{name}.obj').read_text().splitlines() if l.startswith('v ')]
        assert max(z)+.11<.23
    z=[float(l.split()[3]) for l in (PKG/'models/robot_premium/meshes/lidar.obj').read_text().splitlines() if l.startswith('v ')]
    assert max(z)<0


def test_factory_manifest_contacts_match_the_sdf_and_are_still_primitives():
    world=ET.parse(PKG/'worlds/factory.sdf').getroot()
    structure=world.find("world/model[@name='factory_structure']/link")
    collisions=structure.findall('collision')
    manifest=json.loads((PKG/'models/industrial_factory/manifest.json').read_text())
    assert len(collisions)==manifest['static_collision_shapes']==20
    assert not world.findall('.//collision/geometry/mesh')
    for c in manifest['collisions']:
        sdf=structure.find("collision[@name='"+c['name']+"']")
        assert [float(x) for x in sdf.find('geometry/box/size').text.split()]==c['size']
        assert [float(x) for x in sdf.find('pose').text.split()[:3]]==c['xyz']


def test_camera_optical_frame_and_configurable_sensor_exist():
    robot=ET.parse(PKG/'urdf/fleet_bot.urdf.xacro').getroot()
    assert robot.find("link[@name='camera_link']") is not None
    optical=robot.find("joint[@name='camera_optical_joint']/origin")
    assert optical.get('rpy')=='-1.57079632679 0 -1.57079632679'
    gaz=ET.parse(PKG/'urdf/fleet_bot.gazebo.xacro').getroot()
    camera=gaz.find(".//sensor[@type='camera']")
    assert camera.find('camera/clip/near').text=='0.025'
    assert camera.find('camera/image/width').text=='$(arg camera_width)'
    assert camera.find('topic').text=='${ns}/camera/image_raw'
