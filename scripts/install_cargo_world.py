"""Idempotently add collision-free transfer props to the existing factory SDF.

Run after regenerating the factory world. Coordinates and robot deck slots live
in config/cargo.yaml. No robot, map, station goal or hazard policy is altered.
"""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / 'src/factory_bot'
sys.path.insert(0, str(PKG))
from factory_bot.cargo_scene import stock_name, payload_name, offset_pose


def add_model(world, name, mesh, pose):
    model = ET.SubElement(world, 'model', name=name)
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = ' '.join(map(str, [*pose[:3], 0, 0, pose[3]]))
    link = ET.SubElement(model, 'link', name='prop')
    visual = ET.SubElement(link, 'visual', name='cargo')
    ET.SubElement(visual, 'cast_shadows').text = 'true'
    geometry = ET.SubElement(visual, 'geometry')
    ET.SubElement(ET.SubElement(geometry, 'mesh'), 'uri').text = mesh


def main():
    config = yaml.safe_load((PKG/'config/cargo.yaml').read_text())
    path = PKG/'worlds/factory.sdf'; tree = ET.parse(path); world = tree.getroot().find('world')
    for model in list(world.findall('model')):
        if model.get('name', '').startswith('cargo_'):
            world.remove(model)
    for station, spec in config['stations'].items():
        add_model(world, 'cargo_tray_'+station,
                  'model://station_cargo/meshes/station_tray.obj', spec['tray_pose'])
        for i, (dx, dy) in enumerate(config['stock_offsets']):
            add_model(world, stock_name(station,i), config['types'][spec['type']]['mesh'],
                      offset_pose(spec['pickup_pose'],dx,dy))
    for robot in config['robots']:
        for slot in range(len(config['deck_slots'])):
            for kind, spec in config['types'].items():
                add_model(world, payload_name(robot,slot,kind), spec['mesh'], config['parked'])
    ET.indent(tree,space='  '); tree.write(path,encoding='utf-8',xml_declaration=True)
    print('Added seven transfer trays, fourteen supply props and twenty-four reusable payload props')


if __name__ == '__main__':
    main()
