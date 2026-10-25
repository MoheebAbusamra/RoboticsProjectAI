"""Generic material, mesh, lit OBJ export and bake helpers derived from the factory authoring pipeline.
Used only by build_cargo_assets.py inside Blender, independently of ROS.
"""
import argparse
import json
import math
from pathlib import Path
import sys

import bpy
from mathutils import Vector
from bpy_extras.object_utils import world_to_camera_view

argv = sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else []
parser = argparse.ArgumentParser()
parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
args = parser.parse_args(argv)
ROOT = args.root.resolve()
sys.path.insert(0,str(ROOT/'scripts'))
from detail_assets import add_details, add_robot_details, configure_surface_detail, baked_copy
PKG = ROOT / 'src/factory_bot'
FACTORY = PKG / 'models/industrial_factory'
ROBOT = PKG / 'models/robot_premium'
DASH = PKG / 'models/dashboard'

# A separate scene preserves any existing work in the user's Blender session.
scene = bpy.data.scenes.new('NORTHLINE_Studio')
bpy.context.window.scene = scene
scene.render.engine = 'BLENDER_EEVEE'
scene.render.resolution_percentage = 100
scene.world = bpy.data.worlds.new('Northline atmosphere')
scene.world.use_nodes = True
next(n for n in scene.world.node_tree.nodes if n.type=='BACKGROUND').inputs['Color'].default_value = (.13, .17, .20, 1)
next(n for n in scene.world.node_tree.nodes if n.type=='BACKGROUND').inputs['Strength'].default_value = .4
scene.view_settings.view_transform = 'AgX'

def linear(v):
    return v / 12.92 if v <= .04045 else ((v+.055)/1.055)**2.4

def srgb(v):
    return v*12.92 if v <= .0031308 else 1.055*max(v,0)**(1/2.4)-.055

MATS = {}
def material(name, color, metal=0., rough=.5, emission=0.):
    m = bpy.data.materials.new('NL_' + name)
    m.use_nodes = True
    bs = next(n for n in m.node_tree.nodes if n.type=='BSDF_PRINCIPLED')
    rgb = tuple(linear(v) for v in color)
    bs.inputs['Base Color'].default_value = (*rgb,1)
    bs.inputs['Metallic'].default_value = metal
    bs.inputs['Roughness'].default_value = rough
    if emission:
        bs.inputs['Emission Color'].default_value = (*rgb,1)
        bs.inputs['Emission Strength'].default_value = emission
    m.diffuse_color = (*rgb,1)
    m['export_rgb'] = list(color)
    m['export_metal'] = metal
    m['export_rough'] = rough
    m['emission'] = emission
    MATS[name]=m
    return m

for name,color,metal,rough in [
    ('concrete',(.30,.32,.34),.18,.30), ('wall',(.66,.69,.67),0,.7),
    ('steel',(.20,.25,.26),.65,.37), ('edge',(.53,.57,.57),.8,.29),
    ('black',(.10,.12,.13),.05,.6), ('teal',(.24,.40,.39),.4,.35),
    ('orange',(.92,.39,.10),.2,.4), ('yellow',(.94,.72,.19),.2,.4),
    ('wood',(.48,.32,.18),0,.8), ('cardboard',(.67,.53,.35),0,.9),
    ('white',(.81,.84,.80),.15,.45), ('red',(.70,.18,.12),.2,.4),
    ('blue',(.24,.39,.56),.4,.35), ('qc',(.20,.38,.36),0,.7),
    ('slow',(.48,.37,.22),0,.7), ('hazard',(.39,.23,.17),0,.7),
    ('glass',(.13,.26,.29),.45,.16), ('lamp',(.94,.97,.89),0,.4),
    ('signs',(1,1,1),0,.6), ('silver',(.65,.71,.72),.8,.25),
    ('rubber',(.055,.065,.070),0,.78), ('graphite',(.14,.18,.19),.45,.4),
    ('body_blue',(.23,.53,.76),.55,.3), ('body_amber',(.94,.58,.20),.45,.3),
    ('led',(.64,.94,.66),0,.3)]:
    material(name,color,metal,rough, 2.5 if name in ('led','lamp') else 0)


def finish(obj, name, mat, bevel=0.):
    obj.name=name
    obj.data.materials.clear()
    obj.data.materials.append(MATS[mat])
    if bevel:
        b=obj.modifiers.new('Manufactured edge radius','BEVEL')
        b.width=bevel; b.segments=1
        b.limit_method='ANGLE'; b.angle_limit=.5
        b.harden_normals=True
        bpy.context.view_layer.objects.active=obj
        bpy.ops.object.modifier_apply(modifier=b.name)
        for poly in obj.data.polygons: poly.use_smooth=True
        n=obj.modifiers.new('Weighted panel normals','WEIGHTED_NORMAL')
        n.keep_sharp=True; n.weight=50
        bpy.ops.object.modifier_apply(modifier=n.name)
    return obj

def box(name,loc,size,mat='steel',bevel=.008):
    bpy.ops.mesh.primitive_cube_add(size=1,location=loc)
    o=bpy.context.object; o.dimensions=size
    bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
    return finish(o,name,mat,min(bevel,min(size)*.22))

def cyl(name,loc,radius,depth,mat='steel',axis='z',vertices=16,bevel=True):
    rotation=(math.pi/2,0,0) if axis=='y' else ((0,math.pi/2,0) if axis=='x' else (0,0,0))
    bpy.ops.mesh.primitive_cylinder_add(vertices=vertices,radius=radius,depth=depth,location=loc,rotation=rotation)
    return finish(bpy.context.object,name,mat,min(radius*.07,depth*.12) if bevel else 0)

def sphere(name,loc,scale,mat):
    bpy.ops.mesh.primitive_uv_sphere_add(segments=16,ring_count=10,radius=1,location=loc)
    o=bpy.context.object; o.scale=scale
    bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
    for p in o.data.polygons:p.use_smooth=True
    return finish(o,name,mat)

def mtl(path):
    lines=[]
    for key,m in MATS.items():
        if path.parent.parent==ROBOT and not key.startswith(('body_','wheel_','lidar_')):continue
        rgb=m['export_rgb'];rough=m['export_rough'];metal=m['export_metal']
        spec=.05+.43*metal
        lines += ['newmtl '+key,'Ka '+' '.join(f'{v*.35:.5f}' for v in rgb),
                  'Kd '+' '.join(f'{v:.5f}' for v in rgb),f'Ks {spec:.4f} {spec:.4f} {spec:.4f}',
                  f'Ns {max(5,100*(1-rough)):.1f}','d 1','illum 2']
        if m.get('texture'):lines.append('map_Kd '+m['texture'])
        if m.get('emission'):lines.append('Ke '+' '.join(f'{v*.5:.4f}' for v in rgb))
        lines.append('')
    path.write_text('\n'.join(lines))

def export(objects,path,library='premium.mtl'):
    """Explicit normals/UVs, one material batch per mesh; no Blender axis conversion."""
    baked=None
    if path.stem!='static_ground':
        baked=baked_copy(objects,path,globals());objects=[baked]
    verts=[];uvs=[];normals=[];groups={};lookup={}
    for o in objects:
        mesh=o.to_mesh();mesh.calc_loop_triangles()
        normalmat=o.matrix_world.to_3x3().inverted().transposed()
        uv=mesh.uv_layers.active
        for tri in mesh.loop_triangles:
            mat=mesh.materials[tri.material_index] if mesh.materials else MATS['steel']
            key=mat.name.removeprefix('NL_').split('.')[0]
            ids=[]
            for li in tri.loops:
                vi=mesh.loops[li].vertex_index
                v=o.matrix_world@mesh.vertices[vi].co
                n=normalmat@mesh.corner_normals[li].vector;n.normalize()
                t=uv.data[li].uv if uv else (0,0)
                corner=tuple(round(x,6) for x in (*v,*t,*n))
                if corner not in lookup:
                    verts.append(tuple(v));normals.append(tuple(n));uvs.append(tuple(t));lookup[corner]=len(verts)
                ids.append(lookup[corner])
            groups.setdefault(key,[]).append(ids)
        o.to_mesh_clear()
    with path.open('w') as f:
        f.write('mtllib '+library+'\no '+path.stem+'\n')
        for v in verts:f.write('v '+' '.join(f'{x:.6f}' for x in v)+'\n')
        for v in uvs:f.write('vt '+' '.join(f'{x:.6f}' for x in v)+'\n')
        for v in normals:f.write('vn '+' '.join(f'{x:.6f}' for x in v)+'\n')
        for key,faces in groups.items():
            f.write('usemtl '+key+'\n')
            for ids in faces:f.write('f '+' '.join(f'{i}/{i}/{i}' for i in ids)+'\n')
    if baked:
        baked.hide_render=True;baked.hide_viewport=True
    return sum(len(f) for f in groups.values())

