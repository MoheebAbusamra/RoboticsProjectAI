"""Run in Blender: bevel factory visuals, author AMR meshes and render the dashboard atlas.

blender -b -P scripts/build_premium_assets.py -- --root /path/to/factory_ws --source work/visual-dashboard/base
Generate --source first with generate_factory.py --visual-source. Physics is never exported from Blender.
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
parser.add_argument('--source', type=Path, default=Path('work/base'))
args = parser.parse_args(argv)
ROOT = args.root.resolve()
SOURCE = args.source if args.source.is_absolute() else ROOT / args.source
sys.path.insert(0,str(ROOT/'scripts'))
from detail_assets import add_details, add_robot_details, configure_surface_detail, baked_copy
PKG = ROOT / 'src/factory_bot'
FACTORY = PKG / 'models/industrial_factory'
ROBOT = PKG / 'models/robot_premium'
DASH = PKG / 'models/dashboard'
for p in [FACTORY/'meshes', ROBOT/'meshes', DASH]: p.mkdir(parents=True, exist_ok=True)

# A separate scene preserves any existing work in the user's Blender session.
scene = bpy.data.scenes.new('NORTHLINE_Studio')
bpy.context.window.scene = scene
scene.render.engine = 'BLENDER_EEVEE'
scene.render.resolution_percentage = 100
scene.world = bpy.data.worlds.new('Northline atmosphere')
scene.world.use_nodes = True
scene.world.node_tree.nodes['Background'].inputs['Color'].default_value = (.13, .17, .20, 1)
scene.world.node_tree.nodes['Background'].inputs['Strength'].default_value = .4
scene.view_settings.view_transform = 'AgX'

def linear(v):
    return v / 12.92 if v <= .04045 else ((v+.055)/1.055)**2.4

def srgb(v):
    return v*12.92 if v <= .0031308 else 1.055*max(v,0)**(1/2.4)-.055

MATS = {}
def material(name, color, metal=0., rough=.5, emission=0.):
    m = bpy.data.materials.new('NL_' + name)
    m.use_nodes = True
    bs = m.node_tree.nodes.get('Principled BSDF')
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

for name in ('concrete','signs'):
    m=MATS[name]
    tex=m.node_tree.nodes.new('ShaderNodeTexImage')
    tex.image=bpy.data.images.load(str(FACTORY/'textures'/f'{name}.png'),check_existing=True)
    tex.image.reload()
    m.node_tree.links.new(tex.outputs['Color'],m.node_tree.nodes['Principled BSDF'].inputs['Base Color'])
    m['texture']=f'../textures/{name}.png'
    m['export_rgb']=[1.,1.,1.]

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

def imported(name,bevel):
    before=set(scene.objects)
    bpy.ops.wm.obj_import(filepath=str(SOURCE/'meshes'/f'{name}.obj'),forward_axis='Y',up_axis='Z',
                         use_split_objects=False,use_split_groups=False)
    objects=[o for o in scene.objects if o not in before and o.type=='MESH']
    for o in objects:
        for i,m in enumerate(o.data.materials):
            key=m.name.split('.')[0]
            if key in MATS:o.data.materials[i]=MATS[key]
        if bevel:
            bpy.context.view_layer.objects.active=o
            bpy.ops.object.mode_set(mode='EDIT');bpy.ops.mesh.select_all(action='SELECT')
            bpy.ops.mesh.remove_doubles(threshold=.000005);bpy.ops.object.mode_set(mode='OBJECT')
            # Highlight long manufactured edges. Tiny beams, labels and fasteners do
            # not need extra bevel geometry at simulator viewing distances.
            import bmesh
            bm=bmesh.new();bm.from_mesh(o.data)
            weights=bm.edges.layers.float.get('bevel_weight_edge') or bm.edges.layers.float.new('bevel_weight_edge')
            for edge in bm.edges:
                edge[weights]=1. if edge.is_manifold and edge.calc_length()>.16 and edge.calc_face_angle()>.65 else 0.
            bm.to_mesh(o.data);bm.free()
            b=o.modifiers.new('Precision edges','BEVEL');b.width=bevel;b.segments=1;b.limit_method='WEIGHT'
            b.harden_normals=True
            bpy.ops.object.modifier_apply(modifier=b.name)
            for p in o.data.polygons:p.use_smooth=True
            n=o.modifiers.new('Weighted normals','WEIGHTED_NORMAL');n.keep_sharp=True
            bpy.ops.object.modifier_apply(modifier=n.name)
        o.name='Factory_'+name
    return objects

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

configure_surface_detail(globals())
factory=imported('static_factory',.012)
ground=imported('static_ground',0)
for o in ground:
    uv=o.data.uv_layers.active
    if uv:
        for poly in o.data.polygons:
            if o.data.materials[poly.material_index]==MATS['concrete']:
                for li in poly.loop_indices:
                    v=o.matrix_world@o.data.vertices[o.data.loops[li].vertex_index].co
                    uv.data[li].uv=(v.x/16,v.y/14)
before=set(scene.objects)
# Additional visual-only detailing stays above the lidar's .23 m plane and within machine footprints.
manifest=json.loads((FACTORY/'manifest.json').read_text())
for c in manifest['collisions']:
    if c['name'].startswith('cnc_'):
        x,y,z=c['xyz'];sx,sy,_=c['size']
        for dx in [-.36,.36]:
            box('CNC access panel',(x+dx*sx,y+sy/2+.015,1.05),(.55,.018,.67),'teal',.015)
            for dz in range(7):box('Vent louvre',(x+dx*sx,y+sy/2+.030,.81+dz*.065),(.39,.012,.018),'black',.002)
        box('Machining display',(x+sx*.40,y-sy/2-.123,1.32),(.145,.006,.12),'glass',.004)
        for k in range(3):box('Display rows',(x+sx*.40,y-sy/2-.128,1.35-k*.029),(.10,.004,.008),'led',.001)
        for dx in [-sx*.43,sx*.43]:
            for dz in [.6,1.5]:cyl('Panel fastener',(x+dx,y-sy/2-.016,dz),.016,.008,'silver','y',12)
    if c['name'].startswith('rack_'):
        x,y,z=c['xyz']
        for sy in [-1.7,1.7]:
            for h in [.65,1.48,2.28]:
                box('Pallet inventory label',(x-.412,y+sy,h),(.007,.18,.10),'white',.001)
                for k in range(5):box('Barcode',(x-.418,y+sy-.055+k*.025,h),(.004,.009,.064),'black',0)
# Upper-wall conduit and aluminum service trays; no new physical obstacles.
for y in [13.82]:
    box('Cable tray',(8,y,2.84),(15.2,.18,.09),'silver',.012)
    for x in range(1,16):box('Tray mount',(x,y,2.72),(.045,.20,.25),'steel',.004)
for x in [15.80]:
    for z in [2.30,2.42]:cyl('Service pipe',(x,7,z),.029,13.3,'silver','y',16)
factory += [o for o in scene.objects if o not in before and o.type=='MESH']

forklift=imported('forklift',.018)
before=set(scene.objects)
for y in [-.30,.30]:
    for z in [.65,.9,1.15,1.4]:box('Mast chain',(.545,y,z),(.025,.028,.11),'black',.002)
    box('Tail reflector',(-.665,y,.58),(.016,.10,.07),'red',.006)
    box('Head lamp',(.44,y,.65),(.025,.12,.09),'lamp',.009)
sphere('Operator steering wheel',(.16,0,1.19),(.12,.12,.025),'black')
forklift += [o for o in scene.objects if o not in before and o.type=='MESH']
pallet=imported('pallet',.009)

before=set(scene.objects)
# Stylized industrial PPE worker with rounded joints, a hard hat and reflective vest.
for y in [-.105,.105]:
    box('Safety boot',(.04,y,.065),(.28,.165,.13),'rubber',.035)
    box('Trouser leg',(0,y,.42),(.16,.17,.64),'blue',.045)
sphere('Work jacket',(0,0,1.08),(.185,.235,.36),'blue')
box('High visibility vest',(-.005,0,1.12),(.35,.445,.50),'orange',.05)
for z in [.99,1.19]:box('Reflective stripe',(-.005,0,z),(.359,.453,.045),'silver',.008)
for y in [-.275,.275]:
    sphere('Jacket sleeve',(0,y,1.07),(.095,.09,.28),'blue')
    sphere('Glove',(.018,y,.80),(.075,.073,.105),'rubber')
cyl('Neck',(0,0,1.42),.065,.1,'wood')
sphere('Face',(.006,0,1.54),(.13,.12,.16),'cardboard')
box('Safety glasses',(.119,0,1.57),(.027,.20,.053),'glass',.012)
cyl('Hard hat brim',(0,0,1.68),.186,.026,'yellow',vertices=24)
sphere('Hard hat',(0,0,1.69),(.15,.15,.10),'yellow')
worker=[o for o in scene.objects if o not in before and o.type=='MESH']

add_details(globals())
counts={}
for name,objects in [('static_factory',factory),('static_ground',ground),('forklift',forklift),('pallet',pallet),('worker',worker)]:
    counts[name]=export(objects,FACTORY/'meshes'/f'{name}.obj')
mtl(FACTORY/'meshes/premium.mtl')
# Do not leave texture references in robot material libraries: its surfaces are procedural solids.

robot_objects={}
for variant in ['blue','amber']:
    before=set(scene.objects)
    body='body_'+variant
    box('Die cast lower chassis',(0,0,-.015),(.354,.294,.086),'graphite',.022)
    box('Painted upper shell',(0,0,.032),(.348,.286,.057),body,.025)
    box('Recessed payload deck',(-.025,0,.063),(.245,.232,.008),'rubber',.008)
    for y in [-.116,.116]:
        box('Deck rub rail',(-.025,y,.069),(.255,.012,.012),'silver',.004)
    for y in [-.145,.145]:
        box('Bumper band',(0,y,-.025),(.30,.012,.035),'rubber',.006)
        box('Robot ID badge',(-.09,y*1.035,.023),(.085,.006,.028),'white',.003)
        for k in range(6):box('Cooling vent',(.035+k*.012,y*1.028,.005),(.005,.006,.027),'black',.001)
    box('Front bumper',(.177,0,-.027),(.012,.25,.034),'rubber',.006)
    box('Rear bumper',(-.177,0,-.027),(.012,.25,.034),'rubber',.006)
    for y in [-.105,.105]:box('Status light',(.181,y,.021),(.007,.040,.011),'led',.002)
    # Lens is at the front surface; no visual or collision blocks the camera ray.
    cyl('Front camera bezel',(.180,0,.052),.020,.009,'graphite','x',24)
    cyl('Front camera lens',(.185,0,.052),.013,.003,'glass','x',24)
    for x in [-.14,.12]:
        for y in [-.105,.105]:cyl('Chassis fastener',(x,y,.063),.006,.003,'silver',vertices=12)
    cyl('Stop button foot',(-.128,.075,.073),.016,.011,'black')
    cyl('Stop button',(-.128,.075,.081),.012,.008,'red')
    # Shell at z <= .085 means world z <= .195: clear of the .23 m lidar plane.
    bpy.ops.object.select_all(action='DESELECT')
    add_robot_details(globals(),variant)
    robot_objects[variant]=[o for o in scene.objects if o not in before and o.type=='MESH']
    counts['robot_'+variant]=export(robot_objects[variant],ROBOT/'meshes'/f'body_{variant}.obj')
    for o in robot_objects[variant]:o.hide_render=True;o.hide_viewport=True
before=set(scene.objects)
cyl('Tire',(0,0,0),.050,.035,'rubber','y',24)
for y in [-.018,.018]:
    cyl('Machined wheel hub',(0,y,0),.030,.002,'silver','y',16,False)
    cyl('Axle cap',(0,y*1.07,0),.012,.003,'graphite','y',12,False)
    for a in range(6):
        t=a*math.tau/6
        cyl('Lug bolt',(.020*math.cos(t),y*1.1,.020*math.sin(t)),.0035,.002,'black','y',6,False)
wheel=[o for o in scene.objects if o not in before and o.type=='MESH']
counts['wheel']=export(wheel,ROBOT/'meshes/wheel.obj')
for o in wheel:o.hide_render=True;o.hide_viewport=True
before=set(scene.objects)
cyl('Lidar housing',(0,0,-.039),.034,.031,'graphite',vertices=32)
cyl('Lidar window',(0,0,-.022),.033,.007,'glass',vertices=32)
cyl('Lidar cap',(0,0,-.016),.031,.004,'silver',vertices=32)
lidar=[o for o in scene.objects if o not in before and o.type=='MESH']
counts['lidar']=export(lidar,ROBOT/'meshes/lidar.obj')
for o in lidar:o.hide_render=True;o.hide_viewport=True
mtl(ROBOT/'meshes/premium.mtl')

manifest['triangles']= {k:counts[k] for k in ['static_factory','static_ground','forklift','worker','pallet']}
manifest['visual_revision']='baked_detail_2026_10_25'
manifest['baking']='Diffuse color and local contact AO; direct simulator lights remain live'
manifest['texture_sizes']['static_factory_atlas']=[2048,2048]
manifest['texture_sizes']['concrete']=[2048,2048]
manifest['physics_revision']='Hazard fences removed; other primitive contacts unchanged'
(FACTORY/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
(ROBOT/'manifest.json').write_text(json.dumps({'triangles':{k:v for k,v in counts.items() if k.startswith('robot_') or k in ['wheel','lidar']},
    'base_collision_m':[.36,.30,.12],'wheel_radius_m':.05,'wheel_separation_m':.35,'laser_world_height_m':.23,
    'note':'Visual-only shell. Existing inertia, contacts, controllers and scan plane are retained.'},indent=2)+'\n')

# Move individual props off the static factory for its clean atlas render.
for group in [forklift,pallet,worker]:
    for o in group:o.hide_render=True;o.hide_viewport=True

def light(name,loc,energy,color,size=7):
    data=bpy.data.lights.new(name,'AREA');data.energy=energy;data.shape='DISK';data.size=size;data.color=color
    o=bpy.data.objects.new(name,data);scene.collection.objects.link(o);o.location=loc
    return o

light('Large soft skylight',(5,3,13),2700,(.82,.91,1.0),10)
light('Warm task lighting',(12,10,10),1900,(1.,.90,.72),7)
light('Rim on machines',(-2,12,9),2200,(.70,.83,1.),8)
sun_data=bpy.data.lights.new('Daylight key','SUN');sun_data.energy=1.1;sun_data.angle=.16
sun=bpy.data.objects.new('Daylight key',sun_data);scene.collection.objects.link(sun);sun.rotation_euler=(.4,-.5,-.3)

camera_data=bpy.data.cameras.new('Factory atlas orthographic')
camera=bpy.data.objects.new('Factory atlas orthographic',camera_data);scene.collection.objects.link(camera)
camera.location=(24,-22,13.0);target=Vector((8,7,.3))
camera.rotation_euler=(target-camera.location).to_track_quat('-Z','Y').to_euler()
camera_data.type='ORTHO';camera_data.ortho_scale=24.4;scene.camera=camera
scene.render.resolution_x=1500;scene.render.resolution_y=900
scene.render.film_transparent=True
scene.render.image_settings.file_format='PNG';scene.render.filepath=str(DASH/'factory_overview.png')
scene.render.image_settings.color_mode='RGBA'
bpy.context.view_layer.update()
projection={}
for key,xyz in [('origin',(0,0,0)),('x',(1,0,0)),('y',(0,1,0))]:
    v=world_to_camera_view(scene,camera,Vector(xyz));projection[key]=[v.x,1-v.y]
projection.update({'size':[1500,900],'world_offset':[1,1],'kind':'orthographic','source':'Blender rendered static world; live ROS routes and poses are overlaid'})
(DASH/'projection.json').write_text(json.dumps(projection,indent=2)+'\n')
(ROOT/'work/authored').mkdir(parents=True,exist_ok=True)
bpy.ops.wm.save_as_mainfile(filepath=str(ROOT/'work/authored/northline_detail.blend'))
bpy.ops.render.render(write_still=True)
import numpy as np
render=bpy.data.images.load(str(DASH/'factory_overview.png'),check_existing=False)
pixels=np.empty(len(render.pixels),dtype=np.float32);render.pixels.foreach_get(pixels)
rgba=pixels.reshape(scene.render.resolution_y,scene.render.resolution_x,4)
rows,cols=np.where(rgba[:,:,3]>.01)
projection['content_bounds']=[float(cols.min()/1500),float(1-(rows.max()+1)/900),
                              float((cols.max()+1)/1500),float(1-rows.min()/900)]
(DASH/'projection.json').write_text(json.dumps(projection,indent=2)+'\n')
bpy.data.images.remove(render)
factory_scene=scene
studio=bpy.data.scenes.new('NORTHLINE_Robot_Studio')
bpy.context.window.scene=studio
studio.render.engine='BLENDER_EEVEE'
studio.world=factory_scene.world.copy()
studio.world.node_tree.nodes['Background'].inputs['Strength'].default_value=.5
studio.render.film_transparent=True
studio.render.resolution_x=900;studio.render.resolution_y=650;studio.render.resolution_percentage=100
studio.render.image_settings.file_format='PNG';studio.render.image_settings.color_mode='RGBA'
studio.view_settings.view_transform='AgX'
camdata=bpy.data.cameras.new('Robot product camera');camdata.type='ORTHO';camdata.ortho_scale=.70
cam=bpy.data.objects.new('Robot product camera',camdata);studio.collection.objects.link(cam)
cam.location=(.65,-.85,.63);cam.rotation_euler=(Vector((0,0,.015))-cam.location).to_track_quat('-Z','Y').to_euler()
studio.camera=cam
for name,loc,energy,size,col in [('Softbox',(.3,-.4,1),55,.8,(.82,.91,1.)),('Edge light',(-.3,.5,.7),45,.6,(1.,.89,.72))]:
    data=bpy.data.lights.new(name,'AREA');data.energy=energy;data.shape='DISK';data.size=size;data.color=col
    o=bpy.data.objects.new(name,data);studio.collection.objects.link(o);o.location=loc
    o.rotation_euler=(-o.location).to_track_quat('-Z','Y').to_euler()
studio_objects={}
for variant in ['blue','amber']:
    created=[]
    for filename,offset in [(f'body_{variant}',(0,0,0)),('wheel',(0,-.175,-.06)),('wheel',(0,.175,-.06)),('lidar',(.10,0,.12))]:
        before=set(studio.objects)
        bpy.ops.wm.obj_import(filepath=str(ROBOT/'meshes'/f'{filename}.obj'),forward_axis='Y',up_axis='Z',use_split_objects=False,use_split_groups=False)
        for o in studio.objects:
            if o not in before:
                if o.type=='MESH':
                    for i,m in enumerate(o.data.materials):
                        key=m.name.removeprefix('NL_').split('.')[0]
                        if key in MATS:o.data.materials[i]=MATS[key]
                o.location+=Vector(offset);created.append(o)
    studio.render.filepath=str(DASH/f'robot_{variant}.png')
    bpy.ops.render.render(write_still=True)
    studio_objects[variant]=created
    for o in created:o.hide_render=True;o.hide_viewport=True
for o in studio_objects['blue']:o.hide_render=False;o.hide_viewport=False
bpy.context.window.scene=factory_scene
for m in MATS.values():
    for node in m.node_tree.nodes:
        if node.type=='TEX_IMAGE' and node.image and not node.image.packed_file:node.image.pack()
bpy.ops.wm.save_as_mainfile(filepath=str(ROOT/'work/authored/northline_detail.blend'))
bpy.data.libraries.write(str(ROOT/'work/authored/northline_detail_clean.blend'),{factory_scene,studio},fake_user=True)
print('NORTHLINE_ASSET_COUNTS',json.dumps(counts))
