"""Author compact station cargo, bake 512px atlases, export OBJ and a Blender catalog.

blender --factory-startup -b -t 6 -P scripts/build_cargo_assets.py -- --root .
Uses shared visual helpers from the existing factory authoring pipeline.
"""
from pathlib import Path
import json
import math
import bpy
from mathutils import Vector

common = Path(__file__).with_name('cargo_authoring_common.py')
exec(compile(common.read_text(), str(common), 'exec'), globals())
scene.name = 'NORTHLINE_Cargo_Catalog'
CARGO = PKG / 'models/station_cargo'
for folder in ['meshes', 'textures']:
    (CARGO / folder).mkdir(parents=True, exist_ok=True)


def label(text, loc, size=.009, mat='white'):
    curve = bpy.data.curves.new('Cargo label', 'FONT')
    curve.body = text; curve.size = size; curve.align_x = 'CENTER'
    obj = bpy.data.objects.new(text, curve); scene.collection.objects.link(obj)
    obj.location = loc; obj.data.materials.append(MATS[mat])
    bpy.ops.object.select_all(action='DESELECT'); obj.select_set(True)
    bpy.context.view_layer.objects.active = obj; bpy.ops.object.convert(target='MESH')
    obj.select_set(False)


def screwdriver_set():
    box('Tool carrier', (0,0,.003), (.126,.076,.006), 'graphite', .003)
    for y, color in [(-.019,'red'),(.019,'body_blue')]:
        cyl('Driver grip',(-.034,y,.015),.008,.037,color,'x',16)
        for x in [-.046,-.036,-.026]:
            cyl('Grip groove',(x,y,.015),.0085,.002,'rubber','x',12,False)
        cyl('Driver collar',(-.014,y,.015),.004,.005,'silver','x',12,False)
        cyl('Steel shaft',(.015,y,.015),.002,.054,'silver','x',12,False)
        box('Flat blade',(.046,y,.015),(.009,.005,.0018),'silver',0)
    label('TOOLS', (0, -.034, .0062), .007)


def toolbox():
    box('Tool case', (0,0,.013), (.112,.075,.026), 'orange', .005)
    box('Lid seal', (0,0,.025), (.114,.076,.002), 'rubber', .001)
    box('Lid', (0,0,.030), (.112,.075,.008), 'orange', .004)
    for x in [-.019,.019]:
        box('Handle foot',(x,0,.035),(.006,.012,.006),'graphite',.002)
    box('Handle crosspiece',(0,0,.037),(.044,.012,.004),'graphite',.002)
    for x in [-.034,.034]:
        box('Latch',(x,-.038,.025),(.012,.003,.011),'silver',.001)
    label('SERVICE',(0,-.022,.0343),.008,'black')


def sample_vials():
    box('Sample rack',(0,0,.004),(.113,.074,.008),'white',.003)
    for x in [-.035,0,.035]:
        for y in [-.018,.018]:
            cyl('Sealed blue liquid vial',(x,y,.020),.008,.027,'glass',vertices=12)
            cyl('Liquid viewing section',(x,y,.014),.0082,.014,'body_blue',vertices=12,bevel=False)
            cyl('Sample cap',(x,y,.036),.0088,.005,'white',vertices=12,bevel=False)
            box('Sample label',(x,y-.0083,.025),(.008,.001,.007),'white',0)
    label('QC',(0,-.033,.0082),.007,'black')


def parts_crate():
    box('Parts bin base',(0,0,.003),(.122,.079,.006),'teal',.003)
    for x in [-.058,.058]:
        box('Bin side',(x,0,.015),(.006,.079,.025),'teal',.002)
    for y in [-.036,.036]:
        box('Bin end',(0,y,.015),(.110,.006,.025),'teal',.002)
        box('Bin label',(0,y*1.09,.015),(.039,.002,.012),'white',.001)
    for x in [-.034,0,.034]:
        for y in [-.018,.018]:
            cyl('Machined nut',(x,y,.011),.009,.009,'silver',vertices=6,bevel=False)
            cyl('Nut bore',(x,y,.016),.004,.001,'black',vertices=8,bevel=False)


def component_tray():
    box('Component tray',(0,0,.004),(.123,.078,.008),'graphite',.003)
    for y in [-.036,.036]:
        box('Tray rim',(0,y,.009),(.12,.004,.010),'silver',.001)
    for x in [-.033,.031]:
        cyl('Machined wheel',(x,0,.015),.024,.010,'silver',vertices=24)
        cyl('Hub center',(x,0,.021),.009,.003,'graphite',vertices=16,bevel=False)
        for i in range(8):
            a=i*math.tau/8
            tooth=box('Gear tooth',(x+.025*math.cos(a),.025*math.sin(a),.015),(.009,.007,.010),'silver',.001)
            tooth.rotation_euler.z=a
    label('PARTS',(0,-.029,.0082),.007)


def battery_pack():
    box('Battery cartridge',(0,0,.014),(.110,.074,.028),'graphite',.004)
    box('Battery top',(0,0,.029),(.105,.070,.004),'body_blue',.002)
    label('24V',(-.013,-.012,.0312),.016)
    for i in range(4):
        box('Charge segment',(-.031+i*.016,.020,.032),(.011,.006,.002),'led',.0004)
    for x in [-.041,.041]:
        box('Connector',(x,-.038,.017),(.014,.006,.011),'silver',.001)


def station_tray():
    box('Station transfer mat',(0,0,.0015),(.610,.235,.003),'graphite',.001)
    for y in [-.116,.116]:
        box('Transfer mat rim',(0,y,.003),(.61,.003,.006),'silver',.001)
    box('Pickup demarcation',(-.14,0,.0031),(.002,.205,.0004),'yellow',0)
    box('Receive demarcation',(.14,0,.0031),(.002,.205,.0004),'led',0)
    label('PICKUP',(-.14,-.101,.0035),.012)
    label('RECEIVE',(.14,-.101,.0035),.012)


groups = {}; counts = {}; bounds = {}
for name, fn in [('screwdriver_set',screwdriver_set), ('toolbox',toolbox),
                 ('sample_vials',sample_vials), ('parts_crate',parts_crate),
                 ('component_tray',component_tray), ('battery_pack',battery_pack),
                 ('station_tray',station_tray)]:
    before=set(scene.objects); fn()
    objects=[o for o in scene.objects if o not in before and o.type=='MESH']
    groups[name]=objects
    counts[name]=export(objects,CARGO/'meshes'/f'{name}.obj')
    coords=[o.matrix_world@v.co for o in objects for v in o.data.vertices]
    bounds[name]={'min':[min(v[i] for v in coords) for i in range(3)],
                  'max':[max(v[i] for v in coords) for i in range(3)]}
    for o in objects:o.hide_render=True;o.hide_viewport=True

# Keep only materials referenced by the cargo OBJ meshes. Atlas images are lit
# diffuse textures, with distinct specularity for metal and matte surfaces.
mtl(CARGO/'meshes/premium.mtl')
text=(CARGO/'meshes/premium.mtl').read_text()
sections=text.split('newmtl ')
allowed=tuple(n+'_' for n in groups)
(CARGO/'meshes/premium.mtl').write_text(''.join('newmtl '+s for s in sections[1:] if s.splitlines()[0].startswith(allowed)))
(CARGO/'manifest.json').write_text(json.dumps({'triangles':counts,'bounds_m':bounds,
    'atlas_size':512,'collision_shapes':0,'note':'Kinematic visual cargo; deck bottom 0.179m, max cargo top below 0.23m lidar.'},indent=2)+'\n')
(CARGO/'model.config').write_text('<model><name>Station cargo assets</name><version>1.0</version><sdf version="1.9">model.sdf</sdf></model>')
(CARGO/'model.sdf').write_text('<sdf version="1.9"><model name="station_cargo"><static>true</static><link name="cargo"><visual name="sample"><geometry><mesh><uri>meshes/toolbox.obj</uri></mesh></geometry></visual></link></model></sdf>')

# Product catalog: six small items enlarged by camera framing, not runtime scale.
for i,name in enumerate(['parts_crate','screwdriver_set','sample_vials','component_tray','toolbox','battery_pack']):
    x=(i%3)*.22;y=(i//3)*.18
    for o in groups[name]:
        o.hide_render=False;o.hide_viewport=False;o.location+=Vector((x,y,0))
    label(name.replace('_',' ').upper(),(x,y-.062,0),.009)
world=scene.world
world.node_tree.nodes.get('Background').inputs['Strength'].default_value=.45
for loc,energy,size in [((.15,-.1,.7),65,.65),((.5,.5,.5),40,.5)]:
    data=bpy.data.lights.new('Catalog softbox','AREA');data.energy=energy;data.size=size
    obj=bpy.data.objects.new('Catalog softbox',data);scene.collection.objects.link(obj);obj.location=loc
    obj.rotation_euler=(Vector((.22,.09,0))-obj.location).to_track_quat('-Z','Y').to_euler()
data=bpy.data.cameras.new('Cargo catalog camera');data.type='ORTHO';data.ortho_scale=.80
camera=bpy.data.objects.new('Cargo catalog camera',data);scene.collection.objects.link(camera)
camera.location=(.60,-.65,.85);camera.rotation_euler=(Vector((.22,.09,0))-camera.location).to_track_quat('-Z','Y').to_euler()
scene.camera=camera;scene.render.resolution_x=1400;scene.render.resolution_y=1000
scene.render.film_transparent=True;scene.render.image_settings.file_format='PNG'
scene.render.filepath=str(ROOT/'work/cargo_catalog.png')
bpy.ops.render.render(write_still=True)
for image in bpy.data.images:
    if image.filepath and image.has_data and not image.packed_file:
        try:image.pack()
        except RuntimeError:pass
bpy.data.libraries.write(str(ROOT/'work/station_cargo.blend'),{scene},fake_user=True)
print('CARGO_COUNTS',json.dumps(counts),flush=True)
