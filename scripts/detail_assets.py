"""Blender authoring helpers: visual detail and diffuse/AO atlas baking for Ogre1.

No collision meshes, sensor frames, masses or navigation code are authored here.
High-frequency metal finish is baked; live directional/spot lighting remains active.
"""
from pathlib import Path
import math
import bpy


def add_details(api):
    box, cyl, scene = api['box'], api['cyl'], api['scene']
    factory = api['factory']
    before = set(scene.objects)
    for x, y, sx, sy in [(12.5, 11.8, 2., 1.6), (13.65, 3.5, 1.7, 1.7)]:
        # Control keypad, access-door trim and handles, inside existing machine envelope.
        for dx in [-.31, .16]:
            box('Machine door seam', (x+dx*sx,y-sy/2-.012,1.12),(.009,.008,.83),'black',0)
        for dx in [-.24,.23]:
            cyl('Access door handle',(x+dx*sx,y-sy/2-.027,1.02),.015,.24,'silver','z',12)
        for row in range(3):
            for col in range(3):
                box('Control key',(x+sx*.40+(col-1)*.031,y-sy/2-.130,1.215-row*.028),(.020,.005,.016),'white',.002)
        # Hose bundle and upper service panel do not enter the laser-height envelope.
        for k in range(3):
            cyl('Coolant hose',(x-sx*.32+k*.065,y+sy*.37,1.50),.014,.62,'rubber','z',12,False)
        box('Service grille',(x,y+sy/2+.01,1.72),(.72,.025,.25),'steel',.006)
        for k in range(12):
            box('Service grille slot',(x-.31+k*.055,y+sy/2+.026,1.72),(.018,.004,.18),'black',0)
    # Press fittings, hydraulic cylinders and pressure indicator.
    for x in [6.82,8.18]:
        cyl('Hydraulic cylinder',(x,3.58,1.59),.072,.55,'graphite',vertices=16)
        cyl('Piston rod',(x,3.58,1.23),.033,.25,'silver',vertices=16)
        for y in [3.02,3.98]:
            cyl('Press cap bolt',(x,y,2.263),.027,.017,'silver',vertices=6,bevel=False)
    cyl('Pressure dial',(7.85,2.734,1.82),.075,.025,'white','y',24)
    box('Pressure needle',(7.85,2.718,1.84),(.005,.005,.06),'red',0)
    # Hollow-looking ceiling fixtures with LED diffusers; no extra real-time lights.
    for x in [2.,13.8]:
        for y in [5.1,12.9]:
            box('Luminaire backplate',(x,y,3.52),(1.65,.28,.055),'graphite',.014)
            for dx in [-.76,.76]:
                cyl('Luminaire suspension',(x+dx,y,3.70),.009,.34,'silver',vertices=8,bevel=False)
            for dy in [-.085,.085]:
                box('LED diffuser',(x,y+dy,3.479),(1.48,.052,.012),'lamp',.004)
    # Racking gussets and shelf stops add detail without extra physical obstacles.
    for x in [2.7,4.5]:
        for y in [8.,11.4]:
            for z in [.72,1.55,2.35]:
                box('Shelf end stop',(x,y,z),(.70,.032,.065),'orange',.005)
    factory += [o for o in scene.objects if o not in before and o.type=='MESH']


def add_robot_details(api, variant):
    box,cyl=api['box'],api['cyl']
    # All shell detail is below the .23 m scan plane (.12 m above base origin).
    for x in [-.088,-.018,.052]:
        box('Deck anti-slip groove',(x,0,.068),(.006,.19,.003),'graphite',0)
    for y in [-.146,.146]:
        box('Recessed service hatch',(-.085,y,.015),(.105,.003,.047),'graphite',.003)
        for x in [-.126,-.044]:
            cyl('Hatch quarter-turn screw',(x,y*1.023,.016),.004,.002,'silver','y',8,False)
        for k in range(4):
            box('Hatch safety stripe',(-.123+k*.020,y*1.028,.033),(.008,.002,.009),'yellow',0)
        box('Laser warning badge',(.116,y*1.025,.034),(.021,.003,.018),'yellow',.002)
    # Headlights lie beside camera, never across its optical ray.
    for y in [-.105,.105]:
        box('Lamp bezel',(.180,y,-.002),(.009,.048,.022),'graphite',.004)
        box('Headlight',(.186,y,-.002),(.003,.031,.011),'lamp',.002)
    box('Rear docking contacts',(-.181,0,-.008),(.004,.085,.023),'silver',.002)
    for y in [-.023,.023]:
        box('Dock contact insulator',(-.184,y,-.008),(.002,.008,.021),'black',0)
    # Physical geometry is intentionally simple; identity lettering is visual only.
    font=bpy.data.curves.new('Robot identity lettering','FONT')
    font.body='R01' if variant=='blue' else 'R02'
    font.size=.021;font.extrude=0;font.align_x='CENTER'
    o=bpy.data.objects.new('Robot identity '+variant,font)
    api['scene'].collection.objects.link(o);o.location=(-.080,0,.069)
    o.data.materials.append(api['MATS']['white'])
    bpy.context.view_layer.objects.active=o;o.select_set(True)
    bpy.ops.object.convert(target='MESH');o.select_set(False)


def configure_surface_detail(api):
    """CC0 metal finish and subtle paint microtexture; bake into ordinary diffuse maps."""
    cc0=api['ROOT']/'work/cc0'
    metal_path=cc0/'Metal032_1K-JPG_Color.jpg'
    for key in ['steel','edge','silver','graphite','body_blue','body_amber','teal','red','white']:
        m=api['MATS'][key];nodes=m.node_tree.nodes;links=m.node_tree.links
        bs=next(n for n in nodes if n.type=='BSDF_PRINCIPLED')
        tex=nodes.new('ShaderNodeTexImage');tex.image=bpy.data.images.load(str(metal_path),check_existing=True)
        tex.projection='BOX';tex.projection_blend=.1
        coord=nodes.new('ShaderNodeTexCoord');links.new(coord.outputs['Generated'],tex.inputs['Vector'])
        tint=nodes.new('ShaderNodeMixRGB');tint.blend_type='MULTIPLY';tint.inputs[0].default_value=.18
        tint.inputs[1].default_value=bs.inputs['Base Color'].default_value
        links.new(tex.outputs['Color'],tint.inputs[2]);links.new(tint.outputs[0],bs.inputs['Base Color'])


def baked_copy(objects, path, api):
    """Join a disposable export copy, unwrap once and bake material color times local AO.

    Original authored objects and materials remain editable. No directional shadows are
    baked onto moving robots, so illumination still changes when they move in Gazebo.
    """
    scene=api['scene'];size=2048 if path.stem=='static_factory' else (1024 if path.stem.startswith('body_') else 512)
    bpy.ops.object.select_all(action='DESELECT')
    copies=[]
    for obj in objects:
        c=obj.copy();c.data=obj.data.copy();scene.collection.objects.link(c)
        c.hide_render=False;c.hide_viewport=False;c.hide_set(False);c.select_set(True);copies.append(c)
    bpy.context.view_layer.objects.active=copies[0]
    bpy.ops.object.join();joined=bpy.context.object;joined.name='BAKED_'+path.stem
    bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
    # Preserve any old UV used for labels; baking uses a new active render atlas UV.
    old_uv=joined.data.uv_layers.active
    if old_uv:
        old_uv.name='SourceUV'
        for material in joined.data.materials:
            if material:
                for n in material.node_tree.nodes:
                    if n.type=='TEX_IMAGE' and not n.inputs['Vector'].is_linked:
                        uvnode=material.node_tree.nodes.new('ShaderNodeUVMap');uvnode.uv_map='SourceUV'
                        material.node_tree.links.new(uvnode.outputs['UV'],n.inputs['Vector'])
    uv=joined.data.uv_layers.new(name='BakeUV');joined.data.uv_layers.active=uv;uv.active_render=True
    bpy.ops.object.mode_set(mode='EDIT');bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.uv.smart_project(angle_limit=math.radians(66),island_margin=.003)
    bpy.ops.object.mode_set(mode='OBJECT')
    image=bpy.data.images.new('Atlas_'+path.stem,width=size,height=size,alpha=False)
    originals=list(joined.data.materials);temporary=[]
    groups=[]
    for index,mat in enumerate(originals):
        if not mat:continue
        metal=float(mat.get('export_metal',0));emission=float(mat.get('emission',0))
        groups.append('glow' if emission else ('metal' if metal>=.4 else 'matte'))
        m=mat.copy();temporary.append(m);joined.data.materials[index]=m
        nodes,links=m.node_tree.nodes,m.node_tree.links
        bs=next(n for n in nodes if n.type=='BSDF_PRINCIPLED')
        out=next(n for n in nodes if n.type=='OUTPUT_MATERIAL')
        source=bs.inputs['Base Color']
        ao=nodes.new('ShaderNodeAmbientOcclusion');ao.inputs['Distance'].default_value=.32 if path.stem=='static_factory' else .025
        ao.inputs['Color'].default_value=(1,1,1,1)
        mix=nodes.new('ShaderNodeMixRGB');mix.blend_type='MULTIPLY';mix.inputs[0].default_value=.55
        if source.is_linked:links.new(source.links[0].from_socket,mix.inputs[1])
        else:mix.inputs[1].default_value=source.default_value
        links.new(ao.outputs['Color'],mix.inputs[2])
        emit=nodes.new('ShaderNodeEmission');links.new(mix.outputs[0],emit.inputs['Color']);links.new(emit.outputs[0],out.inputs['Surface'])
        target=nodes.new('ShaderNodeTexImage');target.image=image;nodes.active=target
    previous=scene.render.engine
    scene.render.engine='CYCLES';scene.cycles.samples=8;scene.cycles.device='CPU'
    scene.render.bake.margin=6;scene.render.bake.use_clear=True
    # Only the joined copy participates: coincident authoring originals must not
    # darken the atlas by occluding their own disposable duplicate.
    visibility={o:o.hide_render for o in scene.objects}
    for o in scene.objects:o.hide_render=(o!=joined)
    bpy.ops.object.bake(type='EMIT')
    for o,hidden in visibility.items():o.hide_render=hidden
    scene.render.engine=previous
    texture_dir=path.parent.parent/'textures';texture_dir.mkdir(exist_ok=True)
    texture_path=texture_dir/(path.stem+'_atlas.jpg')
    image.filepath_raw=str(texture_path);image.file_format='JPEG';image.save()
    polygon_groups=[groups[p.material_index] for p in joined.data.polygons]
    joined.data.materials.clear()
    for group in ['matte','metal','glow']:
        key=path.stem+'_'+group
        m=api['material'](key,(1,1,1),.65 if group=='metal' else 0,.32 if group=='metal' else .72,.35 if group=='glow' else 0)
        m['texture']='../textures/'+texture_path.name
        bs=next(n for n in m.node_tree.nodes if n.type=='BSDF_PRINCIPLED')
        tex=m.node_tree.nodes.new('ShaderNodeTexImage');tex.image=image
        m.node_tree.links.new(tex.outputs['Color'],bs.inputs['Base Color'])
        joined.data.materials.append(m)
    group_ids={'matte':0,'metal':1,'glow':2}
    for poly,group in zip(joined.data.polygons,polygon_groups):poly.material_index=group_ids[group]
    # Release temporary materials to avoid packing redundant bake node graphs.
    for m in temporary:bpy.data.materials.remove(m)
    print('BAKED',path.stem,size,flush=True)
    return joined
