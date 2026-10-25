#!/usr/bin/env python3
"""Reproducible low-poly factory assets, SDF, and matching geometric reference map.

The reference map is derived from the static collision footprint at lidar height;
it is a simulator ground-truth map, not a claimed SLAM measurement.
"""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageFont, ImageFilter

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / 'src/factory_bot'
ASSET = PKG / 'models/industrial_factory'
WIDTH, HEIGHT = 16.0, 14.0
MATERIALS = {
    'concrete': (.38, .40, .39), 'wall': (.48, .51, .49),
    'steel': (.18, .23, .24), 'edge': (.34, .39, .39),
    'black': (.045, .055, .057), 'teal': (.18, .34, .34),
    'orange': (.72, .29, .075), 'yellow': (.93, .65, .12),
    'wood': (.48, .32, .17), 'cardboard': (.64, .49, .31),
    'white': (.76, .78, .73), 'red': (.60, .16, .10),
    'blue': (.16, .29, .43), 'qc': (.18, .40, .42),
    'slow': (.46, .34, .20), 'hazard': (.37, .22, .17),
    'glass': (.08, .16, .18), 'lamp': (.92, .93, .81),
    'signs': (1., 1., 1.),
}


class Mesh:
    """One mesh per asset, batched into shared material groups."""
    def __init__(self):
        self.verts, self.uvs, self.faces = [], [], []

    def face(self, verts, material, uvs=None):
        start = len(self.verts) + 1
        self.verts.extend(verts)
        self.uvs.extend(uvs or [(0, 0), (1, 0), (1, 1), (0, 1)][:len(verts)])
        for i in range(1, len(verts) - 1):
            self.faces.append((material, (start, start+i, start+i+1)))

    def box(self, xyz, size, material='steel', yaw=0):
        x, y, z = xyz
        a, b, c = [v/2 for v in size]
        co, si = math.cos(yaw), math.sin(yaw)
        def p(v):
            q, r, s = v
            return (x+co*q-si*r, y+si*q+co*r, z+s)
        v = [p(t) for t in [(-a,-b,-c),(a,-b,-c),(a,b,-c),(-a,b,-c),
                            (-a,-b,c),(a,-b,c),(a,b,c),(-a,b,c)]]
        for ids in [(3,2,1,0),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7),(4,5,6,7)]:
            coords=[v[i] for i in ids]
            uv=None
            if material=='concrete':
                uv=[(q[0]/3,q[1]/3) for q in coords]
            self.face(coords,material,uv)

    def cylinder(self, xyz, radius, height, material='steel', segments=12, axis='z'):
        x,y,z=xyz
        def p(t,h):
            a,b=radius*math.cos(t),radius*math.sin(t)
            return (x+a,y+b,z+h) if axis=='z' else (x+a,y+h,z+b)
        for i in range(segments):
            t,u=i*math.tau/segments,(i+1)*math.tau/segments
            self.face([p(t,-height/2),p(u,-height/2),p(u,height/2),p(t,height/2)],material)
            self.face([(x,y,z-height/2) if axis=='z' else (x,y-height/2,z),p(u,-height/2),p(t,-height/2)],material)
            self.face([(x,y,z+height/2) if axis=='z' else (x,y+height/2,z),p(t,height/2),p(u,height/2)],material)

    def sign(self, xyz, size, index, vertical=False):
        x,y,z=xyz; w,h=size
        col,row=index%2,index//2
        u0,u1=col*.5,(col+1)*.5
        v0,v1=1-(row+1)/8,1-row/8
        pts=[(x-w/2,y-h/2,z),(x+w/2,y-h/2,z),(x+w/2,y+h/2,z),(x-w/2,y+h/2,z)]
        if vertical:
            pts=[(x-w/2,y,z-h/2),(x+w/2,y,z-h/2),(x+w/2,y,z+h/2),(x-w/2,y,z+h/2)]
        self.face(pts,'signs',[(u0,v0),(u1,v0),(u1,v1),(u0,v1)])

    def write(self, name):
        path=ASSET/'meshes'/f'{name}.obj'
        with path.open('w') as f:
            f.write('mtllib factory.mtl\no '+name+'\ns off\n')
            for p in self.verts: f.write('v '+' '.join(f'{q:.5f}' for q in p)+'\n')
            for p in self.uvs: f.write('vt '+' '.join(f'{q:.6f}' for q in p)+'\n')
            normals=np.zeros((len(self.verts),3),dtype=float)
            vertices=np.asarray(self.verts)
            for _, ids in self.faces:
                a,b,c=(vertices[i-1] for i in ids)
                normal=np.cross(b-a,c-a)
                length=np.linalg.norm(normal)
                if length>1e-12:
                    normal/=length
                    for i in ids: normals[i-1]+=normal
            lengths=np.linalg.norm(normals,axis=1)
            normals[lengths>0]/=lengths[lengths>0,None]
            for normal in normals: f.write('vn '+' '.join(f'{v:.7f}' for v in normal)+'\n')
            for mat in MATERIALS:
                faces=[ids for m,ids in self.faces if m==mat]
                if not faces: continue
                f.write('usemtl '+mat+'\n')
                for ids in faces: f.write('f '+' '.join(f'{i}/{i}/{i}' for i in ids)+'\n')
        return len(self.faces)


def textures():
    rng=np.random.default_rng(42)
    noise=rng.normal(0,3,(512,512))
    cloud=np.array(Image.fromarray(rng.integers(0,255,(16,16),dtype=np.uint8)).resize((512,512)).filter(ImageFilter.GaussianBlur(12)))
    rgb=np.stack([np.clip(base+noise+cloud*.045,0,255) for base in [112,116,111]],axis=-1).astype(np.uint8)
    image=Image.fromarray(rgb)
    draw=ImageDraw.Draw(image)
    draw.line((0,0,511,0),fill=(80,84,81),width=2)
    draw.line((0,0,0,511),fill=(80,84,81),width=2)
    for _ in range(70):
        x,y=rng.integers(0,512,2); r=int(rng.integers(1,3))
        draw.ellipse((x,y,x+r,y+r),fill=(99,103,99))
    image.save(ASSET/'textures/concrete.png')
    atlas=Image.new('RGB',(1024,1024),(33,40,41)); d=ImageDraw.Draw(atlas)
    fontpath='/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
    labels=['01  STORES','02  TOOL CRIB','03  QC INSPECTION','04  ASSEMBLY A',
            '05  ASSEMBLY B','ROBOT CHARGING','DANGER  /  PRESS CELL','JOB ACCESS ONLY',
            'SLOW  /  SHARED AISLE','DISPATCH  /  RECEIVING','FORKLIFT CROSSING','KEEP AISLE CLEAR',
            'NORTHLINE  /  WORKS','PPE REQUIRED','QUALITY CONTROL','MACHINING']
    for i,t in enumerate(labels):
        x,y=(i%2)*512,(i//2)*128
        d.rectangle((x+2,y+2,x+510,y+126),fill=(37,46,47),outline=(196,149,44),width=5)
        font=ImageFont.truetype(fontpath,30 if len(t)<22 else 25)
        bb=d.textbbox((0,0),t,font=font)
        d.text((x+(512-bb[2])/2,y+(128-bb[3])/2-3),t,fill=(231,220,185),font=font)
    atlas.save(ASSET/'textures/signs.png')
    write_materials()


def write_materials():
    """Lit, moderately reflective materials; emissive color is for lamp faces only."""
    lines=[]
    metal={'steel','edge','teal','blue','white','orange','yellow','red'}
    for name,c in MATERIALS.items():
        colour=(1.,1.,1.) if name in ('concrete','signs') else tuple(v**.60 for v in c)
        spec=.30 if name in metal else (.40 if name=='glass' else .035)
        shine=38 if name in metal else (70 if name=='glass' else 9)
        lines.extend([f'newmtl {name}',
                      'Ka '+' '.join(str(v*.45) for v in colour),
                      'Kd '+' '.join(str(v) for v in colour),
                      f'Ks {spec} {spec} {spec}',f'Ns {shine}','d 1','illum 2'])
        if name in ('concrete','signs'):lines.append(f'map_Kd ../textures/{name}.png')
        if name=='lamp':lines.append('Ke 0.9 0.86 0.70')
        lines.append('')
    (ASSET/'meshes/factory.mtl').write_text('\n'.join(lines))


COLLISIONS=[]


def solid(mesh,name,xyz,size,mat='steel'):
    mesh.box(xyz,size,mat)
    COLLISIONS.append(dict(name=name,shape='box',xyz=list(xyz),size=list(size)))


def rack(mesh,x,y,n):
    # Solid kick plinth matches the box footprint at the 0.23 m laser plane.
    solid(mesh,f'rack_{n}',(x,y,.18),(.8,3.8,.36),'steel')
    for dx in [-.35,.35]:
        for dy in [-1.84,0,1.84]:
            mesh.box((x+dx,y+dy,1.4),(.075,.075,2.8),'blue')
    for z in [.4,1.2,2.0,2.75]:
        mesh.box((x,y,z),(.8,3.8,.09),'orange')
        for j in range(5):
            cy=y-1.45+j*.72
            if z<2.5:
                mesh.box((x,cy,z+.29),(.68,.60,.50),'cardboard' if j%2 else 'wood')
                mesh.box((x-.344,cy,z+.29),(.008,.07,.50),'edge')
                mesh.box((x+.344,cy,z+.29),(.008,.07,.50),'edge')
    mesh.sign((x,y-1.91,2.5),(.62,.16),0,vertical=True)


def machine(mesh,x,y,n,sx=1.8,sy=1.5):
    solid(mesh,f'cnc_{n}',(x,y,.29),(sx,sy,.58),'steel')
    mesh.box((x,y,1.05),(sx,sy,1.22),'teal')
    mesh.box((x,y,1.70),(sx+.03,sy+.03,.12),'edge')
    mesh.box((x,y-sy/2-.012,1.10),(sx*.65,.025,.75),'black')
    mesh.box((x,y-sy/2-.029,1.13),(sx*.54,.014,.59),'glass')
    mesh.box((x+.12,y-sy/2-.04,1.1),(.025,.04,.65),'edge')
    mesh.box((x+sx*.40,y-sy/2-.06,1.25),(.25,.09,.48),'white')
    mesh.box((x+sx*.40,y-sy/2-.111,1.32),(.18,.014,.19),'black')
    for dz,mat in [(0,'red'),(.08,'yellow'),(.16,'white')]:
        mesh.cylinder((x+sx*.38,y,1.90+dz),.055,.065,mat,8)
    for side in [-1,1]:
        mesh.box((x+side*sx*.46,y,1.04),(.04,sy,.95),'edge')
    mesh.sign((x,y-sy/2-.035,1.66),(sx*.6,.15),15,True)


def pallet(mesh,x,y,z=0,loaded=True):
    for dy in [-.38,0,.38]: mesh.box((x,y+dy,z+.08),(1.0,.12,.16),'wood')
    for dx in [-.42,-.21,0,.21,.42]: mesh.box((x+dx,y,z+.18),(.17,.95,.05),'cardboard')
    if loaded:
        for dx in [-.25,.25]:
            mesh.box((x+dx,y,z+.55),(.46,.85,.65),'cardboard')
            mesh.box((x+dx,y,z+.89),(.07,.87,.015),'white')


def border(mesh,bounds,material='yellow',width=.06,z=.014):
    x0,x1,y0,y1=bounds
    mesh.box(((x0+x1)/2,y0,z),(x1-x0,width,.008),material)
    mesh.box(((x0+x1)/2,y1,z),(x1-x0,width,.008),material)
    mesh.box((x0,(y0+y1)/2,z),(width,y1-y0,.008),material)
    mesh.box((x1,(y0+y1)/2,z),(width,y1-y0,.008),material)


def factory_mesh():
    m=Mesh()
    m.box((8,7,-.05),(16,14,.1),'concrete')
    # Walls with a steel kick plate, pilasters and inset industrial doors.
    for name,xyz,size in [('north',(8,14,1.55),(16,.16,3.1)),('south',(8,0,.5),(16,.16,1.0)),
                          ('west',(0,7,.5),(.16,14,1.0)),('east',(16,7,1.55),(.16,14,3.1))]:
        solid(m,name,xyz,size,'wall')
    for y in [0.09,13.91]:
        m.box((8,y,.35),(16,.025,.7),'steel')
        for x in [0.15,4,8,12,15.85]: m.box((x,y,1.75),(.16,.12,3.5),'edge')
    for x in [.09,15.91]:
        m.box((x,7,.35),(.025,14,.7),'steel')
        for y in [3,7,11]: m.box((x,y,1.75),(.12,.16,3.5),'edge')
    # Closed loading door: visual inset, uses existing outer wall collision.
    m.box((3.5,.102,1.35),(3.2,.025,2.7),'edge')
    for z in np.arange(.2,2.7,.18): m.box((3.5,.12,float(z)),(3.15,.018,.018),'steel')
    m.sign((3.5,.145,2.9),(3.2,.35),9,True)
    m.sign((8,13.89,2.6),(4.5,.55),12,True)
    # Open ceiling frame gives a factory silhouette without obscuring the robots.
    for y in [1.4,5.1,9.1,12.9]:
        m.box((8,y,3.8),(16,.12,.16),'steel')
        for x in [2.0,7.8,13.8]:
            m.box((x,y,3.62),(2.2,.25,.08),'steel')
            m.box((x,y,3.57),(2.0,.18,.025),'lamp')
    for x in [.2,15.8]: m.box((x,7,3.80),(.12,14,.18),'steel')
    for n,(x,y) in enumerate([(5.8,8.8),(10.1,5.8)]):
        solid(m,f'column_{n}',(x,y,1.85),(.28,.28,3.7),'steel')
        m.box((x,y,.4),(.31,.31,.8),'yellow')
        for z in [.15,.40,.65]: m.box((x,y,z),(.315,.315,.08),'black')
    rack(m,2.7,9.7,1); rack(m,4.5,9.7,2)
    # Shipping pallets within one conservative collision box per stack.
    for n,(x,y) in enumerate([(2.5,3.8),(4.0,3.8)]):
        pallet(m,x,y)
        COLLISIONS.append(dict(name=f'pallet_stack_{n}',shape='box',xyz=[x,y,.45],size=[1.05,1.0,.9]))
    # Tool storage cabinet along the west wall.
    solid(m,'tool_cabinet',(.46,5.7,.75),(.7,1.8,1.5),'blue')
    for y in np.arange(5,6.6,.25):
        m.box((.82,float(y),.8),(.03,.025,1.2),'steel')
    machine(m,12.5,11.8,1,2.0,1.6)
    machine(m,13.65,3.5,2,1.7,1.7)
    # Conveyor and packing line. Its skirt is a solid sensor-compatible base.
    solid(m,'conveyor',(12.5,8.7,.30),(4.4,.85,.60),'steel')
    m.box((12.5,8.7,.78),(4.4,.85,.18),'edge')
    m.box((12.5,8.7,.885),(4.25,.69,.035),'black')
    for x in np.arange(10.5,14.6,.25): m.box((float(x),8.7,.907),(.012,.67,.008),'edge')
    for x in [11,12.6,14]: m.box((x,8.7,1.10),(.45,.45,.38),'cardboard')
    for x in [10.5,14.5]:
        m.box((x,8.7,.40),(.15,.80,.8),'orange')
    # Press cell: machine inside a painted semantic keep-out zone.
    solid(m,'press_base',(7.5,3.5,.40),(1.8,1.5,.8),'steel')
    for x in [6.8,8.2]: m.box((x,3.5,1.3),(.26,1.2,1.8),'red')
    m.box((7.5,3.5,2.1),(1.8,1.5,.3),'red')
    m.cylinder((7.5,3.5,1.45),.14,.8,'edge')
    # Hazard is enforced by the semantic zone; no perimeter fence or collision.
    # Inspection desk and testing cabinet at the back of the restricted QC bay.
    solid(m,'qc_bench',(7.1,13.0,.43),(1.8,.7,.86),'steel')
    m.box((7.1,13.0,.91),(1.9,.8,.10),'white')
    m.box((7.4,13,1.20),(.55,.3,.5),'blue')
    solid(m,'qc_cabinet',(9.3,13,.80),(.8,.7,1.6),'white')
    m.sign((8,13.88,2.0),(3.3,.4),14,True)
    # Paint is visual only and uses opaque thin surfaces to avoid transparency cost.
    for bounds,mat in [((6,9,2,5),'hazard'),((6,10,10.3,13.65),'qc'),((11.2,15.6,1.8,6.3),'slow')]:
        x0,x1,y0,y1=bounds
        m.box(((x0+x1)/2,(y0+y1)/2,.006),(x1-x0,y1-y0,.006),mat)
        border(m,bounds)
    # Hatching at the press and slow-zone entrances, kept flat to the floor.
    for x in np.arange(6.1,9,.23): m.box((float(x),5,.019),(.10,.26,.008),'black',-.5)
    for x in np.arange(11.35,15.5,.3): m.box((float(x),6.3,.019),(.12,.30,.008),'white',-.5)
    # Two parallel lane lines leave a wide, unpainted central crossing aisle.
    for y in [6.4,7.7]: m.box((8,y,.013),(14.2,.055,.006),'yellow')
    for x in [1.1,5.4]: m.box((x,9.6,.013),(.055,4.6,.006),'yellow')
    for x in np.arange(10.3,11.4,.25): m.box((float(x),7,.019),(.13,1.1,.006),'white')
    m.sign((7.5,1.5,.022),(2.6,.45),6)
    m.sign((8,10.0,.022),(2.5,.42),7)
    m.sign((13.4,6.65,.022),(2.5,.38),8)
    m.sign((7.5,7.9,.022),(2.7,.4),10)
    stations={'HOME_1':(1.,1.,0.),'HOME_2':(2.6,1.,0.),'STORES':(1.3,10.8,1.57),
              'TOOL_CRIB':(1.6,5.5,3.14),'QC_LAB':(8.,11.7,1.57),
              'ASSEMBLY_A':(14.5,11.5,3.14),'ASSEMBLY_B':(14.6,5.2,-1.57)}
    ids={'HOME_1':5,'HOME_2':5,'STORES':0,'TOOL_CRIB':1,'QC_LAB':2,'ASSEMBLY_A':3,'ASSEMBLY_B':4}
    for name,(x,y,yaw) in stations.items():
        border(m,(x-.42,x+.42,y-.40,y+.40),'white',.035,.021)
        m.sign((x,y-.63,.024),(1.45,.24),ids[name])
        if name.startswith('HOME'):
            m.box((x,.25,.22),(.65,.28,.44),'steel')
            COLLISIONS.append(dict(name=name+'_charger',shape='box',xyz=[x,.25,.22],size=[.65,.28,.44]))
    return m,stations


def forklift_mesh():
    m=Mesh()
    m.box((-.12,0,.42),(1.12,.79,.50),'yellow')
    m.box((-.52,0,.55),(.28,.80,.45),'yellow')
    m.box((.15,0,.74),(.40,.72,.23),'steel')
    m.box((-.13,0,.86),(.38,.46,.17),'black')
    m.box((-.30,0,1.07),(.13,.48,.39),'black')
    for x in [-.52,.35]:
        for y in [-.38,.38]:
            m.cylinder((x,y,.24),.235,.14,'black',12,'y')
            m.cylinder((x,y*1.17,.24),.10,.025,'edge',12,'y')
    for x in [-.52,.28]:
        for y in [-.34,.34]: m.box((x,y,1.18),(.06,.06,1.32),'steel')
    m.box((-.12,0,1.86),(1.02,.87,.10),'yellow')
    for y in [-.27,0,.27]: m.box((-.12,y,1.93),(.92,.04,.035),'steel')
    for y in [-.29,.29]:
        m.box((.50,y,1.0),(.09,.09,1.85),'steel')
        m.box((.90,y,.13),(.84,.10,.075),'edge')
    m.box((.51,0,.50),(.10,.68,.19),'black')
    m.cylinder((-.12,0,2.03),.065,.10,'orange',8)
    return m


def worker_mesh():
    m=Mesh()
    for y in [-.11,.11]:
        m.box((.02,y,.07),(.28,.16,.14),'black')
        m.box((0,y,.44),(.17,.16,.63),'blue')
    m.box((0,0,1.08),(.30,.43,.63),'orange')
    for z in [.92,1.18]: m.box((0,0,z),(.31,.44,.055),'yellow')
    for y in [-.28,.28]:
        m.box((0,y,1.03),(.16,.13,.52),'blue')
        m.box((0,y,.73),(.13,.12,.13),'wood')
    m.cylinder((0,0,1.53),.14,.26,'cardboard',10)
    m.cylinder((0,0,1.68),.19,.10,'yellow',12)
    m.cylinder((0,0,1.73),.15,.08,'yellow',10)
    return m


def tag(parent,tagname,text=None,**attrs):
    e=ET.SubElement(parent,tagname,attrs)
    if text is not None: e.text=str(text)
    return e


def collision(parent,c):
    e=tag(parent,'collision',name=c['name'])
    tag(e,'pose',' '.join(map(str,c['xyz']))+' 0 0 0')
    g=tag(e,'geometry')
    if c['shape']=='box': tag(tag(g,'box'),'size',' '.join(map(str,c['size'])))
    else:
        cy=tag(g,'cylinder'); tag(cy,'radius',c['radius']); tag(cy,'length',c['height'])


def visual(parent,name):
    v=tag(parent,'visual',name=name)
    tag(v,'cast_shadows','false' if name=='static_ground' else 'true')
    tag(tag(tag(v,'geometry'),'mesh'),'uri',f'model://industrial_factory/meshes/{name}.obj')


def configure_lighting(w):
    """Low ambient, four real ceiling fixtures and one shadow-casting key light."""
    for child in list(w):
        if child.tag in ('scene','light'):w.remove(child)
    scene=tag(w,'scene')
    tag(scene,'ambient','.21 .235 .26 1')
    tag(scene,'background','.085 .11 .13 1')
    tag(scene,'shadows','true')
    key=tag(w,'light',type='directional',name='overhead_key')
    tag(key,'pose','8 7 8 0 0 0')
    tag(key,'direction','.35 -.55 -1')
    tag(key,'diffuse','.78 .83 .90 1')
    tag(key,'specular','.42 .44 .48 1')
    tag(key,'cast_shadows','true')
    tag(key,'visualize','false')
    fill=tag(w,'light',type='directional',name='wall_fill')
    tag(fill,'pose','8 7 7 0 0 0')
    tag(fill,'direction','-.5 .5 -.8')
    tag(fill,'diffuse','.24 .26 .29 1')
    tag(fill,'specular','.06 .07 .08 1')
    tag(fill,'cast_shadows','false')
    tag(fill,'visualize','false')
    # Match four of the existing light-strip positions; geometry is unchanged.
    for i,(x,y) in enumerate([(2.,5.1),(13.8,5.1),(2.,12.9),(13.8,12.9)]):
        light=tag(w,'light',type='spot',name=f'ceiling_light_{i+1}')
        tag(light,'pose',f'{x} {y} 3.48 0 0 0')
        tag(light,'direction','0 0 -1')
        tag(light,'diffuse','1.0 .95 .86 1')
        tag(light,'specular','.38 .35 .30 1')
        tag(light,'cast_shadows','false')
        tag(light,'visualize','false')
        atten=tag(light,'attenuation')
        tag(atten,'range','13')
        tag(atten,'constant','.65')
        tag(atten,'linear','.06')
        tag(atten,'quadratic','.045')
        spot=tag(light,'spot')
        tag(spot,'inner_angle','.65')
        tag(spot,'outer_angle','1.15')
        tag(spot,'falloff','1.0')



def split_shadow_geometry(mesh):
    """Floor paint receives shadows but must not cast nearly-coplanar shadows."""
    parts=[]
    for ground in (False, True):
        part=Mesh();lookup={}
        for material,ids in mesh.faces:
            on_ground=all(mesh.verts[i-1][2]<=.03 for i in ids)
            if on_ground!=ground:continue
            new=[]
            for i in ids:
                if i not in lookup:
                    part.verts.append(mesh.verts[i-1]);part.uvs.append(mesh.uvs[i-1])
                    lookup[i]=len(part.verts)
                new.append(lookup[i])
            part.faces.append((material,tuple(new)))
        parts.append(part)
    return parts


def write_sdf():
    root=ET.Element('sdf',version='1.9'); w=tag(root,'world',name='factory')
    ph=tag(w,'physics',name='factory_100hz',type='ignored')
    tag(ph,'max_step_size',.01); tag(ph,'real_time_factor',1.)
    for f,n in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster'),('sensors','Sensors'),('imu','Imu')]:
        p=tag(w,'plugin',filename=f'gz-sim-{f}-system',name='gz::sim::systems::'+n)
        if f=='sensors': tag(p,'render_engine','ogre')
    configure_lighting(w)
    model=tag(w,'model',name='factory_structure');tag(model,'static','true');link=tag(model,'link',name='structure')
    collision(link,dict(name='floor',shape='box',xyz=[8,7,-.06],size=[20,18,.12]))
    for c in COLLISIONS: collision(link,c)
    visual(link,'static_factory')
    visual(link,'static_ground')
    movers=[('forklift_1','forklift',19,2,dict(name='body',shape='box',xyz=[.28,0,1.05],size=[2.10,.94,2.10])),
            ('worker_1','worker',19,4,dict(name='body',shape='cylinder',xyz=[0,0,.87],radius=.36,height=1.74)),
            ('worker_2','worker',19,6,dict(name='body',shape='cylinder',xyz=[0,0,.87],radius=.36,height=1.74)),
            ('pallet_1','pallet',19,8,dict(name='body',shape='box',xyz=[0,0,.45],size=[1.05,1.,.9]))]
    for name,mesh,x,y,c in movers:
        model=tag(w,'model',name=name);tag(model,'static','true');tag(model,'pose',f'{x} {y} 0 0 0 0')
        link=tag(model,'link',name='body');collision(link,c);visual(link,mesh)
        if name=='forklift_1':
            proxy=tag(link,'visual',name='lidar_outer_envelope')
            tag(proxy,'pose','0.28 0 1.05 0 0 0');tag(proxy,'cast_shadows','false')
            tag(proxy,'visibility_flags','2147483648')
            tag(tag(tag(proxy,'geometry'),'box'),'size','2.10 0.94 2.10')

    # A useful oblique view into the open-roof workshop.
    gui=tag(w,'gui',fullscreen='false');cam=tag(gui,'camera',name='user_camera')
    tag(cam,'pose','21 -9 18 0 .66 2.28');tag(cam,'view_controller','orbit')
    ET.indent(root,space='  ')
    ET.ElementTree(root).write(PKG/'worlds/factory.sdf',encoding='utf-8',xml_declaration=True)


def write_map():
    res=.05
    # World floor [0,16] x [0,14]; map = world - (1,1).
    origin=(-1.3,-1.3); width,height=332,292
    ys=origin[1]+1+(np.arange(height)+.5)*res
    xs=origin[0]+1+(np.arange(width)+.5)*res
    xx,yy=np.meshgrid(xs,ys)
    occ=(xx<0)|(xx>16)|(yy<0)|(yy>14)
    for c in COLLISIONS:
        x,y,z=c['xyz']
        if c['shape']=='box':
            a,b,h=c['size']
            if z-h/2<=.23<=z+h/2: occ |= (abs(xx-x)<=a/2)&(abs(yy-y)<=b/2)
    pix=np.where(occ,0,254).astype(np.uint8)[::-1]
    image=PKG/'maps/factory_map.pgm'
    image.write_bytes(f'P5\n{width} {height}\n255\n'.encode()+pix.tobytes())
    (PKG/'maps/factory_map.yaml').write_text(yaml.safe_dump(dict(image='factory_map.pgm',mode='trinary',resolution=res,origin=[*origin,0.],negate=0,occupied_thresh=.65,free_thresh=.196),sort_keys=False))


def write_config(stations):
    (PKG/'config/stations.yaml').write_text('# Map frame = Gazebo world minus (1, 1).\n'+yaml.safe_dump({'stations':{k:dict(x=round(v[0]-1,3),y=round(v[1]-1,3),yaw=v[2]) for k,v in stations.items()}},sort_keys=False))
    zones={'HAZARD_PRESS':dict(x_min=5.,x_max=8.,y_min=1.,y_max=4.,rule='forbidden'),
           'QC_LAB':dict(x_min=5.,x_max=9.,y_min=9.3,y_max=12.65,rule='authorized_only'),
           'ASSEMBLY_SLOW':dict(x_min=10.2,x_max=14.6,y_min=.8,y_max=5.3,rule='slow',max_speed=.15)}
    (PKG/'config/zones.yaml').write_text('# Map frame = Gazebo world minus (1, 1).\n'+yaml.safe_dump({'zones':zones},sort_keys=False))
    movers={'forklift_1':dict(route=[[2.4,7.],[14.,7.]],speed=.35,z=0.,parked=[19.,2.],give_way=1.7),
            'worker_1':dict(route=[[9.5,1.5],[9.5,9.8]],speed=.28,z=0.,parked=[19.,4.],give_way=1.),
            'worker_2':dict(route=[[1.2,12.5],[5.2,12.5]],speed=.23,z=0.,parked=[19.,6.],give_way=1.)}
    obstacles=dict(world='factory',rate_hz=5.,movers=movers,placeable={'pallet_1':dict(z=0.,parked=[19.,8.],drop_points={'main_aisle':[8.,7.],'left_aisle':[1.3,6.5]})})
    (PKG/'config/obstacles.yaml').write_text('# Gazebo world coordinates. Movers are excluded from the reference map.\n'+yaml.safe_dump(obstacles,sort_keys=False))
    route=dict(robot='robot1',speed=.20,turn_speed=.30,steer_limit=.30,pause_after_turn=1.,tolerance=.10,stop_distance=.35,
               waypoints_gazebo=[[5.1,1.3],[5.1,6.8],[15.3,6.8],[15.3,13.2],[11.1,13.2],[11.1,10.1],[5.7,10.1],[5.7,12.7],[1.2,12.7],[1.2,7.],[3.6,7.],[3.6,12.4],[1.2,12.4],[1.2,1.]])
    (PKG/'config/mapping_route.yaml').write_text('# Scripted mapping sweep, in Gazebo world coordinates.\n'+yaml.safe_dump(route,sort_keys=False))


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--visual-source', type=Path,
                        help='Export only the original visual meshes/textures to this directory; leave world, map and policies untouched.')
    args = parser.parse_args()
    global ASSET
    if args.visual_source:
        ASSET = args.visual_source.resolve()
    for p in ['meshes','textures']: (ASSET/p).mkdir(parents=True,exist_ok=True)
    textures()
    static,stations=factory_mesh()
    static,ground=split_shadow_geometry(static)
    p=Mesh();pallet(p,0,0)
    counts={name:mesh.write(name) for name,mesh in [('static_factory',static),('static_ground',ground),('forklift',forklift_mesh()),('worker',worker_mesh()),('pallet',p)]}
    if args.visual_source:
        print(json.dumps({'visual_source': str(ASSET), 'triangles': counts}, indent=2))
        return
    write_sdf();write_map();write_config(stations)
    report=dict(triangles=counts,static_collision_shapes=len(COLLISIONS)+1,dimensions=[WIDTH,HEIGHT],
                texture_sizes={'concrete':[512,512],'signs':[1024,1024]},map_source='static collision footprints at 0.23 m lidar height',map_world_offset=[1.,1.],collisions=COLLISIONS)
    (ASSET/'manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    (ASSET/'model.config').write_text('<?xml version="1.0"?><model><name>Industrial factory assets</name><version>1.0</version><sdf version="1.9">model.sdf</sdf><description>Shared low-poly factory meshes. World is in factory_bot/worlds/factory.sdf.</description></model>\n')
    (ASSET/'model.sdf').write_text('<?xml version="1.0"?><sdf version="1.9"><model name="industrial_factory"><static>true</static><link name="visual"><visual name="factory"><cast_shadows>true</cast_shadows><geometry><mesh><uri>meshes/static_factory.obj</uri></mesh></geometry></visual><visual name="ground"><cast_shadows>false</cast_shadows><geometry><mesh><uri>meshes/static_ground.obj</uri></mesh></geometry></visual></link></model></sdf>\n')
    print(json.dumps({k:v for k,v in report.items() if k!='collisions'},indent=2))


if __name__=='__main__': main()
