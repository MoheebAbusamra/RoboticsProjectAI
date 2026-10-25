"""World-aligned concrete with subtle static contact shading; no physics changes."""
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

ROOT=Path(__file__).resolve().parents[1]
ASSET=ROOT/'src/factory_bot/models/industrial_factory'
size=2048
rng=np.random.default_rng(17)
grain=rng.normal(0,1.7,(size,size))
cloud=np.array(Image.fromarray(rng.integers(0,255,(40,40),dtype=np.uint8)).resize((size,size)).filter(ImageFilter.GaussianBlur(20)),dtype=float)
base=np.clip(116+grain+(cloud-128)*.035,0,255)
rgb=np.stack([base,base+3,base],axis=-1).astype(np.uint8)
floor=Image.fromarray(rgb);draw=ImageDraw.Draw(floor)
for x in range(0,size,256):draw.line((x,0,x,size),fill=(97,101,98),width=2)
for y in range(0,size,293):draw.line((0,y,size,y),fill=(97,101,98),width=2)
mask=Image.new('L',(size,size),0);d=ImageDraw.Draw(mask)
manifest=json.loads((ASSET/'manifest.json').read_text())
for c in manifest['collisions']:
    x,y,z=c['xyz'];sx,sy,h=c.get('size',(0,0,0))
    if z-h/2>.35:continue
    d.rectangle(((x-sx/2)/16*size,(14-y-sy/2)/14*size,
                 (x+sx/2)/16*size,(14-y+sy/2)/14*size),fill=145)
soft=np.asarray(mask.filter(ImageFilter.GaussianBlur(12)),dtype=float)/255
pixels=np.asarray(floor,dtype=float)*(1-soft[:,:,None]*.25)
Image.fromarray(np.clip(pixels,0,255).astype(np.uint8)).save(ASSET/'textures/concrete.png')
print('2048 px world-aligned concrete / static contact shading saved')
