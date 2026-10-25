"""Fetch CC0 authoring sources and bake the world-aligned dark concrete surface.

Run before build_premium_assets.py. High-resolution sources stay in work/cc0;
Gazebo loads only the compact runtime textures in the package models folder.
"""
import json
from pathlib import Path
import urllib.request
import zipfile
import numpy as np
from PIL import Image, ImageDraw, ImageFilter

ROOT = Path(__file__).resolve().parents[1]
CC0 = ROOT / 'work/cc0'
ASSET = ROOT / 'src/factory_bot/models/industrial_factory'
CC0.mkdir(parents=True, exist_ok=True)
for archive in ['Metal032_1K-JPG.zip', 'Concrete034_2K-JPG.zip']:
    path = CC0 / archive
    if not path.exists():
        request = urllib.request.Request('https://ambientcg.com/get?file=' + archive,
                                         headers={'User-Agent': 'FactoryAssetAuthoring/1.0'})
        with urllib.request.urlopen(request, timeout=60) as response:
            path.write_bytes(response.read())
    with zipfile.ZipFile(path) as source:
        for name in source.namelist():
            if name.endswith('.jpg') and any(k in name for k in ['Color', 'NormalGL', 'Roughness']):
                (CC0 / Path(name).name).write_bytes(source.read(name))

size = 2048
source = Image.open(CC0 / 'Concrete034_2K-JPG_Color.jpg').convert('RGB')
source = source.resize((512, 512), Image.Resampling.LANCZOS)
luminance = np.asarray(source, dtype=float).mean(axis=2)
luminance = 77 + (luminance - luminance.mean()) * .32
rgb = np.stack([luminance*.96, luminance, luminance*1.035], axis=-1).clip(0,255).astype('uint8')
floor = Image.fromarray(np.tile(rgb, (4,4,1)))
draw = ImageDraw.Draw(floor)
for x in range(0,size,256): draw.line((x,0,x,size),fill=(53,57,60),width=1)
for y in range(0,size,293): draw.line((0,y,size,y),fill=(53,57,60),width=1)
mask = Image.new('L',(size,size)); draw = ImageDraw.Draw(mask)
for contact in json.loads((ASSET/'manifest.json').read_text())['collisions']:
    x,y,z = contact['xyz']; sx,sy,h = contact['size']
    if z-h/2 > .35: continue
    draw.rectangle(((x-sx/2)/16*size,(14-y-sy/2)/14*size,
                    (x+sx/2)/16*size,(14-y+sy/2)/14*size),fill=150)
ao = np.array(mask.filter(ImageFilter.GaussianBlur(10))) / 255
pixels = np.asarray(floor) * (1-ao[:,:,None]*.22)
Image.fromarray(pixels.astype('uint8')).save(ASSET/'textures/concrete.png',optimize=True)
print('CC0 surface sources cached; dark concrete atlas ready')
