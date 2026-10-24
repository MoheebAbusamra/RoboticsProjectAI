"""compare_maps: scores a saved map against a reference map (both saved by map_saver_cli).

  ros2 run factory_bot compare_maps <reference.yaml> <test.yaml>

Both maps must use the same frame (ours do: map (0,0) = robot1's start).
Prints two numbers and writes a picture next to the test map:
  real walls found    = % of reference wall cells that have a test wall within --tolerance
  test walls that fit = % of test wall cells that have a reference wall within --tolerance
Picture (<test>_vs_reference.ppm, open with eog): grey = reference walls,
green = test walls that fit, red = test walls in the wrong place.
AI-assisted (Claude): cite in report."""
import argparse
import os

import numpy as np
import yaml


def read_pgm(path):
    """Read a binary (P5) or text (P2) PGM image into a 2-D uint8 array (row 0 = top)."""
    with open(path, "rb") as f:
        data = f.read()
    tokens, pos = [], 0
    while len(tokens) < 4:                        # magic, width, height, maxval
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b"#":             # comment line
            pos = data.index(b"\n", pos) + 1
            continue
        start = pos
        while not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    magic, w, h = tokens[0], int(tokens[1]), int(tokens[2])
    if magic == b"P5":
        return np.frombuffer(data, np.uint8, w * h, pos + 1).reshape(h, w)
    return np.array(data[pos:].split(), dtype=np.uint8)[:w * h].reshape(h, w)


def load_map(yaml_path):
    """Return (occupied grid with row 0 = BOTTOM, resolution, (origin_x, origin_y))."""
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    image = meta["image"]
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image)
    pix = read_pgm(image).astype(float)
    p = pix / 255.0 if meta.get("negate", 0) else (255.0 - pix) / 255.0
    occupied = p > float(meta.get("occupied_thresh", 0.65))
    return occupied[::-1, :], float(meta["resolution"]), meta["origin"][:2]


def grow(grid, cells):
    """Make every True cell 'cells' cells fatter (square neighbourhood)."""
    out = grid.copy()
    for dr in range(-cells, cells + 1):
        for dc in range(-cells, cells + 1):
            out |= np.roll(np.roll(grid, dr, axis=0), dc, axis=1)
    return out


def fraction_near(points_xy, grid, res, origin, tolerance):
    """For world points, which ones have a True cell of 'grid' within tolerance."""
    fat = grow(grid, int(round(tolerance / res)))
    cols = np.floor((points_xy[:, 0] - origin[0]) / res).astype(int)
    rows = np.floor((points_xy[:, 1] - origin[1]) / res).astype(int)
    inside = (cols >= 0) & (cols < grid.shape[1]) & (rows >= 0) & (rows < grid.shape[0])
    near = np.zeros(len(points_xy), dtype=bool)
    near[inside] = fat[rows[inside], cols[inside]]
    return near


def cell_centres(grid, res, origin):
    rows, cols = np.nonzero(grid)
    return np.column_stack([origin[0] + (cols + 0.5) * res, origin[1] + (rows + 0.5) * res])


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("reference")
    ap.add_argument("test")
    ap.add_argument("--tolerance", type=float, default=0.10, help="metres (default 0.10)")
    a = ap.parse_args()

    ref, ref_res, ref_org = load_map(a.reference)
    test, test_res, test_org = load_map(a.test)
    ref_pts = cell_centres(ref, ref_res, ref_org)
    test_pts = cell_centres(test, test_res, test_org)
    found = fraction_near(ref_pts, test, test_res, test_org, a.tolerance)
    fits = fraction_near(test_pts, ref, ref_res, ref_org, a.tolerance)
    print(f"reference: {a.reference}  ({len(ref_pts)} wall cells)")
    print(f"test:      {a.test}  ({len(test_pts)} wall cells)")
    print(f"real walls found    : {100 * found.mean():5.1f} %")
    print(f"test walls that fit : {100 * fits.mean():5.1f} %   (tolerance {a.tolerance:.2f} m)")

    # picture in the reference map's frame
    img = np.full(ref.shape + (3,), 255, dtype=np.uint8)
    img[ref] = (170, 170, 170)
    cols = np.floor((test_pts[:, 0] - ref_org[0]) / ref_res).astype(int)
    rows = np.floor((test_pts[:, 1] - ref_org[1]) / ref_res).astype(int)
    inside = (cols >= 0) & (cols < ref.shape[1]) & (rows >= 0) & (rows < ref.shape[0])
    img[rows[inside & fits], cols[inside & fits]] = (0, 170, 0)
    img[rows[inside & ~fits], cols[inside & ~fits]] = (220, 0, 0)
    out = os.path.splitext(a.test)[0] + "_vs_reference.ppm"
    with open(out, "wb") as f:
        f.write(f"P6 {img.shape[1]} {img.shape[0]} 255\n".encode())
        f.write(img[::-1].tobytes())              # flip back: image files store the top row first
    print(f"picture: {out}")


if __name__ == "__main__":
    main()
