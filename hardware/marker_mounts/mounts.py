#!/usr/bin/env python3
"""Printable floor stands for the ArUco markers of the fiducial survey (docs/hardware-survey.md).

Four stands hold a 140 mm marker at a different orientation each, so that a run of the
estimator sees marker rotations that differ in pitch and roll and not only in yaw:

    upright    pitch   0, roll  0    the baseline
    overhang   pitch -30, roll  0    face down toward the car
    diamond    pitch   0, roll 45    corners up and down
    faceup     pitch +20, roll 30    face up and rolled

Yaw comes from how a stand is turned on the floor. Each stand is one print with no
joints: the 200 mm marker plate standing on its own lower edge and an open tray behind
it, a foot on the floor and a side wall at each edge of the foot, from the foot up the
back of the plate, flush with the plane of the face. The tray is open at the top and the
back, so that a weight can rest on the foot. Where a rolled plate stands on a corner, the
plate is extended down to the floor across the tray's width, so that the face is one flat
surface down to the floor and the tray is closed in front. The pitch and the roll are the angles the
tray and the marker are cut at, and the marker sits level with the camera, 0.09 to 0.14 m
above the floor.

The overhang stand leans toward the car, so the plate's weight is in front of its lower
edge, which is the front of the stand's footprint, and the print alone tips forward. Its
tray is the full width of the plate, and the table says how much weight on the foot
holds it; nothing stands on the floor in front of the face.

Every stand prints face down: the marker face on the bed, the tray up. On the bed, the
foot and the side walls are walls on the plate, leaning by the pitch, so no part needs
supports. Face down also gives the marker a flat backing with the bed's finish.

Units are millimetres. The stand frame is the paper's robot frame: x toward the car (the
marker faces +x), y left, z up, floor at z = 0, the face's lower edge on the floor at
x = 0. The plate frame is the stand frame pitched: x along the marker normal, y across
the plate, z up the plate, origin at the marker centre; the marker is rolled in it.

    python3 mounts.py --out .            write the STLs and print the table
    python3 mounts.py --out . --preview  also render preview.png (needs matplotlib)

Needs manifold3d and numpy (pip install manifold3d numpy).
"""
from __future__ import annotations

import argparse
import math
import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from manifold3d import CrossSection, Manifold, set_circular_segments

set_circular_segments(48)

# Paper and plate.
MARKER = 140.0               # printed marker side, survey.yaml marker_size
PLATE = 200.0                # plate side; the 30 mm border is the marker's quiet zone
PLATE_T = 4.0

# The tray behind the plate.
TRAY_W = 80.0                # tray width across the stand (y), outside the side walls
TRAY_L = 90.0                # foot length behind the plate's back, on the floor
TRAY_H = 110.0               # height the side walls reach up the back of the plate
WALL_T = 4.0                 # foot and side wall thickness
WALL_H = 40.0                # side wall height at the back end of the foot
MARGIN = 15.0                # tipping margin the counterweight is sized for, mm

# Print estimates.
PLA_DENSITY = 1.24e-3        # g/mm^3
SHELL = 1.2                  # mm of solid skin on every surface: three 0.4 mm walls, six
                             # 0.2 mm top and bottom layers
INFILL = 0.15                # the infill fraction the README recommends


# --- helpers -----------------------------------------------------------------------------

def box(x0, x1, y0, y1, z0, z1) -> Manifold:
    return Manifold.cube((x1 - x0, y1 - y0, z1 - z0)).translate((x0, y0, z0))


def rot_y(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot2(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s], [s, c]])


def pose(R: np.ndarray, t) -> np.ndarray:
    return np.hstack([R, np.asarray(t, float).reshape(3, 1)])


def behind(m: Manifold, normal, point) -> Manifold:
    """The part of m on the side of the plane through `point` opposite to `normal`."""
    n = np.asarray(normal, float)
    return m.trim_by_plane(tuple(-n), -float(n @ np.asarray(point, float)))


def mesh_arrays(m: Manifold):
    mesh = m.to_mesh()
    verts = np.asarray(mesh.vert_properties, dtype=np.float64)[:, :3]
    tris = np.asarray(mesh.tri_verts, dtype=np.int64)
    return verts, tris


def write_stl(path: Path, m: Manifold) -> None:
    verts, tris = mesh_arrays(m)
    v0, v1, v2 = verts[tris[:, 0]], verts[tris[:, 1]], verts[tris[:, 2]]
    n = np.cross(v1 - v0, v2 - v0)
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    with open(path, "wb") as f:
        f.write(b"camslab marker stand".ljust(80, b"\0"))
        f.write(struct.pack("<I", len(tris)))
        data = np.zeros(len(tris), dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
        data["n"] = n
        data["v"][:, 0], data["v"][:, 1], data["v"][:, 2] = v0, v1, v2
        f.write(data.tobytes())


def overhang_area(m: Manifold, limit_deg: float = 45.0) -> float:
    """Area of downward faces flatter than `limit_deg` from horizontal. The bed face and
    the first millimetre above it (engraved grooves) are excluded."""
    verts, tris = mesh_arrays(m)
    v0, v1, v2 = verts[tris[:, 0]], verts[tris[:, 1]], verts[tris[:, 2]]
    cross = np.cross(v1 - v0, v2 - v0)
    area = 0.5 * np.linalg.norm(cross, axis=1)
    nz = cross[:, 2] / np.maximum(2 * area, 1e-12)
    zmin = verts[:, 2].min()
    near_bed = (v0[:, 2] < zmin + 1) & (v1[:, 2] < zmin + 1) & (v2[:, 2] < zmin + 1)
    bad = (nz < -math.cos(math.radians(limit_deg)) - 1e-3) & ~near_bed & (area > 1e-6)
    return float(area[bad].sum())


def centroid(m: Manifold) -> np.ndarray:
    """Centre of volume of a closed mesh (tetrahedra from the origin)."""
    verts, tris = mesh_arrays(m)
    v0, v1, v2 = verts[tris[:, 0]], verts[tris[:, 1]], verts[tris[:, 2]]
    vol = np.einsum("ij,ij->i", v0, np.cross(v1, v2)) / 6.0
    return (vol[:, None] * (v0 + v1 + v2) / 4.0).sum(0) / vol.sum()


def printed_density(m: Manifold) -> float:
    """Fraction of m's volume that is plastic when printed with SHELL of skin and INFILL
    inside: 1 for a solid print. A thin plate is mostly skin, a thick block mostly infill,
    so the two weigh differently per volume."""
    skin = min(1.0, SHELL * m.surface_area() / m.volume())
    return skin + INFILL * (1 - skin)


def cross2(a, b) -> float:
    return float(a[0] * b[1] - a[1] * b[0])


def convex_hull_2d(points: np.ndarray) -> np.ndarray:
    """Counter-clockwise hull of (n, 2) points, Andrew's monotone chain."""
    pts = np.unique(np.round(points, 6), axis=0)
    pts = pts[np.lexsort((pts[:, 1], pts[:, 0]))]
    if len(pts) < 3:
        return pts

    def half(seq):
        out = []
        for p in seq:
            while len(out) >= 2 and cross2(out[-1] - out[-2], p - out[-2]) <= 0:
                out.pop()
            out.append(p)
        return out

    lower, upper = half(pts), half(pts[::-1])
    return np.array(lower[:-1] + upper[:-1])


def bed_footprint(m: Manifold) -> tuple[float, float, float]:
    """Smallest x by y box the print fits in when turned on the bed, and that turn in
    degrees."""
    xy = mesh_arrays(m)[0][:, :2]
    best = (math.inf, math.inf, 0.0)
    for deg in np.arange(0.0, 180.0, 0.5):
        r = rot2(deg) @ xy.T
        ext = r.max(1) - r.min(1)
        if max(ext) < max(best[:2]):
            best = (float(ext[0]), float(ext[1]), float(deg))
    return best


def footprint(parts: list[tuple[Manifold, float]]):
    """The support polygon (the hull of the points on the floor) and the mass and centre
    of mass of the stand made of `parts`, each a solid and its density (PLA_DENSITY
    times the fraction that is plastic, in g/mm^3)."""
    whole = Manifold()
    for m, _ in parts:
        whole += m
    verts, _ = mesh_arrays(whole)
    hull = convex_hull_2d(verts[verts[:, 2] < 0.3][:, :2])
    mass = np.array([m.volume() * d for m, d in parts])
    com = (mass[:, None] * np.array([centroid(m) for m, _ in parts])).sum(0) / mass.sum()
    return hull, float(mass.sum()), com


def tip_margin(parts: list[tuple[Manifold, float]]) -> tuple[float, float]:
    """How far the centre of mass of the stand made of `parts` is inside its footprint,
    in mm, and the floor slope in degrees at which the stand tips."""
    hull, _, com = footprint(parts)
    margin = math.inf
    for a, b in zip(hull, np.roll(hull, -1, axis=0)):
        edge = b - a
        margin = min(margin, cross2(edge, com[:2] - a) / np.linalg.norm(edge))
    return margin, math.degrees(math.atan2(margin, com[2]))


def counterweight(parts: list[tuple[Manifold, float]], point, margin: float = MARGIN) -> float:
    """The least mass at `point`, in g, that puts the centre of mass of `parts` at least
    `margin` inside the footprint; 0 if it already is."""
    hull, mass, com = footprint(parts)
    p = np.asarray(point, float)[:2]
    need = 0.0
    for a, b in zip(hull, np.roll(hull, -1, axis=0)):
        edge = b - a
        length = np.linalg.norm(edge)
        gain = cross2(edge, p - a) - margin * length        # what each gram at p adds
        short = margin * length - cross2(edge, com[:2] - a)  # what the print lacks
        if short > 0:
            if gain <= 0:
                return math.inf                              # the weight is not inside enough
            need = max(need, mass * short / gain)
    return need


# --- the stand ---------------------------------------------------------------------------------

@dataclass
class Stand:
    name: str
    pitch: float           # face up positive
    roll: float            # about the normal
    tray_w: float = TRAY_W  # tray width; the full plate width on a stand that needs a weight

    def rotation(self) -> np.ndarray:
        return rot_y(-self.pitch)

    def normal(self) -> np.ndarray:
        return self.rotation() @ np.array([1.0, 0, 0])

    def plate(self, floor: float | None = None) -> Manifold:
        """The plate in the plate frame with the marker centre at the origin: the rolled
        square with the engraved marker outline and chevron. Built in the (y, z) plane
        and extruded along -x. With `floor`, the plate z of the face's floor line, the
        outline is extended down to the floor across the tray's width (a rolled plate
        stands on a corner), to be trimmed at the floor."""
        outline = CrossSection.square((PLATE, PLATE), center=True).rotate(self.roll)
        if floor is not None:
            w = self.tray_w
            skirt = CrossSection([[(-w / 2, floor - 10), (w / 2, floor - 10), (w / 2, floor), (-w / 2, floor)]])
            outline = (outline + skirt).hull()
        body = Manifold.extrude(outline, PLATE_T)
        groove_w, groove_d = 0.8, 0.6
        square = (CrossSection.square((MARKER + groove_w,) * 2, center=True)
                  - CrossSection.square((MARKER - groove_w,) * 2, center=True)).rotate(self.roll)
        chevron = CrossSection([[(-5.0, 76.0), (5.0, 76.0), (0.0, 84.0)]]).rotate(self.roll)
        grooves = Manifold.extrude(square + chevron, groove_d + 1).translate((0, 0, PLATE_T - groove_d))
        # (u, v, w) with the face at w = PLATE_T -> plate frame (x, y, z) = (w - PLATE_T, u, v)
        M = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float)
        return (body - grooves).transform(pose(M, (-PLATE_T, 0, 0)))

    def centre(self) -> np.ndarray:
        """Marker centre in the stand frame: the plate's lowest point on the floor, the
        face's floor line at x = 0."""
        R = self.rotation()
        zc = -np.asarray(self.plate().transform(pose(R, (0, 0, 0))).bounding_box())[2]
        return np.array([-zc * math.tan(math.radians(self.pitch)), 0.0, zc])

    def parts(self) -> tuple[Manifold, Manifold]:
        """The plate and the tray behind it in the stand frame. The tray reaches the face
        plane, so it overlaps the plate."""
        R, c, n = self.rotation(), self.centre(), self.normal()
        t = math.tan(math.radians(self.pitch))
        plate = self.plate(floor=-c[2] / math.cos(math.radians(self.pitch))).transform(pose(R, c))
        plate = plate.trim_by_plane((0, 0, 1), 0.0)
        x0 = -PLATE_T / math.cos(math.radians(self.pitch)) - TRAY_L   # the foot's back end
        w = self.tray_w
        # The foot, from its back end to the face plane along the floor.
        tray = box(x0, max(0.0, -WALL_T * t) + 1, -w / 2, w / 2, 0, WALL_T)
        # A side wall at each edge: the hull of the foot's back end up to WALL_H, the
        # face's floor line and a point on the face plane at TRAY_H.
        for y in (-(w - WALL_T) / 2, (w - WALL_T) / 2):
            a = box(x0, x0 + 1, y - WALL_T / 2, y + WALL_T / 2, 0, WALL_H)
            b = box(-0.5, 0.5, y - WALL_T / 2, y + WALL_T / 2, 0, 0.5)
            top = box(-TRAY_H * t - 0.5, -TRAY_H * t + 0.5, y - WALL_T / 2, y + WALL_T / 2,
                      TRAY_H - 0.5, TRAY_H + 0.5)
            tray += (a + b + top).hull()
        return plate, behind(tray, n, c).trim_by_plane((0, 0, 1), 0.0)

    def weight_point(self) -> np.ndarray:
        """Where a counterweight on the foot is taken to sit: the middle of the foot's
        free floor, 20 mm up."""
        t = math.tan(math.radians(self.pitch))
        x0 = -PLATE_T / math.cos(math.radians(self.pitch)) - TRAY_L
        x1 = -WALL_T * t - PLATE_T / math.cos(math.radians(self.pitch))  # the plate's back at the foot's top
        return np.array([(x0 + x1) / 2, 0.0, WALL_T + 20.0])

    def solid(self) -> Manifold:
        plate, back = self.parts()
        return plate + back

    def printed(self) -> Manifold:
        """The stand in its print orientation: the face on the bed at z = 0, the plate's
        y along the bed's -x and its z along the bed's y (a rotation, not a mirror)."""
        M = np.array([[0, -1, 0], [0, 0, 1], [-1, 0, 0]], float)   # (x, y, z)_plate -> (-y, z, -x)
        R = M @ self.rotation().T
        return self.solid().transform(pose(R, -R @ self.centre()))


STANDS = [
    Stand("upright", 0.0, 0.0),
    Stand("overhang", -30.0, 0.0, tray_w=PLATE),
    Stand("diamond", 0.0, 45.0),
    Stand("faceup", 20.0, 30.0),
]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", type=Path, default=Path(__file__).parent, help="output directory")
    ap.add_argument("--preview", action="store_true", help="render preview.png with matplotlib")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    print(f"{'stand':9s} {'pitch':>5s} {'roll':>4s} {'centre z [m]':>12s} {'top z [m]':>9s} {'normal x z':>12s}"
          f" {'bed x y [mm] turned':>19s} {'height':>6s} {'solid [g]':>9s} {f'{INFILL:.0%} [g]':>7s}"
          f" {'overhang [mm^2]':>15s} {'margin solid [mm]':>17s} {f'margin {INFILL:.0%} [mm]':>15s}"
          f" {'tips at [deg]':>13s} {f'weight for {MARGIN:.0f} mm [g]':>20s}")
    for s in STANDS:
        plate, tray = s.parts()
        tray -= plate                     # the mass behind the plate, for the weights
        m = s.solid()
        p = s.printed()
        if m.is_empty():
            raise SystemExit(f"{s.name} is empty: {m.status()}")
        c, n = s.centre(), s.normal()
        top = np.asarray(plate.bounding_box())[5]
        b = np.asarray(p.bounding_box())
        bx, by, turn = bed_footprint(p)
        bed = f"{bx:.0f} x {by:.0f} at {turn:.0f}"
        solid = [(plate, PLA_DENSITY), (tray, PLA_DENSITY)]
        infill = [(x, printed_density(x) * PLA_DENSITY) for x in (plate, tray)]
        margin, _ = tip_margin(solid)
        margin_i, angle_i = tip_margin(infill)
        weight = counterweight(infill, s.weight_point())
        print(f"{s.name:9s} {s.pitch:5.0f} {s.roll:4.0f} {c[2]/1000:12.3f} {top/1000:9.3f} {n[0]:6.3f} {n[2]:5.3f}"
              f" {bed:>19s} {b[5]-b[2]:6.0f} {footprint(solid)[1]:9.0f} {footprint(infill)[1]:7.0f}"
              f" {overhang_area(p):15.1f} {margin:17.1f} {margin_i:15.1f} {angle_i:13.1f} {weight:20.0f}")
        write_stl(args.out / f"stand_{s.name}.stl", p)

    if args.preview:
        render(args.out / "preview.png")


# --- preview ---------------------------------------------------------------------------------

SCALE = 1.3                  # preview pixels per millimetre


def raster(m: Manifold, view, size, centre, anchor, scale: float = SCALE,
           colour=(0.80, 0.56, 0.36)) -> np.ndarray:
    """An orthographic z-buffered rendering of m (in mm) seen along `view`, as an RGB
    float image of `size` (w, h), with the world point `centre` at the pixel `anchor`,
    `scale` pixels per millimetre."""
    view = np.asarray(view, float) / np.linalg.norm(view)
    right = np.cross((0, 0, 1.0), view)
    right /= np.linalg.norm(right)
    up = np.cross(view, right)
    light = np.array([0.4, -0.3, 0.85])
    light /= np.linalg.norm(light)
    w, h = size
    image = np.ones((h, w, 3))
    depth = np.full((h, w), -np.inf)
    verts, tris = mesh_arrays(m)
    colour = np.asarray(colour)
    rel = verts - np.asarray(centre, float)
    p = np.stack([rel @ right * scale + anchor[0], anchor[1] - rel @ up * scale, rel @ view], -1)
    t = p[tris]
    n = np.cross(verts[tris[:, 1]] - verts[tris[:, 0]], verts[tris[:, 2]] - verts[tris[:, 0]])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    shade = 0.35 + 0.65 * np.clip(n @ light, 0, 1)
    for (a, b, c), sh in zip(t, shade):
        x0, x1 = int(max(min(a[0], b[0], c[0]), 0)), int(min(max(a[0], b[0], c[0]), w - 1))
        y0, y1 = int(max(min(a[1], b[1], c[1]), 0)), int(min(max(a[1], b[1], c[1]), h - 1))
        if x1 < x0 or y1 < y0:
            continue
        xs, ys = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
        det = (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])
        if abs(det) < 1e-9:
            continue
        l1 = ((xs - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (ys - a[1])) / det
        l2 = ((b[0] - a[0]) * (ys - a[1]) - (xs - a[0]) * (b[1] - a[1])) / det
        l0 = 1 - l1 - l2
        inside = (l0 >= 0) & (l1 >= 0) & (l2 >= 0)
        z = l0 * a[2] + l1 * b[2] + l2 * c[2]
        win = inside & (z > depth[y0:y1 + 1, x0:x1 + 1])
        depth[y0:y1 + 1, x0:x1 + 1][win] = z[win]
        image[y0:y1 + 1, x0:x1 + 1][win] = colour * sh
    return image


def scale_bar(ax, size, mm: float, scale: float = SCALE) -> None:
    """A bar of `mm` millimetres at the bottom left of a raster of `size`, in the image
    plane, where `scale` pixels are one millimetre."""
    w, h = size
    x0, y = 12, h - 12
    x1 = x0 + mm * scale
    ax.plot([x0, x1], [y, y], color="k", lw=2, solid_capstyle="butt")
    ax.plot([x0, x0], [y - 5, y + 5], color="k", lw=1)
    ax.plot([x1, x1], [y - 5, y + 5], color="k", lw=1)
    ax.text((x0 + x1) / 2, y - 6, f"{mm:.0f} mm", ha="center", va="bottom", fontsize=8)


def render(path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    size = (400, 400)
    fig, axes = plt.subplots(2, len(STANDS), figsize=(3.3 * len(STANDS), 7.0))
    for i, s in enumerate(STANDS):
        for row, (m, view, title) in enumerate([
                (s.solid(), (1.0, -0.8, 0.45), f"{s.name}: pitch {s.pitch:+.0f}, roll {s.roll:.0f}\n"
                                                f"centre {s.centre()[2]:.0f} mm above the floor"),
                (s.printed(), (1.0, -0.7, 0.6), f"stand_{s.name}.stl as printed, z up\n"
                                                 f"marker face on the bed")]):
            ax = axes[row, i]
            b = np.asarray(m.bounding_box())
            ax.imshow(raster(m, view, size, (b[:3] + b[3:]) / 2, (size[0] / 2, size[1] / 2),
                             colour=(0.80, 0.56, 0.36) if row == 0 else (0.30, 0.45, 0.70)))
            scale_bar(ax, size, 100.0)
            ax.set_title(title, fontsize=9)
            ax.set_axis_off()
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    print(f"wrote {path}")


if __name__ == "__main__":
    main()
