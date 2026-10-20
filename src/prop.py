"""OpenPlane propeller: 54 x 24 in two-blade fixed-pitch, right-hand tractor (clockwise seen from the seat).
Parametric loft of Clark-Y sections (flat face aft), blade pitch measured to the flat face.
Outputs, prop-local mm (Z = thrust axis forward, X = blade 1):
  out/prop/OpenPlane_prop_54x24.stl (ASCII), .obj, OpenPlane_prop_stations.csv, PROP_README.txt
"""
import numpy as np, os, json, trimesh

from geom import CFG, OUT_DIR
OUT = str(OUT_DIR / "prop")
D_IN, P_IN = float(CFG["propeller"]["diameter_in"]), float(CFG["propeller"]["pitch_in"])
R = D_IN * 25.4 / 2            # 685.8 mm
PITCH = P_IN * 25.4            # 609.6 mm
HUB_R, HUB_T = CFG["propeller"]["hub_dia"] / 2, float(CFG["propeller"]["hub_thickness"])     # hub boss: 160 mm dia, 110 mm thick (along the axis)
STACK = 0.35                   # stacking line at 35 % chord

# r/R, chord mm, max thickness mm
TAB = np.array([
    [0.085, 96, 86], [0.13, 98, 70], [0.17, 100, 58], [0.22, 104, 47], [0.30, 112, 36], [0.40, 118, 27],
    [0.50, 120, 21], [0.60, 116, 17], [0.70, 108, 13.5], [0.80, 95, 11], [0.88, 80, 9.2],
    [0.93, 66, 8.0], [0.965, 50, 6.8], [0.985, 34, 5.2], [1.0, 14, 3.0]])

# Clark Y, % chord (flat lower surface)
CX = np.array([0, 1.25, 2.5, 5, 7.5, 10, 15, 20, 30, 40, 50, 60, 70, 80, 90, 95, 100]) / 100
CU = np.array([3.5, 5.45, 6.5, 7.9, 8.85, 9.6, 10.68, 11.36, 11.7, 11.4, 10.52, 9.15, 7.35, 5.22, 2.8, 1.49, 0.12]) / 100
CL = np.array([3.5, 1.93, 1.47, 0.93, 0.63, 0.42, 0.15, 0.03, 0, 0, 0, 0, 0, 0, 0, 0, 0]) / 100
NS = 28  # points per surface


def section(rR, chord, thick):
    """closed loop, (s, n) in chord units: s from LE (0) to TE (1), n toward the cambered (front) face"""
    s = 0.5 * (1 - np.cos(np.linspace(0, np.pi, NS)))
    up = np.interp(s, CX, CU); lo = np.interp(s, CX, CL)
    k = (thick / chord) / 0.117
    up, lo = up * k, lo * k  # scale thickness; the flat face (lo = 0 aft of 30 %) stays flat
    # inboard: blend to a fat ellipse so the blade root runs into the hub
    e = float(np.clip((0.24 - rR) / 0.12, 0, 1))
    ell = np.sqrt(np.clip(s * (1 - s), 0, None)) * (thick / chord)
    up = (1 - e) * up + e * (ell + 0.0)
    lo = (1 - e) * lo + e * (-ell * 0.6)
    up[-1] = lo[-1] = 0.5 * (up[-1] + lo[-1])  # closed TE
    loop = [(s[i], up[i]) for i in range(NS)] + [(s[i], lo[i]) for i in range(NS - 2, 0, -1)]
    return np.array(loop)


def blade():
    rings = []
    rows = []
    for rR, c, t in TAB:
        r = rR * R
        beta = np.arctan(PITCH / (2 * np.pi * r))
        d = np.array([0, -np.cos(beta), -np.sin(beta)])  # LE -> TE
        n = np.array([0, -np.sin(beta), np.cos(beta)])   # toward the front (cambered) face
        loop = section(rR, c, t)
        pts = [np.array([r, 0, 0]) + (s - STACK) * c * d + h * c * n for s, h in loop]
        rings.append(np.array(pts))
        rows.append((rR, r, c, t, np.degrees(beta)))
    m = len(rings[0])
    V = np.vstack(rings)
    F = []
    for k in range(len(rings) - 1):
        for i in range(m):
            a, b = k * m + i, k * m + (i + 1) % m
            c_, d_ = a + m, b + m
            F += [[a, b, d_], [a, d_, c_]]
    # caps: root (inside the hub) and tip
    c0 = len(V); V = np.vstack([V, rings[0].mean(0), rings[-1].mean(0)])
    last = (len(rings) - 1) * m
    for i in range(m):
        F.append([c0, (i + 1) % m, i])
        F.append([c0 + 1, last + i, last + (i + 1) % m])
    mesh = trimesh.Trimesh(V, np.array(F), process=True)
    if mesh.volume < 0:
        mesh.invert()
    return mesh, rows


def build():
    os.makedirs(OUT, exist_ok=True)
    b1, rows = blade()
    b2 = b1.copy(); b2.apply_transform(trimesh.transformations.rotation_matrix(np.pi, [0, 0, 1]))
    hub = trimesh.creation.cylinder(radius=HUB_R, height=HUB_T, sections=64)
    prop = trimesh.boolean.union([hub, b1, b2], engine="manifold")
    assert prop.is_watertight, "prop mesh not watertight"
    prop.export(f"{OUT}/OpenPlane_prop_{D_IN:g}x{P_IN:g}.stl", file_type="stl_ascii")
    prop.export(f"{OUT}/OpenPlane_prop_{D_IN:g}x{P_IN:g}.obj")
    with open(f"{OUT}/OpenPlane_prop_stations.csv", "w") as f:
        f.write("r/R,radius_mm,chord_mm,max_thickness_mm,t/c,blade_angle_deg (flat face)\n")
        for rR, r, c, t, b in rows:
            f.write(f"{rR:.3f},{r:.1f},{c:.1f},{t:.1f},{t / c:.3f},{b:.2f}\n")
    mass_kg = prop.volume * 0.62e-6  # laminated birch/maple ~620 kg/m3 (rough)
    info = dict(D_in=D_IN, P_in=P_IN, R=R, volume_cm3=prop.volume / 1000, mass_kg=mass_kg,
                faces=len(prop.faces), beta75=float(np.degrees(np.arctan(PITCH / (2 * np.pi * 0.75 * R)))))
    with open(f"{OUT}/PROP_README.txt", "w") as f:
        f.write(f"""OpenPlane propeller, 54 x 24 in (1372 mm dia, 610 mm pitch), two blades, fixed pitch
=============================================================================
Rotation: right-hand tractor, CLOCKWISE seen from the pilot's seat. Check your engine's
rotation before you carve: a half-VW turning the other way needs the mirror image.
Units: mm. Axes: Z = thrust axis (forward is +Z), blade 1 along +X, hub centred on the origin.

Files
  OpenPlane_prop_{D_IN:g}x{P_IN:g}.stl / .obj   watertight solid, {len(prop.faces)} triangles
  OpenPlane_prop_stations.csv       radius, chord, thickness and blade angle at each station:
                                use it to make carving templates if you carve by hand

Section: Clark Y, flat face aft. Blade angle is measured to the flat face:
beta = atan(pitch / (2 pi r)), {info['beta75']:.1f} deg at 75 % radius.
Hub boss: {2 * HUB_R:.0f} mm dia x {HUB_T:.0f} mm thick. NO bolt holes or centre bore in the model:
drill them through a template taken from YOUR prop flange (half-VW hubs vary).

What it is for
  A sensible all-rounder for a 37 hp half-VW turning about 3,000-3,200 rpm direct drive on a
  slow (35-70 kt) aircraft: a bit of climb, a bit of cruise. Expect roughly 2,900-3,100 rpm static.
  Treat the pitch as a starting point. Ground-run it: if static rpm is below about 2,900, the
  prop is too coarse and needs about 1 in less pitch; if the engine over-revs in level flight,
  it needs more. A carver can re-pitch it before it is finished.

How to make it
  CNC-carved laminated hardwood (birch or hard maple, 6-8 laminations, resorcinol or epoxy glued),
  then sealed, with a fibreglass leading-edge sheath. Approximate mass {mass_kg:.1f} kg.
  Do NOT fly a 3D-printed prop. A printed model is fine for fit checks and as a carving pattern.
  Balance it statically and dynamically before first run. Track the tips to within 1/16 in.

This is a concept geometry, not a certified prop design. Have a propeller maker check it, or buy a
proven 54 x 24 (or 54 x 22 for more climb) from an established half-VW prop maker.
""")
    return prop, info


def page_mesh(prop, centre=(-1390.0, 0.0, -550.0)):
    """aircraft frame: local Z (thrust) -> -x, local X -> +z, local Y -> +y. Positions relative to the hub centre."""
    s = prop.simplify_quadric_decimation(face_count=5000) if len(prop.faces) > 5000 else prop
    V = s.vertices
    A = np.stack([-V[:, 2], V[:, 1], V[:, 0]], 1)
    return dict(c=list(centre), v=[round(float(x), 1) for x in A.ravel()], f=[int(i) for i in s.faces.ravel()])


if __name__ == "__main__":
    p, info = build()
    print(json.dumps(info, indent=1), p.is_watertight, p.bounds)
