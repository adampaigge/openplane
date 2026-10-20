"""ORI-1 geometry: every sheet part is defined as a set of planar 3D faces joined
by straight hinges, then *unfolded* to its flat blank. The same data drives the
3D folding animation and the DXF cut files, so they cannot disagree.

Units: millimetres. Wing panel frame: x aft from LE, y outboard from root rib,
z up. Aircraft frame: x aft (wing LE = 0), y right, z up (wing chord line = 0).
"""
import math
import numpy as np

# ------------------------------------------------------------------ parameters
# The design lives in aircraft.yaml at the repository root. Every section is merged
# into one flat dict P, so the rest of the code reads P["chord"], P["x_ms"] and so on.
import os, yaml
from pathlib import Path
ROOT = Path(os.environ.get("OPENPLANE_ROOT", Path(__file__).resolve().parents[1]))
CFG = yaml.safe_load(open(ROOT / "aircraft.yaml", encoding="utf8"))
P = {}
for _sec in ("wing", "sheet", "fuselage", "tail"):
    P.update(CFG[_sec])
OUT_DIR = ROOT / "out"
SITE_DIR = ROOT / "site"
RHO_AL = 2.70e-6  # kg/mm^3
RHO_SS = 7.90e-6
RHO_PC = 1.20e-6


# ------------------------------------------------------------------ airfoils
def naca4(code, c, n=400):
    m = int(code[0]) / 100.0
    p = int(code[1]) / 10.0
    t = int(code[2:]) / 100.0
    beta = np.linspace(0, math.pi, n)
    x = (1 - np.cos(beta)) / 2
    yt = 5 * t * (0.2969 * np.sqrt(x) - 0.1260 * x - 0.3516 * x**2 + 0.2843 * x**3 - 0.1036 * x**4)
    yc = np.zeros_like(x)
    dyc = np.zeros_like(x)
    if m > 0:
        f = x < p
        yc[f] = m / p**2 * (2 * p * x[f] - x[f] ** 2)
        dyc[f] = 2 * m / p**2 * (p - x[f])
        g = ~f
        yc[g] = m / (1 - p) ** 2 * ((1 - 2 * p) + 2 * p * x[g] - x[g] ** 2)
        dyc[g] = 2 * m / (1 - p) ** 2 * (p - x[g])
    th = np.arctan(dyc)
    xu, zu = x - yt * np.sin(th), yc + yt * np.cos(th)
    xl, zl = x + yt * np.sin(th), yc - yt * np.cos(th)
    return (xu * c, zu * c), (xl * c, zl * c)


class Airfoil:
    def __init__(self, code, c):
        (self.xu, self.zu), (self.xl, self.zl) = naca4(code, c)
        self.c = c

    def up(self, x):
        return float(np.interp(x, self.xu, self.zu))

    def lo(self, x):
        return float(np.interp(x, self.xl, self.zl))


WING = Airfoil(str(CFG["wing"]["airfoil"]), P["chord"])
# Facet vertices (x/c). The nose (<15% upper, <10% lower) is a smooth rolled
# zone; aft of it the skin is flat facets broken by soft creases.
UP_ROLL = [0, .005, .0125, .025, .04, .06, .08, .10, .125, .15]
UP_FACET = [.15, .30, .45, .60, .72, .86, 1.0]
LO_ROLL = [0, .005, .0125, .025, .04, .06, .08, .10]
LO_FACET = [.10, .30, .50, .72, 1.0]


def wing_up_pts(off=0.0, xmax=None):
    xs = sorted(set(UP_ROLL + UP_FACET))
    pts = [(x * P["chord"], WING.up(x * P["chord"]) - off) for x in xs]
    if xmax is not None:
        pts = clip_x(pts, xmax)
    return pts


def wing_lo_pts(off=0.0, xmax=None):
    xs = sorted(set(LO_ROLL + LO_FACET))
    pts = [(x * P["chord"], WING.lo(x * P["chord"]) + off) for x in xs]
    if xmax is not None:
        pts = clip_x(pts, xmax)
    return pts


def clip_x(pts, xmax, xmin=-1e9):
    """keep polyline part with xmin<=x<=xmax, interpolating ends."""
    out = []
    for i in range(len(pts)):
        x, z = pts[i]
        if xmin <= x <= xmax:
            out.append((x, z))
        if i + 1 < len(pts):
            x2, z2 = pts[i + 1]
            for b in (xmin, xmax):
                if (x - b) * (x2 - b) < 0:
                    out.append((b, z + (z2 - z) * (b - x) / (x2 - x)))
    out.sort()
    # dedupe
    ded = []
    for p in out:
        if not ded or abs(p[0] - ded[-1][0]) > 1e-6:
            ded.append(p)
    return ded


def z_on(pts, x):
    xs = [p[0] for p in pts]
    zs = [p[1] for p in pts]
    return float(np.interp(x, xs, zs))


def is_roll_x(x, upper=True):
    return x < (0.15 if upper else 0.10) * P["chord"] - 1e-6


# ------------------------------------------------------------------ 3D helpers
def v3(*a):
    return np.array(a, dtype=float)


def unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else v


def newell(poly):
    n = np.zeros(3)
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        n += np.array([(a[1] - b[1]) * (a[2] + b[2]), (a[2] - b[2]) * (a[0] + b[0]), (a[0] - b[0]) * (a[1] + b[1])])
    return unit(n)


def bend_deduction(theta_deg, t):
    th = math.radians(abs(theta_deg))
    if th < math.radians(15) or t < 0.45:
        return 0.0
    R, K = 1.6 * t, 0.40           # Wanttaja Table 5-2 'normal' radius for 6061-T6 ~1.6t
    return 2 * (R + t) * math.tan(th / 2) - th * (R + K * t)


# ------------------------------------------------------------------ Part model
class Part:
    """faces: list of dict(pts=[3D], parent=int|-1, hinge=(i,j) indices into this
    face's pts that lie on the parent face, kind='crease'|'roll', holes=[[3D...]] )"""

    def __init__(self, pid, name, group, material, t, note="", density=RHO_AL):
        self.id, self.name, self.group, self.material, self.t = pid, name, group, material, t
        self.note = note
        self.faces = []
        self.instances = []  # list of (4x4 matrix design->world, label)
        self.rivets3d = []   # rivet pilot holes, design frame 3D
        self.extra_holes = []  # (3D point, dia, face index)
        self.bolts3d = []      # (3D point, dia, kind)
        self.density = density
        self.step = None
        self.color = None

    def add(self, pts, parent=-1, hinge=(0, 1), kind="crease", holes=None):
        self.faces.append(dict(pts=[np.asarray(p, float) for p in pts], parent=parent, hinge=hinge,
                               kind=kind, holes=[[np.asarray(q, float) for q in h] for h in (holes or [])]))
        return len(self.faces) - 1

    # -------------------------------------------------------------- unfold
    def unfold(self, deduct=False):
        F = self.faces
        frames = [None] * len(F)
        out = [None] * len(F)
        root = F[0]
        p = root["pts"]
        n = newell(p)
        eu = unit(p[1] - p[0])
        ev = np.cross(n, eu)
        frames[0] = dict(a3=p[0], E1=eu, E2=ev, A=np.zeros(2), f1=np.array([1.0, 0]), f2=np.array([0, 1.0]), n=n, bd=0.0)
        rootM = np.eye(4)
        rootM[:3, 0], rootM[:3, 1], rootM[:3, 2], rootM[:3, 3] = eu, ev, n, p[0]
        order = sorted(range(len(F)), key=lambda i: self.depth(i))
        for i in order:
            f = F[i]
            if i == 0:
                fr = frames[0]
                flat = [self.to_flat(fr, q) for q in f["pts"]]
                out[i] = (dict(poly=flat, parent=-1, hinge=None, ang=0.0, kind=f["kind"],
                                holes=[[self.to_flat(fr, q) for q in h] for h in f["holes"]]))
                continue
            par = frames[f["parent"]]
            ia, ib = f["hinge"]
            a, b = f["pts"][ia], f["pts"][ib]
            A2, B2 = self.to_flat(par, a), self.to_flat(par, b)
            e1 = unit(b - a)
            nc = newell(f["pts"])
            e2 = np.cross(nc, e1)
            ts = [np.dot(q - a, e2) for q in f["pts"]]
            if np.mean(ts) < 0:
                e2 = -e2
            f1 = unit(B2 - A2)
            left = np.array([-f1[1], f1[0]])
            # parent centroid side
            pc2 = np.mean([self.to_flat(par, q) for q in F[f["parent"]]["pts"]], axis=0)
            side = np.dot(pc2 - A2, left)
            f2 = -left if side > 0 else left
            # dihedral angle
            dp_pts = F[f["parent"]]["pts"]
            pcen = np.mean(dp_pts, axis=0)
            # direction in parent plane, away from edge, into parent
            pn = par["n"]
            d_in = np.cross(pn, e1)
            if np.dot(pcen - a, d_in) < 0:
                d_in = -d_in
            d_c = e2
            # hinge direction (child on left of hinge in flat)
            if np.allclose(f2, left):
                hdir3, hinge2 = e1, (A2, B2)
            else:
                hdir3, hinge2 = -e1, (B2, A2)
            u0 = -d_in
            ang = math.degrees(math.atan2(np.dot(hdir3, np.cross(u0, d_c)), np.dot(u0, d_c)))
            bd = bend_deduction(ang, getattr(self, 'tb', self.t)) if deduct else 0.0
            fr = dict(a3=a, E1=e1, E2=e2, A=A2, f1=f1, f2=f2, n=None, bd=bd)
            # child normal consistent with propagated +w: rotate parent normal by ang about hdir3
            fr["n"] = rotate(pn, hdir3, math.radians(ang))
            frames[i] = fr
            flat = [self.to_flat(fr, q) for q in f["pts"]]
            out[i] = (dict(poly=flat, parent=f["parent"], hinge=[list(hinge2[0]), list(hinge2[1])], ang=ang,
                            kind=f["kind"], holes=[[self.to_flat(fr, q) for q in h] for h in f["holes"]]))
        self._frames = frames
        return out, rootM

    def to_flat(self, fr, q):
        d = np.asarray(q, float) - fr["a3"]
        s, t = np.dot(d, fr["E1"]), np.dot(d, fr["E2"])
        if fr["bd"] and t > 1e-6:
            t = max(t - fr["bd"], 0.2)
        return fr["A"] + s * fr["f1"] + t * fr["f2"]

    def depth(self, i):
        d = 0
        while self.faces[i]["parent"] >= 0:
            i = self.faces[i]["parent"]
            d += 1
        return d

    def face_of(self, q, tol=0.6):
        best = None
        for i, f in enumerate(self.faces):
            n = newell(f["pts"])
            if abs(np.dot(q - f["pts"][0], n)) < tol and point_in_face(q, f["pts"], n):
                best = i
                break
        return best

    def area_mm2(self):
        a = 0.0
        for f in self.faces:
            a += poly_area3(f["pts"])
            for h in f["holes"]:
                a -= poly_area3(h)
        return a

    def mass(self):
        return self.area_mm2() * self.t * self.density


def rotate(v, k, th):
    k = unit(k)
    return v * math.cos(th) + np.cross(k, v) * math.sin(th) + k * np.dot(k, v) * (1 - math.cos(th))


def poly_area3(pts):
    n = newell(pts)
    s = np.zeros(3)
    for i in range(len(pts)):
        s += np.cross(pts[i], pts[(i + 1) % len(pts)])
    return abs(np.dot(s, n)) / 2


def point_in_face(q, pts, n):
    # project to 2D
    a = unit(pts[1] - pts[0])
    b = np.cross(n, a)
    P2 = [(np.dot(p - pts[0], a), np.dot(p - pts[0], b)) for p in pts]
    x, y = np.dot(q - pts[0], a), np.dot(q - pts[0], b)
    inside = False
    for i in range(len(P2)):
        x1, y1 = P2[i]
        x2, y2 = P2[(i + 1) % len(P2)]
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


# ------------------------------------------------------------------ builders
def extrude_path(part, path2, E, A, B, O, u0, u1, root, kinds=None):
    """path2: list of (a,b) points in section plane spanned by A,B (3D unit) at
    origin O; extrude along E from u0 to u1. Facet k joins path2[k]..path2[k+1]."""
    def P3(ab, u):
        return O + ab[0] * A + ab[1] * B + u * E
    nf = len(path2) - 1
    idx = {}
    faces = {}
    for k in range(nf):
        faces[k] = [P3(path2[k], u0), P3(path2[k + 1], u0), P3(path2[k + 1], u1), P3(path2[k], u1)]
    # root first
    idx[root] = part.add(faces[root])
    for k in range(root + 1, nf):  # hinge = start edge (pts 0 and 3)
        idx[k] = part.add(faces[k], parent=idx[k - 1], hinge=(0, 3), kind=(kinds[k] if kinds else "crease"))
    for k in range(root - 1, -1, -1):  # hinge = end edge (pts 1 and 2)
        idx[k] = part.add(faces[k], parent=idx[k + 1], hinge=(1, 2), kind=(kinds[k] if kinds else "crease"))
    return idx


def flanged_web(part, web, normal_dir, flanges, holes=None, rivet_pitch=40.0, rivet_flanges=True):
    """web: planar 3D polygon. flanges: list of (edge_index, width, dir3) where
    dir3 is the unit direction the flange extends from the edge (lies in the
    mating surface). Returns face indices."""
    W = [np.asarray(p, float) for p in web]
    wi = part.add(W, holes=holes)
    n = len(W)
    fl_edges = {e for e, *_ in flanges}
    for (e, w, d) in flanges:
        a, b = W[e], W[(e + 1) % n]
        el = np.linalg.norm(b - a)
        ed = unit(b - a)
        d = unit(np.asarray(d, float))
        # relief/inset at each end depending on neighbouring flange
        def inset(vtx_prev_edge, at_start):
            # angle between this edge and neighbour edge
            if at_start:
                pe = (e - 1) % n
                v1 = unit(W[e] - W[pe])
            else:
                pe = (e + 1) % n
                v1 = unit(W[(pe + 1) % n] - W[pe])
            cosang = np.clip(np.dot(v1, ed), -1, 1)
            turn = math.acos(cosang)
            if pe not in fl_edges:
                return 0.0
            return min(w * math.tan(min(turn, math.radians(80)) / 2) + 1.5, el * 0.3)
        ia, ib = inset(None, True), inset(None, False)
        a2, b2 = a + ed * ia, b - ed * ib
        quad = [a2, b2, b2 + d * w, a2 + d * w]
        part.add(quad, parent=wi, hinge=(0, 1))
        if rivet_flanges:
            L = np.linalg.norm(b2 - a2)
            nr = max(int((L - 20) // rivet_pitch) + 1, 1 if L > 18 else 0)
            for k in range(nr):
                s = L / 2 if nr == 1 else 10 + k * (L - 20) / (nr - 1)
                part.rivets3d.append(a2 + ed * s + d * (w / 2))
    return wi


# ------------------------------------------------------------------ matrices
def mat_T(x, y, z):
    M = np.eye(4)
    M[:3, 3] = (x, y, z)
    return M


def mat_Rx(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    M = np.eye(4)
    M[1, 1], M[1, 2], M[2, 1], M[2, 2] = c, -s, s, c
    return M


def mat_Ry(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    M = np.eye(4)
    M[0, 0], M[0, 2], M[2, 0], M[2, 2] = c, s, -s, c
    return M


MIRROR_Y = np.diag([1.0, -1.0, 1.0, 1.0])


def wing_to_world(side):
    """panel frame -> aircraft. Nose-up incidence rotates about the panel's y
    axis (positive pitch = LE up => rotate by -inc about +y given x aft)."""
    M = mat_T(0, P["cabin_half"], 0) @ mat_Rx(P["dihedral"]) @ mat_Ry(P["incidence"])
    # mat_Ry(+) with x aft, z up: rotates +x toward -z, i.e. TE down = nose up. good.
    return M if side == "R" else MIRROR_Y @ M
