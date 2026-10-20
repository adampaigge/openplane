"""Generate cut files (DXF/SVG), nests, BOM, report and the web-page model JSON."""
import csv
import json
import math
import os
import shutil
import zipfile
import numpy as np
import ezdxf

def dxf_save(doc, path):
    """Write a DXF with every date, timer, GUID and the ezdxf write stamp pinned, so an unchanged
    part produces a byte-identical file and git only shows real geometry changes."""
    import io, re as _re
    buf = io.StringIO(); doc.write(buf); txt = buf.getvalue()
    txt = _re.sub(r"(\n\$(?:TDCREATE|TDUCREATE|TDUPDATE|TDUUPDATE)\n\s*40\n)[^\n]+", r"\g<1>2460676.0", txt)
    txt = _re.sub(r"(\n\$(?:TDINDWG|TDUSRTIMER)\n\s*40\n)[^\n]+", r"\g<1>0.0", txt)
    txt = _re.sub(r"(\n\$(?:FINGERPRINTGUID|VERSIONGUID)\n\s*2\n)[^\n]+", r"\g<1>{00000000-0000-0000-0000-000000000000}", txt)
    txt = _re.sub(r"\n(\d+\.\d+\.\d+) @ \d{4}-\d\d-\d\dT[^\n]+", r"\n\1 @ 2025-01-01T00:00:00+00:00", txt)
    open(path, "w", encoding="utf8", newline="").write(txt)
from shapely.geometry import Polygon, Point, LineString
from shapely.ops import unary_union
from shapely import affinity

from geom import P, newell, wing_to_world, mat_T, MIRROR_Y
from design import run, KT

from geom import OUT_DIR, CFG
CFG_LD = CFG["loading"]
OUT = str(OUT_DIR)
shutil.rmtree(OUT, ignore_errors=True)
for d in ("dxf/parts", "dxf/nests", "svg"):
    os.makedirs(f"{OUT}/{d}", exist_ok=True)

res, parts, items = run(verbose=False)


def col_major(M):
    return [float(v) for v in np.asarray(M).T.reshape(-1)]


# ---------------------------------------------------------------- flat data
def flat_holes_for(p, frames, pts3):
    out = []
    for q in pts3:
        fi = p.face_of(np.asarray(q), tol=2.2)
        if fi is None:
            continue
        out.append(p.to_flat(frames[fi], q))
    return out


spar_extra = {"W-MS": [], "W-RS": []}
by_id = {p.id: p for p in parts}
for rid in ("W-NR", "W-MR", "W-NRe", "W-MRe"):
    r = by_id[rid]
    st = [k * P["rib_pitch"] for k in range(1, 12)] if rid in ("W-NR", "W-MR") else []
    tf = [mat_T(0, s, 0) for s in st]
    if rid.endswith("e"):
        tf = [mat_T(0, 0, 0) @ MIRROR_Y, mat_T(0, P["panel"], 0)]
    for q in r.rivets3d:
        for M in tf:
            w = (M @ np.array([*q, 1.0]))[:3]
            for sid in spar_extra:
                sp = by_id[sid]
                if sp.face_of(w, tol=2.2) == 0:  # web face
                    spar_extra[sid].append(w)

from geom import WING
from parts import UP as _UP, LO as _LO
from geom import z_on as _z_on

MATE = {"W-NR": 0.5, "W-MR": 0.5, "W-NRe": 0.5, "W-MRe": 0.5, "W-MS": 0.5, "W-RS": 0.5}


def rivet_step(p, q, fi):
    if p.id in ("W-NR", "W-MR", "W-NRe", "W-MRe"):
        n = newell(p.faces[fi]["pts"])
        if abs(n[0]) > 0.9:  # attach flange on spar web
            return 4
        mid = 0.5 * (_z_on(_UP, q[0]) + _z_on(_LO, q[0]))
        return 6 if q[2] > mid else 5
    if p.id in ("W-MS", "W-RS"):
        mid = 0.5 * (_z_on(_UP, q[0]) + _z_on(_LO, q[0]))
        return 6 if q[2] > mid else 5
    return p.step


def rivet_grip(p, q, fi):
    t = p.t
    if p.id in ("W-NR", "W-MR", "W-NRe", "W-MRe"):
        n = newell(p.faces[fi]["pts"])
        if abs(n[0]) > 0.9:
            mate = P["t_ms"] if q[0] < 0.5 * (P["x_ms"] + P["x_rs"]) else P["t_rs"]
        else:
            mid = 0.5 * (_z_on(_UP, q[0]) + _z_on(_LO, q[0]))
            mate = P["t_skin_u"] if q[2] > mid else P["t_skin_l"]
    elif p.id == "F-FA" or p.id == "F-RA":
        mate = 0.5
    elif p.material.startswith("Stainless"):
        mate = 0.5
    else:
        mate = 0.5
    stack = t + mate
    return stack, (1 if stack <= 1.6 else 2)


flat_mold, flat_ded, riv_info, bolt_info = {}, {}, {}, {}
for p in parts:
    fm, rootM = p.unfold(deduct=False)
    fm_frames = p._frames
    recs = []
    for q in p.rivets3d:
        fi = p.face_of(np.asarray(q), tol=2.2)
        if fi is None:
            continue
        st, gr = rivet_grip(p, q, fi)
        recs.append(dict(f=fi, xy=[round(v, 1) for v in p.to_flat(fm_frames[fi], q)], step=rivet_step(p, q, fi), grip=gr,
                         firewall=p.material.startswith("Stainless")))
    riv_info[p.id] = recs
    bl = []
    for (q, d, kind) in p.bolts3d:
        fi = p.face_of(np.asarray(q), tol=2.2)
        if fi is None:
            continue
        bl.append(dict(f=fi, xy=[round(v, 1) for v in p.to_flat(fm_frames[fi], q)], d=d, kind=kind))
    bolt_info[p.id] = bl
    fd, _ = p.unfold(deduct=True)
    fd_frames = p._frames
    riv = list(p.rivets3d) + spar_extra.get(p.id, [])
    bolts_flat = []
    for (q, d, kind) in p.bolts3d:
        fi = p.face_of(np.asarray(q), tol=2.2)
        if fi is not None:
            bolts_flat.append((p.to_flat(fd_frames[fi], q), d))
    flat_mold[p.id] = (fm, rootM, flat_holes_for(p, fm_frames, riv))
    flat_ded[p.id] = (fd, flat_holes_for(p, fd_frames, riv), bolts_flat)

total_rivets = sum((len(p.rivets3d)) * len(p.instances) for p in parts)


# ---------------------------------------------------------------- outlines
def outline(panels, relief_d=3.0):
    polys = [Polygon(f["poly"]).buffer(0) for f in panels]
    holes = []
    for f in panels:
        for h in f["holes"]:
            holes.append(Polygon(h).buffer(0))
    u = unary_union([q.buffer(0.02, join_style=2) for q in polys]).buffer(-0.02, join_style=2)
    # relief holes where two creased flanges meet at a web vertex
    root = panels[0]
    kids = [f for f in panels if f["parent"] == 0 and f["hinge"]]
    reliefs = []
    for i, a in enumerate(kids):
        for b in kids[i + 1:]:
            for pa in a["hinge"]:
                for pb in b["hinge"]:
                    if abs(abs(a["ang"]) - 0) > 30 and abs(b["ang"]) > 30:
                        d = math.dist(pa, pb)
                        if d < 30:
                            mid = ((pa[0] + pb[0]) / 2, (pa[1] + pb[1]) / 2)
                            reliefs.append(Point(mid).buffer(relief_d / 2, 16))
    for h in holes + reliefs:
        u = u.difference(h)
    return u


def geoms(u):
    return list(u.geoms) if hasattr(u, "geoms") else [u]


# ---------------------------------------------------------------- DXF per part
LAYERS = {"CUT": 7, "VALLEY": 1, "MOUNTAIN": 5, "ROLL": 3, "HOLE": 6, "ETCH": 8}


def new_doc():
    doc = ezdxf.new("R2010", setup=True)
    doc.units = ezdxf.units.MM
    for n, c in LAYERS.items():
        lt = {"VALLEY": "DASHED", "MOUNTAIN": "DASHDOT", "ROLL": "DOT"}.get(n, "CONTINUOUS")
        doc.layers.add(n, color=c, linetype=lt)
    return doc


def draw_part(msp, p, panels, holes, ox=0.0, oy=0.0, rot=0.0, label=True, qty=None, bolts=()):
    def T(pt):
        x, y = pt
        if rot:
            c, s = math.cos(rot), math.sin(rot)
            x, y = x * c - y * s, x * s + y * c
        return (x + ox, y + oy)
    u = outline(panels)
    for g in geoms(u):
        msp.add_lwpolyline([T(q) for q in g.exterior.coords], close=True, dxfattribs={"layer": "CUT"})
        for r in g.interiors:
            msp.add_lwpolyline([T(q) for q in r.coords], close=True, dxfattribs={"layer": "CUT"})
    for f in panels:
        if f["parent"] < 0 or not f["hinge"] or abs(f["ang"]) < 0.5:
            continue
        a, b = T(f["hinge"][0]), T(f["hinge"][1])
        if f["kind"] == "roll":
            msp.add_line(a, b, dxfattribs={"layer": "ROLL"})
        else:
            lay = "VALLEY" if f["ang"] > 0 else "MOUNTAIN"
            msp.add_line(a, b, dxfattribs={"layer": lay})
            if label:
                m = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
                msp.add_text(f"{'V' if f['ang'] > 0 else 'M'} {abs(f['ang']):.0f}", height=6,
                             dxfattribs={"layer": "ETCH", "insert": m})
    for h in holes:
        msp.add_circle(T(h), radius=1.25, dxfattribs={"layer": "HOLE"})  # #40 pilot, final #30 at assembly
    for (h, d) in bolts:
        msp.add_circle(T(h), radius=d / 2 - 0.25, dxfattribs={"layer": "CUT"})  # cut 0.5 under, ream to size
    if label:
        c = u.centroid
        msp.add_text(f"{p.id}  {p.material}" + (f"  QTY {qty}" if qty else ""), height=12,
                     dxfattribs={"layer": "ETCH", "insert": T((c.x, c.y))})
    return u


bom = []
for p in parts:
    fd, holes, bolts_flat = flat_ded[p.id]
    doc = new_doc()
    msp = doc.modelspace()
    u = draw_part(msp, p, fd, holes, qty=len(p.instances) * getattr(p, "layers", 1), bolts=bolts_flat)
    dxf_save(doc, f"{OUT}/dxf/parts/{p.id}.dxf")
    minx, miny, maxx, maxy = u.bounds
    bom.append(dict(id=p.id, name=p.name, group=p.group, material=p.material, t_mm=p.t, qty=len(p.instances) * getattr(p, "layers", 1), layers=getattr(p, "layers", 1), per_ac=len(p.instances),
                    flat_w=round(maxx - minx), flat_h=round(maxy - miny), folds=sum(1 for f in fd if f["parent"] >= 0 and abs(f["ang"]) >= 0.5),
                    pilot_holes=len(holes), bolt_holes=len(bolts_flat), mass_g=round(p.mass() * 1000), step=p.step, note=p.note))


# ---------------------------------------------------------------- SVG crease patterns (origami-book style)
def svg_part(p, panels, holes, bolts=()):
    u = outline(panels)
    minx, miny, maxx, maxy = u.bounds
    pad = 20
    W, H = maxx - minx + 2 * pad, maxy - miny + 2 * pad
    tx = lambda q: (q[0] - minx + pad, maxy - q[1] + pad)
    s = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W:.1f} {H:.1f}" width="{W:.0f}mm" height="{H:.0f}mm">',
         '<rect width="100%" height="100%" fill="#fbf7ef"/>']
    for g in geoms(u):
        d = "M" + " L".join(f"{tx(q)[0]:.2f},{tx(q)[1]:.2f}" for q in g.exterior.coords) + " Z"
        for r in g.interiors:
            d += " M" + " L".join(f"{tx(q)[0]:.2f},{tx(q)[1]:.2f}" for q in r.coords) + " Z"
        s.append(f'<path d="{d}" fill="#e9e3d6" fill-rule="evenodd" stroke="#111" stroke-width="1.2"/>')
    for f in panels:
        if f["parent"] < 0 or not f["hinge"] or abs(f["ang"]) < 0.5:
            continue
        a, b = tx(f["hinge"][0]), tx(f["hinge"][1])
        if f["kind"] == "roll":
            st = 'stroke="#2a7" stroke-width="0.8" stroke-dasharray="1.5 3"'
        elif f["ang"] > 0:
            st = 'stroke="#c22" stroke-width="1.2" stroke-dasharray="8 5"'
        else:
            st = 'stroke="#25a" stroke-width="1.2" stroke-dasharray="10 4 2 4"'
        s.append(f'<line x1="{a[0]:.2f}" y1="{a[1]:.2f}" x2="{b[0]:.2f}" y2="{b[1]:.2f}" {st}/>')
    for h in holes:
        q = tx(h)
        s.append(f'<circle cx="{q[0]:.2f}" cy="{q[1]:.2f}" r="1.6" fill="#111"/>')
    for (h, d) in bolts:
        q = tx(h)
        s.append(f'<circle cx="{q[0]:.2f}" cy="{q[1]:.2f}" r="{d/2:.2f}" fill="#fbf7ef" stroke="#111" stroke-width="1"/>')
    s.append(f'<text x="{pad}" y="{H - 6:.0f}" font-family="sans-serif" font-size="14">{p.id} {p.name} | {p.material} | qty {len(p.instances)}</text>')
    s.append("</svg>")
    return "\n".join(s)


for p in parts:
    fm, rootM, holes = flat_mold[p.id]
    open(f"{OUT}/svg/{p.id}.svg", "w").write(svg_part(p, fm, holes, [(b["xy"], b["d"]) for b in bolt_info[p.id]]))


# ---------------------------------------------------------------- nesting (shelf packer, bounding boxes)
STOCK = {
    "6061-T6": [(2438, 1219, "8 ft x 4 ft sheet"), (3658, 1219, "12 ft x 4 ft sheet (spar strips)")],
    "Stainless": [(2000, 1000, "2000 x 1000 sheet")],
    "Polycarbonate": [(2050, 1250, "2050 x 1250 sheet")],
    "4130": [(1219, 610, "4130 sheet 4 ft x 2 ft")],
}
GAP = 8
groups = {}
for p in parts:
    groups.setdefault(p.material, []).extend([p] * (len(p.instances) * getattr(p, "layers", 1)))
nest_summary = []
for mat, plist in groups.items():
    fam = next(k for k in STOCK if mat.startswith(k))
    shapes = []
    for p in plist:
        fd, holes, _bf = flat_ded[p.id]
        u = outline(fd)
        mrr = u.minimum_rotated_rectangle
        cs = list(mrr.exterior.coords)
        e = (cs[1][0] - cs[0][0], cs[1][1] - cs[0][1])
        ang = -math.atan2(e[1], e[0])
        ur = affinity.rotate(u, ang, origin=(0, 0), use_radians=True)
        bx = ur.bounds
        w, h = bx[2] - bx[0], bx[3] - bx[1]
        if h > w:
            ang += math.pi / 2
            ur = affinity.rotate(u, ang, origin=(0, 0), use_radians=True)
            bx = ur.bounds
            w, h = bx[2] - bx[0], bx[3] - bx[1]
        shapes.append(dict(p=p, ang=ang, w=w, h=h, bx=bx))
    shapes.sort(key=lambda s: (-max(s["w"], s["h"]), -min(s["w"], s["h"])))
    sheets = []
    for s in shapes:
        best = None
        for rot in (False, True):
            w, h = (s["h"], s["w"]) if rot else (s["w"], s["h"])
            for sh in sheets:
                SW, SH = sh["stock"][0], sh["stock"][1]
                for shelf in sh["shelves"]:
                    if h <= shelf["h"] and shelf["x"] + w <= SW:
                        waste = (shelf["h"] - h) * w
                        if best is None or waste < best[0]:
                            best = (waste, sh, shelf, rot, w, h, None)
                y_next = sum(x["h"] + GAP for x in sh["shelves"])
                if y_next + h <= SH and w <= SW:
                    waste = (SW - w) * h * 0.5 + 1e6
                    if best is None or waste < best[0]:
                        best = (waste, sh, None, rot, w, h, y_next)
        if best is None:
            for rot in (False, True):
                w, h = (s["h"], s["w"]) if rot else (s["w"], s["h"])
                stock = next((st for st in STOCK[fam] if w <= st[0] and h <= st[1]), None)
                if stock and (best is None or h < best[5]):
                    best = (0, dict(stock=stock, shelves=[], placed=[]), None, rot, w, h, 0.0)
            if best is None:
                raise SystemExit(f"{s['p'].id} does not fit any stock")
            sheets.append(best[1])
        _, sh, shelf, rot, w, h, y_next = best
        s2 = dict(s)
        if rot:
            s2["ang"] = s["ang"] + math.pi / 2
            u = outline(flat_ded[s["p"].id][0])
            s2["bx"] = affinity.rotate(u, s2["ang"], origin=(0, 0), use_radians=True).bounds
        if shelf is None:
            shelf = dict(y=y_next, h=h, x=0.0)
            sh["shelves"].append(shelf)
        sh["placed"].append((s2, shelf["x"], shelf["y"]))
        shelf["x"] += w + GAP
    for i, sh in enumerate(sheets):
        doc = new_doc()
        msp = doc.modelspace()
        SW, SH = sh["stock"][0], sh["stock"][1]
        msp.add_lwpolyline([(0, 0), (SW, 0), (SW, SH), (0, SH)], close=True, dxfattribs={"layer": "ETCH"})
        used = 0.0
        for (s, x, y) in sh["placed"]:
            p = s["p"]
            fd, holes, bf = flat_ded[p.id]
            ox, oy = x - s["bx"][0], y - s["bx"][1]
            uu = draw_part(msp, p, fd, holes, ox=ox, oy=oy, rot=s["ang"], label=True, bolts=bf)
            used += uu.area
        tag = mat.replace(" ", "_").replace(".", "")
        fn = f"{tag}_{i + 1:02d}.dxf"
        dxf_save(doc, f"{OUT}/dxf/nests/{fn}")
        nest_summary.append(dict(material=mat, sheet=i + 1, stock=sh["stock"][2], parts=len(sh["placed"]),
                                 util=round(100 * used / (SW * SH)), file=fn))

stock_count = {}
for n in nest_summary:
    k = (n["material"], n["stock"])
    stock_count[k] = stock_count.get(k, 0) + 1


# ---------------------------------------------------------------- cost (rough, GBP, 2026 retail)
price_m2 = {"0.016in": 110, "0.020in": 130, "0.025in": 115, "0.032in": 110, "0.040in": 135, "0.050in": 135}
mat_cost = 0.0
for (mat, stock), n in stock_count.items():
    area = {"8 ft x 4 ft sheet": 2.438 * 1.219, "12 ft x 4 ft sheet (spar strips)": 3.658 * 1.219,
            "2000 x 1000 sheet": 2.0, "2050 x 1250 sheet": 2.56, "4130 sheet 4 ft x 2 ft": 0.74}[stock]
    if mat.startswith("6061"):
        mat_cost += n * area * price_m2.get(mat.split()[-1], 120)
    elif mat.startswith("Stainless"):
        mat_cost += n * 90
    elif mat.startswith("4130"):
        mat_cost += n * 160
    else:
        mat_cost += n * 140


# ---------------------------------------------------------------- bought primitives for 3D
def wp(side, pt):
    return (wing_to_world(side) @ np.array([*pt, 1.0]))[:3].tolist()


def inst_pt(M, q):
    return (M @ np.array([*q, 1.0]))[:3].tolist()


bought3d = []
def prim(kind, step, name, **kw):
    bought3d.append(dict(kind=kind, step=step, name=name, **kw))

ENG = 14
prim("box", ENG, "Half-VW engine", c=[-1120, 0, -550], s=[300, 230, 210], col="eng")
for sgn in (1, -1):
    prim("cyl", ENG, "Cylinder", a=[-1120, sgn * 115, -550], b=[-1120, sgn * 300, -550], r=58, col="eng")
    prim("box", ENG, "Head", c=[-1120, sgn * 320, -550], s=[150, 50, 150], col="eng")
mount = []
for (y, z) in ((250, -700), (-250, -700), (250, -400), (-250, -400)):
    a = [P["x_fw"], y, z]
    b = [-970, y * 0.4, -550 + (z + 550) * 0.4]
    mount.append((a, b))
    prim("cyl", ENG, "Engine mount tube (4130)", a=a, b=b, r=9, col="steel")
prim("cyl", ENG, "Prop drive hub (flange)", a=[-1290, 0, -550], b=[-1340, 0, -550], r=55, col="steel")
prim("cone", ENG, "Spinner", a=[-1450, 0, -550], b=[-1570, 0, -550], r=86, col="steel")
import prop as _prop
_pm, PROP_INFO = _prop.build()
prim("mesh", ENG, "Propeller 54 x 24 in, laminated wood (download the model)", col="wood", spin=True, **_prop.page_mesh(_pm, (-1395.0, 0.0, -550.0)))
prim("box", ENG, "Fuel tank 19 L (rotomoulded polyethylene): header tank behind the firewall, ahead of the panel, Cub-style", c=[-570, 0, -465], s=[150, 660, 190], col="tank")
prim("box", ENG, "Tank strap angle (L)", c=[-570, -335, -565], s=[150, 10, 20], col="steel")
prim("box", ENG, "Tank strap angle (R)", c=[-570, 335, -565], s=[150, 10, 20], col="steel")
prim("cyl", ENG, "Fuel filler neck + cap (through the hood, ahead of the windscreen)", a=[-600, 0, -370], b=[-700, 0, -335], r=22, col="steel")
prim("box", ENG, "Gascolator (fuel strainer) on the firewall", c=[-700, 90, -780], s=[50, 50, 70], col="steel")
prim("box", ENG, "Fuel selector valve: HEADER / AUX / OFF", c=[-600, 90, -600], s=[40, 40, 40], col="fuel")
prim("box", ENG, "Aux fuel tank 20 L (rotomoulded), high behind the seat, gravity feed", c=[865, 40, -300], s=[220, 460, 200], col="tank")
prim("cyl", ENG, "Aux tank filler (through the left cabin side, behind the door)", a=[865, -190, -250], b=[865, -360, -250], r=20, col="steel")
GEAR = 13
legs = []
for sgn in (1, -1):
    a, b = [-20, sgn * 300, -1045], [-110, sgn * 800, -1248]
    legs.append((a, b))
    prim("bar", GEAR, "Main gear spring leg (6061-T6 bar)", a=a, b=b, w=75, t=16, col="steel")
    prim("cyl", GEAR, "Main wheel: 21 x 8.00-6 tundra tyre at 10-12 psi", a=[-110, sgn * 812, -1248], b=[-110, sgn * 1012, -1248], r=267, col="tyre")
    prim("cyl", GEAR, "Wheel hub", a=[-110, sgn * 800, -1248], b=[-110, sgn * 1024, -1248], r=75, col="steel")
    prim("cyl", GEAR, "Brake drum", a=[-110, sgn * 790, -1248], b=[-110, sgn * 812, -1248], r=90, col="steel")
TW = dict(a=[4280, 0, -560], b=[4560, 0, -690])
prim("bar", GEAR, "Tailwheel spring (spring steel leaf)", a=TW["a"], b=TW["b"], w=45, t=8, col="steel")
prim("box", GEAR, "Tailwheel fork", c=[4560, 0, -700], s=[60, 80, 40], col="steel")
prim("cyl", GEAR, "Tailwheel 8 in pneumatic", a=[4560, -32, -710], b=[4560, 32, -710], r=100, col="tyre")
for sgn in (1, -1):
    prim("cyl", GEAR, "Tailwheel steering spring", a=[4560, 0, -690], b=[4420, sgn * 70, -575], r=4, col="steel")

# struts & jury struts: pin-to-pin from the clevis pin centres
from parts import STRUTS
cv = by_id["K-CV"]
cv_pin = [np.mean([q for (q, d, k) in cv.bolts3d if k == "pin"], axis=0)]
pins = {lab: inst_pt(M, cv_pin[0]) for (M, lab) in cv.instances}
STRUT = 16
strut_defs = []
for (side, nm, foot, top, jb, jt) in STRUTS:
    foot, top, jb, jt = [list(map(float, v)) for v in (foot, top, jb, jt)]
    strut_defs.append((side, nm, foot, top, jb, jt))
    d = np.subtract(top, foot) / np.linalg.norm(np.subtract(top, foot))
    prim("cyl", STRUT, f"Lift strut ({nm}) 1.75 x 0.065 in", a=(np.array(foot) + d * 40).tolist(), b=(np.array(top) - d * 40).tolist(), r=22, col="strut")
    for e, sgn in ((foot, 1), (top, -1)):
        prim("cyl", STRUT, "Strut end fork (4130)", a=e, b=(np.array(e) + sgn * d * 60).tolist(), r=10, col="steel")
    dj = np.subtract(jt, jb) / np.linalg.norm(np.subtract(jt, jb))
    prim("cyl", STRUT, "Jury strut 0.75 x 0.049 in", a=(np.array(jb) + dj * 15).tolist(), b=(np.array(jt) - dj * 15).tolist(), r=9, col="strut")
L_main = round(np.linalg.norm(np.subtract(strut_defs[0][3], strut_defs[0][2])))
L_rear = round(np.linalg.norm(np.subtract(strut_defs[1][3], strut_defs[1][2])))
L_jury = round(np.linalg.norm(np.subtract(strut_defs[0][5], strut_defs[0][4])))
L_juryr = round(np.linalg.norm(np.subtract(strut_defs[1][5], strut_defs[1][4])))

CTRL = 17
prim("cyl", CTRL, "Control stick", a=[200, 0, -1040], b=[150, 0, -640], r=12, col="steel")
prim("cyl", CTRL, "Aileron torque tube", a=[200, 0, -1030], b=[900, 0, -1030], r=16, col="steel")
prim("box", CTRL, "Rudder pedals", c=[-300, 0, -980], s=[60, 360, 120], col="steel")
prim("box", CTRL, "Flap lever (overhead, Kitfox-style)", c=[850, -250, -170], s=[30, 20, 160], col="steel")
prim("box", CTRL, "Elevator bellcrank (under seat)", c=[520, 0, -1020], s=[40, 30, 90], col="steel")
SYS = 17
from parts import tc_pt, SEC
from parts import PANEL_HOLES
for (cy, cz, dd, nm) in PANEL_HOLES:
    if dd >= 50:
        prim("cyl", SYS, f"{nm} instrument", a=[-401, cy, cz], b=[-401 - (90 if dd > 60 else 110), cy, cz], r=dd / 2 - 2, col="eng")
prim("cyl", SYS, "Compass (on the glare shield)", a=[-375, 0, -398], b=[-375, 0, -360], r=30, col="eng")
prim("box", SYS, "Battery 12 V LiFePO4 in vented box (under the seat)", c=[450, 150, -1000], s=[150, 90, 90], col="eng")
prim("cyl", SYS, "Pitot tube (left wing, ahead of the LE)", a=wp("L", [-150, 1200, -30]), b=wp("L", [150, 1200, -30]), r=6, col="steel")
prim("cyl", SYS, "Static port", a=[1100, -352, -700], b=[1100, -357, -700], r=12, col="steel")
prim("cyl", SYS, "VHF antenna whip", a=[1600, 0, float(tc_pt(1600, SEC[0])[2])], b=[1600, 0, float(tc_pt(1600, SEC[0])[2]) + 520], r=4, col="steel")
prim("cyl", SYS, "Fire extinguisher 1 kg (clip by the left seat rail)", a=[520, -280, -1040], b=[520, -280, -780], r=40, col="fuel")
prim("box", SYS, "Baggage locker behind the seat (net, 5 kg max)", c=[860, 0, -930], s=[260, 520, 220], col="bag")
prim("box", CTRL, "Rudder pedals", c=[-300, 0, -980], s=[60, 360, 120], col="steel")
# ---- control runs (routed), world mm. Each tailcone run is ONE straight line from its cabin pulley
# to the tail-post grommet: the frustum is convex, so a straight run can't touch the skin, and
# no two runs cross. Fairleads sit where each line passes a bulkhead.
from parts import tc_pt, SEC
BH = [1150.0, 1947.5, 2745.0, 3542.5, 4340.0]
ROUTES = []
STEP_OF = {"fuel": 14}
def route(name, kind, color, pts, pulleys=(), fairleads=(), step=None):
    ROUTES.append(dict(name=name, kind=kind, color=color, step=step or STEP_OF.get(kind, 17),
                       pts=[[round(float(v), 1) for v in p] for p in pts], pulleys=list(pulleys), fairleads=[[round(float(v), 1) for v in f] for f in fairleads]))
def straight_with_fairleads(a, b, xs):
    a, b = np.array(a, float), np.array(b, float)
    out = []
    for x in xs:
        if min(a[0], b[0]) < x < max(a[0], b[0]):
            t_ = (x - a[0]) / (b[0] - a[0]); out.append(a + (b - a) * t_)
    return out
TAIL_X = 4330.0
for sg, nm in ((1, "R"), (-1, "L")):
    A = [1160, sg * 185, -1020]; Bp = [TAIL_X, sg * 40, -545]
    pts = [[-300, sg * 150, -1000], [-260, sg * 200, -1035], [1100, sg * 200, -1035], A, Bp, [4372, sg * 75, -545]]
    route(f"Rudder cable {nm}", "cable", "#f0b35a", pts, pulleys=[1, 3], fairleads=straight_with_fairleads(A, Bp, BH[1:]) + [[1008, sg * 200, -1035], [420, sg * 200, -1035]])
for sg, nm, zA, zB, horn in ((1, "up", -1000, -445, -362), (-1, "down", -1040, -505, -505)):
    A = [520, sg * 30, zA]; Bp = [TAIL_X, sg * 30, zB]
    route(f"Elevator cable ({nm})", "cable", "#5ad6ec", [A, Bp, [4385, sg * 30, horn]], pulleys=[], fairleads=straight_with_fairleads(A, Bp, BH))
route("Elevator pushrod: stick to bellcrank under the seat", "rod", "#5ad6ec", [[200, 0, -1010], [500, 0, -1020], [520, 0, -1020]])
from parts import CONTROL_RODS
for r_ in CONTROL_RODS:
    route(r_["name"], "rod", r_["color"], r_["pts"])
# ---- systems
route("Fuel feed: tank > shut-off > firewall grommet > gascolator > carburettor", "fuel", "#ff5a4f",
      [[-560, 120, -560], [-600, 120, -620], [-650, 120, -700], [-700, 90, -760], [-850, 60, -760], [-1000, 0, -680]])
route("Fuel tank vent (out under the hood)", "fuel", "#ff9b8f", [[-520, -280, -372], [-650, -280, -372], [-760, -280, -420]])
route("Pitot line: left-wing pitot > wing root > A-pillar > ASI", "pitot", "#f4f6f8",
      [wp("L", [150, 1200, -30]), wp("L", [250, 1200, -10]), wp("L", [250, 30, 0]), [250, -330, -80], [-380, -330, -420], [-430, -200, -505]], step=17)
route("Static line: static port on the cabin side > altimeter + ASI", "static", "#9aa3ad",
      [[1100, -352, -700], [900, -330, -700], [-380, -320, -600], [-430, -105, -505]], step=17)
route("12 V: battery (under seat) > master switch > fuses > radio", "wire", "#ffd23f",
      [[450, 150, -1000], [450, 320, -990], [-380, 320, -620], [-430, 255, -455], [-430, 175, -515]], step=17)
route("Mag P-lead: panel mag switch > firewall grommet > magneto", "wire", "#ffd23f",
      [[-430, 255, -505], [-560, 300, -600], [-650, 300, -640], [-900, 150, -600], [-990, 60, -560]], step=17)
route("Antenna coax: radio > cabin top > tailcone antenna", "coax", "#e879f9",
      [[-430, 175, -515], [-380, 300, -380], [1100, 300, -90], [1600, 0, float(tc_pt(1600, SEC[0])[2]) - 10]], step=17)
prim("pilot", 0, "Pilot (95th pct, for scale)", hip=[390, 0, -880], col="ghost")

# ---------------------------------------------------------------- hardware with positions
HW = []  # dict(code, desc, step, where, pos:[...], qty)
def hw(code, desc, step, where, pts, extra_qty=0, show=True, wing=False, ax=(0, 0, 1)):
    ax = np.asarray(ax, float)
    axes = [list(map(float, ax / np.linalg.norm(ax)))] * len(pts) if ax.ndim == 1 else [list(map(float, a_ / np.linalg.norm(a_))) for a_ in ax]
    HW.append(dict(code=code, desc=desc, step=step, where=where, pos=[[round(v, 1) for v in p] for p in pts] if show else [],
                   ax=[[round(v, 3) for v in a_] for a_ in axes] if show else [], qty=len(pts) + extra_qty, wing=wing))

# clevis web bolts and tang bolts (step 12)
cv_bolts = [q for (q, d, k) in cv.bolts3d if k == "bolt"]
hw("AN4", "AN4 bolt (1/4-28), AN365-428 nut, 2 x AN960-416 washers", 12, "Clevis webs to arch tops and arch feet, 2 per clevis",
   [inst_pt(M, q) for (M, lab) in cv.instances if "strut top" not in lab and "jury top" not in lab and "bracket" not in lab for q in cv_bolts],
   ax=np.array([M[:3, 2] for (M, lab) in cv.instances if "strut top" not in lab and "jury top" not in lab and "bracket" not in lab for q in cv_bolts]))
hw("AN4", "AN4 bolt (1/4-28), AN365-428 nut, 2 x AN960-416 washers", 12, "Strut-top and jury-top clevises through the spar caps, 2 each",
   [inst_pt(M, q) for (M, lab) in cv.instances if ("strut top" in lab or "jury top" in lab) for q in cv_bolts], wing=True,
   ax=np.array([M[:3, 2] for (M, lab) in cv.instances if ("strut top" in lab or "jury top" in lab) for q in cv_bolts]))
tg = by_id["K-TG"]
tg_b = [q for (q, d, k) in tg.bolts3d if k == "bolt"]
hw("AN3", "AN3 bolt (10-32), AN365-1032 nut, AN960-10 washers", 12, "Through tang pair + spar web at each spar root, 4 per pair",
   [inst_pt(M, q) for i, (M, lab) in enumerate(tg.instances) if i % 2 == 0 for q in tg_b], wing=True, ax=(1, 0, 0))
# gear (13)
hw("AN5", "AN5 bolt (5/16-24), AN365-524 nut, AN970-5 large washers", 13, "Main gear legs through floor doubler, 2 per leg",
   [[a[0] + dx, a[1], a[2]] for (a, b) in legs for dx in (-30, 30)])
hw("AXLE", "Axle bolt AN6 + castellated nut AN310-6 + cotter MS24665", 13, "Main wheels", [b for (a, b) in legs], ax=(0, 1, 0))
hw("AN4", "AN4 bolt, AN365-428 nut", 13, "Tailwheel spring to tail post, 2", [[TW["a"][0] + dx, 0, TW["a"][2]] for dx in (0, 40)])
# engine (14)
hw("AN4", "AN4 bolt, AN365-428 nut, AN970-4 washer each side", 14, "Engine mount to firewall, 4", [a for (a, b) in mount], ax=(1, 0, 0))
hw("AN5", "AN5 bolt + rubber isolator (Lord-type) + AN365-524", 14, "Engine to mount, 4", [b for (a, b) in mount], ax=(1, 0, 0))
hw("AN5", "AN5 prop bolt, AN310-5 + safety wire (per prop maker)", 14, "Propeller hub, 6",
   [[-1325, 50 * math.cos(a), -550 + 50 * math.sin(a)] for a in np.linspace(0, 2 * math.pi, 6, endpoint=False)], ax=(1, 0, 0))
from parts import COWL_SEC, cowl_pt
cam = []
for k in range(len(COWL_SEC)):
    a_, b_ = COWL_SEC[k], COWL_SEC[(k + 1) % len(COWL_SEC)]
    n_ = 3 if k in (0, 1, 5) else 2
    for t_ in [(i + 0.5) / n_ for i in range(n_)]:
        cam.append([P["x_fw"] - 12, a_[0] + (b_[0] - a_[0]) * t_, a_[1] + (b_[1] - a_[1]) * t_])
for sgn in (1, -1):
    for x in (-800, -1020):
        cam.append([x, sgn * 351, -660])
hw("DZUS", "Quarter-turn camlock (Dzus) + receptacle", 14, "Cowl to firewall flange and along the split line", cam,
   ax=np.array([[0, 1, 0] if c_[0] < P["x_fw"] - 50 else ([1, 0, 0]) for c_ in cam], float))
# wings on (15)
hw("PIN5", "AN5 clevis pin (bolt), AN310-5 castellated nut, MS24665 cotter", 15, "Spar-root pins through tang pair + arch clevis", ax=(1, 0, 0), pts=
   [pins[f"{s} root clevis x{x:.0f}"] for s in "RL" for x in (P["x_ms"], P["x_rs"])])
# struts (16)
hw("PIN5", "AN5 bolt, AN310-5 castellated nut, MS24665 cotter", 16, "Strut ends, both ends of 4 struts", ax=(1, 0, 0), pts=
   [p for d in strut_defs for p in (d[2], d[3])])
hw("AN3", "AN3 bolt, AN365-1032 nut", 16, "Saddle-clamp ears (one also carries the jury-strut fork), 2 per clamp",
   [inst_pt(M, q) for i, (M, lab) in enumerate(by_id["K-JC"].instances) if i % 2 == 0 for (q, d, k) in by_id["K-JC"].bolts3d],
   ax=np.array([M[:3, 2] for i, (M, lab) in enumerate(by_id["K-JC"].instances) if i % 2 == 0 for (q, d, k) in by_id["K-JC"].bolts3d]))
hw("AN3", "AN3 bolt, AN365-1032 nut", 16, "Jury strut top, through the jury clevis", [d[5] for d in strut_defs], ax=(1, 0, 0))
route("Aux fuel feed: aux tank > along the left roof rail > selector valve", "fuel", "#ff5a4f",
      [[865, 0, -405], [760, 0, -420], [600, -300, -330], [-380, -300, -330], [-560, 90, -560], [-600, 90, -600]])
route("Aux tank vent (overboard under the roof)", "fuel", "#ff9b8f", [[865, 220, -200], [865, 330, -150], [960, 345, -120]])
# tail (11)
tail_pts = [[x, s * 112, P["z_stab"]] for s in (1, -1) for x in (3870, 4000, 4150, 4300)]
hw("AN4", "AN4 bolt, AN365-428 nut", 11, "Stabiliser root ribs to tailcone side doublers, 4 per side", tail_pts, ax=(0, 1, 0))
hw("AN4", "AN4 bolt, AN365-428 nut", 11, "Fin spar to tail post, 3", [[4340, 0, z] for z in (-500, -440, -380)], ax=(0, 1, 0))
# controls (17)
_hn = by_id["K-HN"]; _bc = by_id["K-BC"]
horn = [inst_pt(M, _hn.bolts3d[0][0]) for (M, lab) in _hn.instances]
hw("AN3", "AN3 bolt, AN365-1032 nut, through 3/16 in rod end", 17, "Aileron and flap horns (K-HN), 1 per surface", horn, ax=(0, 1, 0), wing=True)
for WG in (False, True):
  _piv, _arm, _pax, _aax = [], [], [], []
  for k in range(0, len(_bc.instances), 2):
    M, lab = _bc.instances[k]; M2 = _bc.instances[k + 1][0]
    if ("wing" in lab) != WG: continue
    c = (np.asarray(M)[:3, 3] + np.asarray(M2)[:3, 3]) / 2
    Mm = np.array(M, float); Mm[:3, 3] = c
    for (q, d, kind) in _bc.bolts3d:
        (_piv if kind == "pivot" else _arm).append(inst_pt(Mm, q))
        (_pax if kind == "pivot" else _aax).append(np.asarray(M)[:3, 2])
  wh = "in the wings" if WG else "in the cabin"
  hw("AN4", "AN4 bolt, AN310-4 castle nut + cotter", 17, f"Bellcrank pivots {wh} (K-BC pair through its K-CV bracket)", _piv, ax=np.array(_pax), wing=WG)
  hw("AN3", "AN3 bolt, AN365-1032 nut, rod end between the plates", 17, f"Bellcrank arms {wh}, 2 per bellcrank", _arm, ax=np.array(_aax), wing=WG)
  _bk = [(M, lab) for (M, lab) in cv.instances if "bracket" in lab and ("wing" in lab) == WG]
  hw("AN4", "AN4 bolt (1/4-28), AN365-428 nut, 2 x AN960-416 washers", 17, f"Bellcrank brackets {wh} to arch / spar, 2 each",
     [inst_pt(M, q) for (M, lab) in _bk for q in cv_bolts], ax=np.array([M[:3, 2] for (M, lab) in _bk for q in cv_bolts]).reshape(-1, 3), wing=WG)
hw("AN4", "AN4 bolt, AN310-4 + cotter (pivot)", 17, "Stick pivot and torque-tube bearings", [[200, 0, -1040], [200, 0, -1000], [900, 0, -1030]], ax=(0, 1, 0))
_tb = [[700, 200, -1035], [700, -200, -1035], (np.array([520, 30, -1000]) + (np.array([4330, 30, -445]) - [520, 30, -1000]) * 0.08).tolist(), (np.array([520, -30, -1040]) + (np.array([4330, -30, -505]) - [520, -30, -1040]) * 0.08).tolist()]
hw("TB", "Turnbuckle AN130 class + safety wire", 17, "Rudder and elevator cables, 1 per cable, in the cabin where you can inspect them", _tb, ax=(1, 0, 0))
hw("NICO", "Nicopress sleeve + thimble (1/8 in cable)", 17, "Each cable end", [], extra_qty=8)
hw("AN5", "AN5 bolt, AN365-524 nut", 9, "Harness anchors to floor and rear arch, 4", [[420, s * 280, -1040] for s in (1, -1)] + [[1000, s * 200, -300] for s in (1, -1)])
# glazing (18)
scr = []
for pid in ("F-DW", "F-FWN"):
    p = by_id[pid]
    for (M, lab) in p.instances:
        scr += [inst_pt(M, q) for (q, d, k) in p.bolts3d]
ws = by_id["F-WS"].faces[0]["pts"]
for i in range(4):
    a, b = np.array(ws[i]), np.array(ws[(i + 1) % 4])
    L = np.linalg.norm(b - a); n = int(L // 90)
    c = np.mean(ws, axis=0)
    for k in range(n):
        q = a + (b - a) * (k + 0.5) / n
        q = q + (c - q) / np.linalg.norm(c - q) * 12
        scr.append(q.tolist())
from geom import newell as _nw
_wsn = _nw(ws)
_n_side = len(scr) - sum(int(np.linalg.norm(np.array(ws[(i + 1) % 4]) - np.array(ws[i])) // 90) for i in range(4))
hw("M4", "M4 pan screw, nylon washer, M4 Nyloc nut, 6 mm oversize hole in polycarbonate", 18, "Windscreen, door and side windows", scr,
   ax=np.array([[0, 1, 0]] * _n_side + [list(_wsn)] * (len(scr) - _n_side), float))
hw("HINGE", "Piano hinge MS20001 + door latch", 18, "Left door front edge", [], extra_qty=1)

hw_total = sum(h["qty"] for h in HW)

# ---------------------------------------------------------------- tools
TOOLS = {
    "brake": "1 m hand bending brake (or hardwood bending blocks in a vise)",
    "drill": "Cordless drill, #40 (2.5 mm) and #30 (3.3 mm) bits",
    "cleco": "1/8 in Clecos (300) + Cleco pliers",
    "puller": "Blind-rivet puller (hand or pneumatic)",
    "deburr": "Deburring tool, 10 mm drill bit (hole deburr), fine file",
    "mark": "Fine felt marker, steel rule, square, MEK to clean",
    "snips": "Aviation snips (straight, left, right)",
    "roll": "75 mm tube + ratchet strap (nose roller)",
    "level": "String line, digital level, plumb bob",
    "spanner": "Spanners, sockets, inch-pound torque wrench",
    "safety": "Cotter-pin and safety-wire pliers",
    "tube": "Tube cutter, hacksaw, drill press",
    "ream": "Drill press, 1/4 in and 5/16 in reamers",
    "prime": "Scotch-Brite, etch/alodine, epoxy primer, spray kit, gloves",
    "nico": "Nicopress tool + cable cutter",
    "plastic": "Plastic-cutting drill bits / step drill",
    "scales": "Three platform scales",
    "sand": "Sandbags and tape measure (proof load)",
    "dup": "1/8 in hole duplicator",
    "vise": "Bench vise with soft jaws",
}

# ---------------------------------------------------------------- steps
riv_by_step = {}
for p in parts:
    for r in riv_info[p.id]:
        riv_by_step[r["step"]] = riv_by_step.get(r["step"], 0) + len(p.instances)
grips = {}
for p in parts:
    for r in riv_info[p.id]:
        k = ("firewall" if r["firewall"] else f"grip {r['grip']}", r["step"])
        grips[k] = grips.get(k, 0) + len(p.instances)


def rivet_hw(step):
    out = []
    for (kind, st), n in sorted(grips.items()):
        if st != step:
            continue
        if kind == "firewall":
            out.append(dict(code="A4-SS", desc="1/8 in stainless/Monel blind rivet (fire-zone)", qty=n, where="Firewall flanges"))
        else:
            g = kind.split()[-1]
            out.append(dict(code=f"A4-{g}", desc=f"1/8 in domed aluminium blind rivet, grip -{g} ({'up to 1.6' if g == '1' else '1.6-3.2'} mm)", qty=n, where=""))
    return out


S = lambda *a: list(a)
STEPS = [
    dict(n=0, title="Primer: build the wing stands", jp=("", "practice piece"), fold="Lipped channels: four 90° brake bends each, two nested layers", parts=["S-BM", "S-LG", "S-FT", "S-GT", "S-GF"],
         tools=["snips", "deburr", "mark", "brake", "drill", "cleco", "puller"],
         sub=S("Why start here: the four stands use every skill the aircraft needs, on parts that don't fly. Get it wrong here, not on a spar. You need the stands anyway: the wings are built and parked on them until step 15.",
               "Cut and deburr: the stand blanks are on the same nest sheets as everything else. File every edge smooth and deburr each pilot hole on both faces. Look for burrs with your fingertip, not your eye.",
               "Mark the folds: red dashed is a valley, blue dash-dot is a mountain. Felt marker on the marked face only, never a scribe.",
               "Bend on the brake: each channel is four 90° bends. Bend the two flanges first, then the two lips. Set the nose radius to 1 mm (about 2t for 0.020 in) and overbend about 2° for springback. Check every bend with a square.",
               "Laminate: each channel is two blanks. Fold both separately, nest the inner one inside the outer one, and check they sit tight with no gap. If the inner one fights, take 0.5 mm off its flange edges.",
               "Drill and cleco: clamp the gussets in place, drill through the pilot holes to #30 (3.3 mm), and put a cleco in every fourth hole as you go. Then take everything apart, deburr the holes and the faying surfaces, and put it back together on clecos.",
               "Rivet: pull 1/8 in blind rivets, working from the middle of each row outwards and swapping clecos for rivets one at a time. A good rivet sits flat, with no gap under the head. Drill out any bad one with a #30 bit and try again: practise this on purpose at least once.",
               "Finish: glue carpet on top of each beam so the wing skin can't be scratched. Load each stand with 40 kg and check it doesn't rack or rock. Keep the offcuts: they are your test pieces for the spar bend in step 2."),
         text="Four stands, 20 blanks, about 200 rivets. The cheapest place to learn to fold, drill and rivet is a part that never leaves the ground."),
    dict(n=1, title="Cut, deburr, mark", jp=("", "crease pattern"), fold="", parts=[],
         tools=["snips", "deburr", "mark"],
         sub=S("Send the nest DXFs to a laser or waterjet service, or cut on a CNC router. Bolt holes come out 0.5 mm undersize; ream them later.",
               "Deburr every edge with a file or deburring tool. A burr is where a fatigue crack starts.",
               "Deburr every pilot hole on both faces with a 10 mm bit turned by hand.",
               "Mark fold lines on the marked face: red dashed = valley, blue dash-dot = mountain. Use a fine felt marker, never a scribe (Wanttaja: scribe lines start cracks).",
               "Write the part number on every blank. Keep the blanks in step order."),
         text="Every crease goes on the blank before anything is folded, the same order the origami book uses."),
    dict(n=2, title="Fold the spars", jp=("", "mountain and valley folds"), fold="Four brake bends each", parts=["W-MS", "W-RS"],
         tools=["brake", "vise", "mark"],
         sub=S("Set the brake nose radius to 0.8 mm, about 1.6 times the 0.51 mm sheet (Table 5-2, 'normal' radius). Never tighter.",
               "Each spar is a laminate: three nested blanks for the main spar, two for the rear. Fold every layer separately, then nest them.",
               "Fold the two caps first (90°), then the two lips. Overbend by about 2° so the 6061-T6 springs back to 90°.",
               "Check the cap angle every 500 mm with a square. Sight along the spar for twist.",
               "Fold a 300 mm offcut of the same strip first and measure it to confirm the bend deduction."),
         text="The lips are what let thin laminated sheet act as a spar. Without them a free cap edge buckles at about 26 MPa. With them the cap reaches yield before it buckles."),
    dict(n=3, title="Fold the ribs, Cleco them on", jp=("", "insert: the lock"), fold="Flanges 90°, all toward the root", parts=["W-NR", "W-MR", "W-NRe", "W-MRe"],
         tools=["brake", "cleco", "mark"],
         sub=S("Fold every rib flange 90° on the brake. Because the airfoil is faceted, each flange is one straight bend.",
               "Fold all 52 ribs before assembling any: one brake set-up does the whole batch.",
               "Lay the spars on a flat table, webs vertical, 3.6 m apart for the two wings.",
               "Fit each rib by its attach flange: Cleco through the pilot holes into the spar web. Nose ribs go on the front face, main ribs on the back.",
               "Root and tip ribs have their flanges facing into the panel."),
         text="All 52 ribs are the same two blanks. The pilot holes locate them, so no jig is needed."),
    dict(n=4, title="Drill, deburr, prime, rivet the skeleton", jp=("", "check, then commit"), fold="", parts=[],
         tools=["level", "drill", "cleco", "deburr", "prime", "puller"],
         sub=S("String-line the spars and check the ribs are square to them. Measure diagonals across each bay; they should match within 1 mm.",
               "Final-drill every Cleco'd hole to #30 (3.3 mm), a few at a time, moving each Cleco to the next hole as you go.",
               "Take the skeleton apart. Deburr every hole on both faces.",
               "Prime the faying surfaces: Scotch-Brite, wash, etch or alodine, then epoxy primer. Wear gloves (Wanttaja: finger oils stop primer sticking).",
               "Reassemble on Clecos, then pull the rivets from the middle of each joint outward.",
               "Inspect each rivet: head seated flat, mandrel broken off flush, no gap under the head."),
         text="This is the sequence for every riveted joint from here on: Cleco, final-drill, take apart, deburr, prime, reassemble, rivet. Later steps call it the riveting loop."),
    dict(n=5, title="Lower skins", jp=("", "make soft creases"), fold="Soft creases 2–6°", parts=["W-SL1", "W-SL2", "W-SL3"],
         tools=["brake", "dup", "drill", "cleco", "deburr", "prime", "puller"],
         sub=S("Put in the soft creases with a light brake hit, or by hand over a straight-edge.",
               "Turn the skeleton upside down. Lay bay 1's skin on it, located at the root rib, with the edge 3 mm short of the rear-spar flange edge.",
               "Back-drill #40 through the rib and spar-cap pilot holes, using the hole duplicator where you can't reach from inside. Cleco as you go.",
               "Skin joints lap on a rib: the outboard skin goes under the inboard one.",
               "Riveting loop: final-drill #30, take apart, deburr, prime, Cleco, rivet."),
         text="The skins carry no holes of their own. Drilling them from the structure means the holes can't be misaligned."),
    dict(n=6, title="Upper skins: roll the nose", jp=("", "give it roundness"), fold="Nose rolled over 75 mm tube", parts=["W-SU1", "W-SU2", "W-SU3"],
         tools=["roll", "brake", "dup", "drill", "cleco", "deburr", "prime", "puller"],
         sub=S("Put in the aft soft creases first.",
               "Roll the green zone: lay the skin over a 75 mm tube and pull it round with a strap, a little at a time. Don't crease it.",
               "Turn the wing upright. Wrap the skin from the trailing-edge shroud, over the nose, and under to lap the lower skin at 8 % chord.",
               "Back-drill through the rib flanges and spar caps. Do the nose last, working from the spar forward.",
               "Riveting loop, rivets from the main spar outward."),
         text="The rolled nose and the main spar together form a closed D-cell, which carries the wing's twisting loads."),
    dict(n=7, title="Flaps and ailerons", jp=("", "pleat"), fold="V-bead pleats + one 165° trailing-edge fold", parts=["W-CSS", "W-CS", "K-HN"],
         tools=["brake", "vise", "drill", "cleco", "puller", "spanner"],
         sub=S("Fold the two V-bead pleats in valley-mountain-valley order. Stop each bead at the marks.",
               "Fold the trailing edge over a 0.6 mm radius bar in the brake, to about 165°. Squeeze gently in soft vise jaws until the TE is straight.",
               "Slide the C-spar in and Cleco it through the flanges. Riveting loop.",
               "Rivet the piano hinge to the spar top, then hang the surface on the rear spar.",
               "Mass-balance each aileron with lead on an arm ahead of the hinge until it hangs level, before it goes on the aircraft."),
         text="Four identical surfaces from one blank. The trailing edge is a single crease, so there's no separate strip to rivet on."),
    dict(n=8, title="Floor, firewall, arches", jp=("", "reference points"), fold="Hat pleats 56°, flanges 90°", parts=["F-FL", "F-FW", "F-FA", "F-RA"],
         tools=["brake", "drill", "cleco", "deburr", "prime", "puller", "level"],
         sub=S("Fold the floor tray's two hat pleats, then the chamfers and side flanges.",
               "Fold the firewall flanges, all aft. It is stainless, so drill it slowly with a sharp bit and cutting oil.",
               "Fold the two arches: outer and inner flanges, all aft.",
               "Stand the arches and firewall on the floor tray, square and plumb. Their stations are 420 mm and 1008 mm aft of the wing LE, and the firewall is at -650 mm.",
               "Riveting loop. The firewall takes stainless or Monel rivets: aluminium melts in a fire."),
         text="The arches sit at the spar stations, so the wing and strut loads go straight into two folded frames."),
    dict(n=9, title="Cabin sides and seat", jp=("", "right angle"), fold="Top flange = longeron", parts=["F-SD", "F-ST"],
         tools=["brake", "dup", "drill", "cleco", "deburr", "prime", "puller", "spanner"],
         sub=S("Fold the 90° top flange and the windscreen flange on each side panel. The top flange is the longeron.",
               "Fit the sides outside the arches, firewall flange and floor flanges. Back-drill and Cleco.",
               "Riveting loop.",
               "Fold the seat: pan and back from one sheet. Rivet its front lip to the floor and lean its back on the rear arch.",
               "Bolt in the four harness anchors."),
         text="Both sides are the same blank, mirrored. The openings have 50 mm corner radii because square corners start cracks."),
    dict(n=10, title="Tailcone frustum", jp=("", "same angle: converging creases"), fold="Converging creases 44°, bulkhead flanges 96–100°", parts=["F-TB1", "F-TS1", "F-TT1", "F-TB2", "F-TS2", "F-TT2", "F-B1", "F-B2", "F-B3", "F-B4", "F-B5"],
         tools=["brake", "drill", "cleco", "deburr", "prime", "puller", "level"],
         sub=S("Fold the bulkhead flanges. They are 96–100°, not 90°, because they follow the taper.",
               "Fold each belly blank: bottom, both chamfers, then the side flanges.",
               "Build each section upside down on a straight edge: bulkheads Clecoed into the belly, then the sides, then the top.",
               "Check twist by sighting a level across bulkheads 1 and 5. Then join the two sections at bulkhead 3.",
               "Riveting loop, then join the tailcone to the cabin at bulkhead 1."),
         text="All six faces aim at one vanishing point, so each face is flat and every crease is straight."),
    dict(n=11, title="Tail surfaces", jp=("", "fold the same way"), fold="One-wrap skins, sharp trailing edges", parts=["T-HS", "T-HR", "T-HK", "T-ES", "T-EK", "T-FS", "T-FR", "T-FK", "T-RS", "T-RK", "K-HD", "K-HR"],
         tools=["brake", "roll", "drill", "cleco", "deburr", "prime", "puller", "spanner", "level"],
         sub=S("Fold the stabiliser and fin spars and their nose ribs.",
               "Roll each skin's leading edge, then wrap it round its ribs and spar. Back-drill and use the riveting loop.",
               "Elevators and rudder: fold them the same way as the ailerons in step 7 (pleats, then the trailing-edge fold, then the C-spar).",
               "Bolt the stabiliser halves to the tailcone side doublers, 4 x AN4 each. Bolt the fin spar to the tail post, 3 x AN4.",
               "Hang the elevators and rudder on piano hinges. Check ±25° rudder travel clears the elevator halves."),
         text="The stabiliser and fin use one wrapped skin each."),
    dict(n=12, title="4130 fittings", jp=("", "turn it over"), fold="Clevis: two 90° bends at 3t radius", parts=["K-CV", "K-TG"],
         tools=["vise", "brake", "ream", "spanner", "prime"],
         sub=S("Fold the 12 clevises in the vise over a 5 mm radius block. Normalised 4130 needs at least 3t (Wanttaja).",
               "Ream the pin holes to 5/16 in and the bolt holes to 1/4 in and 3/16 in, through both ears at once so they line up.",
               "Clean and prime. Steel fittings rust under paint if they aren't primed.",
               "Bolt the four root clevises to the arch tops and the four strut-foot clevises to the arch feet with AN4 bolts.",
               "Bolt the tang pairs either side of each spar web at the root (4 x AN3), and the strut-top clevises under the spars at the 2.4 m rib."),
         text="Every steel fitting is also a flat blank folded at most twice."),
    dict(n=13, title="Landing gear and tailwheel", jp=("", "turn it over"), fold="", parts=[],
         tools=["spanner", "safety", "level", "ream"],
         sub=S("Turn the fuselage upside down on padded trestles.",
               "Bolt each main-gear spring leg through the floor doubler under the front arch: 2 x AN5 each, large washers both sides.",
               "Fit the axles, wheels and drum brakes. Castellated nut and cotter on each axle. The 21 x 8.00-6 tundra tyres run at 10-12 psi: check pressure before every flight, they are soft by design.",
               "Bolt the tailwheel spring to the tail post (2 x AN4). Fit the fork and the two steering springs to the rudder horn.",
               "Turn it back over. On its three wheels it should sit about 9–11° nose-up."),
         text="Tailwheel: a leaf spring, a steerable fork and two springs to the rudder."),
    dict(n=14, title="Engine, cowl, prop", jp=("", "fold to the marks"), fold="Box-pleat hood 45°, nose bowl converging creases", parts=["F-CU", "F-CL", "F-NU", "F-NL", "F-NF"],
         tools=["spanner", "safety", "brake", "drill", "cleco", "puller", "ream"],
         sub=S("Bolt the 4130 engine mount through the firewall (4 x AN4) with large washers on the back.",
               "Hang the half-VW on rubber isolators (4 x AN5).",
               "Fuel, header tank: the 19 L rotomoulded tank sits behind the firewall, above the pilot's shins and under the windscreen, strapped to two angles riveted to the side panels (as on a J-3 Cub). It gravity-feeds: tank outlet, selector valve (HEADER / AUX / OFF), a fire-sleeved hose through a firewall grommet, the gascolator on the firewall, then the carburettor. The tank bottom is about 200 mm above the carburettor. The filler neck comes up through the hood just ahead of the windscreen, and the vent runs forward under the hood.",
               "Fuel, aux tank: a second 20 L tank strap-mounts high behind the seat, its bottom above the header tank top so it also gravity-feeds. Its line runs forward along the left roof rail to the selector; filler through the left cabin side, vent overboard under the roof. Never in the wings. Fill it only as far as the 300 kg MTOW allows: with a 100 kg pilot that is none at all. Burn the aux tank first, which keeps the CG moving forward.",
               "Fold the upper and lower cowls (four 45° creases each) and the nose-bowl halves. Rivet the nose bowls to the nose ring.",
               "Fit the camlock receptacles on the firewall flange and along the split line. Trim the cowl until it clears the cylinders by at least 10 mm.",
               "Bolt on the propeller and spinner. Torque to the prop maker's figure, then safety-wire."),
         text="The cowl and nose bowl are flat blanks too: a box-pleated hood and a frustum, with straight creases throughout."),
    dict(n=15, title="Wings on", jp=("", "open out"), fold="", parts=["W-CL", "W-CU"],
         tools=["spanner", "safety", "level"],
         sub=S("With a helper at the tip, lift each wing until its root tangs slide into the arch-top clevises.",
               "Push the AN5 root pins through. Fit castellated nuts and cotter pins.",
               "Support each tip on a trestle at 1.5° dihedral until the struts are on.",
               "Fit the cabin roof and the centre-section upper skin, back-drilled through the root ribs. Use the riveting loop."),
         text="Each wing pins on at two points. The struts set its angle."),
    dict(n=16, title="Lift struts and jury struts", jp=("", "same length"), fold="Saddle clamps: rolled half-shells", parts=["K-JC"],
         tools=["tube", "ream", "spanner", "safety", "level"],
         sub=S(f"Cut the main struts to {L_main} mm and the rear struts to {L_rear} mm pin-to-pin, minus the end-fitting length. All four are 6061-T6 1.75 x 0.065 in tube.",
               "Press or bolt a 4130 end fork into each tube end. Drill the AN5 pin hole after the final length check on the aircraft.",
               "Set dihedral to 1.5° and wash-out to 0° with a digital level on the root and tip ribs. Then pin the struts on: AN5, castellated nut, cotter pin.",
               "Roll the eight saddle-clamp halves round a 45 mm bar, then bolt a pair round each strut at mid-length (2 x AN3 through the ears).",
               f"Fit the jury struts ({L_jury} mm main, {L_juryr} mm rear) from the clamp ear to the jury-top clevis under the spar, AN3 at each end. They double the strut's buckling strength.",
               "Clip on the strut fairings."),
         text="Under negative g the struts go into compression. The jury struts halve their unsupported length, which multiplies their buckling load by four."),
    dict(n=17, title="Controls, instruments and systems", jp=("", "same angle"), fold="Panel: glare-shield fold 90°, side tabs 90°", parts=["F-IP", "K-BC", "W-CT"],
         tools=["spanner", "safety", "nico", "level", "tube", "drill", "puller"],
         sub=S("Fit the stick, the aileron torque tube under the seat and the elevator bellcrank. Connect the stick to the bellcrank with a short pushrod.",
               "Elevator cables: run one straight line each from the bellcrank to the tail-post grommet, 60 mm apart, up cable above down cable. Fit a phenolic fairlead where each line passes a bulkhead.",
               "Rudder cables: pedal horn, then a pulley at the pedal, along the floor at y = ±200 mm through fairleads on both arch feet, a pulley at bulkhead 1, then one straight line each to the tail post. Because the tailcone is a frustum, a straight run between interior points never touches the skin, so no cable needs a mid-cone pulley.",
               "Tension each cable with its turnbuckle in the cabin, where you can see it, then safety-wire it. Nicopress every end over a thimble.",
               "Bellcranks: each is a pair of K-BC plates, 16 mm apart, on an AN4 pivot through a folded K-CV bracket. The rod ends sit between the plates on AN3 bolts. Five per side: aileron arch-foot, arch-top and outboard (at the rear spar); flap torque-tube arm and flap bellcrank.",
               "Ailerons: a short link from the torque-tube arm to the arch-foot bellcrank, a pushrod up the rear-arch leg to the arch-top bellcrank, then out through the rib lightening holes to the outboard bellcrank, which drives the K-HN horn on the aileron. Flaps: overhead lever, a short rod to the centre-section torque-tube arms, then pushrods to the flap bellcranks and K-HN horns.",
               "Close the bay behind the centre-section rear spar with the W-CT trailing-edge fairing (same wedge fold as the flaps). Do it last, after the flap linkage is rigged and checked.",
               "Set the travels: aileron +20/-15°, elevator +25/-20°, rudder ±25°, flaps 0/20°. Fit the stops.",
               "Fold the instrument panel (glare shield and side tabs) and rivet it to the cabin sides at x = -400 mm. Fit the ASI, altimeter, tach, CHT/EGT, radio, master and mag switches and fuel valve, and put the compass on the glare shield.",
               "Pitot-static: pitot tube under the left wing at the 1.2 m rib. Run its line inside the wing ahead of the main spar to the root and down the A-pillar to the ASI. A static port on the left cabin side feeds the altimeter and ASI. Secure everything with Adel clamps every 300 mm.",
               "Electrics: 12 V LiFePO4 battery in a vented box under the seat. Run it via the master switch and fuses to the radio, with the antenna coax to a whip on the tailcone top. The mag P-lead goes through a firewall grommet. The half-VW's magneto needs no battery to run."),
         text="With X-ray on, every run is colour-coded: orange rudder, cyan elevator, purple aileron, green flap, red fuel, white pitot, grey static, yellow 12 V, pink coax."),
    dict(n=18, title="Windscreen, side windows, door", jp=("", "close up"), fold="", parts=["F-WS", "F-DW", "F-FWN"],
         tools=["plastic", "spanner", "mark"],
         sub=S("Leave the protective film on. Drill the polycarbonate with plastic bits, holes 6 mm for M4 screws so it can expand.",
               "Windscreen: flat sheet, bedded on foam tape on the firewall and windscreen flanges.",
               "Front side windows: screw over the cut-outs with a 20 mm lap.",
               "Right side: fixed window over the door opening. Left side: the same blank on a piano hinge at its front edge, with a latch at the back.",
               "Don't overtighten: nylon washers, and stop when the washer just grips."),
         text="All the glazing is flat sheet: no forming and no moulds."),
    dict(n=19, title="Weigh, test, fly-off", jp=("", "finished"), fold="", parts=[],
         tools=["scales", "sand", "level", "spanner"],
         sub=S("Paint, then weigh on three scales in the level attitude. Work out the empty CG and check every loading case falls inside the design range in DESIGN_REPORT.md (about 28–33 % MAC).",
               "Proof-load the wing upside down with sandbags to +4 g limit load, spread spanwise. It must show no permanent set.",
               "Engine runs: full-power static run, mag check, then check cylinder head temperatures.",
               "Taxi tests, then fast taxi with the tail up.",
               "Have a BMAA or LAA inspector look at it before the first flight. Do first flights in calm air, over a long field."),
         text="SSDR means no airworthiness approval is required, so all the testing is yours to do."),
]
for s in STEPS:
    s["rivets"] = riv_by_step.get(s["n"], 0)
    s["blanks"] = sum(len(by_id[i].instances) * getattr(by_id[i], "layers", 1) for i in s["parts"])
    s["hw"] = rivet_hw(s["n"]) + [dict(code=h["code"], desc=h["desc"], qty=h["qty"], where=h["where"]) for h in HW if h["step"] == s["n"]]
    s["hwcount"] = sum(h["qty"] for h in HW if h["step"] == s["n"])
    # merge identical hardware lines within a step
    mg = {}
    for h in s["hw"]:
        k = (h["code"], h["desc"])
        if k in mg:
            mg[k]["qty"] += h["qty"]; mg[k]["where"] = (mg[k]["where"] + "; " + h["where"]).strip("; ")
        else:
            mg[k] = dict(h)
    s["hw"] = list(mg.values())
def RT(prefix):
    return next(r for r in ROUTES if r["name"].startswith(prefix))
def plen(pts):
    return sum(np.linalg.norm(np.subtract(pts[i + 1], pts[i])) for i in range(len(pts) - 1))
def seg(pts, i):
    return np.linalg.norm(np.subtract(pts[i], pts[i - 1]))
DRAW = {
    13: [dict(kind="bar", label="Main gear leg x2: 6061-T6 bar 75 x 16 mm", len=round(np.linalg.norm(np.subtract(legs[0][1], legs[0][0]))) + 120, note="2 x AN5 holes at the top, axle boss at the bottom"),
         dict(kind="bar", label="Tailwheel spring: spring-steel leaf 45 x 8 mm", len=round(np.linalg.norm(np.subtract(TW["b"], TW["a"]))) + 60, note="2 x AN4 at the tail post")],
    16: [dict(kind="tube", label="Main lift strut x2: 6061-T6 1.75 x 0.065 in", len=L_main, note="pin-to-pin; AN5 each end"),
         dict(kind="tube", label="Rear lift strut x2: 6061-T6 1.75 x 0.065 in", len=L_rear, note="pin-to-pin; AN5 each end"),
         dict(kind="tube", label="Jury strut x2 (main): 6061-T6 0.75 x 0.049 in", len=L_jury, note="AN3 each end"),
         dict(kind="tube", label="Jury strut x2 (rear): 6061-T6 0.75 x 0.049 in", len=L_juryr, note="AN3 each end")],
    17: [dict(kind="cable", label="Rudder cable x2: 1/8 in 7x19 galvanised", len=round(plen(RT("Rudder cable R")["pts"])) + 200, note="via floor pulley + 3 fairleads; Nicopress + thimble each end"),
         dict(kind="cable", label="Elevator cable x2: 1/8 in 7x19 galvanised", len=round(plen(RT("Elevator cable (up)")["pts"])) + 200, note="via pulley at bulkhead 1 + 3 fairleads; turnbuckle"),
         dict(kind="tube", label="Aileron pushrod (arch) x2: 6061-T6 3/8 x 0.035 in", len=round(seg(RT("Aileron pushrods R")["pts"], 3)), note="arch-foot bellcrank up the rear-arch leg to the arch-top bellcrank"),
         dict(kind="tube", label="Aileron pushrod (wing) x2: 6061-T6 1/2 x 0.035 in", len=round(seg(RT("Aileron pushrods R")["pts"], 5)), note="arch-top bellcrank to outboard bellcrank, through rib lightening holes"),
         dict(kind="tube", label="Aileron link (cabin) x2: 6061-T6 3/8 x 0.035 in", len=round(seg(RT("Aileron pushrods R")["pts"], 1)), note="torque-tube arm to arch-foot bellcrank"),
         dict(kind="tube", label="Flap pushrod (wing) x2: 6061-T6 3/8 x 0.035 in", len=round(seg(RT("Flap pushrods R")["pts"], 1)), note="centre torque-tube arm to flap bellcrank")],
}
for s in STEPS:
    s["draw"] = DRAW.get(s["n"], [])

# ---------------------------------------------------------------- full BOM
BOM = []
def B(cat, item, spec, qty, unit, step, note=""):
    BOM.append(dict(cat=cat, item=item, spec=spec, qty=qty, unit=unit, step=step, note=note))

for b in bom:
    B("Wing stands (step 0)" if b["group"] == "Stands" else "Folded blanks" if b["group"] != "Fittings" else "4130 fittings", f"{b['id']} {b['name']}", b["material"], b["qty"], "blanks", b["step"],
      f"{b['flat_w']} x {b['flat_h']} mm flat, {b['folds']} folds, {b['pilot_holes']} pilot holes" + (f"; {b['per_ac']} parts x {b['layers']} laminated layers" if b['layers'] > 1 else ""))
tot_r = {}
for (kind, st), n in grips.items():
    tot_r[kind] = tot_r.get(kind, 0) + n
for kind, n in sorted(tot_r.items()):
    if kind == "firewall":
        B("Fasteners", "A4-SS blind rivet", "1/8 in stainless/Monel, domed (firewall)", n, "pcs", 8, "+15 % spares")
    else:
        g = kind.split()[-1]
        B("Fasteners", f"A4-{g} blind rivet", f"1/8 in aluminium, domed, grip -{g}", n, "pcs", "4-15", "+15 % spares")
agg = {}
for h in HW:
    k = (h["code"], h["desc"])
    a = agg.setdefault(k, dict(qty=0, steps=set(), where=[]))
    a["qty"] += h["qty"]; a["steps"].add(h["step"]); a["where"].append(h["where"])
for (code, desc), a in agg.items():
    B("Fasteners", code, desc, a["qty"], "sets", ",".join(str(x) for x in sorted(a["steps"])), "; ".join(a["where"]))
B("Fasteners", "Clecos", "1/8 in spring Clecos + pliers", 300, "pcs", "3-15", "tool, reused")
for d in DRAW.values():
    for x in d:
        qty = 2 if "x2" in x["label"] else 1
        B("Tube, bar and cable", x["label"].split(":")[0], x["label"].split(":")[-1].strip(), qty, f"x {x['len']} mm", "", x["note"])
B("Tube, bar and cable", "Strut end forks", "4130, tube insert + tang, AN5 hole", 8, "pcs", 16, "bought or welded")
B("Tube, bar and cable", "Strut fairings", "Clip-on extruded plastic fairing for 1.75 in tube", 4, "x 2.6 m", 16, "")
B("Tube, bar and cable", "Engine mount", "4130 tube, welded, 4-point", 1, "pcs", 14, "fabricated or bought for half-VW")
for (n, m) in res["bought"]:
    if any(k in n for k in ("Rivets", "AN bolts", "Lift struts", "Jury")):
        continue
    B("Bought assemblies", n, "", 1, "set", "", f"{m} kg")
B("Bought assemblies", "Rod ends", "3/16 in bore male rod end + check nut", 12, "pcs", 17, "")
B("Bought assemblies", "Systems fit-out", "Adel P-clamps x40, firewall grommets x4, pitot tube, static port, 4 m pitot/static tubing, 10 m tefzel wire, fuses, master + mag switches", 1, "set", 17, "")
B("Bought assemblies", "Battery + radio + antenna", "12 V LiFePO4 + vented box, panel-mount VHF, whip antenna, coax", 1, "set", 17, "radio optional in SSDR")
B("Bought assemblies", "Extinguisher and PLB", "1 kg extinguisher + personal locator beacon", 1, "set", 17, "")
B("Bought assemblies", "Fuel system parts", "Shut-off valve, gascolator, 1.5 m 1/4 in fuel hose + fire sleeve, firewall grommet, vented filler cap", 1, "set", 14, "gravity feed")
B("Bought assemblies", "Piano hinge", "MS20001, 0.040 in", 14, "m", "7,11,18", "4 x 1.79 m wing, 2 x 1.0 m elevators, 1.24 m rudder, door")
B("Bought assemblies", "Fairleads and pulleys", "Phenolic fairleads, 3 in pulleys", 1, "set", 17, "")
B("Consumables", "Epoxy primer + etch/alodine", "2-part epoxy primer, chromate-free", 4, "L", "4-15", "faying surfaces and interior")
B("Consumables", "Cotter pins", "MS24665 assorted", 50, "pcs", "13-17", "")
B("Consumables", "Safety wire", "0.032 in stainless", 1, "roll", "14-17", "")
B("Consumables", "Foam tape", "Closed-cell, windscreen bedding", 5, "m", 18, "")
B("Consumables", "MEK / cleaner, Scotch-Brite, gloves", "", 1, "set", "1-19", "")
B("Consumables", "Paint", "Polyurethane topcoat", 4, "L", 19, "")
for k, v in TOOLS.items():
    B("Tools", v, "", 1, "set", ",".join(str(s["n"]) for s in STEPS if k in s["tools"]), "")

# ---------------------------------------------------------------- line prices (GBP, UK retail incl. 20 % VAT, estimates)
M2 = {"0.016in": 110, "0.020in": 130, "0.025in": 115, "0.032in": 110, "0.040in": 135, "0.050in": 135}
_mat_cost, _mat_area = {}, {}
for (mat, stock), n in stock_count.items():
    area = {"8 ft x 4 ft sheet": 2.438 * 1.219, "12 ft x 4 ft sheet (spar strips)": 3.658 * 1.219,
            "2000 x 1000 sheet": 2.0, "2050 x 1250 sheet": 2.56, "4130 sheet 4 ft x 2 ft": 0.74}[stock]
    rate = M2.get(mat.split()[-1], 120) if mat.startswith("6061") else (216 if mat.startswith("4130") else (45 if mat.startswith("Stainless") else 55))
    _mat_cost[mat] = _mat_cost.get(mat, 0) + n * area * rate
for b_ in bom:
    _mat_area[b_["material"]] = _mat_area.get(b_["material"], 0) + b_["flat_w"] * b_["flat_h"] * b_["qty"]
def sheet_price(mat, w, h):
    """share of the nested stock-sheet cost of that material, by blank area (so the BOM adds up to the sheets you buy)"""
    return _mat_cost[mat] * (w * h) / _mat_area[mat], mat.startswith("6061") or mat.startswith("4130")
FP = {"A4-1": .12, "A4-2": .12, "A4-SS": .35, "AN3": .9, "AN4": 1.2, "AN5": 1.8, "PIN5": 2.6, "AXLE": 9, "DZUS": 7, "M4": .25, "TB": 18, "NICO": 1.6, "HINGE": 45, "Clecos": .45}
def tube_price(label, length):
    m = length / 1000
    for k, v in (("1.75", 22), ("0.75", 8), ("1/2", 6), ("3/8", 5), ("bar 75", 60), ("leaf", 40), ("7x19", 1.8)):
        if k in label:
            return m * v
    return m * 10
BOUGHT_P = [("Battery 12 V", 0, False), ("Radio (panel", 0, False), ("Wiring, switches", 0, False), ("Pitot-static", 0, False), ("Fire extinguisher 1 kg + PLB", 0, False), ("Half-VW", 5500, True), ("Propeller", 550, False), ("Engine mount, 4130", 0, True), ("Main gear spring legs", 0, False),
            ("Wheels", 980, False), ("Tailwheel", 210, False), ("Mass balance", 40, False), ("Controls:", 250, False), ("Seat", 220, False),
            ("Fuel tank", 180, False), ("Aux tank", 190, False), ("Instruments", 600, False), ("Primer", 0, False), ("Piano hinges (wing", 0, False)]
SUND = {"Systems fit-out": (220, False), "Battery + radio + antenna": (520, False), "Extinguisher and PLB": (260, False), "Fuel system parts": (160, False), "Rod ends": (9, True), "Piano hinge": (12, True), "Fairleads and pulleys": (120, True), "Strut end forks": (18, True), "Strut fairings": (30, False)}
CONS = {"Epoxy primer": 45, "Cotter pins": .1, "Safety wire": 18, "Foam tape": 2, "MEK": 40, "Paint": 35}
TOOLP = {"brake": 300, "drill": 60, "cleco": 25, "puller": 45, "deburr": 20, "mark": 15, "snips": 45, "roll": 20, "level": 40, "spanner": 120,
         "safety": 35, "tube": 180, "ream": 60, "prime": 80, "nico": 90, "plastic": 25, "scales": 150, "sand": 60, "dup": 25, "vise": 60}
tool_by_desc = {v: k for k, v in TOOLS.items()}
bom_by_id = {b["id"]: b for b in bom}
keep = []
for r in BOM:
    unit, us = 0.0, False
    if r["cat"] in ("Folded blanks", "4130 fittings", "Wing stands (step 0)"):
        pid = r["item"].split()[0]
        b_ = bom_by_id[pid]
        unit, us = sheet_price(b_["material"], b_["flat_w"], b_["flat_h"])
    elif r["cat"] == "Fasteners":
        code = r["item"].split()[0]
        unit, us = FP.get(code, 1.0), code not in ("M4",)
        if code.startswith("A4"):
            r["qty"] = int(round(r["qty"] * 1.15)); r["note"] = "includes 15 % spares"
    elif r["cat"] == "Tube, bar and cable":
        if r["item"] in SUND:
            unit, us = SUND[r["item"]]
        elif r["item"] == "Engine mount":
            unit, us = 320, True
        else:
            L = float(r["unit"].split()[1]); unit, us = tube_price(r["item"] + r["spec"], L), True
    elif r["cat"] == "Bought assemblies":
        hit = next(((p_, u_) for k_, p_, u_ in BOUGHT_P if r["item"].startswith(k_) or k_ in r["item"]), None)
        if r["item"] in SUND:
            unit, us = SUND[r["item"]]
        elif hit:
            unit, us = hit
        if unit == 0 and not hit and r["item"] not in SUND:
            unit = 50
        if hit and hit[0] == 0:
            continue  # counted elsewhere
    elif r["cat"] == "Consumables":
        unit = next((v for k, v in CONS.items() if r["item"].startswith(k)), 20)
    elif r["cat"] == "Tools":
        unit = TOOLP.get(tool_by_desc.get(r["item"], ""), 40)
    q = r["qty"] if isinstance(r["qty"], (int, float)) else 1
    r["gbp"] = round(unit, 2); r["gbp_total"] = round(unit * q, 2); r["us"] = us
    keep.append(r)
BOM[:] = keep
BOM.append(dict(cat="Bought assemblies", item="Laser / waterjet cutting service", spec="all nests, incl. 4130 and polycarbonate", qty=1, unit="job", step=1, note="or a club CNC router", gbp=900, gbp_total=900, us=False))
cats = []
for r in BOM:
    if r["cat"] not in cats:
        cats.append(r["cat"])
cost = [(c, round(sum(r["gbp_total"] for r in BOM if r["cat"] == c), -1)) for c in cats]
cost_total = sum(c for _, c in cost)

# ---------------------------------------------------------------- model export
from parts import STAND_POS, STAND_SIDE
def _stand_alt(lab):
    k = int(lab.split()[-1]); (cx, cy), (sx, sy) = STAND_POS[k], STAND_SIDE[k]
    return [sx - cx, sy - cy, 0.0]
model = dict(parts=[], bought=bought3d, steps=STEPS, hw=[h for h in HW if h["pos"]], tools=TOOLS, bomx=BOM, routes=ROUTES)
for p in parts:
    fm, rootM, holes = flat_mold[p.id]
    model["parts"].append(dict(
        id=p.id, name=p.name, group=p.group, material=p.material, t=p.t, step=p.step, note=p.note,
        mass=round(p.mass() * 1000), qty=len(p.instances), layers=getattr(p, "layers", 1),
        panels=[dict(poly=[[round(v, 2) for v in q] for q in f["poly"]], parent=f["parent"],
                     hinge=([[round(v, 2) for v in q] for q in f["hinge"]] if f["hinge"] else None),
                     ang=round(f["ang"], 2), kind=f["kind"],
                     holes=[[[round(v, 2) for v in q] for q in h] for h in f["holes"]]) for f in fm],
        rivets=[[round(q[0], 1), round(q[1], 1)] for q in holes],
        rv=[[r["f"], r["xy"][0], r["xy"][1], r["step"]] for r in riv_info[p.id]],
        bolts=[[b["f"], b["xy"][0], b["xy"][1], b["d"]] for b in bolt_info[p.id]],
        inst=[[round(v, 4) for v in col_major(M @ rootM)] for (M, lab) in p.instances],
        alt=[_stand_alt(lab) for (M, lab) in p.instances] if p.group == "Stands" else None,
        iw=[1 if (p.group == "Wing" or p.id == "K-TG" or "strut top" in lab or "jury top" in lab or "wing" in lab) else 0 for (M, lab) in p.instances],
    ))

kt = lambda v: round(v / KT, 1)
g0 = 9.80665
H = dict(
    span=round(res["span"] / 1000, 2), S=round(res["S"], 2), AR=round(res["AR"], 2), chord=P["chord"] / 1000,
    length=round((4560 + 75 + 1480) / 1000, 2),
    empty=round(res["mass_empty"]), sheet=round(res["mass_sheet"], 1), mtow=res["MTOW"], useful=round(res["useful"]),
    Vs=kt(res["Vs"]), Vs0=kt(res["Vs0"]), Vmax=kt(res["Vmax"]), Vcr=kt(res["Vcr"]), Vy=kt(res["Vy"]),
    roc=round(res["ROC"] * 196.85, -1), LD=round(res["LDmax"], 1), CD0=round(res["CD0"], 4), f=round(res["f"], 2),
    T0=round(res["T0"]), to_grass=round(res["s_to_grass"], -1), to_hard=round(res["s_to_hard"], -1), land=round(res["s_land"], -1),
    fuel_flow=round(res["fuel_flow"], 1), endurance=round(res["endurance"], 1), range=round(res["range_km"], -1), endurance_both=round(res["endurance_both"], 1), range_both=round(res["range_both"], -1), prop_beta75=round(PROP_INFO["beta75"], 1), prop_d_in=PROP_INFO["D_in"], prop_p_in=PROP_INFO["P_in"], header_l=CFG_LD["header_tank_l"], aux_l=CFG_LD["aux_tank_l"], fuel_rho=CFG_LD["fuel_density"], version=CFG["version"],
    WS=round(res["WS"] / g0, 1), PW=round(res["MTOW"] / 37, 1),
    Vh=round(res["Vh"], 2), Vv=round(res["Vv"], 3), hn=round(100 * res["hn"], 1), cg_fwd=round(100 * res["cg_fwd"], 1),
    cg_aft=round(100 * res["cg_aft"], 1), SM_min=round(100 * res["SM_min"], 1), SM_max=round(100 * res["SM_max"], 1),
    n_pos=res["n_pos"], n_neg=res["n_neg"], Va=kt(res["Va"]), Vne=kt(res["Vne"]), Vd=kt(res["Vd"]),
    gust_c=round(res["n_gust_c"], 2), gust_d=round(res["n_gust_d"], 2),
    ms_sig=round(res["spar_main"]["sig_ult"]), ms_cc=round(res["spar_main"]["s_cc"]), ms_MS=round(res["spar_main"]["MS_comp"], 2),
    ms_tau=round(res["spar_main"]["tau_lim"], 1), ms_taucr=round(res["spar_main"]["tau_cr"], 1), ms_MSs=round(res["spar_main"]["MS_shear_buckle_limit"], 2),
    rs_sig=round(res["spar_rear"]["sig_ult"]), rs_cc=round(res["spar_rear"]["s_cc"]), rs_MS=round(res["spar_rear"]["MS_comp"], 2),
    strut_L=round(res["strut_L"]), strut_T=round(res["strut_T_ult"]), strut_C=round(res["strut_C_ult"]), strut_Pcr=round(res["strut_Pcr"]),
    strut_MSb=round(res["strut_MS_buckle"], 2), strut_MSt=round(res["strut_MS_ten"], 2), strut_phi=round(res["strut_phi"], 1),
    rivets=total_rivets, hw_total=hw_total, nparts=len([p for p in parts if p.group != "Stands"]), nblanks=sum(len(p.instances) * getattr(p, "layers", 1) for p in parts if p.group != "Stands"), nblanks_stands=sum(len(p.instances) * getattr(p, "layers", 1) for p in parts if p.group == "Stands"),
    cost_total=round(cost_total, -2), cg_cases=res["cg_cases"], sheets=[dict(material=k[0], stock=k[1], n=v) for k, v in stock_count.items()],
    h_ms=round(res["h_ms"]), h_rs=round(res["h_rs"]),
)
model["H"] = H
model["cost"] = cost
model["bom"] = bom
json.dump(model, open(f"{OUT}/ori1_model.json", "w"), separators=(",", ":"), default=float)

with open(f"{OUT}/BOM.csv", "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["Category", "Item", "Spec", "Qty", "Unit", "Build step", "Notes"])
    for b in BOM:
        w.writerow([b["cat"], b["item"], b["spec"], b["qty"], b["unit"], b["step"], b["note"]])
    w.writerow([])
    w.writerow(["Stock sheets (from nest)"])
    for k, v in stock_count.items():
        w.writerow(["Stock", f"{k[0]}", k[1], v, "sheets", "1", ""])
    w.writerow([])
    w.writerow(["Rough cost (GBP, 2026 UK retail, estimate)"])
    for n, c in cost:
        w.writerow(["Cost", n, "", c, "GBP", "", ""])
    w.writerow(["Cost", "TOTAL", "", round(cost_total, -2), "GBP", "", ""])
with open(f"{OUT}/HARDWARE_BY_STEP.csv", "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["Step", "Title", "Code", "Description", "Qty", "Where"])
    for s in STEPS:
        for h in s["hw"]:
            w.writerow([s["n"], s["title"], h["code"], h["desc"], h["qty"], h["where"]])

json.dump(dict(nests=nest_summary), open(f"{OUT}/nests.json", "w"), indent=1)
print("parts", len(parts), "blanks", H["nblanks"], "rivets", total_rivets, "hw", hw_total, "cost", cost_total)
print({s["n"]: (s["rivets"], s["hwcount"], s["blanks"]) for s in STEPS})
print({k: H[k] for k in ("empty", "Vs", "Vs0", "Vmax", "roc", "cg_fwd", "cg_aft", "SM_min")})
print("struts", L_main, L_rear, L_jury, L_juryr)
