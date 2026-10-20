"""All OpenPlane sheet parts, built as folded 3D faces in their design frames."""
import math
import numpy as np
from geom import (mat_Rx, P, WING, Part, extrude_path, flanged_web, v3, unit, wing_up_pts, wing_lo_pts, clip_x,
                  z_on, is_roll_x, Airfoil, mat_T, mat_Rx, mat_Ry, MIRROR_Y, wing_to_world, RHO_SS, RHO_PC,
                  newell)

PARTS = []
EX, EY, EZ = v3(1, 0, 0), v3(0, 1, 0), v3(0, 0, 1)
O = v3(0, 0, 0)


def rivet_line(part, face_idx, pitch=40.0, end=15.0):
    f = part.faces[face_idx]["pts"]
    a = 0.5 * (f[0] + f[1])
    b = 0.5 * (f[3] + f[2])
    L = np.linalg.norm(b - a)
    n = max(int((L - 2 * end) // pitch) + 1, 2)
    for k in range(n):
        part.rivets3d.append(a + (b - a) * (end + k * (L - 2 * end) / (n - 1)) / L)


LAYERS = {"W-CSS": 2, "T-HS": 2, "T-FS": 2, "T-ES": 1, "T-RS": 1, "F-B2": 1, "F-B3": 1, "F-B4": 1, "F-B5": 2, "F-NF": 1, "F-ST": 1}


def single_gauge(p):
    """Every aluminium part is cut from one stock: 6061-T6 0.020 in. Parts that need more
    thickness are made as n nested layers folded together and riveted as one (a laminated fold)."""
    p.layers = 1
    if not p.material.startswith("6061"):
        return
    n = LAYERS.get(p.id) or max(1, math.ceil(p.t / 0.5 - 0.3))
    p.layers = n
    p.tb = 0.5
    p.t = 0.5 * n
    p.material = "6061-T6 0.020in"
    if n > 1:
        p.note = (p.note + " " if p.note else "") + f"Laminated: {n} nested layers of 0.020 in, folded separately and riveted as one; trim 0.5 mm per inner layer off flange edges."


def reg(p, step, color=None):
    single_gauge(p)
    p.step = step
    p.color = color
    PARTS.append(p)
    return p


# =============================================================== WING
tsu, tsl = P["t_skin_u"], P["t_skin_l"]
UP = wing_up_pts(off=tsu)        # inner mould line under upper skin
LO = wing_lo_pts(off=tsl)
UPo = wing_up_pts()              # outer surface (skins)
LOo = wing_lo_pts()
xms, xrs = P["x_ms"], P["x_rs"]
tms, trs = P["t_ms"], P["t_rs"]
stations = [i * P["rib_pitch"] for i in range(int(P["panel"] / P["rib_pitch"]) + 1)]


def spar_main():
    p = Part("W-MS", "Main spar (lipped C, opens aft)", "Wing", "6061-T6 0.050in", tms,
             "Order as 12 ft strip. Lips turn the free flange edges into supported edges (k 0.43 -> ~4).")
    b, d = P["cap_b"], P["lip"]
    x0, x1 = xms, xms + b
    path = [(x1, z_on(LO, x1) + d), (x1, z_on(LO, x1)), (x0, z_on(LO, x0)), (x0, z_on(UP, x0)),
            (x1, z_on(UP, x1)), (x1, z_on(UP, x1) - d)]
    ix = extrude_path(p, path, EY, EX, EZ, O, 0, P["panel"], root=2)
    rivet_line(p, ix[1]); rivet_line(p, ix[3])
    return p


def spar_rear():
    p = Part("W-RS", "Rear spar (lipped C, opens forward)", "Wing", "6061-T6 0.032in", trs,
             "Carries flap/aileron hinges and rear strut fitting.")
    b, d = P["cap_b"], P["lip"]
    x0, x1 = xrs, xrs - b
    path = [(x1, z_on(LO, x1) + d), (x1, z_on(LO, x1)), (x0, z_on(LO, x0)), (x0, z_on(UP, x0)),
            (x1, z_on(UP, x1)), (x1, z_on(UP, x1) - d)]
    ix = extrude_path(p, path, EY, EX, EZ, O, 0, P["panel"], root=2)
    rivet_line(p, ix[1]); rivet_line(p, ix[3])
    return p


def rib_poly(x0, x1, notch0=0.0, notch1=0.0):
    """Rib web outline in panel frame (x,z) between x0 and x1, upper then lower.
    notchN lowers/raises the edge near the spar caps it must pass under."""
    up = clip_x(UP, x1, x0)
    lo = clip_x(LO, x1, x0)
    def adj(pts, sgn):
        out = []
        for x, z in pts:
            dz = 0.0
            if notch0 and x <= x0 + P["cap_b"] + 3:
                dz = notch0
            if notch1 and x >= x1 - P["cap_b"] - 3:
                dz = notch1
            out.append((x, z - sgn * dz))
        return out
    up, lo = adj(up, 1), adj(lo, -1)
    return up, lo


def make_rib(pid, name, x0, x1, notch0, notch1, front_flange, rear_flange, lighten):
    p = Part(pid, name, "Wing", "6061-T6 0.020in", P["t_rib"],
             "Faceted rib: every flange is a straight brake bend, no form block or fluting needed.")
    up, lo = rib_poly(x0, x1, notch0, notch1)
    # polygon CCW in (x,z): along lower from x0 to x1, then upper back
    loop = lo + list(reversed(up))
    if x0 < 1:  # nose rib: LE point shared
        loop = lo[1:] + list(reversed(up))
    pts3 = [v3(x, 0, z) for x, z in loop]
    # orientation: make CCW about +y? flange dir is -y (inboard). compute edges
    n = len(pts3)
    holes = []
    for (cx, cz, r) in lighten:
        holes.append([v3(cx + r * math.cos(a), 0, cz + r * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 24, endpoint=False)])
    fl = []
    for e in range(n):
        a, b = pts3[e], pts3[(e + 1) % n]
        if np.linalg.norm(b - a) < 6:
            continue
        vertical = abs(b[0] - a[0]) < 1e-6
        if vertical:
            if (abs(a[0] - x0) < 1e-6 and front_flange) or (abs(a[0] - x1) < 1e-6 and rear_flange):
                fl.append((e, 25.0, -EY))
            continue
        fl.append((e, 15.0, -EY))
    flanged_web(p, pts3, -EY, fl, holes=holes, rivet_pitch=45.0)
    return p


def wing_ribs():
    nose = make_rib("W-NR", "Nose rib", 0.0, xms, 0, 0, False, True,
                    lighten=[(250, z_on(UP, 250) * 0.5 + z_on(LO, 250) * 0.5, 45)])
    xa, xb = xms + tms, xrs - trs
    zc = lambda x: 0.5 * (z_on(UP, x) + z_on(LO, x))
    main = make_rib("W-MR", "Main rib", xa, xb, tms, trs, True, True,
                    lighten=[(600, zc(600), 55), (800, zc(800), 40)])
    for r in (nose, main):
        for s in stations[1:-1]:
            for side in "RL":
                r.instances.append((wing_to_world(side) @ mat_T(0, s, 0), f"{side} y={s:.0f}"))
    # root / tip ribs (same blanks, flanges face into the panel)
    rootn = make_rib("W-NRe", "Nose rib, root/tip", 0.0, xms, 0, 0, False, True, lighten=[])
    rootm = make_rib("W-MRe", "Main rib, root/tip", xa, xb, tms, trs, True, True, lighten=[])
    for r in (rootn, rootm):
        for side in "RL":
            r.instances.append((wing_to_world(side) @ mat_T(0, 0, 0) @ MIRROR_Y, f"{side} root"))
            r.instances.append((wing_to_world(side) @ mat_T(0, P["panel"], 0), f"{side} tip"))
    return [nose, main, rootn, rootm]


def skin_upper(y0, y1, pid, name):
    p = Part(pid, name, "Wing", "6061-T6 0.020in", tsu,
             "Nose zone is rolled over a 75 mm tube; aft creases are soft brake hits (2-6 deg).")
    pts = clip_x(UPo, xrs + 40)
    lo = clip_x(LOo, 140)
    path = list(reversed(pts)) + lo[1:]       # TE-ish -> LE -> lower 10%
    kinds = []
    for k in range(len(path) - 1):
        xm = 0.5 * (path[k][0] + path[k + 1][0])
        kinds.append("roll" if xm < 0.15 * P["chord"] else "crease")
    # root facet: the one spanning the main spar
    root = next(k for k in range(len(path) - 1) if path[k + 1][0] <= xms <= path[k][0])
    extrude_path(p, path, EY, EX, EZ, O, y0, y1, root=root, kinds=kinds)
    return p


def skin_lower(y0, y1, pid, name):
    p = Part(pid, name, "Wing", "6061-T6 0.016in", tsl, "Laps 30 mm under the upper skin at 8% chord.")
    lo = clip_x(LOo, xrs, 110)
    kinds = ["roll" if 0.5 * (lo[k][0] + lo[k + 1][0]) < 0.10 * P["chord"] else "crease" for k in range(len(lo) - 1)]
    root = next(k for k in range(len(lo) - 1) if lo[k][0] <= xms <= lo[k + 1][0])
    extrude_path(p, lo, EY, EX, EZ, O, y0, y1, root=root, kinds=kinds)
    return p


def control_surface(pid, name, x0, chord, af_up, af_lo, span, group, note):
    """Wrapped single-sheet control surface: one sharp TE crease (the origami
    edge), one pleated bead top and bottom, closed by a C spar."""
    p = Part(pid, name, group, "6061-T6 0.016in", P["t_cs"], note)
    x1 = x0 + chord
    zu0, zl0 = af_up(x0), af_lo(x0)
    zte = 0.5 * (af_up(x1) + af_lo(x1))
    xb = x0 + 0.40 * chord   # bead position
    def lerp(a, b, t):
        return a + (b - a) * t
    # straight wedge faces (developable), with a V bead in each
    def face_pts(z0, sgn):
        # points from spar (x0) to TE along a straight line, bead at xb
        tb = (xb - x0) / chord
        zb = lerp(z0, zte, tb)
        w = 8.0
        dz = sgn * 4.0
        return [(x0, z0), (xb - w, lerp(z0, zte, (xb - w - x0) / chord)), (xb, zb - dz),
                (xb + w, lerp(z0, zte, (xb + w - x0) / chord)), (x1, zte)]
    top = face_pts(zu0, 1)
    bot = face_pts(zl0, -1)
    path = top + list(reversed(bot))[1:]
    # spar flange overlap: start/end 0 offset; root = top face first panel
    extrude_path(p, path, EY, EX, EZ, O, 0, span, root=0)
    return p


def cs_spar(pid, name, x0, af_up, af_lo, span, group):
    p = Part(pid, name, group, "6061-T6 0.025in", 0.63, "Closes the control surface; carries hinge and mass-balance arm.")
    b = 30.0
    path = [(x0 + b, af_lo(x0 + b) + 0.4), (x0, af_lo(x0) + 0.4), (x0, af_up(x0) - 0.4), (x0 + b, af_up(x0 + b) - 0.4)]
    ix = extrude_path(p, path, EY, EX, EZ, O, 0, span, root=1)
    rivet_line(p, ix[0]); rivet_line(p, ix[2])
    return p


def wing_control_surfaces():
    # straight-line wedge between hinge and TE (outer mould)
    x0, ch = P["x_cs"], P["chord"] - P["x_cs"]
    zte = 0.5 * (WING.up(P["chord"]) + WING.lo(P["chord"]))
    up = lambda x: WING.up(x0) + (zte - WING.up(x0)) * (x - x0) / ch
    lo = lambda x: WING.lo(x0) + (zte - WING.lo(x0)) * (x - x0) / ch
    span = P["flap"][1] - P["flap"][0]
    skin = control_surface("W-CS", "Aileron / flap skin (identical x4)", x0, ch, up, lo, span, "Wing",
                           "Same blank for both flaps and both ailerons: modular-origami units.")
    spar = cs_spar("W-CSS", "Aileron / flap spar (identical x4)", x0, up, lo, span, "Wing")
    for part in (skin, spar):
        for side in "RL":
            for nm, (a, b) in (("flap", P["flap"]), ("aileron", P["aileron"])):
                part.instances.append((wing_to_world(side) @ mat_T(0, a, 0), f"{side} {nm}"))
    return skin, spar


# =============================================================== FUSELAGE
hw, zt, zb, ch = P["cabin_half"], P["z_top"], P["z_bot"], P["chamfer"]
xfw, xce, xtail = P["x_fw"], P["x_cab_end"], P["x_tail"]


def cabin_section():
    """(y,z) loop, CCW seen from aft."""
    return [(-hw + ch, zb), (hw - ch, zb), (hw, zb + ch), (hw, zt), (-hw, zt), (-hw, zb + ch)]


def firewall():
    p = Part("F-FW", "Firewall", "Fuselage", "Stainless 304 0.5 mm", 0.5,
             "Flanges all round fold aft; engine mount bolts through 4 doublers.", density=RHO_SS)
    zf = P["z_fw_top"]
    loop = [(-hw + ch, zb), (hw - ch, zb), (hw, zb + ch), (hw, zf), (-hw, zf), (-hw, zb + ch)]
    pts = [v3(xfw, y, z) for y, z in loop]
    flanged_web(p, pts, EX, [(e, 20.0, EX) for e in range(len(pts))], rivet_pitch=40)
    p.instances.append((np.eye(4), ""))
    return p


def u_frame(pid, name, x, leg=60.0, top=60.0, bottom=-930.0):
    p = Part(pid, name, "Fuselage", "6061-T6 0.040in", 1.0,
             "Arch over the pilot. Spar-root fittings bolt to its top corners, strut fittings to its feet.")
    o = [(-hw, bottom), (-hw, zt), (hw, zt), (hw, bottom), (hw - leg, bottom), (hw - leg, zt - top),
         (-hw + leg, zt - top), (-hw + leg, bottom)]
    pts = [v3(x, y, z) for y, z in o]
    fl = [(0, 20, EX), (1, 20, EX), (2, 20, EX), (4, 15, EX), (5, 15, EX), (6, 15, EX)]
    flanged_web(p, pts, EX, fl, rivet_pitch=40)
    p.instances.append((np.eye(4), ""))
    return p


def rounded_rect_x(x0, x1, z0, z1, y, r):
    ring = []
    for (cx, cz, a0) in ((x1 - r, z0 + r, -90), (x1 - r, z1 - r, 0), (x0 + r, z1 - r, 90), (x0 + r, z0 + r, 180)):
        for a in np.linspace(a0, a0 + 90, 7):
            ring.append(v3(cx + r * math.cos(math.radians(a)), y, cz + r * math.sin(math.radians(a))))
    return ring


DOOR = (440.0, 960.0, -850.0, -200.0)
FWIN = [(-420, -460), (360, -460), (360, -140), (330, -110), (-30, -110), (-420, -299)]


def cabin_side():
    p = Part("F-SD", "Cabin side (x2, mirrored): door/window opening + front window", "Fuselage", "6061-T6 0.020in", 0.5,
             "Top flange is the longeron: the fold is the stiffener. Openings have 50 mm corner radii (no square corners).")
    zf, xw = P["z_fw_top"], P["x_ws_top"]
    loop = [(xfw, zb + ch), (xce, zb + ch), (xce, zt), (xw, zt), (xfw, zf)]
    pts = [v3(x, hw, z) for x, z in loop]
    holes = [rounded_rect_x(*DOOR, hw, 50), [v3(x, hw, z) for x, z in FWIN]]
    fl = [(2, 25.0, -EY), (3, 20.0, -EY)]
    flanged_web(p, pts, -EY, fl, holes=holes, rivet_pitch=40)
    p.instances.append((np.eye(4), "R"))
    p.instances.append((MIRROR_Y, "L"))
    return p


def glazing():
    out = []
    d = Part("F-DW", "Door (left, hinged) / side window (right, fixed)", "Fuselage", "Polycarbonate 2 mm", 2.0,
             "Same blank both sides. 20 mm lap all round. Left one hangs on a piano hinge at its front edge.", density=RHO_PC)
    x0, x1, z0, z1 = DOOR
    d.add(rounded_rect_x(x0 - 20, x1 + 20, z0 - 20, z1 + 20, hw + 1.5, 70))
    d.instances.append((np.eye(4), "R fixed window"))
    d.instances.append((MIRROR_Y, "L door"))
    for k in range(9):
        t = (k + 0.5) / 9
        d.bolts3d.append((v3(x0 - 10 + (x1 - x0 + 20) * t, hw + 1.5, z1 + 10), 4.5, "screw"))
        d.bolts3d.append((v3(x0 - 10 + (x1 - x0 + 20) * t, hw + 1.5, z0 - 10), 4.5, "screw"))
    out.append(d)
    w = Part("F-FWN", "Front side window (x2, mirrored)", "Fuselage", "Polycarbonate 2 mm", 2.0,
             "Flat polycarbonate over the front cut-out, 20 mm lap, screws in 4.5 mm oversize holes.", density=RHO_PC)
    poly = [(-445, -480), (385, -480), (385, -130), (345, -90), (-40, -90), (-445, -280)]
    w.add([v3(x, hw + 1.5, z) for x, z in poly])
    for i in range(len(poly)):
        a, b = np.array(poly[i]), np.array(poly[(i + 1) % len(poly)])
        L = np.linalg.norm(b - a); n = max(int(L // 110), 1)
        for k in range(n):
            q = a + (b - a) * (k + 0.5) / n
            c = np.mean(poly, axis=0); q = q + (c - q) / np.linalg.norm(c - q) * 10
            w.bolts3d.append((v3(q[0], hw + 1.5, q[1]), 4.5, "screw"))
    w.instances.append((np.eye(4), "R"))
    w.instances.append((MIRROR_Y, "L"))
    out.append(w)
    return out


def floor_tray():
    p = Part("F-FL", "Floor tray with two hat pleats", "Fuselage", "6061-T6 0.032in", 0.8,
             "Two hat pleats (step-folds) are the floor stiffeners; no separate stringers.")
    z = zb
    path = [(-hw, zb + ch + 40), (-hw, zb + ch), (-hw + ch, z), (-160, z), (-150, z + 15), (-110, z + 15), (-100, z),
            (100, z), (110, z + 15), (150, z + 15), (160, z), (hw - ch, z), (hw, zb + ch), (hw, zb + ch + 40)]
    root = 6
    ix = extrude_path(p, path, EX, EY, EZ, O, xfw, xce, root=root)
    rivet_line(p, ix[0]); rivet_line(p, ix[12])
    p.instances.append((np.eye(4), ""))
    return p


def seat():
    p = Part("F-ST", "Seat (pan + back, one sheet)", "Fuselage", "6061-T6 0.032in", 0.8, "Leans on the rear arch.")
    path = [(300, -960), (320, -905), (620, -935), (660, -430)]
    extrude_path(p, path, EY, EX, EZ, O, -250, 250, root=1)
    p.instances.append((np.eye(4), ""))
    return p


PANEL_HOLES = [(-200, -505, 80, "ASI"), (-105, -505, 80, "ALT"), (-10, -505, 80, "TACH"), (85, -505, 57, "CHT/EGT"),
               (175, -515, 57, "RADIO"), (255, -455, 13, "MASTER"), (255, -505, 13, "MAG"), (255, -555, 13, "FUEL VALVE")]


def instrument_panel():
    p = Part("F-IP", "Instrument panel (glare-shield fold, side tabs)", "Fuselage", "6061-T6 0.032in", 0.8,
             "Flat panel; the top flange folds aft as a glare shield, side tabs fold forward onto the cabin sides.")
    x = -400.0
    loop = [(-hw + 2, -620), (hw - 2, -620), (hw - 2, -400), (-hw + 2, -400)]
    pts = [v3(x, y, z) for y, z in loop]
    holes = []
    for (cy, cz, d, nm) in PANEL_HOLES:
        r = d / 2
        holes.append([v3(x, cy + r * math.cos(a), cz + r * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 24, endpoint=False)])
    fl = [(2, 45.0, EX), (1, 20.0, -EX), (3, 20.0, -EX)]
    flanged_web(p, pts, EX, fl, holes=holes, rivet_pitch=45)
    p.instances.append((np.eye(4), ""))
    return p


def windscreen():
    p = Part("F-WS", "Windscreen (cold-bent flat polycarbonate)", "Fuselage", "Polycarbonate 2 mm", 2.0,
             "Planar: no forming, drill with plastic-cutting bits, slotted holes for expansion.", density=RHO_PC)
    zf, xw = P["z_fw_top"], P["x_ws_top"]
    p.add([v3(xfw, -hw, zf), v3(xfw, hw, zf), v3(xw, hw, zt), v3(xw, -hw, zt)])
    p.instances.append((np.eye(4), ""))
    return p


COWL_SEC = [(-350, -360), (350, -360), (350, -970), (270, -1050), (-270, -1050), (-350, -970)]
COWL_OCT = COWL_SEC  # kept name for importers
X_COWL_NOSE, X_COWL_FRONT, Z_THRUST, COWL_NOSE_SCALE = -1160.0, -1350.0, -550.0, 0.48
NS = len(COWL_SEC)


def cowl_pt(x, yz):
    if x >= X_COWL_NOSE:
        s = 1.0
    else:
        s = 1 + (COWL_NOSE_SCALE - 1) * (X_COWL_NOSE - x) / (X_COWL_NOSE - X_COWL_FRONT)
    return v3(x, s * yz[0], Z_THRUST + s * (yz[1] - Z_THRUST))


def cowl():
    """The cowl section is the firewall outline itself, so the whole front panel is covered."""
    out = []
    zf = P["z_fw_top"]
    up = Part("F-CU", "Upper cowl (hood, two 90 deg folds)", "Fuselage", "6061-T6 0.025in", 0.63,
              "Covers the top half of the firewall. Camlocks along the firewall flange and the split line.")
    path = [(-hw, -660), (-hw, zf), (hw, zf), (hw, -660)]
    extrude_path(up, path, -EX, EY, EZ, O, -xfw, -X_COWL_NOSE, root=1)
    lo = Part("F-CL", "Lower cowl (box-pleat pan to the cabin floor line)", "Fuselage", "6061-T6 0.025in", 0.63,
              "Five facets, chamfers matching the cabin floor. A louvre slot at the rear is the cooling-air exit.")
    path = [(-hw, -660), (-hw, -970), (-270, -1050), (270, -1050), (hw, -970), (hw, -660)]
    extrude_path(lo, path, -EX, EY, EZ, O, -xfw, -X_COWL_NOSE, root=2)
    out += [up, lo]
    def face(k):
        a, b = COWL_SEC[k], COWL_SEC[(k + 1) % NS]
        return [cowl_pt(X_COWL_NOSE, a), cowl_pt(X_COWL_NOSE, b), cowl_pt(X_COWL_FRONT, b), cowl_pt(X_COWL_FRONT, a)]
    nu = Part("F-NU", "Nose bowl, upper (3 converging facets)", "Fuselage", "6061-T6 0.025in", 0.63,
              "Frustum facets aimed at one point, so every crease is straight.")
    r0 = nu.add(face(0))
    nu.add(face(1), parent=r0, hinge=(0, 3)); nu.add(face(5), parent=r0, hinge=(1, 2))
    nl = Part("F-NL", "Nose bowl, chin (3 converging facets)", "Fuselage", "6061-T6 0.025in", 0.63, "")
    r3 = nl.add(face(3))
    nl.add(face(2), parent=r3, hinge=(1, 2)); nl.add(face(4), parent=r3, hinge=(0, 3))
    out += [nu, nl]
    fr = Part("F-NF", "Nose ring (prop-shaft hole + two cooling inlets)", "Fuselage", "6061-T6 0.032in", 0.8,
              "Flanges follow the nose taper.")
    ring = [cowl_pt(X_COWL_FRONT, q) for q in COWL_SEC]
    holes = []
    for (cy, cz, r) in ((0, Z_THRUST, 72), (-115, Z_THRUST - 40, 40), (115, Z_THRUST - 40, 40)):
        holes.append([v3(X_COWL_FRONT, cy + r * math.cos(a), cz + r * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 28, endpoint=False)])
    fl = []
    for e in range(NS):
        d = unit(cowl_pt(X_COWL_NOSE, COWL_SEC[e]) - cowl_pt(X_COWL_FRONT, COWL_SEC[e]))
        fl.append((e, 15.0, d))
    flanged_web(fr, ring, EX, fl, holes=holes, rivet_pitch=35)
    out.append(fr)
    for q in out:
        q.instances.append((np.eye(4), ""))
    return out


# ------------------------------------------------ 4130 fittings (flat, folded once or twice)
CLV_PIN_H = 30.0


def clevis():
    p = Part("K-CV", "Clevis, 4130 0.063in (U, two folds)", "Fittings", "4130 steel 0.063in", 1.6,
             "Ears take an AN5 pin (fore-aft axis); web bolts with 2 x AN4. Bend radius 3t for normalised 4130 (Wanttaja).", density=7.85e-6)
    w, L, hgt = 40.0, 60.0, 45.0
    path = [(-w / 2, hgt), (-w / 2, 0), (w / 2, 0), (w / 2, hgt)]
    extrude_path(p, path, EX, EY, EZ, v3(-L / 2, 0, 0), 0, L, root=1)
    p.bolts3d.append((v3(0, -w / 2, CLV_PIN_H), 8.0, "pin"))
    p.bolts3d.append((v3(0, w / 2, CLV_PIN_H), 8.0, "pin"))
    p.bolts3d.append((v3(-15, 0, 0), 6.4, "bolt"))
    p.bolts3d.append((v3(15, 0, 0), 6.4, "bolt"))
    return p


def frame_at(pin_axis, ear_dir, pin_center):
    """Clevis placement: local y = pin axis, local z = ear direction, pin at local (0,0,CLV_PIN_H)."""
    ly = unit(np.asarray(pin_axis, float))
    lz = np.asarray(ear_dir, float)
    lz = unit(lz - ly * np.dot(lz, ly))
    lx = np.cross(ly, lz)
    M = np.eye(4)
    M[:3, 0], M[:3, 1], M[:3, 2] = lx, ly, lz
    M[:3, 3] = np.asarray(pin_center, float) - lz * CLV_PIN_H
    return M


TANG_PIN = (-60.0, -30.0)


def tang():
    p = Part("K-TG", "Spar-root tang, 4130 0.063in (flat, pairs either side of web)", "Fittings", "4130 steel 0.063in", 1.6,
             "Bolts to the spar web with 4 x AN3 and reaches inboard past the spar end to the arch-top clevis pin.", density=7.85e-6)
    poly = [(-95, -55), (-25, -55), (0, -15), (120, -15), (120, 60), (-95, 60)]
    p.add([v3(0, y, z) for y, z in poly])
    for (y, z) in ((30, 40), (95, 40), (95, 5), (30, 5)):
        p.bolts3d.append((v3(0, y, z), 4.8, "bolt"))
    p.bolts3d.append((v3(0, TANG_PIN[0], TANG_PIN[1]), 8.0, "pin"))
    return p


def saddle_clamp():
    p = Part("K-JC", "Jury-strut saddle clamp, 4130 0.063in (rolled half-shell with ears)", "Fittings", "4130 steel 0.063in", 1.6,
             "Two halves round the 1.75 in strut tube; the ear bolts also carry the jury-strut end.", density=7.85e-6)
    r = 22.6
    arc = [(r * math.cos(a), r * math.sin(a)) for a in np.linspace(math.pi, 0, 9)]
    path = [(-44, 0)] + arc + [(44, 0)]
    kinds = ["crease"] + ["roll"] * (len(arc) - 1) + ["crease"]
    extrude_path(p, path, EX, EY, EZ, v3(-15, 0, 0), 0, 30, root=len(path) // 2 - 1, kinds=kinds)
    p.bolts3d.append((v3(0, -34, 0), 4.8, "bolt"))
    p.bolts3d.append((v3(0, 34, 0), 4.8, "bolt"))
    return p


STRUTS = []  # filled by fittings(): (side, name, foot_pin, top_pin, mid, jury_top_pin)


def fittings():
    cv, tg, jc = clevis(), tang(), saddle_clamp()
    from geom import wing_to_world as w2w
    STRUTS.clear()
    for side, sg in (("R", 1), ("L", -1)):
        W = w2w(side)
        wx = W[:3, 0]; wz = W[:3, 2]
        if side == "L":
            wz = W[:3, 2]
        for x in (xms, xrs):
            tw = tms if x == xms else trs
            xmid = x + tw / 2 if x == xms else x - tw / 2
            # spar-root: tang pair on the web, pin inboard of the spar end, clevis on the arch top
            pin_root = (W @ np.array([xmid, TANG_PIN[0], TANG_PIN[1], 1.0]))[:3]
            cv.instances.append((frame_at([1, 0, 0], [0, 0, 1], pin_root), f"{side} root clevis x{x:.0f}"))
            for off in (-1.6 - 0.2, tw + 0.2):
                x0 = x if x == xms else x - tw
                tg.instances.append((W @ mat_T(x0 + off + (0.8 if off > 0 else 0.8), 0, 0), f"{side} tang x{x:.0f}"))
            # strut foot: ears outward from the cabin side at the arch foot
            foot = np.array([x, sg * (hw + CLV_PIN_H), -950.0])
            cv.instances.append((frame_at([1, 0, 0], [0, sg, 0], foot), f"{side} strut foot x{x:.0f}"))
            # strut top: ears down from the spar cap at the 2.4 m rib
            xs = x + 20 if x == xms else x - 20
            zc = z_on(LO, xs)
            top = (W @ np.array([xs, 2400.0, zc - CLV_PIN_H, 1.0]))[:3]
            down = -wz
            cv.instances.append((frame_at(wx, down, top), f"{side} strut top x{x:.0f}"))
            # jury top: same clevis at the 1.05 m rib
            jt = (W @ np.array([xs, 1050.0, zc - CLV_PIN_H, 1.0]))[:3]
            cv.instances.append((frame_at(wx, down, jt), f"{side} jury top x{x:.0f}"))
            # saddle clamp at strut mid-length, ears in the plane toward the jury strut
            mid = 0.5 * (foot + top)
            d = unit(top - foot)
            toward = jt - mid
            lz = unit(toward - d * np.dot(toward, d))
            lzz = np.cross(d, lz)
            for flip in (1, -1):
                M = np.eye(4)
                M[:3, 0], M[:3, 1], M[:3, 2] = d, lz * flip, lzz * flip
                M[:3, 3] = mid
                jc.instances.append((M, f"{side} jury clamp x{x:.0f}"))
            STRUTS.append((side, "main" if x == xms else "rear", foot, top, mid + lz * 34.0, jt))
    return [cv, tg, jc]


# ------------------------------------------------ control linkage: bellcranks, horns, their brackets
CONTROL_RODS = []   # dict(name, color, pts) in world mm, consumed by the build for routing
BC_L = 55.0


def bellcrank_plate():
    p = Part("K-BC", "Bellcrank plate, 4130 0.063in (pairs, 55 mm arms)", "Fittings", "4130 steel 0.063in", 1.6,
             "Two plates sandwich each rod end; AN4 pivot, AN3 at each arm.", density=7.85e-6)
    L, m = BC_L, 15.0
    poly = [(-m, -m), (L + m, -m), (L + m, m), (m, L + m), (-m, L + m)]
    p.add([v3(u, v, 0) for u, v in poly])
    p.bolts3d.append((v3(0, 0, 0), 6.4, "pivot"))
    p.bolts3d.append((v3(0, L, 0), 4.8, "arm"))
    p.bolts3d.append((v3(L, 0, 0), 4.8, "arm"))
    return p


def bellcrank_at(Q, a, b):
    """incoming rod arrives at Q along a; outgoing rod leaves along b. Returns pivot O, out-hole H2, frame."""
    a, b = unit(np.asarray(a, float)), unit(np.asarray(b, float))
    ap = a - b * np.dot(a, b)
    if np.linalg.norm(ap) < 1e-6:
        ap = np.cross(b, [0, 0, 1.0])
    ap = unit(ap)
    O = np.asarray(Q, float) - b * BC_L
    H2 = O + ap * BC_L
    n = np.cross(ap, b)
    M = np.eye(4); M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = ap, b, n, O
    return O, H2, M, n, ap


def horn_part(pid, name, poly_uv, holes_uv, flange_edge=None):
    p = Part(pid, name, "Fittings", "4130 steel 0.063in", 1.6, "Horn: rod end bolts through the outer hole (AN3).", density=7.85e-6)
    pts = [v3(u, 0, v) for u, v in poly_uv]
    fl = [(flange_edge, 18.0, EY)] if flange_edge is not None else []
    flanged_web(p, pts, EY, fl, rivet_pitch=30)
    for (u, v, d) in holes_uv:
        p.bolts3d.append((v3(u, 0, v), d, "arm"))
    return p


def control_linkage():
    from geom import wing_to_world as w2w
    CONTROL_RODS.clear()
    bc, cv_extra = bellcrank_plate(), []
    # wing horn (aileron and flap): hole at local (0,0); top edge on the control-surface lower skin
    hn = horn_part("K-HN", "Aileron / flap horn, 4130 0.063in (one fold onto the lower skin)",
                   [(-22, -14), (30, -14), (40, 34), (-22, 34)], [(0, 0, 4.8)], flange_edge=2)
    def add_bc(Q, a, b, label):
        O, H2, M, n, ap = bellcrank_at(Q, a, b)
        for off in (-8.0, 8.0):
            bc.instances.append((mat_T(*(n * off)) @ M, label))
        away = -(ap + b)
        cv_extra.append((frame_at(n, away, O), label + " bracket"))
        return H2
    for side, sg in (("R", 1), ("L", -1)):
        W = w2w(side)
        # ailerons: torque-tube arm -> arch-foot bellcrank -> up the arch leg -> arch-top bellcrank
        #           -> spanwise through the rib lightening holes -> outboard bellcrank -> horn
        TT = np.array([900.0, sg * 60, -1030.0])
        Q1 = np.array([990.0, sg * 300, -1000.0])
        H = add_bc(Q1, Q1 - TT, [0, 0, 1], f"{side} aileron foot bellcrank")
        Q2 = np.array([H[0], H[1], -95.0])
        tgt3 = (W @ np.array([995.0, 2650.0, 5.0, 1.0]))[:3]
        b2 = unit(tgt3 - Q2)
        H2 = add_bc(Q2, [0, 0, 1], b2, f"{side} aileron arch-top bellcrank")
        Q3 = H2 + b2 * np.linalg.norm(tgt3 - Q2)
        b3 = W[:3, 0]  # aft, chordwise
        H3 = add_bc(Q3, b2, b3, f"{side} wing aileron outboard bellcrank")
        horn_hole = H3 + b3 * 50.0
        CONTROL_RODS.append(dict(name=f"Aileron pushrods {side}", color="#9b8cff", pts=[TT, Q1, H, Q2, H2, Q3, H3, horn_hole]))
        Hp = np.linalg.solve(W, np.array([*horn_hole, 1.0]))[:3]
        hn.instances.append((W @ mat_T(Hp[0], Hp[1], Hp[2]), f"{side} wing aileron horn"))
        # flaps: overhead lever -> centre torque tube arm -> spanwise -> flap bellcrank -> horn
        LV = np.array([850.0, -250, -140])
        TA = np.array([975.0, sg * 320, -60.0])
        tgt4 = (W @ np.array([985.0, 860.0, 0.0, 1.0]))[:3]
        b4 = unit(tgt4 - TA)
        H4 = add_bc(TA, [0, 0, 1], b4, f"{side} flap torque-tube arm")
        Q5 = H4 + b4 * np.linalg.norm(tgt4 - TA)
        H5 = add_bc(Q5, b4, W[:3, 0], f"{side} wing flap bellcrank")
        fh = H5 + W[:3, 0] * 50.0
        CONTROL_RODS.append(dict(name=f"Flap pushrods {side}", color="#7fd49a", pts=[H4, Q5, H5, fh]))
        Hp = np.linalg.solve(W, np.array([*fh, 1.0]))[:3]
        hn.instances.append((W @ mat_T(Hp[0], Hp[1], Hp[2]), f"{side} wing flap horn"))
    CONTROL_RODS.append(dict(name="Flap lever rod", color="#7fd49a", pts=[np.array([850.0, -250, -140]), np.array([975.0, -250, -60])]))
    # elevator double horn on the elevator torque tube, rudder horn at the foot of the rudder post
    hd = horn_part("K-HD", "Elevator horn, 4130 0.063in (double-ended, on the elevator torque tube)",
                   [(-20, -152), (20, -152), (20, 10), (-20, 10)], [(0, 0, 4.8), (0, -143, 4.8), (-15, -58, 20.0)])
    hd.instances.append((mat_T(4385, 0, -362), "elevator horn"))
    hr = Part("K-HR", "Rudder horn, 4130 0.063in (flat, cable each side)", "Fittings", "4130 steel 0.063in", 1.6, "", density=7.85e-6)
    hr.add([v3(-22, -92, 0), v3(22, -92, 0), v3(22, 92, 0), v3(-22, 92, 0)])
    for y in (-75, 75):
        hr.bolts3d.append((v3(0, y, 0), 4.8, "arm"))
    hr.bolts3d.append((v3(-8, 0, 0), 12.0, "post"))
    hr.instances.append((mat_T(4372, 0, -545), "rudder horn"))
    return [hn, hd, hr, bc], cv_extra



# ---------------------------------------------------- step 0: wing stands (the primer project)
STAND_TOP = 855.0          # floor to saddle top, mm (wing parked on top)
STAND_POS = [(2700.0, s * yy) for s in (1, -1) for yy in (1650.0, 3650.0)]
STAND_SIDE = [(-2600.0 + 1700.0 * k, 5600.0) for k in range(4)]   # where they stand during steps 0-1
STAND_FZ = -1515.0
BMW, BMD, LIP = 60.0, 35.0, 10.0   # beam web, flange depth, lip
LGD = 30.0                          # leg flange depth
XL = 650.0                          # leg web position


def _lipped_c(p, path, E, A, B, u0, u1):
    return extrude_path(p, path, E, A, B, O, u0, u1, root=2)


def stands():
    top = STAND_TOP
    bm = Part("S-BM", "Stand beam (lipped C, saddle on top)", "Stands", "6061-T6 0.020in", 1.0,
              "Primer part: two nested layers folded on the brake and riveted as one. Glue carpet on the web.")
    path = [(-BMW / 2 + LIP, top - BMD), (-BMW / 2, top - BMD), (-BMW / 2, top), (BMW / 2, top), (BMW / 2, top - BMD), (BMW / 2 - LIP, top - BMD)]
    ix = extrude_path(bm, [(b, a) for a, b in path], EX, EZ, EY, O, -750.0, 750.0, root=2)
    lg = Part("S-LG", "Stand leg (lipped C)", "Stands", "6061-T6 0.020in", 1.0, "")
    z0, z1 = 1.0, top - BMD
    lpath = [(XL - LGD, -BMW / 2 + LIP), (XL - LGD, -BMW / 2), (XL, -BMW / 2), (XL, BMW / 2), (XL - LGD, BMW / 2), (XL - LGD, BMW / 2 - LIP)]
    extrude_path(lg, lpath, EZ, EX, EY, O, z0, z1, root=2)
    ft = Part("S-FT", "Stand foot (lipped C, opens up)", "Stands", "6061-T6 0.020in", 1.0, "")
    fpath = [(XL - 60 + LIP, 35.0), (XL - 60, 35.0), (XL - 60, 0.0), (XL, 0.0), (XL, 35.0), (XL - LIP, 35.0)]
    extrude_path(ft, fpath, EY, EX, EZ, O, -300.0, 300.0, root=2)
    # top gusset: right triangle in the x-z plane on the outside of the leg/beam flanges, flanged hypotenuse
    gt = Part("S-GT", "Stand top gusset (flanged hypotenuse)", "Stands", "6061-T6 0.020in", 1.0, "One 90 degree flange stiffens the free edge.")
    yo = BMW / 2 + 0.6
    tri = [v3(XL, yo, top - 180), v3(XL, yo, top), v3(XL - 180, yo, top)]
    flanged_web(gt, tri, EY, [(2, 14.0, unit(v3(0, 1, 0)))], rivet_flanges=False)
    for k in range(5):
        gt.rivets3d.append(v3(XL - 15, yo, top - 165 + k * 28))
        gt.rivets3d.append(v3(XL - 165 + k * 28, yo, top - 17))
    # foot gusset: flat isosceles plate on the outside of the leg web and foot flange
    gf = Part("S-GF", "Stand foot gusset (flat)", "Stands", "6061-T6 0.020in", 1.0, "")
    xo = XL + 0.6
    gf.add([v3(xo, -190, 4), v3(xo, 190, 4), v3(xo, 22, 210), v3(xo, -22, 210)])
    for y in (-170, -130, -90, -50, 50, 90, 130, 170):
        gf.rivets3d.append(v3(xo, y, 18))
    for z in (60, 100, 140, 180):
        for y in (-12, 12):
            gf.rivets3d.append(v3(xo, y, z))
    MX = np.diag([-1.0, 1.0, 1.0, 1.0])
    MY = np.diag([1.0, -1.0, 1.0, 1.0])
    for k, (cx, cy) in enumerate(STAND_POS):
        W = mat_T(cx, cy, STAND_FZ)
        lab = f"stand {k}"
        bm.instances.append((W, lab))
        for mx in (np.eye(4), MX):
            lg.instances.append((W @ mx, lab)); ft.instances.append((W @ mx, lab)); gf.instances.append((W @ mx, lab))
            for my in (np.eye(4), MY):
                gt.instances.append((W @ mx @ my, lab))
    return [bm, lg, ft, gt, gf]


def center_skins():
    up = skin_upper(-hw, hw, "W-CU", "Centre-section upper skin")
    up.group = "Fuselage"
    lo = Part("W-CL", "Cabin roof (centre lower skin)", "Fuselage", "6061-T6 0.016in", tsl, "")
    lop = clip_x(LOo, xrs, 110)
    root = next(k for k in range(len(lop) - 1) if lop[k][0] <= xms <= lop[k + 1][0])
    extrude_path(lo, lop, EY, EX, EZ, O, -hw, hw, root=root)
    M = mat_Ry(P["incidence"])
    up.instances.append((M, ""))
    lo.instances.append((M, ""))
    # fixed trailing-edge closure between the flaps: the same wedge-and-bead fold as W-CS, aft of the rear spar
    x0, ch_ = xrs, P["chord"] - xrs
    zte = 0.5 * (WING.up(P["chord"]) + WING.lo(P["chord"]))
    fu = lambda x: WING.up(x0) + (zte - WING.up(x0)) * (x - x0) / ch_
    fl = lambda x: WING.lo(x0) + (zte - WING.lo(x0)) * (x - x0) / ch_
    te = control_surface("W-CT", "Centre-section trailing-edge fairing (fixed, between the flaps)", x0, ch_, fu, fl, 2 * hw, "Fuselage",
                         "Rivets to the rear-spar flanges. Closes the bay behind the centre-section rear spar.")
    te.instances.append((M @ mat_T(0, -hw, 0), ""))
    return up, lo, te


# ---------------------------------------------------- tailcone frustum
def tc_scale(x):
    return 1 + (P["tail_scale"] - 1) * (x - xce) / (xtail - xce)


def tc_axis(x):
    return P["axis0"] + (P["axis1"] - P["axis0"]) * (x - xce) / (xtail - xce)


def tc_pt(x, yz):
    s, a = tc_scale(x), tc_axis(x)
    return v3(x, s * yz[0], a + s * (yz[1] - P["axis0"]))


SEC = [(-hw, zt), (hw, zt), (hw, zb + ch), (hw - ch, zb), (-hw + ch, zb), (-hw, zb + ch)]  # top, R side, R chamfer, bottom, L chamfer, L side


def tc_face(x0, x1, k):
    a, b = SEC[k], SEC[(k + 1) % len(SEC)]
    return [tc_pt(x0, a), tc_pt(x0, b), tc_pt(x1, b), tc_pt(x1, a)]


def lateral_dir(yz, x0, x1):
    return unit(tc_pt(x1, yz) - tc_pt(x0, yz))


def tailcone_section(sx0, sx1, tag):
    parts = []
    # top panel with flanges lying on side faces
    p = Part(f"F-TT{tag}", f"Tailcone top, section {tag}", "Fuselage", "6061-T6 0.016in", 0.4, "")
    top = tc_face(sx0, sx1, 0)
    ti = p.add(top)
    for k in (5, 1):  # neighbouring side faces
        side = tc_face(sx0, sx1, k)
        # flange = strip of the side face 25 mm below the shared edge
        if k == 1:
            a0, a1 = top[1], top[2]
            d0 = unit(side[1] - side[0]); d1 = unit(side[2] - side[3])
        else:
            a0, a1 = top[0], top[3]
            d0 = unit(side[0] - side[1]); d1 = unit(side[3] - side[2])
        fl = [a0, a1, a1 + d1 * 25, a0 + d0 * 25]
        p.add(fl, parent=ti, hinge=(0, 1))
        for t_ in np.linspace(0.04, 0.96, max(int(np.linalg.norm(a1 - a0) / 45), 2)):
            p.rivets3d.append(a0 + (a1 - a0) * t_ + 0.5 * (d0 * 25 * (1 - t_) + d1 * 25 * t_))
    p.instances.append((np.eye(4), ""))
    parts.append(p)
    s = Part(f"F-TS{tag}", f"Tailcone side, section {tag} (x2, mirrored)", "Fuselage", "6061-T6 0.016in", 0.4, "")
    s.add(tc_face(sx0, sx1, 1))
    s.instances.append((np.eye(4), "R"))
    s.instances.append((MIRROR_Y, "L"))
    parts.append(s)
    b = Part(f"F-TB{tag}", f"Tailcone belly, section {tag}", "Fuselage", "6061-T6 0.020in", 0.5,
             "Bottom + both chamfers + side flanges from one sheet: converging creases, still developable (frustum).")
    bi = b.add(tc_face(sx0, sx1, 3))
    for k, sk in ((2, 1), (4, 5)):
        f = tc_face(sx0, sx1, k)
        if k == 2:
            ci = b.add(f, parent=bi, hinge=(1, 2))
            a0, a1 = f[0], f[3]
            d0 = unit(tc_pt(sx0, SEC[1]) - tc_pt(sx0, SEC[2])); d1 = unit(tc_pt(sx1, SEC[1]) - tc_pt(sx1, SEC[2]))
        else:
            ci = b.add(f, parent=bi, hinge=(0, 3))
            a0, a1 = f[1], f[2]
            d0 = unit(tc_pt(sx0, SEC[0]) - tc_pt(sx0, SEC[5])); d1 = unit(tc_pt(sx1, SEC[0]) - tc_pt(sx1, SEC[5]))
        fl = [a0, a1, a1 + d1 * 25, a0 + d0 * 25]
        b.add(fl, parent=ci, hinge=(0, 1))
        for t_ in np.linspace(0.04, 0.96, max(int(np.linalg.norm(a1 - a0) / 45), 2)):
            b.rivets3d.append(a0 + (a1 - a0) * t_ + 0.5 * (d0 * 25 * (1 - t_) + d1 * 25 * t_))
    b.instances.append((np.eye(4), ""))
    parts.append(b)
    return parts


def tc_bulkhead(x, pid, ring=True):
    p = Part(pid, f"Tailcone bulkhead x={x:.0f}" + ("" if ring else " (tail post)"), "Fuselage",
             "6061-T6 0.032in", 0.8, "Flanges follow the frustum taper (fold < 90 deg).")
    outer = [tc_pt(x, yz) for yz in SEC]
    x2 = min(x + 50, xtail + 50)
    holes = []
    if ring:
        s = tc_scale(x)
        inset = 0.80
        cy, cz = 0.0, tc_axis(x)
        holes = [[v3(x, cy + (q[1]) * inset, cz + (q[2] - cz) * inset) for q in outer]]
    fl = []
    for e in range(len(outer)):
        yz = SEC[e]
        d = lateral_dir(yz, x, x + 100) * (1 if ring else -1)
        fl.append((e, 18.0, d))
    flanged_web(p, outer, EX, fl, holes=holes, rivet_pitch=40)
    p.instances.append((np.eye(4), ""))
    return p


# =============================================================== TAIL
TAILF = Airfoil("0010", 1.0)


def sym(c):
    return (lambda x: TAILF.up(x / c) * c), (lambda x: TAILF.lo(x / c) * c)


def tail_surface(pid, name, c_fixed, c_total, span, t_sk=0.4):
    """Fixed surface (stab or fin) in its own frame: x aft from LE, y span, z thickness.
    Skin wraps spar-top -> LE -> spar-bottom in one sheet. Spar at x=c_fixed."""
    up, lo = sym(c_total)  # NACA 0010 over fixed + moving chord
    xs = [0, .01, .025, .05, .08, .12, .2, .35, .5, .7, .85, 1.0]
    upp = [(f * c_fixed, up(f * c_fixed)) for f in xs]
    lop = [(f * c_fixed, lo(f * c_fixed)) for f in xs]
    path = list(reversed(upp)) + lop[1:]
    kinds = ["roll" if 0.5 * (path[k][0] + path[k + 1][0]) < 0.12 * c_fixed else "crease" for k in range(len(path) - 1)]
    sk = Part(pid + "K", name + " skin (one wrap)", "Tail", "6061-T6 0.016in", t_sk, "")
    extrude_path(sk, path, EY, EX, EZ, O, 0, span, root=0, kinds=kinds)
    sp = Part(pid + "S", name + " spar", "Tail", "6061-T6 0.032in", 0.8, "")
    b = 25.0
    x0 = c_fixed
    spath = [(x0 - b, lo(x0 - b) + 0.4), (x0, lo(x0) + 0.4), (x0, up(x0) - 0.4), (x0 - b, up(x0 - b) - 0.4)]
    ix = extrude_path(sp, spath, EY, EX, EZ, O, 0, span, root=1)
    rivet_line(sp, ix[0]); rivet_line(sp, ix[2])
    rb = Part(pid + "R", name + " nose rib", "Tail", "6061-T6 0.020in", 0.5, "")
    up_in = [(f * c_fixed, up(f * c_fixed) - 0.4) for f in xs[:-1]] + [(x0 - 0.8, up(x0 - 0.8) - 0.4)]
    lo_in = [(f * c_fixed, lo(f * c_fixed) + 0.4) for f in xs[1:-1]] + [(x0 - 0.8, lo(x0 - 0.8) + 0.4)]
    poly = list(reversed(up_in)) + lo_in
    pts = [v3(x, 0, z) for x, z in poly]
    n = len(pts)
    fl = []
    for e in range(n):
        a, bb = pts[e], pts[(e + 1) % n]
        if np.linalg.norm(bb - a) < 6:
            continue
        if abs(bb[0] - a[0]) < 1e-6:
            fl.append((e, 20.0, EY))
        else:
            fl.append((e, 12.0, EY))
    flanged_web(rb, pts, EY, fl, rivet_pitch=40)
    return sk, sp, rb


def tail_parts():
    out = []
    c_st = P["x_stab_spar"] - P["x_stab_le"]
    span = P["stab_semispan"] - P["stab_root"]
    c_ht = c_st + 20 + P["elev_chord"]
    sk, sp, rb = tail_surface("T-H", "Stabiliser", c_st, c_ht, span)
    for part in (sk, sp):
        part.instances.append((mat_T(P["x_stab_le"], P["stab_root"], P["z_stab"]), "R"))
        part.instances.append((MIRROR_Y @ mat_T(P["x_stab_le"], P["stab_root"], P["z_stab"]), "L"))
    for yy in (0, span / 2, span):
        rb.instances.append((mat_T(P["x_stab_le"], P["stab_root"] + yy, P["z_stab"]), f"R {yy:.0f}"))
        rb.instances.append((MIRROR_Y @ mat_T(P["x_stab_le"], P["stab_root"] + yy, P["z_stab"]), f"L {yy:.0f}"))
    out += [sk, sp, rb]
    # elevator
    up, lo = sym(c_ht)
    ce = P["elev_chord"]
    zte = 0.0
    eu = lambda x: up(c_st) + (zte - up(c_st)) * (x - 0) / ce
    el = lambda x: lo(c_st) + (zte - lo(c_st)) * (x - 0) / ce
    espan = P["elev_span"][1] - P["elev_span"][0]
    es = control_surface("T-EK", "Elevator skin", 0.0, ce, eu, el, espan, "Tail", "Mass-balance horn at outboard end.")
    esp = cs_spar("T-ES", "Elevator spar", 0.0, eu, el, espan, "Tail")
    for part in (es, esp):
        part.instances.append((mat_T(P["x_elev"], P["elev_span"][0], P["z_stab"]), "R"))
        part.instances.append((MIRROR_Y @ mat_T(P["x_elev"], P["elev_span"][0], P["z_stab"]), "L"))
    out += [es, esp]
    # fin: build in stab frame then rotate so span -> +z
    c_fin = P["x_stab_spar"] - P["x_fin_le"]
    fspan = P["fin_top"] - P["fin_root"]
    c_vt = c_fin + 20 + P["rud_chord"]
    fk, fs, fr = tail_surface("T-V", "Fin", c_fin, c_vt, fspan)
    Rz = np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1.0]])  # y->z, z->-y
    Mfin = mat_T(P["x_fin_le"], 0, P["fin_root"]) @ Rz
    fk.instances.append((Mfin, ""))
    fs.instances.append((Mfin, ""))
    for yy in (0, fspan / 2, fspan):
        fr.instances.append((Mfin @ mat_T(0, yy, 0), f"{yy:.0f}"))
    for q in (fk, fs, fr):
        q.id = q.id.replace("T-V", "T-F")
    out += [fk, fs, fr]
    rspan = P["fin_top"] - P["rud_bot"]
    cr = P["rud_chord"]
    fu, _fl = sym(c_vt)
    ru = lambda x: fu(c_fin) + (0 - fu(c_fin)) * x / cr
    rl = lambda x: -ru(x)
    rk = control_surface("T-RK", "Rudder skin", 0.0, cr, ru, rl, rspan, "Tail", "Bottom extends between the elevator halves.")
    rsp = cs_spar("T-RS", "Rudder spar", 0.0, ru, rl, rspan, "Tail")
    Mr = mat_T(P["x_rud"], 0, P["rud_bot"]) @ Rz
    rk.instances.append((Mr, ""))
    rsp.instances.append((Mr, ""))
    out += [rk, rsp]
    return out


# =============================================================== assemble list
def build_all():
    PARTS.clear()
    ms, rs = spar_main(), spar_rear()
    for s in (ms, rs):
        for side in "RL":
            s.instances.append((wing_to_world(side), side))
    reg(ms, 2); reg(rs, 2)
    for r in wing_ribs():
        reg(r, 3)
    for i, (a, b) in enumerate(P["bays"]):
        su = skin_upper(a, b, f"W-SU{i+1}", f"Upper skin, bay {i+1}")
        sl = skin_lower(a, b, f"W-SL{i+1}", f"Lower skin, bay {i+1}")
        for s in (su, sl):
            for side in "RL":
                s.instances.append((wing_to_world(side), side))
        reg(sl, 5); reg(su, 6)
    sk, sp = wing_control_surfaces()
    reg(sp, 7); reg(sk, 7)
    reg(floor_tray(), 8); reg(firewall(), 8)
    reg(u_frame("F-FA", "Front arch (main-spar frame)", xms), 8)
    reg(u_frame("F-RA", "Rear arch (rear-spar frame)", xrs), 8)
    reg(cabin_side(), 9); reg(seat(), 9)
    xm = 0.5 * (xce + xtail)
    for tag, (a, b) in (("1", (xce, xm)), ("2", (xm, xtail))):
        for q in tailcone_section(a, b, tag):
            reg(q, 10)
    for i, x in enumerate([xce, xce + (xm - xce) / 2, xm, xm + (xtail - xm) / 2]):
        reg(tc_bulkhead(x, f"F-B{i+1}"), 10)
    reg(tc_bulkhead(xtail, "F-B5", ring=False), 10)
    for q in tail_parts():
        reg(q, 11)
    for q in cowl():
        reg(q, 14)
    fit = fittings()
    links, cv_extra = control_linkage()
    fit[0].instances += cv_extra
    for q in fit:
        reg(q, 16 if q.id == "K-JC" else 12)
    for q in links:
        reg(q, {"K-HN": 7, "K-HD": 11, "K-HR": 11, "K-BC": 17}[q.id])
    for q in stands():
        reg(q, 0)
    cu, cl, ct = center_skins()
    reg(cl, 15); reg(cu, 15); reg(ct, 17)
    reg(instrument_panel(), 17)
    reg(windscreen(), 18)
    for q in glazing():
        reg(q, 18)
    return PARTS
