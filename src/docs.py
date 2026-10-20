import json, os, zipfile, shutil
from geom import OUT_DIR, SITE_DIR, CFG
OUT = str(OUT_DIR)
m = json.load(open(f"{OUT}/ori1_model.json"))
H, cost, bom = m["H"], m["cost"], m["bom"]

cg = "\n".join(f"| {c['pilot']} kg | {c['fuel']} kg{' (' + str(c['aux']) + ' kg in aux)' if 'aux' in c else ''}{' + 5 kg baggage' if c.get('bag') else ''} | {c['mass']} kg | {c['x']} mm | {c['mac']} % |" for c in H["cg_cases"])
sheets = "\n".join(f"| {s['material']} | {s['stock']} | {s['n']} |" for s in H["sheets"])
costs = "\n".join(f"| {n} | £{c:,.0f} |" for n, c in cost)
report = f"""# OpenPlane: design report

OpenPlane is a single-seat, strut-braced, high-wing microlight. Every structural sheet part is a flat blank with straight folds. It was designed against the UK SSDR limits (300 kg MTOW landplane, stall at or below 35 kt CAS). It does **not** meet US FAR Part 103: its empty mass of {H['empty']} kg is over the 254 lb (115 kg) limit.

This is a conceptual design with first-pass calculations. It is not an approved or flight-tested design. See "Limits of this work" before cutting metal.

## Geometry
| | |
|---|---|
| Span / area / AR | {H['span']} m / {H['S']} m² / {H['AR']} |
| Chord | {H['chord']} m constant. NACA 4415, faceted aft of 15 % chord. 1.5° dihedral, 2° incidence |
| Length | {H['length']} m |
| Flaps / ailerons | Four identical 1.79 m × 0.38 m surfaces. Flaps inboard, ailerons outboard |
| Tail | Stabiliser + elevator 2.6 m × 0.92 m (Vh {H['Vh']}). Fin + rudder ≈1.05 m² (Vv {H['Vv']}) |
| Engine | Half-VW, 37 hp (27.6 kW), 54 x 24 in fixed-pitch wood prop (model in prop/) |
| Undercarriage | Spring-leg taildragger on 21 x 8.00-6 tundra tyres (10-12 psi), 8 in pneumatic tailwheel |
| Fuel | 19 L header tank behind the firewall + 20 L aux tank high behind the seat. Both gravity-feed through a HEADER / AUX / OFF selector. No fuel in the wings |

## Mass
Sheet parts {H['sheet']} kg (computed from the unfolded blanks). Empty {H['empty']} kg. MTOW {H['mtow']} kg. Useful load {H['useful']} kg.

| Pilot | Fuel | Mass | CG (aft of LE) | % MAC |
|---|---|---|---|---|
{cg}

Neutral point (stick-fixed estimate): {H['hn']} % MAC. CG range {H['cg_fwd']}–{H['cg_aft']} % MAC gives a static margin of {H['SM_min']}–{H['SM_max']} %.

## Aerodynamics and performance (ISA sea level, MTOW)
| | |
|---|---|
| CLmax clean / flaps 20° (assumed) | 1.30 / 1.50 |
| Stall clean / flaps | {H['Vs']} kt / **{H['Vs0']} kt** (SSDR limit 35 kt) |
| Equivalent flat-plate drag, CD0 | {H['f']} m², {H['CD0']} (component build-up) |
| e, L/Dmax | 0.75, {H['LD']} |
| Static thrust (momentum theory × 0.70) | {H['T0']} N |
| Best climb | {H['roc']:.0f} ft/min at {H['Vy']} kt |
| Vmax level / 75 % cruise | {H['Vmax']} kt / {H['Vcr']} kt |
| Take-off ground roll, flaps 20 | {H['to_hard']:.0f} m hard, {H['to_grass']:.0f} m short grass |
| Landing roll (light braking) | {H['land']:.0f} m |
| Fuel 8.7 L/h at 75 %, header tank only | ~{H['endurance']} h, ~{H['range']:.0f} km still air, no reserve |
| Both tanks (only with a light pilot, see below) | ~{H['endurance_both']} h, ~{H['range_both']:.0f} km still air, no reserve |

**What limits fuel is the 300 kg MTOW, not tank size.** Empty mass is {H['empty']} kg, so pilot + fuel + baggage must not exceed {H['useful']} kg. A 75 kg pilot can fill both tanks. A 100 kg pilot can carry only about {max(0, H['useful'] - 100) / 0.72:.0f} L, which is not a practical flight. Ways to claw back mass: fit 6.00-6 tyres instead of the tundras (about 4 kg), leave the aux tank out (2.4 kg), or a lighter seat and instruments. Wing tanks would add volume, not payload.

## Propeller
54 x 24 in two-blade, right-hand tractor (clockwise from the seat), Clark-Y sections with the flat face aft, about {H['prop_beta75']} deg at 75 % radius. It is an all-rounder pitch for a 37 hp direct-drive half-VW at 35-70 kt. Files are in prop/ (STL, OBJ, station table, notes). Carve it in laminated hardwood; do not fly a printed one. Treat the pitch as a starting point and trim it after a ground run (see prop/PROP_README.txt).

## Structure (first pass)
Load factors +{H['n_pos']} / {H['n_neg']} (BCAR Section S microlight values), ultimate factor 1.5. Va {H['Va']} kt, Vne {H['Vne']} kt, Vd {H['Vd']} kt. Gust load factors {H['gust_c']} at Vc and {H['gust_d']} at Vd are both below the +4 manoeuvre case, so manoeuvre loads govern.

Spanwise lift is a Schrenk distribution. Each wing is pinned at the cabin arch and supported by a V-strut at the 2.4 m rib. The chordwise split between spars uses NACA 4415 centre of pressure: the main spar takes 0.95 of the load at +4 g and 1.64 of it at −2 g at Vd (nose-down torsion). The rear spar takes up to 0.64 in reverse.

| Item | Ultimate stress / load | Allowable | Margin |
|---|---|---|---|
| Main spar, lipped C, 3 x 0.020 in laminated, {H['h_ms']} mm deep: bending + strut compression | {H['ms_sig']} MPa | {H['ms_cc']} MPa (cap crippling) | +{H['ms_MS']} |
| Main spar web shear at limit load (stiffened at 300 mm by rib flanges) | {H['ms_tau']} MPa | {H['ms_taucr']} MPa (elastic shear buckling) | +{H['ms_MSs']} |
| Rear spar, lipped C, 2 x 0.020 in laminated, {H['h_rs']} mm deep | {H['rs_sig']} MPa | {H['rs_cc']} MPa | +{H['rs_MS']} |
| Lift strut 1.75 × 0.065 in 6061-T6, {H['strut_L']} mm at {H['strut_phi']}°, tension | {H['strut_T']} N | 0.85 Ftu | +{H['strut_MSt']} |
| Same strut, −2 g compression, jury strut at mid-length | {H['strut_C']} N | Euler {H['strut_Pcr']} N | +{H['strut_MSb']} |

One sheet stock: every aluminium part is cut from 6061-T6 0.020 in (0.51 mm). Thicker parts are 2 or 3 nested layers of that sheet, each folded separately and riveted as one. The laminates are checked as solid sections, which assumes the rivet pitch is close enough to stop the layers buckling separately. Exceptions to one stock: the stainless firewall (fire), the 4130 fittings (bolted joints) and the polycarbonate glazing.

Why the lip matters: a free 40 mm flange of 1.5 mm sheet buckles at about 37 MPa (k = 0.43). Fold a 12 mm lip on it and that edge is supported (k ≈ 4). The cap then reaches yield before it buckles. One extra crease gives about nine times the buckling stress.

## Materials and stock (bounding-box shelf nest, ~60 % utilisation)
| Material | Stock | Sheets |
|---|---|---|
{sheets}

## Rough cost (GBP, UK retail incl. VAT; sheet priced from the nested stock at an aero retailer's list price)
| Item | £ |
|---|---|
{costs}
| **Total** | **£{H['cost_total']:,.0f}** |

## Limits of this work
- Not analysed: ribs, skin buckling, fittings and bolts, landing gear, tail loads, control circuits, the engine mount, or flutter. Every control surface must be mass-balanced. Flutter needs a proper check or a ground vibration test.
- CLmax, CD0, propeller efficiency and the neutral point are handbook estimates. Measure stall speed and CG limits in flight testing.
- The flat patterns come from the same 3D model as the drawings. Bend deduction assumes R = 1.6 t and K = 0.40 for 6061-T6. Fold lines sit on the web's outer mould line, and the deduction is taken from the flange. Cut and fold one rib and one spar offcut before batch-cutting.
- Rib-to-spar holes are coordinated in the DXFs. Skins carry no holes: back-drill them through the substructure at assembly, which is standard kit practice.
- SSDR aircraft need no airworthiness approval in the UK, but you take full responsibility. Before any flight, do at least a sandbag proof load to +4 g limit, a full control-system check and taxi tests. Get an experienced inspector (BMAA or LAA) to look at it.
"""
open(f"{OUT}/DESIGN_REPORT.md", "w").write(report)

readme = f"""OpenPlane cut files
================
dxf/parts/   one DXF per unique blank, 1:1 in mm. S- parts are the four wing stands (step 0, the practice project). Layers:
             CUT       outline and cut-outs (includes corner relief holes)
             VALLEY    fold toward the marked face (red, dashed)
             MOUNTAIN  fold away from the marked face (blue, dash-dot)
             ROLL      rolled zone boundary lines (leading edges): roll, don't crease
             HOLE      2.5 mm (#40) pilot holes; final-drill 3.3 mm (#30) at assembly for 1/8 in blind rivets
                       (bolt holes on 4130 fittings are on CUT, 0.5 mm undersize: ream to size)
             ETCH      part ID, fold angle labels (V 90 = valley 90 deg). Do not cut.
dxf/nests/   the same blanks shelf-nested onto stock sheets (one file per sheet)
svg/         crease-pattern drawings of every blank (origami-book convention)
BOM.csv      full bill of materials: folded blanks, 4130 fittings, rivets by grip, AN hardware, tube/bar/cable cut lengths,
             bought assemblies, consumables, tools, stock sheets, rough cost
HARDWARE_BY_STEP.csv  every fastener by build step with where it goes
LICENSE.txt  CC0 1.0 Universal public-domain dedication
prop/        54 x 24 in propeller: STL + OBJ solid, station table (CSV), carving and trimming notes
DESIGN_REPORT.md  mass, balance, aero, performance, structure, limits
ori1_model.json   the folded-geometry model used by the web page (panels, hinges, angles, instances)

Fold radius: 6061-T6 normal radius from Wanttaja Table 5-2 (about 1.6 t). Do not go tighter.
Quantities in the DXF titles are per aircraft. Mirror pairs are the same blank, flipped.
This is a concept design. Read DESIGN_REPORT.md "Limits of this work".
"""
open(f"{OUT}/README.txt", "w").write(readme)
open(f"{OUT}/LICENSE.txt", "w").write("""OpenPlane: CC0 1.0 Universal (public domain dedication)

To the extent possible under law, the authors have waived all copyright and related or
neighbouring rights to OpenPlane: the cut files (DXF), crease patterns (SVG), the 3D models
(including the propeller), the bill of materials, the design report and the web page.
You may copy, modify, distribute, build and sell it, even commercially, without asking permission.

Full legal text: https://creativecommons.org/publicdomain/zero/1.0/legalcode

No warranty. This is a concept design, not an approved or flight-tested aircraft.
Nothing in it is airworthiness advice. Whoever builds or flies it is responsible for doing so safely
and lawfully under their own country's rules.
""")

zp = str(SITE_DIR / "OpenPlane-files.zip")
os.makedirs(SITE_DIR, exist_ok=True)
with zipfile.ZipFile(zp, "w", zipfile.ZIP_DEFLATED) as z:
    for root, _, files in os.walk(OUT):
        for f in files:
            if f in ("preview.html", "nests.json"):
                continue
            p = os.path.join(root, f)
            z.write(p, "OpenPlane/" + os.path.relpath(p, OUT))
print(os.path.getsize(zp))
