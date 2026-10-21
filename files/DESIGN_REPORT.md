# OpenPlane: design report

OpenPlane is a single-seat, strut-braced, high-wing microlight. Every structural sheet part is a flat blank with straight folds. It was designed against the UK SSDR limits (300 kg MTOW landplane, stall at or below 35 kt CAS). It does **not** meet US FAR Part 103: its empty mass of 198 kg is over the 254 lb (115 kg) limit.

This is a conceptual design with first-pass calculations. It is not an approved or flight-tested design. See "Limits of this work" before cutting metal.

## Geometry
| | |
|---|---|
| Span / area / AR | 7.9 m / 11.06 m² / 5.64 |
| Chord | 1.4 m constant. NACA 4415, faceted aft of 15 % chord. 1.5° dihedral, 2° incidence |
| Length | 6.12 m |
| Flaps / ailerons | Four identical 1.79 m × 0.38 m surfaces. Flaps inboard, ailerons outboard |
| Tail | Stabiliser + elevator 2.6 m × 0.92 m (Vh 0.57). Fin + rudder ≈1.05 m² (Vv 0.044) |
| Engine | Half-VW, 37 hp (27.6 kW), 54 x 24 in fixed-pitch wood prop (model in prop/) |
| Undercarriage | Spring-leg taildragger on 21 x 8.00-6 tundra tyres (10-12 psi), 8 in pneumatic tailwheel |
| Fuel | 19 L header tank behind the firewall + 20 L aux tank high behind the seat. Both gravity-feed through a HEADER / AUX / OFF selector. No fuel in the wings |

## Mass
Sheet parts 100.9 kg (computed from the unfolded blanks). Empty 198 kg. MTOW 300.0 kg. Useful load 102 kg.

| Pilot | Fuel | Mass | CG (aft of LE) | % MAC |
|---|---|---|---|---|
| 55 kg | 0.0 kg | 252.5 kg | 450 mm | 32.1 % |
| 55 kg | 13.0 kg | 265.5 kg | 400 mm | 28.6 % |
| 75 kg | 0.0 kg | 272.5 kg | 445 mm | 31.8 % |
| 75 kg | 13.0 kg | 285.5 kg | 399 mm | 28.5 % |
| 100 kg | 0.0 kg | 297.5 kg | 439 mm | 31.4 % |
| 100 kg | 13.0 kg | 310.5 kg | 397 mm | 28.4 % |
| 55 kg | 0.0 kg + 5 kg baggage | 257.5 kg | 458 mm | 32.7 % |
| 55 kg | 27.4 kg (14.4 kg in aux) | 279.9 kg | 424 mm | 30.3 % |
| 75 kg | 27.4 kg (14.4 kg in aux) | 299.9 kg | 421 mm | 30.1 % |
| 100 kg | 2.5 kg (0.0 kg in aux) | 300.0 kg | 431 mm | 30.8 % |

Neutral point (stick-fixed estimate): 38.6 % MAC. CG range 28.4–32.7 % MAC gives a static margin of 5.9–10.2 %.

## Aerodynamics and performance (ISA sea level, MTOW)
| | |
|---|---|
| CLmax clean / flaps 20° (assumed) | 1.30 / 1.50 |
| Stall clean / flaps | 35.5 kt / **33.1 kt** (SSDR limit 35 kt) |
| Equivalent flat-plate drag, CD0 | 0.58 m², 0.0524 (component build-up) |
| e, L/Dmax | 0.75, 8.0 |
| Static thrust (momentum theory × 0.70) | 1106 N |
| Best climb | 450 ft/min at 42.8 kt |
| Vmax level / 75 % cruise | 67.2 kt / 58.7 kt |
| Take-off ground roll, flaps 20 | 110 m hard, 120 m short grass |
| Landing roll (light braking) | 90 m |
| Fuel 8.7 L/h at 75 %, header tank only | ~2.1 h, ~190 km still air, no reserve |
| Both tanks (only with a light pilot, see below) | ~4.3 h, ~400 km still air, no reserve |

**What limits fuel is the 300 kg MTOW, not tank size.** Empty mass is 198 kg, so pilot + fuel + baggage must not exceed 102 kg. A 75 kg pilot can fill both tanks. A 100 kg pilot can carry only about 3 L, which is not a practical flight. Ways to claw back mass: fit 6.00-6 tyres instead of the tundras (about 4 kg), leave the aux tank out (2.4 kg), or a lighter seat and instruments. Wing tanks would add volume, not payload.

## Propeller
54 x 24 in two-blade, right-hand tractor (clockwise from the seat), Clark-Y sections with the flat face aft, about 10.7 deg at 75 % radius. It is an all-rounder pitch for a 37 hp direct-drive half-VW at 35-70 kt. Files are in prop/ (STL, OBJ, station table, notes). Carve it in laminated hardwood; do not fly a printed one. Treat the pitch as a starting point and trim it after a ground run (see prop/PROP_README.txt).

## Structure (first pass)
Load factors +4.0 / -2.0 (BCAR Section S microlight values), ultimate factor 1.5. Va 71.1 kt, Vne 75.0 kt, Vd 85.0 kt. Gust load factors 3.37 at Vc and 2.72 at Vd are both below the +4 manoeuvre case, so manoeuvre loads govern.

Spanwise lift is a Schrenk distribution. Each wing is pinned at the cabin arch and supported by a V-strut at the 2.4 m rib. The chordwise split between spars uses NACA 4415 centre of pressure: the main spar takes 0.95 of the load at +4 g and 1.64 of it at −2 g at Vd (nose-down torsion). The rear spar takes up to 0.64 in reverse.

| Item | Ultimate stress / load | Allowable | Margin |
|---|---|---|---|
| Main spar, lipped C, 3 x 0.020 in laminated, 209 mm deep: bending + strut compression | 78 MPa | 241 MPa (cap crippling) | +2.1 |
| Main spar web shear at limit load (stiffened at 300 mm by rib flanges) | 7.4 MPa | 23.8 MPa (elastic shear buckling) | +2.23 |
| Rear spar, lipped C, 2 x 0.020 in laminated, 119 mm deep | 73 MPa | 159 MPa | +1.18 |
| Lift strut 1.75 × 0.065 in 6061-T6, 2587 mm at 22.0°, tension | 15506 N | 0.85 Ftu | +2.53 |
| Same strut, −2 g compression, jury strut at mid-length | 13384 N | Euler 20677 N | +0.54 |

One sheet stock: every aluminium part is cut from 6061-T6 0.020 in (0.51 mm). Thicker parts are 2 or 3 nested layers of that sheet, each folded separately and riveted as one. The laminates are checked as solid sections, which assumes the rivet pitch is close enough to stop the layers buckling separately. Exceptions to one stock: the stainless firewall (fire), the 4130 fittings (bolted joints) and the polycarbonate glazing.

Why the lip matters: a free 40 mm flange of 1.5 mm sheet buckles at about 37 MPa (k = 0.43). Fold a 12 mm lip on it and that edge is supported (k ≈ 4). The cap then reaches yield before it buckles. One extra crease gives about nine times the buckling stress.

## Materials and stock (bounding-box shelf nest, ~60 % utilisation)
| Material | Stock | Sheets |
|---|---|---|
| 6061-T6 0.020in | 12 ft x 4 ft sheet (spar strips) | 3 |
| 6061-T6 0.020in | 8 ft x 4 ft sheet | 27 |
| Stainless 304 0.5 mm | 2000 x 1000 sheet | 1 |
| 4130 steel 0.063in | 4130 sheet 4 ft x 2 ft | 2 |
| Polycarbonate 2 mm | 2050 x 1250 sheet | 2 |

## Rough cost (GBP, UK retail incl. VAT; sheet priced from the nested stock at an aero retailer's list price)
| Item | £ |
|---|---|
| Folded blanks | £11,460 |
| 4130 fittings | £320 |
| Wing stands (step 0) | £1,080 |
| Fasteners | £1,270 |
| Tube, bar and cable | £1,000 |
| Bought assemblies | £11,230 |
| Consumables | £390 |
| Tools | £1,460 |
| **Total** | **£28,200** |

## Limits of this work
- Not analysed: ribs, skin buckling, fittings and bolts, landing gear, tail loads, control circuits, the engine mount, or flutter. Every control surface must be mass-balanced. Flutter needs a proper check or a ground vibration test.
- CLmax, CD0, propeller efficiency and the neutral point are handbook estimates. Measure stall speed and CG limits in flight testing.
- The flat patterns come from the same 3D model as the drawings. Bend deduction assumes R = 1.6 t and K = 0.40 for 6061-T6. Fold lines sit on the web's outer mould line, and the deduction is taken from the flange. Cut and fold one rib and one spar offcut before batch-cutting.
- Rib-to-spar holes are coordinated in the DXFs. Skins carry no holes: back-drill them through the substructure at assembly, which is standard kit practice.
- SSDR aircraft need no airworthiness approval in the UK, but you take full responsibility. Before any flight, do at least a sandbag proof load to +4 g limit, a full control-system check and taxi tests. Get an experienced inspector (BMAA or LAA) to look at it.
