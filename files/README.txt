OpenPlane cut files
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
