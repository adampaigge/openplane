"""OpenPlane design calculations: mass & balance, aerodynamics, performance,
stability, and first-pass structural sizing. Everything is printed and
returned as a dict so the web page and the report quote the same numbers."""
import math
import numpy as np
from geom import P, WING, newell, poly_area3, CFG
LD_ = CFG["loading"]
from parts import build_all

g, rho = 9.80665, 1.225
E_AL, NU = 68.9e3, 0.33            # MPa
FTU, FTY, FCY, FSU = 290.0, 241.0, 241.0, 186.0   # 6061-T6 sheet, MPa (MMPDS-typical)
KT = 0.514444


def plate(k, t, b):
    return k * math.pi**2 * E_AL / (12 * (1 - NU**2)) * (t / b) ** 2


def part_centroid_world(p, M):
    a_tot, c = 0.0, np.zeros(3)
    for f in p.faces:
        a = poly_area3(f["pts"])
        cen = np.mean(f["pts"], axis=0)
        a_tot += a
        c += a * cen
    c /= a_tot
    w = M @ np.array([*c, 1.0])
    return w[:3]


def run(verbose=True):
    allparts = build_all()
    parts = [q for q in allparts if q.group != "Stands"]
    out = {}
    # ------------------------------------------------------------- masses
    items = []  # (name, kg, x_mm, z_mm, category)
    sheet = 0.0
    rivets_flange = 0
    for p in parts:
        m = p.mass()
        for (M, lab) in p.instances:
            x, y, z = part_centroid_world(p, M)
            items.append((p.id, m, x, z, "sheet"))
            sheet += m
            rivets_flange += len(p.rivets3d)
    # skin rivets (back-drilled through substructure) are already counted on
    # the flanges; add spar-cap lines and control-surface/spar lines (pitch 40)
    rivets = rivets_flange
    out["rivets"] = rivets

    # struts (sized below; geometry here)
    ys_panel = 2400.0
    yS = P["cabin_half"] + ys_panel * math.cos(math.radians(P["dihedral"]))
    zS = ys_panel * math.sin(math.radians(P["dihedral"])) - 45.0
    y_lo, z_lo = P["cabin_half"], -950.0
    Ls = math.hypot(yS - y_lo, zS - z_lo)
    phi = math.atan2(zS - z_lo, yS - y_lo)
    strut = dict(od=44.45, t=1.65)   # 1.75 x 0.065 in 6061-T6 round, clip-on fairing
    A_st = math.pi / 4 * (strut["od"] ** 2 - (strut["od"] - 2 * strut["t"]) ** 2)
    m_strut = 4 * Ls * A_st * 2.70e-6 + 4 * 0.35  # tubes + fairings
    m_jury = 4 * 0.45 * 0.6
    bought = [
        ("Half-VW engine, dressed (carb, mag, exhaust)", 40.0, -1160, -550),
        ("Propeller 54 in wood + hub", 2.5, -1410, -550),
        ("Engine mount, 4130 welded", 2.5, -880, -550),
        ("Main gear: 2 spring legs + axles", 5.0, -60, -1250),
        ("Wheels, 21 x 8.00-6 tundra tyres, drum brakes x2", 9.0, -110, -1248),
        ("Tailwheel assembly, 8 in pneumatic", 1.8, 4450, -650),
        ("Lift struts x4 + fairings", m_strut, 720, -470),
        ("Jury struts x4", m_jury, 720, -250),
        ("AN bolts, nuts, washers, camlocks, screws", 1.6, 700, -300),
        ("Piano hinges (wing + tail)", 1.6, 2000, -150),
        ("Mass balance weights", 1.6, 2600, -200),
        ("Controls: stick, torque tube, cables, pushrods", 5.0, 900, -800),
        ("Seat cushion + 4-pt harness", 2.5, 510, -880),
        ("Fuel tank 19 L (rotomoulded) + lines, valve, gascolator", 2.8, -570, -465),
        ("Aux tank 20 L behind the seat + selector valve + line", 2.4, 865, -300),
        ("Instruments (ASI, alt, tach, CHT, compass)", 1.5, -440, -500),
        ("Primer / corrosion protection", 2.0, 1300, -400),
        ("Battery 12 V LiFePO4 + box (under seat)", 1.4, 450, -1000),
        ("Radio (panel-mount) + antenna + coax", 0.9, 200, -400),
        ("Wiring, switches, fuses, P-lead", 0.7, -200, -600),
        ("Pitot-static: pitot tube, static port, tubing", 0.4, 0, -300),
        ("Fire extinguisher 1 kg + PLB", 1.3, 500, -950),
        ("Rivets (%d x 0.35 g)" % rivets, rivets * 0.35e-3, 1200, -300),
    ]
    for (n, m, x, z) in bought:
        items.append((n, m, x, z, "bought"))
    empty = sum(i[1] for i in items)
    xe = sum(i[1] * i[2] for i in items) / empty
    ze = sum(i[1] * i[3] for i in items) / empty
    out["mass_sheet"] = sheet
    out["mass_empty"] = empty
    out["cg_empty_x"] = xe
    out["cg_empty_z"] = ze
    out["bought"] = [(n, round(m, 2)) for (n, m, x, z) in bought]
    groups = {}
    for p in parts:
        groups.setdefault(p.group, 0.0)
        groups[p.group] += p.mass() * len(p.instances)
    out["mass_groups"] = groups

    # ------------------------------------------------------------- loading
    fuel_l = float(LD_["header_tank_l"])
    fuel_kg = fuel_l * LD_["fuel_density"]
    pilot_x, pilot_z = float(LD_["pilot_x"]), float(LD_["pilot_z"])
    fuel_x = float(LD_["header_tank_x"])
    MTOW = float(LD_["mtow"])
    c = P["chord"]
    cases = []
    for pilot in LD_["pilots"]:
        for fuel in (0.0, fuel_kg):
            m = empty + pilot + fuel
            x = (empty * xe + pilot * pilot_x + fuel * fuel_x) / m
            cases.append(dict(pilot=pilot, fuel=round(fuel, 1), mass=round(m, 1), x=round(x), mac=round(100 * x / c, 1)))
    # worst aft: light pilot, no fuel, 5 kg in the baggage locker behind the seat
    m = empty + 55 + 5
    x = (empty * xe + 55 * pilot_x + 5 * 850.0) / m
    cases.append(dict(pilot=55, fuel=0.0, mass=round(m, 1), x=round(x), mac=round(100 * x / c, 1), bag=5))
    # aux tank behind the seat (20 L at x 865): fuel load limited by MTOW, not tank size
    aux_l, aux_x = float(LD_["aux_tank_l"]), float(LD_["aux_tank_x"])
    for pilot in LD_["pilots"]:
        room = MTOW - empty - pilot
        f_main = min(fuel_kg, max(room, 0)); f_aux = min(aux_l * 0.72, max(room - f_main, 0))
        m = empty + pilot + f_main + f_aux
        x = (empty * xe + pilot * pilot_x + f_main * fuel_x + f_aux * aux_x) / m
        cases.append(dict(pilot=pilot, fuel=round(f_main + f_aux, 1), aux=round(f_aux, 1), mass=round(m, 1), x=round(x), mac=round(100 * x / c, 1)))
    out["aux_l"] = aux_l
    out["cg_cases"] = cases
    out["MTOW"] = MTOW
    out["useful"] = MTOW - empty

    # ------------------------------------------------------------- aero
    b = 2 * P["panel"] + 2 * P["cabin_half"]
    S = b * c / 1e6
    AR = (b / 1000) ** 2 / S
    W = MTOW * g
    CLmax, CLmax_f = 1.30, 1.50
    Vs = math.sqrt(2 * W / (rho * S * CLmax))
    Vs0 = math.sqrt(2 * W / (rho * S * CLmax_f))
    # drag build-up (equivalent flat-plate areas, m^2)
    Sexp = S - 0.7 * c / 1000
    f_items = {
        "wing (Cf .0055, FF 1.30, x1.1 rivets/facets/gaps)": 0.0055 * 2.05 * Sexp * 1.30 * 1.10,
        "tail surfaces": 0.0055 * 2.05 * (2.34 + 1.05) * 1.15 * 1.10,
        "fuselage box + tailcone (FF 1.5)": 0.0065 * 12.4 * 1.5,
        "main gear, unfaired 21 in tundra tyres": 0.075,
        "tailwheel": 0.005,
        "lift struts with fairings, jury struts": 4 * 0.0445 * Ls / 1000 * 0.12 + 0.01,
        "engine cooling + exposed cylinders": 0.05,
    }
    f_sum = sum(f_items.values()) * 1.10  # +10 % interference/leakage
    CD0 = f_sum / S
    e = 0.75
    k = 1 / (math.pi * e * AR)
    LDmax = 0.5 / math.sqrt(CD0 * k)
    out.update(span=b, S=S, AR=AR, CLmax=CLmax, CLmax_f=CLmax_f, Vs=Vs, Vs0=Vs0, f=f_sum, f_items=f_items,
               CD0=CD0, e=e, LDmax=LDmax, WS=W / S)

    # ------------------------------------------------------------- performance
    Pmax = 37 * 745.7
    D = 1.372
    Adisk = math.pi * D**2 / 4
    eta_fac = 0.70

    def thrust(V, Pw):
        # actuator disc with a 0.70 loss factor (fixed-pitch wood prop)
        T = 800.0
        for _ in range(200):
            Vi = max(V, 0.5)
            # ideal: P_ideal = T*(V + vi), vi from momentum
            vi = 0.5 * (-V + math.sqrt(V * V + 2 * T / (rho * Adisk)))
            Tn = eta_fac * Pw / (V + vi) if V + vi > 0 else T
            T = 0.5 * T + 0.5 * Tn
        return T

    def drag(V, n=1.0):
        q = 0.5 * rho * V * V
        CL = n * W / (q * S)
        return q * S * (CD0 + k * CL * CL)

    Vs_grid = np.linspace(Vs * 1.05, 45, 400)
    roc = [(thrust(V, Pmax) * V - drag(V) * V) / W for V in Vs_grid]
    i = int(np.argmax(roc))
    Vy, ROC = Vs_grid[i], roc[i]
    vmax = [V for V in Vs_grid if thrust(V, Pmax) >= drag(V)]
    Vmax = max(vmax) if vmax else float("nan")
    vcr = [V for V in Vs_grid if thrust(V, 0.75 * Pmax) >= drag(V)]
    Vcr = max(vcr) if vcr else float("nan")
    T0 = thrust(0.0, Pmax)
    # take-off (grass mu .08, hard .04), flaps 20
    def to_roll(mu):
        VLO = 1.2 * Vs0
        V7 = 0.7 * VLO
        q = 0.5 * rho * V7**2
        CLg = 0.6
        L = q * S * CLg
        Dg = q * S * (CD0 + 0.02 + k * CLg**2)
        a = (thrust(V7, Pmax) - Dg - mu * (W - L)) / MTOW
        return VLO**2 / (2 * a)
    s_hard, s_grass = to_roll(0.04), to_roll(0.08)
    # landing roll, light braking mu .25 from 1.15 Vs0
    Vtd = 1.15 * Vs0
    s_land = Vtd**2 / (2 * 0.25 * g * 0.85)
    Vbg = math.sqrt(2 * W / (rho * S) * math.sqrt(k / CD0))
    sink_min = min(drag(V) * V / W for V in Vs_grid)
    fuel_flow = 0.75 * 37 * 0.50 * 0.4536 / 0.72  # L/h, BSFC 0.50 lb/hp/h
    out.update(Pmax_kW=Pmax / 1000, T0=T0, Vy=Vy, ROC=ROC, Vmax=Vmax, Vcr=Vcr, s_to_hard=s_hard,
               s_to_grass=s_grass, s_land=s_land, Vbg=Vbg, sink_min=sink_min, fuel_flow=fuel_flow,
               endurance=fuel_l / fuel_flow, range_km=fuel_l / fuel_flow * Vcr * 3.6 * 0.85,
               endurance_both=(fuel_l + LD_["aux_tank_l"]) / fuel_flow, range_both=(fuel_l + LD_["aux_tank_l"]) / fuel_flow * Vcr * 3.6 * 0.85)

    # ------------------------------------------------------------- stability
    aw = 2 * math.pi * AR / (2 + math.sqrt(AR**2 + 4))
    Sh = 2 * P["stab_semispan"] * (P["x_stab_spar"] - P["x_stab_le"] + 20 + P["elev_chord"]) / 1e6
    ch_t = (P["x_stab_spar"] - P["x_stab_le"] + 20 + P["elev_chord"])
    ARt = (2 * P["stab_semispan"] / 1000) ** 2 / Sh
    at = 2 * math.pi * ARt / (2 + math.sqrt(ARt**2 + 4))
    x_ac_w = 0.25 * c
    x_ac_t = P["x_stab_le"] + 0.25 * ch_t
    lt = (x_ac_t - x_ac_w) / 1000
    Vh = Sh * lt / (S * c / 1000)
    deda = 2 * aw / (math.pi * AR)
    eta_t = 0.9
    d_tail = eta_t * Vh * (at / aw) * (1 - deda)
    Lf = (P["x_tail"] + 1410) / 1000
    Kf = 0.011
    CMa_f = Kf * 0.7**2 * Lf / (c / 1000 * S) * 180 / math.pi
    d_fus = CMa_f / aw
    d_prop = 0.025
    hn = 0.25 + d_tail - d_fus - d_prop
    c_rud = P["rud_chord"]
    Sv = ((P["x_stab_spar"] - P["x_fin_le"]) * (P["fin_top"] - P["fin_root"]) + c_rud * (P["fin_top"] - P["rud_bot"])) / 1e6
    x_ac_v = P["x_fin_le"] + 0.25 * (P["x_stab_spar"] - P["x_fin_le"] + 20 + c_rud)
    Vv = Sv * (x_ac_v - x_ac_w) / 1000 / (S * b / 1000)
    fwd = min(cc["mac"] for cc in cases) / 100
    aft = max(cc["mac"] for cc in cases) / 100
    out.update(aw=aw, at=at, Sh=Sh, ARt=ARt, lt=lt, Vh=Vh, deda=deda, d_tail=d_tail, d_fus=d_fus, d_prop=d_prop,
               hn=hn, Sv=Sv, Vv=Vv, cg_fwd=fwd, cg_aft=aft, SM_min=hn - aft, SM_max=hn - fwd)

    # ------------------------------------------------------------- structure
    n_pos, n_neg, FoS = 4.0, -2.0, 1.5
    Va = Vs * math.sqrt(n_pos)
    Vd = 85 * KT
    Vne = 75 * KT
    # gust (CS-VLA style, 15.24 m/s at Vc, 7.62 at Vd)
    mu_g = 2 * (W / S) / (rho * c / 1000 * aw * g)
    Kg = 0.88 * mu_g / (5.3 + mu_g)
    Vc = max(Vcr, 30 * KT)
    n_gust_c = 1 + rho * aw * Vc * 15.24 * Kg / (2 * W / S)
    n_gust_d = 1 + rho * aw * Vd * 7.62 * Kg / (2 * W / S)
    # spanwise load: Schrenk over full semi-span, panel carries y from hw to b/2
    semi = b / 2
    ys = np.linspace(P["cabin_half"], semi, 721)
    yp = ys - P["cabin_half"]  # panel coordinate
    ell = 4 / (math.pi * semi) * np.sqrt(np.clip(1 - (ys / semi) ** 2, 0, None))
    rect = np.ones_like(ys) / semi
    shape = 0.5 * (ell + rect)
    shape /= np.trapezoid(shape, ys)
    # fraction of semi-span lift on the panel (rest on centre section, carried to cabin)
    def spar_analysis(n_equiv, t, h, bcap=P["cap_b"], lip=P["lip"]):
        # n_equiv: ultimate load factor share on this spar
        Wn = n_equiv * W / 2           # N on half wing (includes centre section share -> conservative)
        w = Wn * shape                 # N/mm along y
        yy = yp
        # pinned at yp=0, simple support (strut) at ys_panel
        Rs = np.trapezoid(w * yy, yy) / ys_panel
        R0 = np.trapezoid(w, yy) - Rs
        M = np.zeros_like(yy)
        V = np.zeros_like(yy)
        for j, y in enumerate(yy):
            sel = yy >= y
            # moment from outboard side about y (sagging +)
            Mo = -np.trapezoid(w[sel] * (yy[sel] - y), yy[sel])
            if y < ys_panel:
                Mo += Rs * (ys_panel - y)
            M[j] = Mo
            V[j] = np.trapezoid(w[sel], yy[sel]) - (Rs if y < ys_panel else 0)
        Pax = Rs / math.tan(phi)  # spar compression inboard of strut
        # section: lipped channel
        I = t * h**3 / 12 + 2 * bcap * t * (h / 2) ** 2 + 2 * (t * lip**3 / 12 + lip * t * (h / 2 - lip / 2) ** 2)
        A = t * (h + 2 * bcap + 2 * lip)
        cdist = h / 2
        sig_b = np.abs(M) * cdist / I
        sig = sig_b + np.where(yy < ys_panel, Pax / A, 0)
        jmax = int(np.argmax(sig))
        # allowables
        s_fl = plate(4.0, t, bcap)        # flange, lip-stiffened edge
        s_lip = plate(0.43, t, lip)
        s_cc = min(FCY, s_fl, s_lip * 1.3)  # crude crippling cut-off
        s_ten = 0.85 * FTU
        # shear (limit) with stiffeners at rib pitch
        a = P["rib_pitch"]
        hs = min(h, a)
        ks = 5.35 + 4 * (hs / max(h, a)) ** 2
        tau_cr = plate(ks, t, hs)
        tau_lim = np.max(np.abs(V)) / FoS / (h * t)
        return dict(Rs=Rs, R0=R0, Mmax=float(np.max(np.abs(M))), y_Mmax=float(yy[int(np.argmax(np.abs(M)))]),
                    Pax=Pax, sig_ult=float(sig[jmax]), y_sig=float(yy[jmax]), s_cc=s_cc, s_ten=s_ten,
                    MS_comp=s_cc / float(sig[jmax]) - 1, MS_ten=s_ten / float(sig[jmax]) - 1,
                    tau_lim=float(tau_lim), tau_cr=tau_cr, MS_shear_buckle_limit=tau_cr / tau_lim - 1,
                    I=I, A=A, Vmax=float(np.max(np.abs(V))),
                    # spanwise profile for the viewer: panel y (mm), ultimate cap stress (MPa), limit web shear (MPa)
                    prof=[[round(float(yy[j]), 1), round(float(sig[j]), 2), round(float(abs(V[j]) / FoS / (h * t)), 3)]
                          for j in sorted(set(range(0, len(yy), 12)) | {jmax, len(yy) - 1})])
    h_ms = WING.up(P["x_ms"]) - WING.lo(P["x_ms"]) - P["t_skin_u"] - P["t_skin_l"]
    h_rs = WING.up(P["x_rs"]) - WING.lo(P["x_rs"]) - P["t_skin_u"] - P["t_skin_l"]
    share_main = max(0.95 * n_pos, 1.64 * abs(n_neg)) * FoS
    share_rear = max(0.14 * n_pos, 0.64 * abs(n_neg)) * FoS
    ms = spar_analysis(share_main, P["t_ms"], h_ms)
    rs = spar_analysis(share_rear, P["t_rs"], h_rs)
    # strut: tension at +4g (main share .95), compression at -2g (main share 1.64)
    main_pos = spar_analysis(0.95 * n_pos * FoS, P["t_ms"], h_ms)
    main_neg = spar_analysis(1.64 * abs(n_neg) * FoS, P["t_ms"], h_ms)
    T_ult = main_pos["Rs"] / math.sin(phi)
    C_ult = main_neg["Rs"] / math.sin(phi)
    Do, ts = strut["od"], strut["t"]
    I_st = math.pi / 64 * (Do**4 - (Do - 2 * ts) ** 4)
    Le = Ls / 2  # jury strut at mid-length
    Pcr = math.pi**2 * E_AL * I_st / Le**2
    sig_ten = T_ult / A_st
    # stress view: each ultimate load case on its own, with the same spar shares as above
    def case(label, n_main, n_rear, tension):
        m = spar_analysis(n_main * FoS, P["t_ms"], h_ms)
        r = spar_analysis(n_rear * FoS, P["t_rs"], h_rs)
        def strut_u(R):
            F = R / math.sin(phi)
            return dict(F=round(F), u=round(F / A_st / (0.85 * FTU) if tension else F / Pcr, 3),
                        allow=round(0.85 * FTU * A_st) if tension else round(Pcr))
        return dict(label=label, tension=tension,
                    main=dict(prof=m["prof"], s_cc=round(m["s_cc"], 1), tau_cr=round(m["tau_cr"], 1)),
                    rear=dict(prof=r["prof"], s_cc=round(r["s_cc"], 1), tau_cr=round(r["tau_cr"], 1)),
                    strut_main=strut_u(m["Rs"]), strut_rear=strut_u(r["Rs"]))
    stress = dict(y0=P["cabin_half"], cases=[case("+4 g", 0.95 * n_pos, 0.14 * n_pos, True),
                                            case("−2 g", 1.64 * abs(n_neg), 0.64 * abs(n_neg), False)])
    out.update(n_pos=n_pos, n_neg=n_neg, FoS=FoS, stress=stress, Va=Va, Vd=Vd, Vne=Vne, n_gust_c=n_gust_c, n_gust_d=n_gust_d,
               spar_main=ms, spar_rear=rs, h_ms=h_ms, h_rs=h_rs, share_main=share_main, share_rear=share_rear,
               strut_L=Ls, strut_phi=math.degrees(phi), strut_T_ult=T_ult, strut_C_ult=C_ult, strut_Pcr=Pcr,
               strut_MS_buckle=Pcr / C_ult - 1, strut_MS_ten=0.85 * FTU / sig_ten - 1, strut=strut, ys_panel=ys_panel)
    if verbose:
        pr(out)
    return out, allparts, items


def pr(o):
    f = lambda v, d=1: f"{v:.{d}f}"
    print(f"sheet {f(o['mass_sheet'])} kg  empty {f(o['mass_empty'])} kg  CGx {f(o['cg_empty_x'],0)}  useful {f(o['useful'])}")
    print({k: round(v, 1) for k, v in o['mass_groups'].items()})
    for c in o["cg_cases"]:
        print("  ", c)
    print(f"S {f(o['S'],2)} m2 b {f(o['span']/1000,2)} AR {f(o['AR'],2)} W/S {f(o['WS'],0)} N/m2")
    print(f"Vs {f(o['Vs']/KT)} kt  Vs0 {f(o['Vs0']/KT)} kt  CD0 {f(o['CD0'],4)} f {f(o['f'],3)} L/D {f(o['LDmax'])}")
    print(f"T0 {f(o['T0'],0)} N  ROC {f(o['ROC']*196.85,0)} fpm @ {f(o['Vy']/KT)} kt  Vmax {f(o['Vmax']/KT)} kt  Vcr75 {f(o['Vcr']/KT)} kt")
    print(f"TO hard {f(o['s_to_hard'],0)} m grass {f(o['s_to_grass'],0)} m  land {f(o['s_land'],0)} m  Vbg {f(o['Vbg']/KT)} kt sink {f(o['sink_min'],2)} m/s")
    print(f"fuel {f(o['fuel_flow'])} L/h endurance {f(o['endurance'],2)} h range {f(o['range_km'],0)} km")
    print(f"aw {f(o['aw'],2)} at {f(o['at'],2)} Vh {f(o['Vh'],3)} Vv {f(o['Vv'],3)} deda {f(o['deda'],2)}")
    print(f"hn {f(o['hn'],3)} (tail {f(o['d_tail'],3)} fus {f(o['d_fus'],3)} prop {o['d_prop']}) CG {o['cg_fwd']}-{o['cg_aft']} SM {f(o['SM_min'],3)}..{f(o['SM_max'],3)}")
    print(f"Va {f(o['Va']/KT)} kt gust n @Vc {f(o['n_gust_c'],2)} @Vd {f(o['n_gust_d'],2)}")
    for nm in ("spar_main", "spar_rear"):
        s = o[nm]
        print(nm, {k: (round(v, 3) if isinstance(v, float) else v) for k, v in s.items()})
    print(f"h_ms {f(o['h_ms'])} h_rs {f(o['h_rs'])}")
    print(f"strut L {f(o['strut_L'],0)} phi {f(o['strut_phi'])} T_ult {f(o['strut_T_ult'],0)} C_ult {f(o['strut_C_ult'],0)} Pcr {f(o['strut_Pcr'],0)} MSb {f(o['strut_MS_buckle'],2)} MSt {f(o['strut_MS_ten'],2)}")


if __name__ == "__main__":
    run()
