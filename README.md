# OpenPlane

A single-seat microlight in which every structural sheet part is a flat blank with straight folds, cut from one gauge of aluminium (6061-T6, 0.020 in). Thicker parts are nested laminations of the same sheet.

**The aircraft is code.** [`aircraft.yaml`](aircraft.yaml) is the design. Change a number and push: the build regenerates every flat pattern, the 3D model, mass and balance, performance, first-pass structure checks, the bill of materials and the website.

| | |
|---|---|
| Span / chord | 7.9 m / 1.4 m, NACA 4415 |
| Empty / MTOW | about 198 kg / 300 kg (UK SSDR) |
| Stall, flaps | 33 kt |
| Engine | Half-VW, 37 hp, 54 × 24 in wood prop |
| Fuel | 18 L header tank + 20 L aux tank, both in the cabin |
| Parts | 218 blanks, about 4,400 rivets, plus 4 wing stands as a practice project |

The live figures are on the website and in each build's run summary; the table above is a guide.

## Build it yourself

```sh
pip install -r requirements.txt
python run.py          # refold check, cut files, report, website
python run.py check    # refold check only
```

Outputs:

- `out/dxf/parts/` one DXF per unique blank, 1:1 in mm (layers CUT, VALLEY, MOUNTAIN, ROLL, HOLE, ETCH)
- `out/dxf/nests/` the blanks nested onto stock sheets, ready for a laser or waterjet
- `out/svg/` crease patterns
- `out/prop/` the propeller as STL, OBJ and a station table
- `out/BOM.csv`, `out/HARDWARE_BY_STEP.csv`, `out/DESIGN_REPORT.md`
- `site/` the website: open `site/index.html`

Builds are reproducible: an unchanged part produces a byte-identical DXF, so a diff shows only real changes.

## How it works

| File | What it does |
|---|---|
| `aircraft.yaml` | The design parameters |
| `qualification.yaml` | The test programme: phases, tests and pass criteria, which use live numbers from the build |
| `src/geom.py` | Airfoils, the `Part` class, and unfolding: each part is planar 3D faces joined by straight hinges |
| `src/parts.py` | Every part: spars, ribs, skins, control surfaces, fuselage, tail, cowl, fittings, linkage, glazing, stands |
| `src/check.py` | Refolds every flat pattern and compares it with the 3D part. CI fails above 0.01 mm |
| `src/design.py` | Mass and balance, aerodynamics, performance, stability, structure |
| `src/build.py` | DXF and SVG output, nesting, hardware, routes, build steps, BOM, model JSON |
| `src/prop.py` | Parametric propeller geometry |
| `src/docs.py` | Design report, readme and download zip |
| `src/site_build.py` | Assembles the website and `live.js` |
| `web/page.html` | The website template: 3D build viewer, numbers, cost, BOM |

## The live website

Every push to `main` runs [`.github/workflows/build.yml`](.github/workflows/build.yml):

1. Runs the refold check, then the full build.
2. Writes the key numbers to the run summary and flags a stall over 35 kt, a static margin under 5 %, or no room for a 55 kg pilot.
3. Pushes `site/` to the `live` branch and deploys it to GitHub Pages.

Copies of the page hosted elsewhere load `https://cdn.jsdelivr.net/gh/<owner>/<repo>@live/live.js`, so they show the latest build of `main`. The badge at the top of the page shows the commit the numbers came from. If the live build can't be reached, the page falls back to the snapshot built into it.

One-time setup: in the repository's Settings → Pages, set the source to **GitHub Actions**.

## Translations

The site offers the languages in [`i18n/config.yaml`](i18n/config.yaml). Each has a file, `i18n/<lang>.json`, with one entry per English line:

```json
"Fold the spars": {"t": "Doblar los largueros", "by": "claude-sonnet-5-5", "reviewed": false}
```

- **Machine translation in CI:** if the repository has an `ANTHROPIC_API_KEY` secret, every build translates only the new or changed lines and commits them. Lines whose English has gone are dropped.
- **Human review:** fix a line with a pull request and set `"reviewed": true`. Machine updates never overwrite a reviewed line.
- **Browser fallback:** lines not in the file yet are translated on the visitor's device by the browser's built-in translator where there is one (desktop Chrome), and labelled as such. Otherwise they stay in English.

The page always tells readers that the English is the reference for anything safety-critical.

```sh
python src/i18n_tool.py status      # coverage per language
python run.py translate             # translate locally (needs ANTHROPIC_API_KEY)
```

## Status

A concept design with first-pass calculations. It has not been built, load-tested or flown. See `DESIGN_REPORT.md` → "Limits of this work" before cutting metal.

## Licence

[CC0 1.0](LICENSE): public domain. Copy it, change it, build it, sell it.
