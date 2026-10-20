"""Markdown summary of the key numbers, printed into the GitHub Actions run page."""
import json
from geom import OUT_DIR

H = json.load(open(OUT_DIR / "ori1_model.json"))["H"]
rows = [
    ("Empty / MTOW", f"{H['empty']} / {H['mtow']} kg"),
    ("Stall, flaps / clean", f"{H['Vs0']} / {H['Vs']} kt (SSDR limit 35 kt)"),
    ("Best climb", f"{H['roc']:.0f} ft/min"),
    ("Cruise 75 %", f"{H['Vcr']} kt"),
    ("Static margin", f"{H['SM_min']}–{H['SM_max']} % MAC"),
    ("Endurance, header / both tanks", f"{H['endurance']} / {H['endurance_both']} h"),
    ("Blanks / rivets / bolts", f"{H['nblanks']} / {H['rivets']} / {H['hw_total']}"),
    ("Rough cost", f"£{H['cost_total']:,.0f}"),
]
print(f"## OpenPlane v{H.get('version', '')}\n\n| | |\n|---|---|")
for k, v in rows:
    print(f"| {k} | {v} |")
flags = []
if H["Vs0"] > 35:
    flags.append(f"Stall with flaps is {H['Vs0']} kt: over the 35 kt SSDR limit.")
if H["SM_min"] < 5:
    flags.append(f"Minimum static margin is {H['SM_min']} %: below 5 %.")
if H["empty"] + 55 > H["mtow"]:
    flags.append("Empty mass leaves no room for a 55 kg pilot.")
if flags:
    print("\n**Check:**\n" + "\n".join(f"- {f}" for f in flags))
