"""Translations live in i18n/<lang>.json, one entry per English string:

    "Fold the spars": {"t": "Doblar los largueros", "by": "claude-sonnet-5-5", "reviewed": false}

`reviewed: true` marks a line a person has checked; machine updates never overwrite it.
When the English changes, the old entry is dropped and the new line is translated again.

    python src/i18n_tool.py extract     write i18n/strings.json from the current build
    python src/i18n_tool.py translate   fill missing lines with Claude (needs ANTHROPIC_API_KEY)
    python src/i18n_tool.py status      show coverage per language
"""
import json, os, re, sys, time, urllib.request
import yaml
from geom import ROOT, OUT_DIR

I18N = ROOT / "i18n"
CFG = yaml.safe_load(open(I18N / "config.yaml", encoding="utf8"))
MODEL = os.environ.get("OPENPLANE_TRANSLATE_MODEL", "claude-sonnet-5-5")
BATCH = 40


def extract():
    M = json.load(open(OUT_DIR / "ori1_model.json", encoding="utf8"))
    s = set()
    for st in M["steps"]:
        s.update([st["title"], st["jp"][1], st["text"], st["fold"], *st["sub"]])
        for h in st["hw"]:
            s.update([h["desc"], h.get("where", "")])
    s.update(M["tools"].values())
    s.update(b["name"] for b in M["bought"])
    for b in M["bomx"]:
        s.update([b["item"], b.get("note", ""), b["cat"]])
    Q = yaml.safe_load(open(ROOT / "qualification.yaml", encoding="utf8"))
    s.add(Q["intro"])
    for ph in Q["phases"]:
        s.update([ph["title"], ph["when"], *ph["retires"]])
        for t in ph["tests"]:
            s.update([t["name"], t["method"], t["pass"]])
    page = open(ROOT / "web" / "page.html", encoding="utf8").read()
    s.update(x.strip() for x in re.findall(r"data-i18n[^>]*>([^<]+)<", page))
    s.update(re.findall(r"T\('((?:[^'\\]|\\.)*)'\)", page))
    strings = sorted(x for x in s if x and re.search(r"[A-Za-z]{3}", x))
    json.dump(strings, open(I18N / "strings.json", "w", encoding="utf8"), ensure_ascii=False, indent=0)
    return strings


def load(lang):
    p = I18N / f"{lang}.json"
    return json.load(open(p, encoding="utf8")) if p.exists() else {}


def save(lang, d):
    json.dump(dict(sorted(d.items())), open(I18N / f"{lang}.json", "w", encoding="utf8"), ensure_ascii=False, indent=1)


def ask_claude(lang_name, batch):
    prompt = (f"Translate each string in the JSON array below from English into {lang_name}. "
              "They come from the build manual and website of OpenPlane, a home-built single-seat microlight aircraft "
              "made from folded 6061-T6 aluminium sheet. Rules: keep part codes (W-MS, K-CV, F-B1, S-GT), AN/MS hardware "
              "codes (AN4, AN365-428, A4-3), numbers, units, currency amounts, file names and symbols exactly as written; "
              "use standard aviation and sheet-metal terminology; translate precisely, because people will build an "
              "aircraft from this; keep it concise; add nothing. "
              f"Reply with only a JSON array of {len(batch)} strings in the same order.\n\n" + json.dumps(batch, ensure_ascii=False))
    body = json.dumps({"model": MODEL, "max_tokens": 16000, "messages": [{"role": "user", "content": prompt}]}).encode()
    req = urllib.request.Request("https://api.anthropic.com/v1/messages", data=body, headers={
        "content-type": "application/json", "x-api-key": os.environ["ANTHROPIC_API_KEY"], "anthropic-version": "2023-06-01"})
    for attempt in range(4):
        try:
            with urllib.request.urlopen(req, timeout=300) as r:
                txt = "".join(c.get("text", "") for c in json.load(r)["content"])
            out = json.loads(txt[txt.index("["): txt.rindex("]") + 1])
            if isinstance(out, list) and len(out) == len(batch) and all(isinstance(x, str) for x in out):
                return out
            raise ValueError("wrong shape")
        except Exception as e:
            print(f"  retry {attempt + 1}: {e}", flush=True)
            time.sleep(5 * (attempt + 1))
    return None


def translate():
    strings = extract()
    if not os.environ.get("ANTHROPIC_API_KEY"):
        print("ANTHROPIC_API_KEY not set: skipping machine translation (the site falls back to the browser's translator).")
        prune(strings)
        return
    for lang, meta in CFG["languages"].items():
        d = load(lang)
        missing = [s for s in strings if s not in d]
        print(f"{lang}: {len(missing)} new lines", flush=True)
        for i in range(0, len(missing), BATCH):
            batch = missing[i:i + BATCH]
            out = ask_claude(meta["english"], batch)
            if out is None:
                print(f"  {lang}: batch {i // BATCH + 1} failed, left for next build")
                continue
            for s, t in zip(batch, out):
                d[s] = {"t": t, "by": MODEL, "reviewed": False}
            save(lang, d)
    prune(strings)


def prune(strings):
    keep = set(strings)
    for lang in CFG["languages"]:
        d = load(lang)
        if d:
            save(lang, {k: v for k, v in d.items() if k in keep})


def status():
    strings = extract()
    for lang in CFG["languages"]:
        d = load(lang)
        have = sum(1 for s in strings if s in d)
        rev = sum(1 for s in strings if s in d and d[s].get("reviewed"))
        print(f"{lang}: {have}/{len(strings)} translated, {rev} reviewed")


if __name__ == "__main__":
    {"extract": extract, "translate": translate, "status": status}[sys.argv[1] if len(sys.argv) > 1 else "status"]()
