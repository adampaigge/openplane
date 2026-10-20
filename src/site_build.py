"""Assemble the website from web/page.html and the build outputs.

Writes:
  site/index.html   the page, with the model embedded (works offline and as a snapshot)
  site/live.js      the same model as a script: window.OPENPLANE_LIVE = {...}
  site/files/...    every generated file (DXF, SVG, CSV, STL, report), for the zip download
The CI publishes site/ to the `live` branch and to GitHub Pages, so a copy of the page
hosted elsewhere can load https://cdn.jsdelivr.net/gh/<repo>@live/live.js and track main.
"""
import datetime, glob, json, os, shutil, subprocess
from geom import ROOT, OUT_DIR, SITE_DIR, CFG
import yaml


def git(*a):
    try:
        return subprocess.check_output(["git", *a], cwd=ROOT, text=True, stderr=subprocess.DEVNULL).strip()
    except Exception:
        return ""


def main():
    repo = os.environ.get("OPENPLANE_REPO") or CFG.get("repo") or ""
    sha = os.environ.get("GITHUB_SHA") or git("rev-parse", "HEAD")
    when = git("log", "-1", "--format=%cI") or datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds")
    import hashlib
    page_src = open(ROOT / "web" / "page.html", "rb").read()
    meta = dict(repo=repo, sha=sha, short=sha[:7], date=when, version=CFG.get("version", ""),
                page=hashlib.sha1(page_src).hexdigest()[:12])   # live data is only used by the page code that made it

    paths = []
    for root, _, fs in os.walk(OUT_DIR):
        for f in fs:
            if f in ("preview.html", "nests.json"):
                continue
            paths.append(os.path.relpath(os.path.join(root, f), OUT_DIR))
    paths.sort()
    model = open(OUT_DIR / "ori1_model.json", encoding="utf8").read()
    flags = {os.path.basename(f)[:2]: open(f, encoding="utf8").read().strip()
             for f in sorted(glob.glob(str(ROOT / "web" / "flags" / "[A-Z][A-Z].svg"))) if not f.endswith("EU.svg")}
    langs = yaml.safe_load(open(ROOT / "i18n" / "config.yaml", encoding="utf8"))
    page = open(ROOT / "web" / "page.html", encoding="utf8").read()
    page = (page.replace("/*MODEL*/", model.replace("</", "<\\/"))
                .replace("/*MANIFEST*/", json.dumps(paths))
                .replace("/*FLAGS*/", json.dumps(flags).replace("</", "<\\/"))
                .replace("/*LANGS*/", json.dumps(langs, ensure_ascii=False))
                .replace("/*QUAL*/", json.dumps(yaml.safe_load(open(ROOT / "qualification.yaml", encoding="utf8")), ensure_ascii=False).replace("</", "<\\/"))
                .replace("/*META*/", json.dumps(meta))
                .replace("__OPENPLANE_REPO__", repo))

    os.makedirs(SITE_DIR, exist_ok=True)
    if (SITE_DIR / "files").exists():
        shutil.rmtree(SITE_DIR / "files")
    shutil.copytree(OUT_DIR, SITE_DIR / "files", ignore=shutil.ignore_patterns("preview.html", "nests.json"))
    open(SITE_DIR / "index.html", "w", encoding="utf8").write(page)
    os.makedirs(SITE_DIR / "i18n", exist_ok=True)
    for lang in langs["languages"]:
        src = ROOT / "i18n" / f"{lang}.json"
        if src.exists():
            shutil.copy(src, SITE_DIR / "i18n" / f"{lang}.json")
    open(SITE_DIR / "live.js", "w", encoding="utf8").write(
        "window.OPENPLANE_LIVE=" + json.dumps(dict(meta=meta, manifest=paths)) [:-1] + ',"model":' + model + "};\n")
    open(SITE_DIR / ".nojekyll", "w").write("")
    print(f"site: {len(paths)} files, page {len(page) // 1024} KB, commit {meta['short'] or '(none)'}")


if __name__ == "__main__":
    main()
