"""Build everything from aircraft.yaml.

    python run.py            check, build, report, site
    python run.py check      refold check only (fast)
    python run.py translate  translate new or changed text (needs ANTHROPIC_API_KEY), then rebuild the site

Outputs go to out/ (cut files, BOM, report) and site/ (the website).
"""
import os, subprocess, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SRC = ROOT / "src"
STEPS = {"check": ["check.py"], "all": ["check.py", "build.py", "docs.py", "site_build.py"],
         "translate": ["i18n_tool.py translate", "site_build.py"]}


def main():
    which = sys.argv[1] if len(sys.argv) > 1 else "all"
    env = dict(os.environ, OPENPLANE_ROOT=str(ROOT), PYTHONPATH=str(SRC))
    if which == "all" and (ROOT / "out").exists():
        import shutil
        shutil.rmtree(ROOT / "out")
    for s in STEPS[which]:
        print(f"== {s}", flush=True)
        name, *args = s.split()
        r = subprocess.run([sys.executable, str(SRC / name), *args], cwd=ROOT, env=env)
        if r.returncode:
            sys.exit(f"{s} failed")


if __name__ == "__main__":
    main()
