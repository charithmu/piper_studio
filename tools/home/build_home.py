#!/usr/bin/env python3
"""Build the Piper Studio home page (https://<host>:8700/piper-studio/): every Studio page under one roof, with a live up/down status for each, what to run when one is down,
the tools to run from a terminal, and the key documents. Generated from this repo's artifacts.yaml, so a page or tool registered there shows up here on the next build.

    source scripts/env.sh && python tools/home/build_home.py [OUT_DIR]        # default ~/data/ml/isaac/piper_studio/home (served by the always-on static unit, no GPU)

Static HTML; the browser probes each page's route (same origin) to show whether it is running. Nothing here starts, stops or exposes anything.
"""
import html
import json
import subprocess
import sys
import time
from pathlib import Path

import yaml

WS = Path(__file__).resolve().parents[2]
SELF_ID = "piper-studio-home"

# pages of other projects that belong with Studio (the clients of its model and wheels); not generated: they are not in this repo's registry
RELATED = [
    ("Piper Pick demo", "The pick-and-place system built on Studio's wheels and model.", "/piper-pick/demo/"),
    ("cuRobo study videos", "Recorded cuRobo plans executed in MuJoCo and Isaac (a client of Studio's cuRobo model).", "/curobo-piper/videos/"),
    ("Pick pipeline bake-off", "Results of the pick pipeline study.", "/pick-bakeoff/results/"),
    ("Isaac Sim demo playlist", "General Isaac Sim videos of this workstation.", "/isaac-sim/videos/"),
]

DOCS = [
    ("README.md", "what this is, layout, quick start"), ("docs/STATE.md", "start here: orientation and the hard-won gotchas"),
    ("docs/ARCHITECTURE.md", "layers, interfaces, repository map"), ("docs/INTEGRATION.md", "the contract other repos rely on (names, wheels, camera)"),
    ("docs/CUROBO.md", "the cuRobo model: ownership, use, regeneration"), ("docs/CLEANUP.md", "state, to-do list and the verification log"),
    ("docs/VERSIONS.md", "qualified versions"), ("docs/real_arm/REAL_ARM.md", "the real arm: rules, tools, numbers"),
    ("docs/real_arm/CHECKLIST.md", "the ladder to the real arm, one rung at a time"), ("docs/real_arm/ZED_BOX.md", "the ZED X Mini on Atlas and on the ZED Box"),
    ("docs/description/DECISIONS.md", "design decisions D1 to D14"), ("HANDOFF.md", "notes for downstream projects"),
]

CSS = """
:root{--bg:#f6f7f9;--card:#fff;--ink:#16191d;--mut:#5d6773;--line:#dde1e6;--acc:#1f5fd1;--ok:#17803d;--bad:#b3261e;--idle:#9aa3ad}
@media(prefers-color-scheme:dark){:root{--bg:#121417;--card:#1b1f24;--ink:#e8eaed;--mut:#9aa3ad;--line:#2b3138;--acc:#7aa7ff;--ok:#5fd08a;--bad:#ff8b83;--idle:#6b7580}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.5 system-ui,-apple-system,"Segoe UI",sans-serif}
main{max-width:1060px;margin:0 auto;padding:20px 16px 48px}h1{font-size:26px;margin:4px 0 2px}h2{font-size:17px;margin:30px 0 10px;padding-bottom:5px;border-bottom:1px solid var(--line)}
.muted{color:var(--mut);font-size:13px}.lead{color:var(--mut);margin:2px 0 6px;max-width:780px}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(310px,1fr));gap:12px}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:14px 15px;display:flex;flex-direction:column;gap:6px}
.card h3{font-size:16px;margin:0;display:flex;align-items:center;gap:8px}.card p{margin:0;color:var(--mut);font-size:13.5px}
.card a.open{margin-top:auto;color:var(--acc);text-decoration:none;font-weight:600;font-size:14px}.card a.open:hover{text-decoration:underline}
.dot{width:10px;height:10px;border-radius:50%;background:var(--idle);flex:none}.dot.up{background:var(--ok)}.dot.down{background:var(--bad)}
.state{font-size:12px;color:var(--mut)}code{font:12.5px ui-monospace,SFMono-Regular,Menlo,monospace;background:var(--bg);border:1px solid var(--line);border-radius:5px;padding:1px 5px;word-break:break-all}
.how{font-size:12.5px;color:var(--mut)}table{border-collapse:collapse;width:100%;font-size:13.5px}td,th{text-align:left;padding:6px 8px;border-bottom:1px solid var(--line);vertical-align:top}th{color:var(--mut);font-weight:600;font-size:12px}
.meta span{margin-right:16px}details summary{cursor:pointer;color:var(--acc);margin:6px 0}
"""


def git(*a):
    try:
        return subprocess.run(["git", "-C", str(WS), *a], capture_output=True, text=True, check=True).stdout.strip()
    except Exception:       # noqa: BLE001
        return "?"


def short(text, n=190):
    text = " ".join(str(text).split())
    return text if len(text) <= n else text[:n].rsplit(" ", 1)[0] + " ..."


def esc(s):
    return html.escape(str(s))


def card(title, desc, route, start=None, always=False):
    how = "" if always or not start else f'<div class="how">If it is down, start it: <code>{esc(start)}</code></div>'
    return (f'<div class="card" data-probe="{esc(route)}"><h3><span class="dot"></span>{esc(title)}</h3><p>{esc(desc)}</p>{how}'
            f'<div class="state">checking ...</div><a class="open" href="{esc(route)}">Open &rarr;</a></div>')


def main(out: Path):
    arts = yaml.safe_load((WS / "artifacts.yaml").read_text())["artifacts"]
    web = [a for a in arts if a.get("type") == "web" and a["id"] != SELF_ID and a.get("route")]
    tools = [a for a in arts if a.get("type") in ("job", "cli")]
    wheels = sorted(p.name.split("-")[1] for p in (WS / "dist").glob("piper_core-*.whl")) if (WS / "dist").exists() else []
    meta = (f'<span>version <b>{esc(wheels[-1] if wheels else "?")}</b> (wheels)</span><span>commit <code>{esc(git("rev-parse", "--short", "HEAD"))}</code> '
            f'{esc(git("log", "-1", "--format=%cs"))}</span><span>built {time.strftime("%Y-%m-%d %H:%M")}</span>')
    cards = "".join(card(a["title"], short(a.get("desc", "")), a["route"].rstrip("/") + "/", a.get("start"), bool(a.get("autostart"))) for a in web)
    related = "".join(card(t, d, r, always=True) for t, d, r in RELATED)
    rows = "".join(f'<tr><td>{esc(a["title"])}</td><td><code>{esc(a.get("start", ""))}</code></td>'
                   f'<td class="muted">{esc(", ".join(a.get("needs", []) or []))}</td></tr>' for a in tools)
    docs = "".join(f'<tr><td><code>{esc(p)}</code></td><td>{esc(d)}</td></tr>' for p, d in DOCS)
    page = f"""<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Piper Studio</title>
<style>{CSS}</style></head><body><main>
<h1>Piper Studio</h1>
<p class="lead">The AgileX Piper arm as importable parts and a standalone setup: description, control stack, a ROS-free rules package, a Python client, simulators (mock, Gazebo, MuJoCo, Isaac Sim), the real arm,
and the cuRobo model. Everything below is one place for its pages. Tailnet only.</p>
<div class="muted meta">{meta}</div>
<h2>Pages</h2><div class="grid">{cards}</div>
<h2>Built on Studio (other projects)</h2><div class="grid">{related}</div>
<h2>Run from a terminal</h2><p class="muted">Start each from <code>~/projects/ros/piper_studio</code> after <code>source scripts/session.sh &lt;ROS_DOMAIN_ID&gt;</code> (the full list: <code>RUNBOOK.md</code>). Anything that can move the real arm needs you at the arm.</p>
<details><summary>{len(tools)} tools and jobs</summary><table><tr><th>What</th><th>Command</th><th>Needs</th></tr>{rows}</table></details>
<h2>Documents</h2><p class="muted">Files in the repository <code>~/projects/ros/piper_studio</code>.</p><table>{docs}</table>
</main>
<script>
document.querySelectorAll('.card[data-probe]').forEach(async c=>{{
  const dot=c.querySelector('.dot'), st=c.querySelector('.state');
  try{{ const r=await fetch(c.dataset.probe,{{method:'GET',cache:'no-store'}}); if(r.ok){{dot.className='dot up';st.textContent='running';}} else {{dot.className='dot down';st.textContent='not running (HTTP '+r.status+')';}} }}
  catch(e){{ dot.className='dot down'; st.textContent='not reachable'; }}
}});
</script></body></html>"""
    out.mkdir(parents=True, exist_ok=True)
    (out / "index.html").write_text(page)
    (out / "SOURCE.md").write_text("Owner: ~/projects/ros/piper_studio (tools/home/build_home.py). Generated static home page of Piper Studio; no user recordings.\n")
    print(f"wrote {out / 'index.html'}: {len(web)} pages, {len(RELATED)} related, {len(tools)} tools")


if __name__ == "__main__":
    main(Path(sys.argv[1]) if len(sys.argv) > 1 else Path.home() / "data/ml/isaac/piper_studio/home")
