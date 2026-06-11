#!/usr/bin/env python3
"""SkateArm cell dashboard — Flask + SQLite SCADA-style monitor.

Ingests the sequencer's JSON cycle logs (logs/cycle_*.json) and serves:

  /            overview: KPI cards (cycles, accept rate, avg cycle time,
               QC residuals), cycle-time trend, cycles table
  /cycle/<id>  step timeline (GRAFCET S0..S7) + QC measurements of one cycle

Usage:
    pip install flask
    python app.py --ingest ../logs/cycle_001.json   # import a log (repeatable)
    python app.py                                   # serve on :5000

The schema mirrors the sequencer's event stream — the same code will ingest
logs from the real cell once skate_ros2 streams them.
"""
import argparse
import json
import os
import sqlite3

from flask import Flask, g, render_template_string, abort

DB = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cell.db")
app = Flask(__name__)


# ---------------------------------------------------------------- storage --
def db():
    conn = getattr(g, "_db", None)
    if conn is None:
        conn = g._db = sqlite3.connect(DB)
        conn.row_factory = sqlite3.Row
    return conn


def init_db(conn):
    conn.executescript("""
    CREATE TABLE IF NOT EXISTS cycles(
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        source TEXT, cycle_time_s REAL, verdict TEXT,
        cam_align_mm REAL, cam_depth_mm REAL,
        oracle_align_mm REAL, oracle_depth_mm REAL, oracle_tilt_deg REAL,
        residual_align_mm REAL, residual_depth_mm REAL);
    CREATE TABLE IF NOT EXISTS events(
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        cycle_id INTEGER REFERENCES cycles(id),
        t REAL, step TEXT, msg TEXT, data TEXT);
    """)
    conn.commit()


def ingest(path):
    conn = sqlite3.connect(DB)
    init_db(conn)
    log = json.load(open(path))
    verdict, ctime = "UNKNOWN", None
    qc = {}
    for e in log:
        if "result" in e:
            verdict = e["result"]
        if "cam_result" in e:
            verdict = e["cam_result"]
        if "cycle_time_s" in e:
            ctime = e["cycle_time_s"]
        for k in ("cam_align_mm", "cam_depth_mm", "oracle_align_mm",
                  "oracle_depth_mm", "oracle_tilt_deg",
                  "residual_align_mm", "residual_depth_mm"):
            if k in e and e[k] is not None:
                qc[k] = e[k]
        if "depth_mm" in e and "oracle_depth_mm" not in qc:
            qc["oracle_depth_mm"] = e["depth_mm"]
        if "err_xy_mm" in e and "oracle_align_mm" not in qc:
            qc["oracle_align_mm"] = e["err_xy_mm"]
        if "tilt_deg" in e and "oracle_tilt_deg" not in qc:
            qc["oracle_tilt_deg"] = e["tilt_deg"]
    cur = conn.execute(
        "INSERT INTO cycles(source, cycle_time_s, verdict, cam_align_mm,"
        " cam_depth_mm, oracle_align_mm, oracle_depth_mm, oracle_tilt_deg,"
        " residual_align_mm, residual_depth_mm) VALUES(?,?,?,?,?,?,?,?,?,?)",
        (os.path.basename(path), ctime, verdict,
         qc.get("cam_align_mm"), qc.get("cam_depth_mm"),
         qc.get("oracle_align_mm"), qc.get("oracle_depth_mm"),
         qc.get("oracle_tilt_deg"),
         qc.get("residual_align_mm"), qc.get("residual_depth_mm")))
    cid = cur.lastrowid
    for e in log:
        extra = {k: v for k, v in e.items() if k not in ("t", "step", "msg")}
        conn.execute("INSERT INTO events(cycle_id, t, step, msg, data)"
                     " VALUES(?,?,?,?,?)",
                     (cid, e.get("t"), e.get("step"), e.get("msg"),
                      json.dumps(extra) if extra else None))
    conn.commit()
    print(f"ingested {path} as cycle #{cid} ({verdict}, {ctime}s)")


# ----------------------------------------------------------------- pages --
BASE = """<!doctype html><html><head><meta charset="utf-8">
<title>SkateArm cell dashboard</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4"></script>
<style>
 :root{--bg:#0d1117;--panel:#161b22;--line:#30363d;--txt:#c9d1d9;
       --accent:#58a6ff;--ok:#3fb950;--bad:#f85149;--dim:#8b949e}
 body{background:var(--bg);color:var(--txt);font-family:'Segoe UI',Roboto,sans-serif;
      margin:0;padding:24px 32px}
 h1{font-size:20px;margin:0 0 4px} h1 a{color:var(--txt);text-decoration:none}
 .sub{color:var(--dim);font-size:13px;margin-bottom:20px}
 .cards{display:flex;gap:14px;flex-wrap:wrap;margin-bottom:22px}
 .card{background:var(--panel);border:1px solid var(--line);border-radius:8px;
       padding:14px 20px;min-width:150px}
 .card .v{font-size:26px;font-weight:600;color:var(--accent)}
 .card .v.ok{color:var(--ok)} .card .v.bad{color:var(--bad)}
 .card .l{font-size:12px;color:var(--dim);margin-top:2px}
 .panel{background:var(--panel);border:1px solid var(--line);border-radius:8px;
        padding:16px 20px;margin-bottom:20px}
 table{border-collapse:collapse;width:100%;font-size:13px}
 th,td{border-bottom:1px solid var(--line);padding:7px 10px;text-align:left}
 th{color:var(--dim);font-weight:500}
 a{color:var(--accent)} .ok{color:var(--ok)} .bad{color:var(--bad)}
 .bar{height:18px;border-radius:3px;background:var(--accent);opacity:.85}
 .steplbl{font-size:12px;color:var(--dim);width:210px;display:inline-block}
</style></head><body>
<h1><a href="/">SkateArm — cell dashboard</a></h1>
<div class="sub">GRAFCET sequencer telemetry · sim work-cell · Phase 1</div>
{{ body }}
</body></html>"""

OVERVIEW = """
<div class="cards">
 <div class="card"><div class="v">{{n}}</div><div class="l">cycles logged</div></div>
 <div class="card"><div class="v {{'ok' if rate>=90 else 'bad'}}">{{rate}}%</div>
   <div class="l">accept rate</div></div>
 <div class="card"><div class="v">{{avg_t}} s</div><div class="l">avg cycle time (takt ≤ 60 s)</div></div>
 <div class="card"><div class="v">{{res_a}} mm</div><div class="l">QC residual · alignment</div></div>
 <div class="card"><div class="v">{{res_d}} mm</div><div class="l">QC residual · depth</div></div>
</div>
<div class="panel"><canvas id="trend" height="70"></canvas></div>
<div class="panel"><table>
 <tr><th>#</th><th>source</th><th>verdict</th><th>cycle time</th>
     <th>cam align</th><th>cam depth</th><th>oracle align</th><th>oracle depth</th></tr>
 {% for c in cycles %}
 <tr><td><a href="/cycle/{{c['id']}}">{{c['id']}}</a></td><td>{{c['source']}}</td>
  <td class="{{'ok' if c['verdict']=='ACCEPT' else 'bad'}}">{{c['verdict']}}</td>
  <td>{{c['cycle_time_s'] or '—'}} s</td>
  <td>{{c['cam_align_mm'] if c['cam_align_mm'] is not none else '—'}}</td>
  <td>{{c['cam_depth_mm'] if c['cam_depth_mm'] is not none else '—'}}</td>
  <td>{{c['oracle_align_mm'] if c['oracle_align_mm'] is not none else '—'}}</td>
  <td>{{c['oracle_depth_mm'] if c['oracle_depth_mm'] is not none else '—'}}</td></tr>
 {% endfor %}
</table></div>
<script>
new Chart(document.getElementById('trend'),{type:'line',
 data:{labels:{{labels}},datasets:[{label:'cycle time, s',data:{{times}},
  borderColor:'#58a6ff',backgroundColor:'rgba(88,166,255,.15)',fill:true,tension:.3},
  {label:'takt target',data:{{takt}},borderColor:'#f8514966',borderDash:[6,4],pointRadius:0}]},
 options:{plugins:{legend:{labels:{color:'#c9d1d9'}}},
  scales:{x:{ticks:{color:'#8b949e'},grid:{color:'#21262d'}},
          y:{ticks:{color:'#8b949e'},grid:{color:'#21262d'},suggestedMax:70}}}});
</script>"""

DETAIL = """
<div class="panel"><b>cycle #{{c['id']}}</b> · {{c['source']}} ·
 <span class="{{'ok' if c['verdict']=='ACCEPT' else 'bad'}}">{{c['verdict']}}</span>
 · {{c['cycle_time_s'] or '—'}} s</div>
<div class="panel">
 <b>step timeline</b><br><br>
 {% for s in spans %}
  <div><span class="steplbl">{{s.step}} — {{s.label}}</span>
   <div class="bar" style="margin-left:{{s.left}}px;width:{{s.width}}px;display:inline-block"></div>
   <span style="color:var(--dim);font-size:12px"> {{s.dur}} s</span></div>
 {% endfor %}
</div>
<div class="panel"><b>QC</b><br><br><table>
 <tr><th>metric</th><th>camera</th><th>oracle (sim truth)</th><th>residual</th></tr>
 <tr><td>alignment, mm</td><td>{{c['cam_align_mm'] or '—'}}</td>
     <td>{{c['oracle_align_mm'] or '—'}}</td><td>{{c['residual_align_mm'] or '—'}}</td></tr>
 <tr><td>insertion depth, mm</td><td>{{c['cam_depth_mm'] or '—'}}</td>
     <td>{{c['oracle_depth_mm'] or '—'}}</td><td>{{c['residual_depth_mm'] or '—'}}</td></tr>
 <tr><td>tilt, deg</td><td>v2 (resolution)</td><td>{{c['oracle_tilt_deg'] or '—'}}</td><td>—</td></tr>
</table></div>
<div class="panel"><b>event log</b><br><br><table>
 <tr><th>t, s</th><th>step</th><th>message</th><th>data</th></tr>
 {% for e in events %}
 <tr><td>{{e['t']}}</td><td>{{e['step']}}</td><td>{{e['msg']}}</td>
     <td style="color:var(--dim);font-size:12px">{{e['data'] or ''}}</td></tr>
 {% endfor %}
</table></div>"""

STEP_LABELS = {"S0": "home / parts check", "S1": "approach + grasp",
               "S2": "carry to fixture", "S3": "align peg/pocket",
               "S4": "insert (guarded)", "S5": "QC verify",
               "S6": "place to bin", "S7": "complete"}


@app.route("/")
def overview():
    conn = db()
    init_db(conn)
    cycles = conn.execute("SELECT * FROM cycles ORDER BY id").fetchall()
    n = len(cycles)
    acc = sum(1 for c in cycles if c["verdict"] == "ACCEPT")
    times = [c["cycle_time_s"] for c in cycles if c["cycle_time_s"]]
    res_a = [c["residual_align_mm"] for c in cycles if c["residual_align_mm"] is not None]
    res_d = [c["residual_depth_mm"] for c in cycles if c["residual_depth_mm"] is not None]
    body = render_template_string(
        OVERVIEW, n=n, rate=round(100 * acc / n) if n else 0,
        avg_t=round(sum(times) / len(times), 1) if times else "—",
        res_a=round(sum(res_a) / len(res_a), 1) if res_a else "—",
        res_d=round(sum(res_d) / len(res_d), 1) if res_d else "—",
        cycles=cycles,
        labels=[f"#{c['id']}" for c in cycles],
        times=[c["cycle_time_s"] for c in cycles],
        takt=[60] * n)
    return render_template_string(BASE, body=body)


@app.route("/cycle/<int:cid>")
def cycle(cid):
    conn = db()
    init_db(conn)
    c = conn.execute("SELECT * FROM cycles WHERE id=?", (cid,)).fetchone()
    if not c:
        abort(404)
    events = conn.execute(
        "SELECT * FROM events WHERE cycle_id=? ORDER BY id", (cid,)).fetchall()
    # build step spans from first/last event times per step
    spans, seen = [], {}
    for e in events:
        if e["step"] and e["t"] is not None:
            seen.setdefault(e["step"], [e["t"], e["t"]])
            seen[e["step"]][1] = e["t"]
    order = sorted(seen.items(), key=lambda kv: kv[1][0])
    total = max((v[1] for _, v in order), default=1) or 1
    scale = 420.0 / total
    for i, (step, (t0, t1)) in enumerate(order):
        t_end = order[i + 1][1][0] if i + 1 < len(order) else t1
        spans.append(type("S", (), {
            "step": step, "label": STEP_LABELS.get(step, ""),
            "left": int(t0 * scale), "width": max(4, int((t_end - t0) * scale)),
            "dur": round(t_end - t0, 1)}))
    body = render_template_string(DETAIL, c=c, events=events, spans=spans)
    return render_template_string(BASE, body=body)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--ingest", help="path to a cycle_*.json log")
    ap.add_argument("--port", type=int, default=5000)
    args = ap.parse_args()
    if args.ingest:
        ingest(args.ingest)
    else:
        app.run(host="127.0.0.1", port=args.port, debug=False)
