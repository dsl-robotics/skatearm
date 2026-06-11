# dashboard — cell SCADA monitor

Flask + SQLite dashboard over the sequencer's cycle logs.

```bash
pip install flask
python app.py --ingest ../logs/cycle_001.json          # import logs (repeatable)
python app.py --ingest ../logs/cycle_002_camera_qc.json
python app.py                                          # serve on http://127.0.0.1:5000
```

Pages:

- **Overview** — KPI cards (cycles logged, accept rate, average cycle time vs
  the 60 s takt target, camera-vs-oracle QC residuals), cycle-time trend chart,
  cycles table.
- **Cycle detail** (`/cycle/<id>`) — GRAFCET step timeline, camera vs oracle QC
  comparison, full event log.

`preview_overview.html` / `preview_cycle.html` are statically rendered samples
of both pages with the two reference cycles loaded (no server needed to peek).

The ingest schema mirrors the sequencer's event stream, so the same dashboard
will ingest logs from the real cell once `skate_ros2` streams them.
