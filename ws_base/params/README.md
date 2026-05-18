# ws_base/params — Tunable Parameters for POC Runs

This directory holds **operator-editable** YAML parameter files for both
laboratory and on-field POC runs.  These are the files you edit during
on-field tuning — no need to navigate into `src/*/config/`.

## Structure

```
params/
├── lab/
│   ├── mission_command.yaml     — Base PC: destination & speed (lab indoor/bench test)
│   ├── camera.yaml              — Jetson: camera config with video-file fallback enabled
│   └── control.yaml             — Jetson: conservative lab-tuned controller gains
└── field/
    ├── mission_command.yaml     — Base PC: real field GPS coordinates & speed limit
    ├── camera.yaml              — Jetson: camera-only (no fallback), full-res 30 FPS
    └── control.yaml             — Jetson: field-tuned controller gains
```

## How the POC Scripts Use These

| Script | Params directory |
|--------|-----------------|
| `launch_poc_lab.sh`   | `params/lab/`   |
| `launch_poc_field.sh` | `params/field/` |

The scripts SCP `camera.yaml` and `control.yaml` to `~/almondmatcha_poc/` on the
Jetson **before** launching nodes, then pass `CAMERA_PARAMS_FILE` and
`CONTROL_PARAMS_FILE` env vars to `ws_jetson/launch_jetson_single_domain.sh`.

`mission_command.yaml` is passed via `MISSION_CMD_PARAMS` env var to
`ws_base/launch_base_single_domain.sh`.

## On-Field Tuning Workflow

1. **Edit gains before a run:**
   ```bash
   nano ~/almondmatcha/ws_base/params/field/control.yaml
   # adjust k_p, k_i, k_d, ema_alpha as needed
   ```

2. **Run the field script** — it picks up the new values automatically:
   ```bash
   bash ~/almondmatcha/ws_base/launch_poc_field.sh
   ```

3. **Emergency interrupt:** press `Ctrl-C` — all nodes and collectors are
   cleanly stopped across all machines.

4. **Check results:**
   ```bash
   ls ~/almondmatcha/ws_base/runs/single_domain/
   ```

## Source-of-Truth Defaults

The original in-source defaults remain at:
- `ws_base/src/mission_control/config/params.yaml`
- `ws_jetson/src/vision_navigation/config/rover_kinematic_control_params.yaml`
- `ws_jetson/src/vision_navigation/config/vision_nav_headless.yaml`

These are used by non-POC launches and serve as the baseline reference.
