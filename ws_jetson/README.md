# Jetson Workspace - Visual Navigation System

Real-time lane detection and steering control for autonomous rovers on NVIDIA Jetson.

## Quick Start

### Build
```bash
cd ~/almondmatcha/ws_jetson
./build_clean.sh   # Clean build (recommended)
./build_inc.sh     # Incremental build (faster)
```

### Launch
```bash
./launch_gui.sh      # GUI mode (with visualization)
./launch_headless.sh # Headless mode (no GUI)
```

## Configuration

System config and control tuning kept separate (no duplication):
- `vision_navigation/config/vision_nav_gui.yaml` — GUI mode settings
- `vision_navigation/config/vision_nav_headless.yaml` — Headless mode settings
- `vision_navigation/config/steering_control_params.yaml` — PID tuning

See [Configuration Guide](docs/configuration.md) for detailed tuning and custom configurations.

## Documentation

- **[Architecture](docs/architecture.md)** — System design, data flow, message types, sign conventions
- **[Node Details](docs/nodes.md)** — Individual node parameters, launching, and testing
- **[Configuration Guide](docs/configuration.md)** — Parameter tuning, custom configs, examples
- **[Troubleshooting](docs/troubleshooting.md)** — Issues, solutions, performance baselines

## Quick References

- **Main project**: `/home/yupi/almondmatcha/README.md`
- **Build scripts**: `build_clean.sh`, `build_inc.sh`
- **Launch scripts**: `launch_gui.sh`, `launch_headless.sh`
- **Config directory**: `vision_navigation/config/`
- **Session notes**: `/home/yupi/almondmatcha/copilot-session-note/WORK_SESSION_2025-11-04.md`

## Package Structure

```
ws_jetson/
├── README.md                    (This file - quick start only)
├── docs/                        (Detailed documentation)
│   ├── architecture.md          (System design & data flow)
│   ├── nodes.md                 (Node details & testing)
│   ├── configuration.md         (Parameter tuning & examples)
│   └── troubleshooting.md       (Issues & solutions)
├── build_clean.sh               (Clean build script)
├── build_inc.sh                 (Incremental build script)
├── launch_gui.sh                (Launch GUI mode)
├── launch_headless.sh           (Launch headless mode)
├── vision_navigation/
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   ├── config/                  (YAML configuration files)
│   │   ├── vision_nav_gui.yaml
│   │   ├── vision_nav_headless.yaml
│   │   └── steering_control_params.yaml
│   ├── launch/                  (ROS2 launch files)
│   │   ├── vision_nav_gui.launch.py
│   │   ├── vision_nav_headless.launch.py
│   │   └── vision_navigation.launch.py
│   └── vision_navigation_pkg/   (Python source code)
├── build/                       (Auto-generated)
├── install/                     (Auto-generated)
├── log/                         (Auto-generated)
└── logs/                        (Data logs - CSV files)
```

## License
Apache 2.0
