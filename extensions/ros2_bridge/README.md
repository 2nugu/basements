# basements_ros2_bridge

ROS 2 bridge layer between the Basements MPM-rigid coupling solver and
real robot hardware (myCobot reference target; UR / Franka adaptable).

**Status:** W3 scaffold. Offline-coupled mode functional (record arm
trajectory → replay in sim → diff). Runtime-coupled mode out-of-scope
for Phase 1 (see `docs/notes/l3_pipeline_design.md` §3).

## Layout

```
extensions/ros2_bridge/
├── package.xml                      ROS 2 ament_python metadata
├── setup.py                         Python entry points
├── basements_ros2/
│   ├── __init__.py
│   ├── sim_node.py                  MPMSolver wrapper, /tf in → /sim/wrench out
│   ├── trajectory_logger.py         pymycobot → trajectory.csv
│   ├── trajectory_replayer.py       trajectory.csv → MPMSolver
│   ├── driver_wrapper.py            pymycobot vendoring + reconnect logic
│   ├── force_estimators/
│   │   ├── __init__.py              Strategy registry
│   │   ├── base.py                  ForceEstimator ABC
│   │   ├── kinematic_only.py        No force; depth tracking only
│   │   ├── joint_current.py         pymycobot motor current → end-effector force
│   │   └── wrench_sensor.py         /wrench topic passthrough
│   └── config_loader.py             YAML → dataclass
├── config/
│   ├── mycobot_280_kinematic.yaml
│   ├── mycobot_280_joint_current.yaml
│   ├── mycobot_320_joint_current.yaml
│   ├── mycobot_pro600_joint_current.yaml
│   ├── mycobot_with_ft.yaml
│   ├── sandbox_ottawa_f65.yaml
│   └── sandbox_jumunjin.yaml
└── launch/
    ├── log_only.launch.py           Record only, no sim
    ├── replay_only.launch.py        Replay logged trajectory in sim
    └── runtime_coupled.launch.py    Phase 3 — not yet operational
```

## Quick start

```bash
# install editable
pip install -e .

# Record a myCobot trajectory (offline)
ros2 run basements_ros2 log_trajectory \
    --config config/mycobot_280_kinematic.yaml \
    --output trajectory_$(date +%F).csv

# Replay in sim and diff
ros2 run basements_ros2 replay_in_sim \
    --trajectory trajectory_2026-05-15.csv \
    --sandbox config/sandbox_ottawa_f65.yaml \
    --output sim_prediction.csv
```

## Anti-foreclosure compliance

Per `docs/notes/l3_pipeline_design.md` §8, this scaffold:
- Does not modify `MPMSolver` public API
- Treats `RigidColliderState` as serialisable plain struct
- Selects force estimator at runtime via YAML, not compile-time
- Does NOT depend on `mycobot_ros2` package (uses vendored pymycobot
  per `docs/notes/l3_pipeline_design.md` §Q2 decision)
