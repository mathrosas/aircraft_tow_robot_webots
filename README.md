# Aircraft Tow Robot

Autonomous mobile robot that tows aircraft (a Learjet 35) inside an aerodrome and parks them in a hangar. Built in [Webots](https://cyberbotics.com/) with Python controllers as part of my Mechatronics Engineering thesis at UTEC.

The robot maps the aerodrome with a 2D Hokuyo lidar, builds an occupancy grid, and then plans a path to the hangar. Four classical path planners are implemented and compared in the same world: **A\***, **Dijkstra**, **D\* Lite**, and **RRT**.

## Demo

![Aircraft tow robot mapping the aerodrome and towing a Learjet 35 into the hangar](media/aircraft_tow_gif.gif)

## Repository layout

```
.
├── assets/meshes/        STL meshes shared by all worlds (hangar, robot, Learjet 35)
├── docs/
│   ├── thesis-es.pdf         Full thesis (Spanish)
│   └── defense-slides-es.pdf Defense presentation (Spanish)
├── webots/
│   ├── astar/            Webots project — A* planner
│   ├── dijkstra/         Webots project — Dijkstra planner
│   ├── dstar_lite/       Webots project — D* Lite planner
│   └── rrt/              Webots project — RRT planner
├── prototypes/
│   └── algorithm_visualizer.py   Standalone 2D path-planning visualizer (BFS / Dijkstra / A*)
└── results/
    ├── algorithms/       Comparison plots across the four planners
    ├── motors/           Motor / load characterization
    └── pid/              PID tuning sweeps
```

Each `webots/<algorithm>/` folder is a self-contained Webots project. They share the same robot, same hangar, same world layout — they only differ in the planning algorithm and a thin `planning.py` glue layer. The 3D meshes live once in `assets/meshes/` and every world references them via relative paths.

## Architecture

The robot controller is structured as a **behavior tree** (`py_trees`) with three behaviors:

1. **Mapping** — drives a fixed waypoint loop around the aerodrome perimeter, fuses lidar scans into a 300×300 occupancy grid, dilates obstacles to a configuration space, and saves it as `cspace.npy`.
2. **Navigation** — pure waypoint following with a P-controller on heading and forward speed, used during the mapping phase.
3. **Planning** — once a map exists, plans a path from the current pose to the hangar entrance using one of the four algorithms. Re-plans on lidar-detected obstacles using a SLAM-style local rescan.

The robot has a 4-wheel differential-style drive with a hinge joint towing the Learjet 35.

```
            ┌─ Mapping ───────────┐
Behavior    │  (build cspace.npy) │
Tree   ──── ├─ Navigation ────────┤ ──── Planning ──── Hangar
            │  (perimeter loop)   │      (A*/D*/RRT)
            └─────────────────────┘
```

## Path planners

| Algorithm | File                      | Notes                                                                 |
|-----------|---------------------------|-----------------------------------------------------------------------|
| A\*       | `webots/astar/.../a_star.py`         | Euclidean heuristic, 8-connected grid                       |
| Dijkstra  | `webots/dijkstra/.../dijkstra.py`    | Uniform-cost reference                                      |
| D\* Lite  | `webots/dstar_lite/.../d_star_lite.py` | Incremental re-planner, ideal for unknown / changing maps |
| RRT       | `webots/rrt/.../rrt.py`              | Sampling-based, with an A\* smoothing pass                  |

Comparison plots (trajectory / distance / speed) are in [`results/algorithms/`](results/algorithms).

## Requirements

- [Webots R2023b](https://cyberbotics.com/) or newer
- Python 3.9+
- See [`requirements.txt`](requirements.txt) for Python packages

```bash
pip install -r requirements.txt
```

## Running a simulation

1. Open Webots.
2. **File → Open World…** and pick e.g. `webots/astar/worlds/Towbot.wbt`.
3. Make sure Webots is using the Python interpreter you installed the requirements into (`Tools → Preferences → Python command`).
4. Hit play.

The mapping phase runs until `cspace.npy` is saved (a few minutes of sim time). On subsequent runs the map is reused and the robot heads straight into planning.

To compare algorithms, just open a different `webots/<algo>/worlds/Towbot.wbt`. The shared meshes are pulled from `../../../assets/meshes/...`.

## Standalone path-planning visualizer

`prototypes/algorithm_visualizer.py` is a tiny matplotlib demo that runs BFS / Dijkstra / A\* on a random 20×30 grid — useful for sanity-checking the algorithms without spinning up Webots.

```bash
python prototypes/algorithm_visualizer.py
```

## Roadmap

- [ ] Replace the hand-coded perimeter sweep with frontier-based exploration
- [ ] Add Hybrid A\* / kinodynamic planning to respect the trailer's turning radius

## License

[MIT](LICENSE)

## Author

Mathias Rosas — Mechatronics Engineering thesis, UTEC.
