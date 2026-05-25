# caatingarobotics — Sowbot Jazzy fork

**Fork of [`caatingarobotics`](https://github.com/joaodemouragy-hash/caatingarobotics)
retargeted to ROS 2 Jazzy + Gazebo Harmonic, for the Sowbot platform.**

The upstream project targets ROS 2 Humble / earlier Gazebo. This fork ports the
workspace to **Jazzy** (Ubuntu 24.04 Noble) and **Gazebo Harmonic**, and adds the
`sowbot_row_follow` package for monocular crop-row following.

> ⚠️ **Experimental.** The Jazzy/Harmonic port is a work in progress and has not
> been validated on physical hardware.

## How this repo is used — read this first

**You do not clone or build this repository directly.** It is consumed by the
**[`feldfreund_devkit_ros`](https://github.com/Agroecology-Lab/feldfreund_devkit_ros)**
devkit, which is the supported way to build and run everything.

The devkit's `docker/Dockerfile` clones this fork automatically during the image
build:

```dockerfile
git clone --depth 1 -b main \
  https://github.com/samuk/caatingarobotics src/caatingarobotics
```

All four packages here are then built by `colcon` inside the devkit's
`ros:jazzy`-based Docker image, alongside the rest of the Sowbot stack
(`devkit_launch`, `devkit_driver`, `fusioncore`, `topological_navigation`,
`virtual_maize_field`, etc.).

**Practical consequence:** to pick up changes made in this fork, push them to the
`main` branch and rebuild the devkit image.

## Running (via the devkit)

Everything runs inside the devkit's Docker container, orchestrated by its
`manage.py`. In short:

```bash
# 1. Clone the devkit (not this repo)
git clone https://github.com/Agroecology-Lab/feldfreund_devkit_ros.git
cd feldfreund_devkit_ros

# 2. Build the image — the Dockerfile pulls this fork in automatically.
#    Add +sim to also install Gazebo Harmonic (needed for the simulation).
./manage.py build           # or:  ./manage.py build +sim
#    ./manage.py full-build  # --no-cache rebuild

# 3. Run
./manage.py                 # sim / limbic stack — exercises agro_robot_sim
./manage.py neo             # Neo row-follow stack — runs sowbot_row_follow
```

`./manage.py neo` launches the devkit's `neo.launch.py`, which delegates to
`sowbot_row_follow/launch/crop_row_nav.launch.py` — i.e. the `crop_row_node` from
this repo. Launch arguments pass straight through, e.g.:

```bash
./manage.py neo camera_index:=2
```

For full setup (the `.env` hardware-detection step, USB/GNSS handling, the
NiceGUI cockpit, sim vs. hardware mode, etc.) see the **devkit README**. This
README only covers what the `caatingarobotics` packages contribute.

## Packages in this repo

The devkit image builds all four packages, but the current Sowbot work only
launches two of them:

| Package | Purpose | Status |
|---|---|---|
| `agro_robot_sim` | Robot URDF/Xacro, Gazebo Harmonic worlds, simulation assets | **Active** — used by the sim stack |
| `sowbot_row_follow` | Monocular RGB crop-row following (`crop_row_node`, `limbic_row_follow`) | **Active** — used by `./manage.py neo` |
| `caatinga_vision` | Vision / inference pipeline | Not used yet — candidate for future work |
| `caatinga_nav` | Standalone Nav2 navigation stack | Not used (the devkit uses LCAS `topological_navigation` instead) |

### `sowbot_row_follow`

Monocular RGB crop-row following, derived from
[visual-multi-crop-row-navigation](https://github.com/Agricultural-Robotics-Bonn/visual-multi-crop-row-navigation)
(Agricultural-Robotics-Bonn, BSD-2-Clause). ExG vegetation index + Otsu
threshold, scan-window line fitting, Cherubini & Chaumette visual servoing.
Publishes `/aoc/conditions/row_offset`, `/aoc/conditions/row_heading_error`, and
the `/aoc/heartbeat/neo_vision` perception heartbeat into the AOC navigation
layer.

It contains two nodes:

- `crop_row_node` — runs on the **Neo** SBC; opens the camera directly and does
  the row-following. This is what `./manage.py neo` runs.
- `limbic_row_follow` — runs on **Limbic** as a Nav2 `NavigateToPose` action
  server, enabling/monitoring Neo over the crossover link.

**Camera calibration is mandatory before any field use** — set `camera_height_m`
and `camera_tilt_deg` in `src/sowbot_row_follow/config/crop_row_params.yaml`. See
that package's [`README.md`](src/sowbot_row_follow/README.md) for the full
parameter and tuning notes.

### `agro_robot_sim`

Robot description (Xacro/URDF), Gazebo Harmonic worlds, and simulation assets for
the Caatinga agro-robot. In the devkit's sim mode the robot model from this
package is spawned into a generated `virtual_maize_field` world. Requires the
devkit image to be built with `+sim`.

## Repository structure

```text
caatingarobotics/
├── datasets/
│   └── agro_v1/data.yaml
├── src/
│   ├── agro_robot_sim/        # Active — simulation (URDF, worlds, launch)
│   │   ├── launch/
│   │   ├── config/
│   │   ├── maps/
│   │   ├── urdf/
│   │   └── worlds/
│   ├── sowbot_row_follow/     # Active — monocular crop-row following
│   │   ├── sowbot_row_follow/
│   │   ├── launch/
│   │   └── config/
│   ├── caatinga_vision/       # Not used yet — vision/inference pipeline
│   └── caatinga_nav/          # Not used — standalone Nav2 stack
├── CHANGELOG.md
└── README.md
```

## Developing a package standalone

The supported run path is the devkit image above. If you need to iterate on a
single package in isolation, you can still build it in a local ROS 2 Jazzy
environment:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/sowbot_row_follow --ignore-src -r -y
colcon build --symlink-install --packages-select sowbot_row_follow
```

Note this does not reproduce the devkit runtime (CycloneDDS config, drivers,
launch orchestration). Rebuild the devkit image to run changes for real.

## Licensing

Licensing is mixed across the workspace:

- `agro_robot_sim`, `caatinga_vision`, `caatinga_nav` — Apache-2.0
- `sowbot_row_follow` — BSD-2-Clause (derived from the Bonn package; see its
  README for full attribution)

## Notes and future work

- The CI workflow under `.github/workflows/` still targets ROS 2 Humble and has
  not been updated for the Jazzy port.
- The devkit image already ships a CPU PyTorch + **YOLOX** runtime and
  YOLOX-Nano weights, so revisiting `caatinga_vision` for perception work later
  should not need new dependencies.
- `cv_bridge` must match the system NumPy ABI. A runtime error like *"a module
  compiled using NumPy 1.x cannot be run in NumPy 2.x"* means the `cv_bridge`
  build and the runtime `numpy` disagree — this is resolved in the devkit's
  Docker image, not in this repo.
