# Weakly electric fish biomechanical model

A reproducible MuJoCo model of **Gnathonemus petersii**, built from the supplied textured fish mesh. The model has 14 rigid body regions, seven whole fins, a separate chin with three rotational DOFs, and 40 position actuators. Swimming uses coordinated body, caudal and pectoral strokes.

[Pipeline documentation](fish_mujoco/README.md) · [Forward swimming](fish_mujoco/videos/03_forward_swim.mp4) · [Chin demonstration](fish_mujoco/videos/08_chin_3dof.mp4) · [Force measurements](fish_mujoco/reports/propulsion_revision.md)

![Swimming model with world-fixed ground grid](fish_mujoco/reports/behavior_forward.png)

## Get the complete assets

Meshes, textures, reports, publication figures and videos are included. Binary assets use **Git LFS**, including the original texture archive. Install Git LFS before cloning, then pull its objects:

```sh
git lfs install
git clone git@github.com:gizemozd/wef_biomechanical_model.git
cd wef_biomechanical_model
git lfs pull
```

## Rebuild and run

```sh
make setup
make all
```

The root Makefile forwards to `fish_mujoco/`. Setup uses `uv` to create a Python 3.12 environment with pinned dependencies. Full regeneration starts from `data/`, evaluates the configured controllers, renders videos, and runs validation. Headless rendering uses CGL on macOS or EGL/OSMesa on Linux. System `ffmpeg` and `ffprobe` are required.

For a focused update:

```sh
cd fish_mujoco
.venv/bin/python evaluate_behaviors.py
.venv/bin/python render_videos.py --only swimming
.venv/bin/python -m pytest tests -q
```

`config.yaml` controls geometry, joints, fluid parameters, behavior overrides and cameras. Videos 03–06 hold the chin neutral, retain both caudal and pectoral actuation, and use a stationary ground grid plus a fixed top camera. The dedicated chin clips demonstrate independent yaw, pitch and roll.

## Repository contents

| Path | Contents |
|---|---|
| `data/` | Original model formats, texture maps and source archives |
| `fish_mujoco/` | Executable segmentation, model generation, controllers and rendering pipeline |
| `fish_mujoco/assets/`, `fish.xml` | Generated textured meshes and MuJoCo model |
| `fish_mujoco/reports/` | Anatomy references, geometry checks, simulation results and render verification |
| `fish_mujoco/videos/` | H.264 demonstrations and preview GIF |
| `paper/` | Publication figure generator, configuration and exports |
| `blender/`, `docs/legacy_rigging_plan.md` | Earlier Blender rigging work and historical planning notes |

The model is a mechanical approximation of an external mesh. Joint limits and controller settings are engineering assumptions; the ellipsoid fluid model does not resolve wakes or flexible fins. Hover remains approximate, and the reports record actual displacement and forces rather than assuming that commands achieve ideal swimming.
