# Forward swimming from the video reference

The requested priority is body and tail motion during forward swimming. The model now combines slower caudal strokes, stronger posterior-trunk bending and a separate pectoral cadence. The root moves under native MuJoCo fluid forces. No swimming path or extra thrust is prescribed.

## Reference and interpretation

Reference: [AquaVerse, “Elephant Nose Fish: The Mysterious Trunked Swimmer!”](https://www.youtube.com/shorts/Vf1ye8vfjK0), uploaded 2024-10-31, inspected 2026-10-02. The inspected video is 720×1280 at 30 fps, duration 21.667 s. Frame inspection covered the whole clip, with denser sampling of 0–3.7 s and 4.2–7.6 s. The former provides a front-quarter view of caudal strokes; the latter shows a side view before a nose-down transition. Later bottom probing is excluded from the requested behavior. The source clip and extracted frames are not redistributed in this repository.

| Visible feature | Model approximation |
|---|---|
| Tail sweeps are individually visible; the front of the body stays relatively quiet | 2.5 Hz posterior body wave, low anterior command envelope |
| Rear body translates as the caudal fin rotates | Wider posterior yaw ROM with coordinated caudal counter-bending |
| Pectorals remain active without an obvious fixed phase relationship to the tail | Independent 3.5 Hz pectoral cycle |
| Stroke strength varies through the footage | Smooth 25% body-wave amplitude modulation at 0.5 Hz |

The frequencies, modulation, phase and angular amplitudes are engineering choices guided by qualitative inspection. Perspective, partial occlusion and edits prevent a reliable calibrated 3D pose or source swimming-speed estimate. This is not a frame-by-frame reconstruction, a fitted tailbeat measurement, or a claim that the source swims at the simulated speed.

## Geometry and actuation

The existing five posterior joints (`j_body_07_yaw` through `j_body_11_yaw`) now permit ±1.432° instead of ±0.716°. Anterior and peduncle yaw remain ±0.716°, and pitch remains ±0.5°. These effective ranges are below the requested regional caps and come from `joints.seam_displacement_fraction_by_region`. Changing ranges uses the same rigid meshes, textures, chin attachment, mass, fluid proxies and actuator gains.

There are 130 individual-axis seam poses and 52 combined yaw/pitch corners. The largest sampled gap is **2.111% of local body height**, below the unchanged 3% bent limit. The largest rest gap is 0.000%, below 1%. These finite checks do not prove arbitrary surface tangency. No flexible cover or skin is introduced. Whole dorsal/anal fins remain neutral; their long rigid bases cannot follow a deforming body perfectly. Chin commands remain zero in videos 03–06; the fixed ground grid remains visible.

The selected forward gait uses wavelength 1.2 BL, body fraction 0.95, caudal counter-bending gain 1.6, phase -10°, and command cap ±12°. Pectoral amplitude is 16° before the shared 24/22 scaling, with feathering fraction 0.3. Defaults for the other behaviors retain their previous cadences; their trajectories and videos are regenerated because the physical posterior ranges changed.

## Measured outcome

| Gait | Body/tail Hz | Caudal sweep, peak-to-peak (°) | Forward travel in 6 s (mm) | Mean axial speed, final 3 s (BL/s) |
|---|---:|---:|---:|---:|
| Previous fast gait | 6 | 16.77 | 42.89 | 0.04358 |
| Reference-inspired gait | 2.5 | 22.90 | 32.07 | 0.03467 |

Tail-base lateral sweep relative to the head increases from **4.11 to 10.11 mm**. Actual caudal hinge sweep is **22.90°**. Broader, slower strokes approximate the visible motion better, but do not outperform the previous fast gait in forward travel. The forward distance changes by -25.2%.

The tail contributes **+0.2022 mN** and paired pectorals **+0.1663 mN** mean anterior force over the final half of the run. The tail supplies 54.9% of those two groups' combined axial force, not a percentage of all forces or biological efficiency. Holding the caudal target neutral reduces speed to **0.00206 BL/s**; holding the pectorals neutral gives **0.02316 BL/s**. Both groups therefore contribute in independent free-root runs. Reynolds number based on body length and mean axial speed is approximately 1387.

![Achieved body/tail motion and free-root displacement](reference_swim_comparison.png)

![Forward swimming render](behavior_forward.png)

## Focused trials and limits

`reference_swim_trials.json` records 13 focused controller trials, separating the old and expanded posterior ranges. Slowing the earlier rigid-tail stroke without enough tail-base translation produced drag. Expanded posterior ranges let the tail translate farther while counter-rotating, producing positive forward force at the slower cadence. The selected gain avoids the slight caudal-limit overshoots of larger-gain trials. No general tuning grid, mesh recutting, fin-proxy resizing or fluid-coefficient search was run.

[MuJoCo's ellipsoid fluid model](https://mujoco.readthedocs.io/en/stable/computation/fluid.html) uses local stateless force approximations. It does not resolve wakes, flexible fins or hydrodynamic interaction between surfaces. Realistic-looking kinematics do not establish realistic thrust or efficiency. The model retains passive forebody movement, modest upward/lateral drift and open-loop control. Exact 3D agreement with the reference would require calibrated multi-view measurements and a richer body/fin model.

## Reproduce without optimization

```sh
.venv/bin/python segment_mesh.py --body-rom-only
.venv/bin/python build_model.py
.venv/bin/python evaluate_behaviors.py
.venv/bin/python render_videos.py --only swimming
.venv/bin/python render_videos.py --only body_undulation
.venv/bin/python render_videos.py --only rom
.venv/bin/python -m pytest tests -q
```

`make all` applies the same regional range checks during segmentation and reproduces the configured motion and videos from raw assets. `reference_swim.py` regenerates this report and comparison from the archived pre-reference measurements and current simulated trajectory; it is called after the phase-4 audit. Previous gait reports remain historical records.
