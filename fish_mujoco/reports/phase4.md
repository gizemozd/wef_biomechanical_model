# Tail and pectoral contribution revision

The previous forward controller commanded only 1° at the caudal hinge and disabled the tail during hover. Independent caudal pitching generated backward force in this approximation. The revised forward/turning gait counter-rotates the caudal fin against the summed body-yaw wave. This lets rear-body translation and caudal rotation cooperate. The body amplitude envelope increases posteriorly within the existing seam-safe ROM. The caudal fin and every other exterior mesh remain rigid. The dorsal and anal fins remain single neutral meshes.

In videos 03–06 the chin's three position targets now stay at zero; active sensory scanning is reserved for the dedicated chin demonstration. The current force/trajectory measurements are re-simulated with this neutral chin command.

Pectoral rotation about the span (global local-body y axis) now uses the same sign on left and right, as required by sagittal reflection of an axial vector. Its sign reverses for backward strokes. Previously that component was mirrored incorrectly; it did not reliably reverse pectoral thrust. Backward uses an independent lateral tail stroke together with reverse-feathered pectorals. Hover keeps both groups active with approximately opposing mean forces; it is open-loop and drifts. Turning retains coordinated tail strokes plus body/tail bias and unequal pectoral amplitudes.

The reference-video revision expands the five posterior body yaw limits from ±0.716° to ±1.432° after individual and compound seam checks. Meshes, actuator gains, inertia, fluid proxies and fluid coefficients remain unchanged. Native MuJoCo ellipsoid forces drive a free root; no trajectory prescription or added thrust is used. The base gaits use 3 Hz and 1.2 BL wavelength. The forward gait uses its own frequency, amplitude and caudal phase overrides from config.yaml. `reference_swim.md` describes the current slower body/tail cadence, independently cycling pectorals and stronger posterior bending. `carangiform.md` and `tail_increment.md` document the earlier gaits. `forward_improvement.md` and `larger_tail.md` preserve the earlier speed and tail-amplitude comparisons. Historical `tuning.json`/`tuning.png` are retained and are no longer implicit overrides of config.yaml.

## Measured contributions

Native fluid forces are measured by subtracting each fin group's contribution at identical simulated positions and velocities. Statistics average the final half of a six-second run. Force is projected onto the head's anterior axis. Positive means forward; negative means backward. The other bodies, fins and chin also interact with the fluid, so these are fin-force comparisons, not percentages of swimming speed or biological efficiency.

Independent ablations hold the selected group's position targets at neutral; their passive geometry and fluid forces remain present. The other controller targets are unchanged. These experiments measure the effect of active strokes, rather than removing anatomy.

| Gait | Tail force (mN) | Pectoral force (mN) | Both active (BL/s) | Tail neutral (BL/s) | Pectorals neutral (BL/s) |
|---|---:|---:|---:|---:|---:|
| forward | +0.20217 | +0.16628 | +0.03467 | +0.00206 | +0.02316 |
| backward | -0.13530 | -0.16906 | -0.03384 | -0.02380 | -0.02725 |
| hover | -0.10293 | +0.09132 | -0.00384 | +0.01244 | -0.01771 |
| turning | +0.10290 | +0.01276 | +0.01376 | -0.02089 | +0.01075 |

![Native fin forces and control ablations](propulsion_contributions.png)

Complete traces, achieved joint ranges, displacement, yaw change, Reynolds number and per-group RMS forces are in `propulsion_audit.json`, `behaviors.json`, and `trajectory_*.npz`. The JSON beside each trajectory fingerprints the current MJCF, controller, joint definitions and simulation config. Rendering automatically re-simulates stale trajectories without tuning. Videos 03–06 use side, top and caudal-close-up views, with instantaneous measured axial COM velocity and actual joint angles.

## Evidence and limits

[Lannoo & Lannoo (1993), pp. 163–164](https://www.ikhebeenvraag.be/mediastorage/FSDocument/56/Lannoo-157.pdf) discusses carangiform movements in Gnathonemus, a semi-stiff body/peduncle, and lateral tail strokes during backward probing. This supports including active body/caudal motion. It does not establish the amplitudes, phase relationship or thrust shares used here; those remain engineering settings.

[MuJoCo's fluid documentation](https://mujoco.readthedocs.io/en/stable/computation/fluid.html) describes stateless ellipsoid approximations. This model lacks resolved wakes, fin flexibility and fluid coupling between separate surfaces. Speeds are low, and lateral/vertical drift remains. A positive measured tail force and a successful ablation establish contribution in this simulation, not quantitatively validated fish biomechanics. Hover is approximate balance, not feedback station keeping. Historical tuning results describe the previous controller.

Reproduce without optimization:

```sh
.venv/bin/python evaluate_behaviors.py
.venv/bin/python render_videos.py --only swimming
.venv/bin/python -m pytest tests -q
```

`run_pipeline.py` / `make all` now evaluates the configured gaits in phase 4 rather than replacing them with a new search winner.
