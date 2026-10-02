# Larger forward tail stroke

Historical comparison before the posterior-body revision. The current forward video uses the carangiform-style envelope documented in [carangiform.md](carangiform.md). The measurements below describe the preceding larger-tail gait.

The forward tail movement is larger, but **the extra amplitude does not improve forward swimming in this model**. The selected stroke increases the caudal hinge sweep by **31.8%** and approximately doubles the tail-tip lateral excursion. Six-second forward travel decreases **4.6%**, from **43.85 to 41.84 mm**. The tail remains useful: holding its position target neutral reduces speed by **31.4%** relative to the selected gait.

| Measurement | Previous stroke | Larger stroke (current) |
|---|---:|---:|
| Caudal counterbend gain | 2.2 | 3.0 |
| Caudal command amplitude cap | 8° | 11° |
| Caudal phase offset | −60° | −45° |
| Actual caudal hinge range | −5.094 to +5.107° | −6.704 to +6.739° |
| Actual hinge peak-to-peak sweep | 10.201° | 13.444° |
| Tail-tip lateral excursion relative to head | 0.793 mm | 1.683 mm |
| Forward COM displacement after 6 s | 43.849 mm | 41.843 mm |
| Mean anterior speed during final 3 s | 0.04424 BL/s | 0.04254 BL/s |
| Mean caudal anterior force during final 3 s | +0.12308 mN | +0.10364 mN |
| Mean paired-pectoral anterior force during final 3 s | +0.32592 mN | +0.33603 mN |

![Measured tail motion and forward travel](larger_tail_comparison.png)

Joint sweep and tail-tip excursion use the final three seconds. The tip is the most posterior vertex of the caudal visual mesh, transformed through its simulated body pose into the head's frame; root translation and rotation are excluded. Forward distance is whole-fish COM displacement along world x. Speed and fin force are projected onto the instantaneous head anterior axis. These quantities need not have the same percentage change.

The new gait retains 6 Hz, a 1.2 BL body wavelength, body amplitude fraction 0.95, pectoral amplitude 14° before CPG scaling and pectoral feathering fraction 0.3. The caudal physical range remains ±12°; meshes, body joint limits, mass, actuator gains and fluid proxies are unchanged. Chin targets remain zero, and the fixed ground grid is retained. No artificial thrust or prescribed root movement is applied.

## Both fin groups contribute

In separate free-root runs with the same passive anatomy, holding the caudal target neutral gives **0.02918 BL/s**, versus **0.04254 BL/s** with both fin groups active. Holding the pectoral targets neutral gives **0.01947 BL/s**. The tail supplies **23.6%** of the combined caudal-plus-pectoral mean anterior force. This percentage excludes the drag of the remaining fish and is not a percentage of swimming efficiency. Native same-state force measurements and independent actuation ablations are in [propulsion_audit.json](propulsion_audit.json).

## Focused amplitude/timing comparison

Only caudal gain, command cap and phase changed in these eight six-second comparisons. Frequency, body wave, pectorals and physics were fixed. This is a small comparison around the existing forward gait, not a new whole-model optimization.

| Gain | Cap | Phase | Actual hinge sweep | Forward travel | Mean tail anterior force |
|---:|---:|---:|---:|---:|---:|
| 2.2 | 8° | −60° | 10.20° | 43.85 mm | +0.12308 mN |
| 3.0 | 11° | −60° | 12.35° | 41.87 mm | +0.10377 mN |
| 3.6 | 11° | −60° | 13.88° | 38.82 mm | +0.05972 mN |
| 3.0 | 11° | −90° | 9.73° | 36.05 mm | −0.02308 mN |
| **3.0** | **11°** | **−45°** | **13.44°** | **41.84 mm** | **+0.10364 mN** |
| 3.6 | 11° | −45° | 14.96° | 39.50 mm | +0.07599 mN |
| 4.2 | 11° | −45° | 15.95° | 37.04 mm | +0.04332 mN |
| 4.2 | 11° | −30° | 16.87° | 35.73 mm | +0.02362 mN |

Larger strokes reduced travel; the −90° phase trial even made the tail's mean axial force negative. The selected setting gives a visibly wider stroke while keeping the distance penalty below 5% and a positive contribution from both fin groups. The previous setting remains the faster one among these trials.

This result is specific to the rigid fins, stiff body ranges and MuJoCo ellipsoid fluid approximation. The solver does not resolve wakes or fin-to-fin hydrodynamic interactions. It does not establish that larger tail strokes are unhelpful in real fish. The selected run also rises 12.10 mm and drifts laterally 0.41 mm in six seconds; it is not straight, constant-depth swimming.

## Reproduction and validation

The current gait regenerates with the following commands from `fish_mujoco/`; the plotting command reproduces this historical comparison:

```sh
.venv/bin/python evaluate_behaviors.py --only forward
.venv/bin/python render_videos.py --only forward
.venv/bin/python -m pytest tests -q
.venv/bin/python reports/plot_larger_tail.py
```

`larger_tail_trials.json` stores each trial's resolved `parameters` dictionary. Pass that dictionary as `overrides` to `evaluate_behaviors.evaluate(config(), model, 'forward', overrides=parameters)` to replay any comparison. The two saved trajectories and head-relative tip measurements used by the figure are in `larger_tail_comparison_traces.npz`.

All **14 tests passed in 26.81 s**. The new `03_forward_swim.mp4` is H.264, **1920×1080, 60 fps, 360 frames, six seconds**, and every frame decoded without errors. The preview GIF and report images were refreshed. Backward, hover and turning videos are byte-identical to their prior exports; their regenerated simulated positions are also identical. Chin controls remain zero in videos 03–06 with passive movement below 0.1°. See [video validation](swimming_video_validation.json) and [the verified frame](03_forward_swim_verified_frame.png).
