# Carangiform-style posterior body actuation

Concentrating bending in the posterior trunk improves this model's forward propulsion. The caudal-fin base sweeps **18.4% farther sideways**, mean tail thrust more than doubles, and six-second forward travel increases **8.3%**, from **41.84 to 45.34 mm**. This comparison changes the body amplitude envelope alone; the tail-fin, pectoral, frequency, wavelength and physics settings are retained.

## Anatomical motivation and implementation

[Lannoo & Lannoo (1993), pp. 163–164](https://www.ikhebeenvraag.be/mediastorage/FSDocument/56/Lannoo-157.pdf) discusses carangiform movements in Gnathonemus, a semi-stiff body axis, and Gemminger bones maintaining a relatively rigid electric-organ peduncle. This supports moving that peduncle sideways through bending ahead of it. It does not establish numerical species-specific amplitudes or joint ranges.

The `carangiform` controller profile uses piecewise-linear amplitude fractions along the existing body hinges:

```yaml
controller:
  gaits:
    forward:
      profile: carangiform
      carangiform_envelope: [[0.0, 0.03], [0.3, 0.03], [0.55, 1.0], [0.82, 1.0], [1.0, 0.3]]
```

The first coordinate runs from the first body hinge (0) to the last (1), not from snout to tail tip. The second coordinate is a fraction of each hinge's existing yaw limit, further scaled by `body_amplitude_fraction: 0.95`. Low anterior commands rise to stronger bending in posterior joints 07–11, then taper across peduncle joints 12–13. The forebody retains passive compliance; small commanded anterior motion does not make it perfectly rigid. Actual anterior bending remains smaller than posterior bending, although some anterior hinges move more than in the previous gait because their mechanical loading changes.

The caudal target still counter-rotates against the traveling body wave with gain 3, an 11° command cap and a −45° phase offset. Frequency is 6 Hz and wavelength 1.2 BL. The tail now both translates laterally with the rear body and rotates at its base. Pectorals remain active, dorsal/anal fins remain whole neutral meshes, and all three chin targets remain zero. No external thrust or prescribed root trajectory is applied.

## Measured comparison

| Measurement | Previous larger-tail gait | Carangiform-style gait |
|---|---:|---:|
| Six-second forward COM displacement | 41.843 mm | 45.337 mm |
| Mean anterior speed, final 3 s | 0.04254 BL/s | 0.04573 BL/s |
| Tail-base lateral sweep relative to head | 3.231 mm | 3.826 mm |
| Body joint 09 yaw sweep | 0.662° | 1.123° |
| Body joint 10 yaw sweep | 0.747° | 1.137° |
| Last peduncle joint 13 yaw sweep | 0.981° | 0.422° |
| Caudal-fin hinge sweep | 13.444° | 15.566° |
| Mean caudal anterior force | +0.10364 mN | +0.22159 mN |
| Mean paired-pectoral anterior force | +0.33603 mN | +0.31402 mN |

Sweeps and forces use the final three seconds. Sweep means peak-to-peak actual motion, not the command cap. Tail-base position is transformed into the head frame, excluding root translation and rotation. Forward displacement is measured along world x; speed and forces are projected onto the instantaneous head anterior axis.

![Achieved bending and forward displacement](carangiform_comparison.png)

The tail supplies **41.4%** of the combined tail-plus-pectoral mean anterior force. Holding the tail targets neutral reduces mean speed to **0.02945 BL/s**; holding the pectoral targets neutral reduces it to **0.02719 BL/s**. Holding body targets neutral while retaining the same fin commands produces **−0.00772 BL/s**. These separate free-root runs show that body/fin coordination matters and that both fin groups contribute. They preserve passive anatomy and hydrodynamic forces.

Six focused comparisons are recorded in `carangiform_trials.json`: the previous gait, the selected body-envelope change, and four variants of caudal gain/phase/wavelength. The selected envelope-only change increases both tail-base movement and forward travel while retaining the previous larger-tail settings. Some variants reach 47.18 mm, but have smaller tail-base excursions. This is not a claim of globally optimal speed.

## Limits and verification

No meshes, textures, physical joint ranges, actuator gains or fluid proxies changed. Existing body yaw limits remain approximately ±0.716° and pitch limits ±0.5°. The new motion stays within them. The rigid interfaces retain their existing seam tests; arbitrary compound poses are not analytically guaranteed seamless. The entire exterior remains rigid.

The amplitude envelope is an engineering approximation, not measured Gnathonemus kinematics. MuJoCo's [stateless ellipsoid fluid model](https://mujoco.readthedocs.io/en/stable/computation/fluid.html) does not resolve wakes or fluid-mediated interactions between fins. The run still has **12.21 mm upward drift** and **1.76 mm lateral drift**; it is not constant-depth or closed-loop swimming.

All **15 tests passed in 29.15 s**, including a new check of actual posterior-dominant bending, peduncle taper and body joint limits. Video 03 and the preview GIF are re-rendered with the neutral chin and fixed ground references. The H.264 export is 1920×1080 at 60 fps, with all 360 frames successfully decoded. Backward, hover and turning videos and simulated positions remain unchanged. Detailed video checks are in `swimming_video_validation.json`; test output is in `carangiform_pytest.txt`.

## Reproduction

Run from `fish_mujoco/`:

```sh
.venv/bin/python evaluate_behaviors.py
.venv/bin/python render_videos.py --only forward
.venv/bin/python -m pytest tests -q
.venv/bin/python reports/plot_carangiform.py
```

`make all` rebuilds the current model and videos from the configured profile. To reproduce a historical trial, pass its resolved `parameters` from `carangiform_trials.json` as `overrides` to `evaluate_behaviors.evaluate(config(), model, 'forward', overrides=parameters)`. `carangiform_comparison.json` stores selected/baseline statistics and the additional body-neutral experiment. `carangiform_comparison_traces.npz` supplies the two saved motion traces used by the plot.
