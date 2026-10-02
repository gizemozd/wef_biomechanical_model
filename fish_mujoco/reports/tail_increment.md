# Modest tail-amplitude increase with the carangiform body wave

The forward caudal-fin swing increases from approximately **±7.8° to ±8.4°**, a **7.8% wider peak-to-peak sweep**. The tail base's lateral sweep relative to the head also increases from **3.83 to 4.11 mm**. The carangiform body envelope, frequency, timing and pectoral commands are retained.

Only two forward-gait settings change: `caudal_counterbend_gain` increases from **3.0 to 3.6** and `caudal_amplitude_deg` from **11° to 12°**. The command cap now equals the existing caudal hinge limit; the achieved motion remains inside that range. No joint limits, meshes, textures, actuator gains, masses or fluid coefficients change. Other gaits are unchanged.

| Measurement | Before | Increased stroke |
|---|---:|---:|
| Actual caudal hinge range | −7.777 to +7.789° | −8.381 to +8.394° |
| Actual caudal hinge sweep | 15.566° | 16.775° |
| Tail-base lateral sweep relative to head | 3.826 mm | 4.112 mm |
| Forward COM displacement after 6 seconds | 45.337 mm | 42.885 mm |
| Mean anterior speed, final 3 seconds | 0.04573 BL/s | 0.04358 BL/s |
| Mean caudal anterior force | +0.22159 mN | +0.18575 mN |
| Mean paired-pectoral anterior force | +0.31402 mN | +0.32598 mN |

The larger stroke **does not increase forward performance**: travel decreases **5.4%**. Both fin groups still propel the fish. Holding the tail targets neutral lowers speed to **0.02945 BL/s**; holding the pectorals neutral lowers it to **0.02179 BL/s**. The tail supplies **36.3%** of the combined tail-plus-pectoral mean anterior force. These are measurements within the current ellipsoid fluid approximation, not validated biological efficiencies. The six-second run also has 12.01 mm upward drift and 1.89 mm lateral drift.

`tail_increment.json` stores the before/after measurements and complete resolved controller settings. Sweeps and forces use the final three seconds; forward displacement uses world x, while force and speed use the head's anterior axis. The chin targets remain zero, and the world-fixed ground grid remains visible in the video.

All **15 tests passed in 25.23 s**, including actual posterior-body motion and both fin groups' propulsion checks (`tail_increment_pytest.txt`). Video 03 and the preview GIF were re-rendered. The final H.264 video is 1920×1080 at 60 fps; validation is recorded in `swimming_video_validation.json`.

![Updated forward swimming](03_forward_swim_verified_frame.png)

Reproduce from `fish_mujoco/`:

```sh
.venv/bin/python evaluate_behaviors.py
.venv/bin/python render_videos.py --only forward
.venv/bin/python -m pytest tests -q
```

`make all` also regenerates the current configured gait. To reproduce the preceding stroke, set the forward caudal gain to 3.0 and its command cap to 11.0; all other controller parameters are unchanged.
