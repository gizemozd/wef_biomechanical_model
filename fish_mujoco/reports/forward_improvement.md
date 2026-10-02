# More effective forward swimming

Historical speed improvement before the requested larger tail stroke. The current video uses the wider stroke measured in [larger_tail.md](larger_tail.md), with slightly less forward travel. The measurements below describe the previous, faster gait.

The fish advances **43.85 mm in six seconds**, compared with **6.64 mm** for the prior gait: **6.61× farther** at the same simulation/video duration. Mean speed along the head axis over the last three seconds increases from **0.00811 to 0.04424 BL/s**.

The forward-only overrides are 6 Hz, body amplitude fraction 0.95, pectoral amplitude 14° before the 24/22 CPG scale, pectoral feathering fraction 0.3, and a −60° caudal phase offset. The caudal target remains counter-bending, with gain 2.2 and an 8° cap. The phase offset compensates relative body/fin tracking lag: simply increasing frequency made the tail resist forward motion. Nine targeted six-second comparisons, including the original baseline, are recorded with complete parameter overrides in `forward_improvement_trials.json`.

The tail's mean anterior force is **0.12308 mN** and the pectorals' is **0.32592 mN**. With the tail held neutral, speed falls to **0.02918 BL/s**; with pectorals held neutral it falls to **0.02454 BL/s**. Both groups therefore contribute to the improved gait. No external thrust or prescribed root motion is used.

Joint limits, meshes, mass, actuator gains and fluid proxies/coefficient values are unchanged. The chin targets remain zero, and the fixed ground grid/camera are retained. Backward, hover and turning parameters are unchanged; regenerated trajectories match their prior versions to within 1.2e-15 in generalized position. Video 03 and the preview GIF are re-rendered at the original 1920×1080, 60 fps and six-second duration.

This is improved performance within the simplified ellipsoid model, not validation against live-fish measurements. Final displacement includes **12.24 mm upward drift** and **-0.33 mm lateral drift**. The frequency, amplitudes and delay are engineering settings. No claim of straight, constant-depth or steady-state biological swimming is made.

Reproduce this historical gait by passing its `parameters` dictionary from `forward_improvement_trials.json` as `overrides` to `evaluate_behaviors.evaluate`; the first trial's dictionary reproduces the original slow baseline. The normal evaluation/rendering commands now use the wider tail stroke from the current config.

See [the current forward frame](behavior_forward.png) for the latest video.
