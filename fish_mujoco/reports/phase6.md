# Phase 6 — verification and deliverables

- Model compiles; explicit mass and inertia are finite. 22 closed segment solids retain OBJ face-corner UV coordinates.
- Independent tests: see `pytest.txt` for the complete result. Tests cover compile/forward, topology/UVs, joint and actuator ranges, seam thresholds, a 5 s driven simulation, 5 s passive buoyancy, controller clipping, actual joint-limit response, three independent chin DOFs and single-mesh median fins.
- Total mass **60.130 g**. Passive root drift after 5 s: **5.31e-14 m**; max speed 1.84e-13; simulation rate **3.0× real time** (rendering excluded).
- Maximum sampled body seam gap: **0.996% of local height**. Maximum measured penetration: 0.1750 mm, including designed overlap. Rest and full-ROM thresholds: 1% and 3%.
- Nine H.264 videos checked with ffprobe at 1920×1080, 60 fps. `videos/preview.gif` is a small behavior preview. Per-video durations and frame counts are in `validation.json`.
- Raw OBJ SHA256: `af2867bb01610841bea88f0473e1165c8035b9f42066587b28e8066437851e1c`. Installed package versions are pinned in `requirements.txt`; all derived assets are generated from the raw OBJ, textures and config.

The model is a tested mechanical approximation, not a validated biological digital twin. Body seams are checked at finitely sampled individual-joint poses; deforming fin attachments, arbitrary compound bends, CFD thrust, electrical fields and empirical muscle parameters are not established. Backward/hover labels name control commands; read actual measured displacement in `behaviors.json`.
