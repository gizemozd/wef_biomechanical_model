# Phase 5 — rendering

Offscreen `mujoco.Renderer`, 1920×1080, 60 fps, H.264/yuv420p MP4. Backend: `cgl`. Linux launcher selects EGL and retries OSMesa if rendering initialization fails; this macOS build uses CGL.

Videos 03–06 were regenerated with the chin held neutral and a world-fixed 10 mm ground grid. Heavier lines appear every 50 mm, and gold lines mark the world origin. The side and caudal-fin views track the fish; the top camera stays fixed. Overlays show x/y/z displacement from the start, measured COM velocity along the head's anterior axis, and actual fin angles. Both fin groups remain active. See [the ground and chin report](swimming_ground.md) and [the current force measurements](propulsion_revision.md). All four videos were checked at 1920×1080, 60 fps, 360 frames; see swimming_video_validation.json.

The swimming root is freely simulated, not prescribed. The ROM/exploded clips are explicitly kinematic demonstrations. A 20 mm perspective scale is drawn at the top view's center depth. No tank or misleading bubbles/current visualization is added. Video 07 retains its previous export; re-rendering regenerates it from the current configured trajectory. The forward preview GIF is updated.

![Verified video frames](swimming_videos_preview.png)

`texture_continuity.png` compares the same camera/lighting at rest; `texture_difference_x4.png` exposes differences from normal splitting, caps and repairs. `seam_closeups.png` shows a representative body interface at rest and both ROM limits. `chin_dofs.png` and the extra `08_chin_3dof.mp4` show independent yaw, pitch and roll. Dorsal and anal fins are whole rigid meshes. Their long bases follow one body segment each, so body bending can change attachment alignment; no flexible skin is claimed.
