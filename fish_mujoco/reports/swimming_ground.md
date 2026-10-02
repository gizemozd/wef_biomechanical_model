# Stationary chin and ground reference: videos 03–06

All three chin position targets are held at zero for forward, backward, hover and turning. Tiny passive actuator compliance is measured below; there is no commanded scan. The separate chin demonstration retains its three active DOFs. Both tail and pectoral actuation remain active; trajectories use the current per-gait configuration. See `reference_swim.md` for the current reference-inspired posterior-body wave and measured contributions. `forward_improvement.md` and `larger_tail.md` preserve the earlier controller comparisons.

The ground grid is fixed in world coordinates at z = -50 mm, with 10 mm spacing and a heavier line every 5 cells. Gold lines mark x = 0 and y = 0. It is decorative render geometry and adds no contact, fluid force or mass. Its lines are never translated or rotated with the camera or fish.

The top panel has a stationary camera centered on the initial whole-fish COM; the side and tail views track the fish. The overlays give x/y/z COM displacement relative to the initial pose. These fixed references make the measured forward, backward and lateral movement visible even in tracking views.

| Behavior | Peak absolute chin angle (degrees) | Final Δx (mm) | Final Δy (mm) | Final Δz (mm) |
|---|---:|---:|---:|---:|
| forward | 0.0618 | +32.07 | +0.06 | +14.98 |
| backward | 0.0597 | -34.66 | -2.41 | -3.86 |
| hover | 0.0115 | -3.33 | +0.47 | +2.78 |
| turning | 0.0306 | +11.81 | +4.99 | +4.83 |

Four H.264 videos rendered at 1920×1080, 60 fps. Configure `controller.gaits.<behavior>.chin_amplitude_fraction`, `render.swimming_ground`, `render.swimming_fixed_top` and `render.swimming_side_elevation_deg` in config.yaml. Regenerate with `.venv/bin/python render_videos.py --only swimming`.

![Verified video frames](swimming_videos_preview.png)
