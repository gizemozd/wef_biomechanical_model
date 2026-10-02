# Stationary chin and ground reference: videos 03–06

All three chin position targets are held at zero for forward, backward, hover and turning. Tiny passive actuator compliance is measured below; there is no commanded scan. The separate chin demonstration retains its three active DOFs. Both tail and pectoral actuation remain active. The carangiform body wave is documented in [carangiform.md](carangiform.md); [tail_increment.md](tail_increment.md) records its latest amplitude increase.

The ground grid is fixed in world coordinates at z = -50 mm, with 10 mm spacing and a heavier line every 5 cells. Gold lines mark x = 0 and y = 0. It is decorative render geometry and adds no contact, fluid force or mass. Its lines are never translated or rotated with the camera or fish.

The top panel has a stationary camera centered on the initial whole-fish COM; the side and tail views track the fish. The overlays give x/y/z COM displacement relative to the initial pose. These fixed references make the measured forward, backward and lateral movement visible even in tracking views.

| Behavior | Peak absolute chin angle (degrees) | Final Δx (mm) | Final Δy (mm) | Final Δz (mm) |
|---|---:|---:|---:|---:|
| forward | 0.0921 | +42.89 | +1.89 | +12.01 |
| backward | 0.0594 | -35.16 | -2.14 | -3.93 |
| hover | 0.0111 | -2.17 | +0.44 | +2.78 |
| turning | 0.0297 | +4.87 | +5.44 | +3.92 |

Four H.264 videos rendered at 1920×1080, 60 fps. Configure `controller.gaits.<behavior>.chin_amplitude_fraction`, `render.swimming_ground`, `render.swimming_fixed_top` and `render.swimming_side_elevation_deg` in config.yaml. Regenerate with `.venv/bin/python render_videos.py --only swimming`.

![Verified video frames](swimming_videos_preview.png)
