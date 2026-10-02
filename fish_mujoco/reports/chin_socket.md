# Rigid rounded chin attachment

The moving chin now contains a complete spherical ball of radius **1.839 mm**. The head contains the matching concave socket. Both surfaces share the actual three-axis joint center. A radial overlap of 0.0147 mm covers faceting tolerances. The chin is one rigid textured mesh; **no deformable visual skin is loaded**. This removes the stretched, kinked collar from the previous revision.

The center is at source coordinates [9.05, 0, 2.65], at the mouthward root indicated in the user's marked image. Relative to the previous [9.50, 0, 2.60] center, it moves 3.68 mm toward the mouth and 0.41 mm dorsally. The source radius is 0.225, a 15.09% reduction from 0.265.

The nearby exterior is permanently recontoured into a smooth, narrow rigid waist using `chin_root_profile_source`. The head rim and chin root shelter the smaller ball, removing the protruding bead appearance at rest. **The complete exterior stays rigid**, with no flexible cover. The protected upper mouth and distal shaft retain their source geometry. The changed root surface receives outward radial UV projection from the source exterior, avoiding accidental projection onto the inside of the mouth. This is an engineering adjustment to match the marked location and appearance, not a recovered anatomical joint. Inertias and displaced volumes are regenerated from the new solids; swimming tuning is not rerun. `chin_root_contour.json` records the local volume removed.

Full requested limits are retained: yaw ±45°, pitch ±45°, roll ±30°. All 125 combinations of rest, half and full angles were checked against the **compiled MuJoCo mesh**. Maximum sampled socket-rim opening: 0 mm across 1835 rim samples. Maximum departure from a rigid transform: 2.82e-14 mm. All new head cut faces near the joint are also checked to lie on the socket sphere, excluding hidden planar leftovers.

The metric checks socket coverage, not exact tangency or uninterrupted texture markings between separate rotating parts. A small rounded base remains visible, and extreme upward combinations may intersect nearby head geometry because self-contact is disabled. There is no elastic tissue model or claim of anatomically validated ROM. The old skin reports describe superseded revisions.

Rebuild without swimming optimization:

```sh
.venv/bin/python segment_mesh.py
.venv/bin/python build_model.py
.venv/bin/python render_videos.py --only chin
.venv/bin/python render_videos.py --only exploded
.venv/bin/python -m pytest tests -q
```

![Rest and full-range views](chin_socket_preview.png)

![Rigid joint pieces, separated](chin_socket_parts.png)
