# Continuous chin attachment

The mechanical cut was already a concentric sphere. Rotating finite patches of that sphere cannot preserve a nonspherical exterior silhouette. The revised visual surface joins the original head and chin exterior, removes the internal cap/socket faces with a boolean union, and blends its movement smoothly between the existing head and chin bodies. The joint remains spherical underneath. The mouth and proximal head stay fixed to the head; the distal chin follows its three existing actuators.

The local skin uses the original texture coordinates. Its transition is confined to a short collar at the base: the configured radii are [0.0015, 0.0055] m, replacing the former 0.002–0.014 m blend that made the shaft look S-shaped. The mouth fade is confined to source z=[2.85, 3.0]. The remaining shaft follows the chin rigidly, preserving the source shape rather than introducing extra curvature. `segmentation.chin_skin`, `chin_skin_blend_radii_m`, `chin_skin_mouth_fade_source_z`, and `chin_skin_subdivisions` control it. `build_model.py` regenerates the skin from the current segment meshes; `make all` includes it automatically. Setting `chin_skin: false` restores rigid visuals. The exploded view deliberately displays rigid parts.

**Validation:** 125 combinations of zero, half and full yaw/pitch/roll limits were checked using MuJoCo's actual updated skin vertices. The visible surface is closed by position topology, with maximum separation between coincident UV-boundary copies **0 mm**. Maximum rest-position error versus the union mesh is 1.11856e-05 mm. Rendered positions and normals remain finite, with positive enclosed volume. See `chin_surface_validation.json` for all sampled poses.

**Straight shaft:** all 2066 rendered shaft vertices beyond the first 2 mm of the chin mesh match the rigid chin body's transform at every sampled pose, to within 1.00441e-05 mm. This verifies a 15.02 mm length of shaft keeps its shape rather than bending with the visual collar.

The previous rigid-interface gap measurements remain in `chin_seam_metrics.json` and `chin_compound_metrics.json`; they describe hidden rigid parts, not the continuous visible surface. Joint ranges remain ±45° yaw, ±45° pitch and ±30° roll. No swimming tuning, masses, fluid proxies, or actuator gains were changed.

This uses [MuJoCo visual skinning](https://mujoco.readthedocs.io/en/stable/XMLreference.html#deformable-skin), not a mechanical soft-tissue model. It does not add contact geometry or enforce tissue incompressibility. Severe combined bends can compress, stretch, or self-intersect the skin; the tests establish continuity at sampled poses, not anatomical accuracy or freedom from all intersections.

![Identical-camera comparison: rigid left, continuous right](chin_surface_comparison.png)
