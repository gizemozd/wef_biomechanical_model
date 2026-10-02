# Joint placed at the marked root, with a smaller rigid sphere

The pivot moved **3.68 mm toward the mouth** and **0.41 mm dorsally**, from source [9.50, 0, 2.60] to **[9.05, 0, 2.65]**, matching the marked root in the supplied image. Sphere radius decreased **15.09%**, from **2.166 to 1.839 mm**. Placement is inferred from the external marked view, not an anatomical scan.

The entire exterior remains rigid as requested. No skin, flexible collar, or added deformable body is present. The short head/chin root is permanently recontoured into a narrow waist, so the small ball sits within the surrounding contours rather than protruding as a bead. The protected upper mouth and distal shaft retain their original geometry. New exterior faces use projected source texture coordinates and consistent normals. The contour removes approximately **27.69 mm³**, about 0.046% of the original fish volume; mass and inertia are regenerated from the resulting solids. Degenerate coplanar boolean slivers are discarded before export.

The three actuated ranges remain yaw ±45°, pitch ±45°, roll ±30°. **12 tests passed**; UV/topology and protected-region checks passed after the final texture/normal changes. All **125** sampled combined poses have **zero measured socket-rim gap**, and the compiled mesh follows a rigid transform to within 2.82e-17 m. This verifies socket coverage; it does not claim perfectly tangent outer skin or absence of all intersections at extreme poses. A rigid articulation line can remain visible.

Actuator, controller, fluid, simulation and render settings are unchanged. The saved swimming tuning JSON is byte-identical; no tuning or optimization was run. The model, chin videos 08/09, exploded video 01, current stills and Figure 1 PDF/SVG/PNG were updated. Other swimming clips retain their earlier geometry until explicitly rerendered. Checks and artifact metadata are in `chin_marked_joint.json`.

Rebuild the geometry with `.venv/bin/python segment_mesh.py`, then `.venv/bin/python build_model.py`. Render the chin using `.venv/bin/python render_videos.py --only chin`. `chin_root_profile_source` contains the configurable static root contour. The full pipeline regenerates it automatically.

![Same-camera comparison; gold crosses indicate actual internal pivots](chin_marked_joint_comparison.png)

![Rest and full-range checks](chin_socket_preview.png)
