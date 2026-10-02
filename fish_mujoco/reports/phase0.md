# Phase 0 — inspection and proposed layout

Source: `Elephant_Nose_Fish.obj`; species established by the existing project README: **Gnathonemus petersii**.

- 7,621 OBJ vertices (UV splits included), 13,948 triangles, two materials; UVs present.
- Source bounds: [[-4.635, -11.233, -0.001], [4.635, 13.241, 7.545]]; source units are unspecified.
- Scale: 0.00817194 m/source unit; full mesh extent **0.200 m**, including projecting chin. This is a chosen specimen size, not maximum adult or standard length.
- Coordinates: source -Y → anterior +x, source +X → left +y, source +Z → dorsal +z; origin at bounding-box mid-body.
- Body + two eyes form three shells. Two 14-edge openings near the eyes are fan-capped; normals corrected. Open transparent sclera overlays are omitted; textured eyeballs remain.
- Six 4096² maps: diffuse, opacity, normal, roughness, specular, glossiness. MuJoCo uses the diffuse atlas; full PBR/opacity maps are archived but not reproduced by its classic shader.
- Proposed 14 body regions; head/girdle kept together, trunk mildly flexible, electric-organ peduncle stiff. Dorsal and anal fins are one rigid mesh each, with paired pectorals, paired pelvic fins, and a caudal fin. The chin is a separate segment with yaw, pitch and roll actuators.
- Centerline uses section polygon area centroids and a smoothing spline. Preliminary dimensions include median fins; final sphere dimensions will use the extracted body.
- Spherical interfaces do **not** mathematically ensure continuity of an arbitrary outer surface. Seam tests will limit actual joint ranges, including pitch.

![Original textured render](original_render.png)
![Texture atlas](texture_atlas.png)
![Cut layout](phase0_layout.png)
![Centerline](centerline.png)

See [anatomy notes](anatomy_notes.md) for sources and estimated values.
