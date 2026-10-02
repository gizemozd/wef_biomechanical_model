# Textured elephantnose fish in MuJoCo

Reproducible conversion of the supplied **Gnathonemus petersii** OBJ to an actuated, textured MuJoCo model. The revised model has **14 body regions, 7 whole fins, and a separate chin segment**. Its **40 position actuators** include independent chin yaw, pitch and roll. The dorsal and anal fins are one mesh each and are held neutral by default.

Start with [the preview](videos/preview.gif), [the chin demonstration](videos/08_chin_3dof.mp4), [the original/segmented comparison](reports/texture_continuity.png), or [the validation report](reports/phase6.md).

## Rebuild

```sh
cd fish_mujoco
make setup     # uv, Python 3.12, pinned packages
make all      # raw assets -> repaired meshes -> segments -> MJCF -> gait audits -> videos -> tests
```

After setup, `.venv/bin/python run_pipeline.py` is equivalent. From the containing workspace, `make all` forwards here. `ffmpeg` and `ffprobe` must be on PATH; video encoding uses imageio-ffmpeg. The raw OBJ and textures remain in `../data/`; paths in `config.yaml` are relative to that config file. No source asset is edited.

The current machine uses macOS CGL for headless rendering. Linux selects EGL; the launcher retries OSMesa if initialization fails. No Blender or GPU display window is needed. `requirements.txt` pins the tested environment, including MuJoCo 3.14.0.

```sh
.venv/bin/python run_pipeline.py --config config.yaml
.venv/bin/python run_pipeline.py --from-phase 3    # rebuild physics, gait audits, videos, tests
.venv/bin/python evaluate_behaviors.py            # current config; no optimization
.venv/bin/python render_videos.py --only swimming # videos 03–06
.venv/bin/python render_videos.py --only chin
.venv/bin/python -m pytest tests -q
```

`--quick` explicitly generates a reduced 640×360/15 fps smoke build **in the same output directories**; run `make all` again to restore the final 1920×1080/60 fps deliverables. Phase 2 texture generation is included in `segment_mesh.py` (phase 1).

## Chin and fin controls

The chin mesh is `assets/meshes/chin_0.obj`, attached to `body_00_head`. Its three hinges share a pivot:

| Joint | Rotation axis | Actuator |
|---|---|---|
| `j_chin_yaw` | z | `a_j_chin_yaw` |
| `j_chin_pitch` | y | `a_j_chin_pitch` |
| `j_chin_roll` | x | `a_j_chin_roll` |

Position actuator commands are **radians**, including when the MJCF compiler interprets joint ranges in degrees. Each DOF has named position and velocity sensors. Chin limits are now **yaw ±45°, pitch ±45°, roll ±30°**. The three-axis scan uses `controller.chin_frequency_hz: 1.0` and `controller.chin_amplitude_fraction: 0.85`, commanding ±38.25° yaw/pitch and ±25.5° roll; set the fraction to zero to hold it still. These are adjustable engineering values, not measured anatomical limits. The dedicated `09_chin_scan.mp4` shows an actuated free-root simulation with other joint targets held at rest.

**Videos 03–06 hold the chin neutral.** Their per-gait `chin_amplitude_fraction: 0.0` overrides the general scan amplitude. Actual passive compliance stays below 0.1° in the current six-second runs. The three DOFs remain available for the dedicated chin demonstration.

The current attachment uses **`segmentation.chin_attachment: ball_socket`** and **`chin_skin: false`**. The chin owns a complete spherical ball seated in a concave socket in the head. The entire chin, including its base, is a rigid mesh. There is no deforming collar or visual skin. The small rounded base rotates without stretching the mouth or bending the shaft.

The ball radius is **1.839 mm**, reduced by 15.09% from 2.166 mm. Its center is `chin_pivot_source: [9.05, 0, 2.65]`, about **3.68 mm toward the mouth** and 0.41 mm higher than the previous center, matching the marked root location. The adjacent rigid contours form a narrow waist around the smaller ball so it does not protrude as a bead. No flexible cover is used. The upper mouth and distal shaft retain the original geometry; the short root is deliberately recontoured. The placement is inferred from the marked surface image, not a measured anatomical joint. See [the current socket report](reports/chin_socket.md) and [before/after](reports/chin_marked_joint_comparison.png).

`chin_radius_source` controls the rounded root, and `chin_upper_limit_source` separates the appendage from the mouth. A small concentric radial overlap compensates for triangulated sphere facets. Socket-rim coverage is enforced even when `chin_auto_reduce_for_seams` is false. The original head-cap/concave-chin and optional visual-skin implementations remain available as legacy comparisons; their old reports do not describe the current model. The legacy skin parameters have no effect in the current mode.

`chin_root_profile_source` lists [anterior offset from pivot, radius] pairs in source units. A smooth revolved envelope trims only the lower root; the model is static after this geometry operation. Projected source UVs texture the new exterior. Removing the profile restores the original root contour, which may require a larger ball to cover its wider attachment.

The upper and lower fins remain `fin_dorsal_0` and `fin_anal_0`, one complete rigid mesh each. No swimming tuning is rerun for a chin geometry update.

## Tail and pectoral propulsion

Videos 03–06 use coordinated caudal and pectoral strokes. The forward/turning caudal target counter-rotates against the sum of the traveling body-yaw targets. Forward swimming uses `caudal_counterbend_gain: 3.0`, an 11° command cap, and a **−45° caudal phase offset**; turning retains gain 2.2 and an 8° cap. The body wave grows toward the rear within the existing seam-safe joint limits, with a 1.2 BL wavelength. Base gaits run at 3 Hz; forward uses **6 Hz**, body amplitude fraction **0.95**, pectoral amplitude **14°** before CPG scaling, and feathering fraction **0.3**. Both pectoral fins feather with matching span-axis rotation; this sign reverses in the backward gait. `controller.gaits` keeps each behavior independently configurable.

The forward tail now swings about **±6.7°**, a **32% wider hinge sweep** than the previous ±5.1°. Tail-tip lateral excursion relative to the head increases from 0.79 to 1.68 mm. The larger stroke **does not improve speed**: six-second forward displacement falls from 43.85 to **41.84 mm**, and mean anterior speed over the final three seconds falls from 0.04424 to **0.04254 BL/s**. It retains most of the earlier improvement from 6.64 mm without extending the video or prescribing root motion. See [the larger-tail comparison](reports/larger_tail.md) and [the earlier speed improvement](reports/forward_improvement.md). These are controller engineering settings, not measured species-specific kinematics.

Native force measurements with a neutral chin show that the tail supplies about **24% of combined tail-and-pectoral axial force in forward swimming**, **46% backward**, and **47% turning**. These compare only those fin groups in this model, not their share of all hydrodynamic forces or biological efficiency. Independent runs with either group's targets held neutral weaken propulsion; holding the forward tail neutral reduces speed by about 31%. Hover uses opposing active fin forces and retains small drift; it is not closed-loop station keeping. See [the contribution report](reports/propulsion_revision.md), [force/ablation plot](reports/propulsion_contributions.png), and [full measurements](reports/propulsion_audit.json).

Phase 4 now evaluates the configured gaits without a parameter search. Historical `tuning.json` and `tuning.png` are preserved for reference; they do not override `config.yaml`. The optional `tune.py` explores candidate CPG parameters separately. The rendering command checks each trajectory's model/controller signature and regenerates stale trajectories before rendering. It does not silently replay old motion with new geometry.

```bash
.venv/bin/python segment_mesh.py
.venv/bin/python build_model.py
.venv/bin/python render_videos.py --only chin
.venv/bin/python render_videos.py --only exploded
.venv/bin/python -m pytest tests -q
```

`segment_mesh.py --chin-rom-only` updates angle diagnostics from existing meshes; changing the pivot or radius requires normal segmentation. The focused chin rendering commands update videos 01, 08 and 09. Swimming videos 03–06 also include the current rigid chin/root geometry after the propulsion revision.

## Configuration and outputs

- `source`: input mesh/textures, right-handed orientation and 0.20 m full-mesh extent. This includes the chin and tail fin; it is **not** standard length or a maturity claim.
- `segmentation`: body count, sphere/cylinder cuts, radii, overlap, anatomical landmarks and seam thresholds. Defaults use spherical cuts. Cylinder mode removes body pitch; the three-DOF chin still uses a spherical interface.
- `joints`: requested regional limits, stiffness, damping, armature and position-actuator gains. `auto_reduce_for_seams` governs the body. The ball/socket chin enforces socket coverage at its configured ranges; `chin_auto_reduce_for_seams` controls range shrinking only for the legacy attachment.
- `fluid`: water/tissue density, viscosity, gravity, explicit buoyancy and ellipsoid coefficients.
- `controller`: CPG frequency, amplitude scale, body wavelength, body amplitude envelope, caudal counter-bending, pectoral feathering, per-behavior `gaits`, chin scan and optional tuning grid. Pectoral abduction amplitude = `pec_amplitude_deg × amplitude_deg / 22`, clipped to its joint range. `pec_pitch_fraction` scales span-axis feathering. `disabled_groups` holds selected position targets neutral for ablations while preserving passive anatomy.
- `render`: resolution, fps, clip duration, lighting quality and camera defaults. `swimming_ground` controls the world-fixed decorative floor, grid spacing (10 mm), stronger lines (every 50 mm), origin lines and colors. `swimming_fixed_top` enables a stationary overhead camera; `swimming_side_elevation_deg` controls the side view angle.

`fish.xml`, all segment meshes, padded texture atlas and manifests are generated. Segment OBJ files have shared geometry vertices and independent face-corner UV/normal indices. Position-only welding restores their closed topology when a loader splits vertices at UV seams. PLY counterparts carry geometry for validation; OBJ is used for rendering. `original.obj` is the unsegmented reference and intentionally retains original openings; topology tests apply to generated segments.

The atlas keeps the original 4096×4096 pixels and adds a 128-pixel strip of flat interior colors. Barycentric transfer preserves UV islands and original surface normals. Proximity calculations are rescaled internally to millimetres to avoid loss of accuracy on small triangles; all assets and physics use metres. Source normal/roughness/specular/glossiness/opacity maps are copied but classic MuJoCo renders only the diffuse material.

## Videos and reports

| File | Content |
|---|---|
| `00_original_vs_segmented.mp4` | Same-camera rotation comparison |
| `01_exploded_segments.mp4` | Colored parts separate and reassemble |
| `02_joint_rom.mp4` | Every hinge swept individually, with close-up |
| `03_forward_swim.mp4` | Forward controller, measured COM speed |
| `04_backward_swim.mp4` | Backward controller |
| `05_hover.mp4` | Open-loop hover command |
| `06_turning.mp4` | Coordinated tail stroke, body/tail bias and asymmetric pectorals |
| `07_body_undulation.mp4` | Body-wave comparison |
| `08_chin_3dof.mp4` | Dedicated yaw, pitch and roll demonstration |
| `09_chin_scan.mp4` | Simulated active chin scan with side/top close-ups |
| `preview.gif` | Short forward behavior preview |

Updated behavior clips 03–06 show a tracking side view, **fixed top view**, and caudal-fin close-up, with time, actual fin angles, signed COM velocity along the head's anterior axis, and x/y/z displacement from the start. A stationary **10 mm ground grid** has stronger 50 mm lines and gold world-origin lines. The floor is render-only geometry and contributes no collisions or fluid forces. See [the ground/chin report](reports/swimming_ground.md). The videos are H.264, 1920×1080, 60 fps. Kinematic ROM/explosion clips are distinct from simulated free-root behaviors; no external thrust or prescribed swimming trajectory is used. Video 07 keeps its earlier export until regenerated; its current-config trajectory is available.

`reports/phase0.md` through `phase6.md` record each phase. Biology citations are in `reports/anatomy_notes.md`, regenerated from `references/anatomy_notes.md`. `seam_metrics.json`, `chin_seam_metrics.json`, and `chin_compound_metrics.json` contain measured gaps and penetration. `tuning.json` and `controller_diagnostics.json` are historical. Current results are in `behaviors.json`, `propulsion_audit.json` and saved trajectories with provenance JSON. `validation.json` includes passive drift, runtime, source checksum and ffprobe output.

## Assumptions and limitations

This is a mechanical approximation of an artistic external mesh, not a CT-based anatomical reconstruction. Species-specific vertebral count and regional ROM were not verified, so rigid body regions are not labeled as individual vertebrae. Fin bases and the chin attachment are inferred from visible geometry. Stiffness, actuator gains, fin angles and fluid coefficients are engineering settings, not measured muscle parameters. Literature supports a stiffened mormyrid electric-organ peduncle; no electric-field simulation is included.

Concentric cut surfaces do not guarantee continuity of a nonspherical outer skin. Finite seam samples cover individual body yaw/pitch poses, all three chin axes and eight combined chin extrema. Body ranges are approximately ±0.72° yaw and ±0.5° pitch. They deliberately prioritize the requested gap limits. The current chin is a rigid ball/socket assembly, checked at 125 combinations of yaw, pitch and roll using the compiled MuJoCo mesh. The tests establish socket-rim coverage and rigid movement, not perfect outer-surface tangency or continuous texture markings across rotating parts. Extreme upward combinations can intersect nearby head geometry because self-contact is disabled. Arbitrary compound body bends are not proven seamless. Whole dorsal/anal fins follow one central body segment each; their long bases cannot deform with multiple body segments. The previous local MuJoCo `<skin>` is disabled; the whole current fish is rigidly segmented.

Explicit gravity compensation supplies Archimedean buoyancy from corrected displaced volume at each uniform-density segment COM. Native MuJoCo fluid density does not supply buoyancy. Full submersion and coincident centers of mass/buoyancy are assumed. Segment overlap is corrected globally in mass/volume; its local distribution is approximate. Extremely small fin-tip principal inertias are floored only when necessary, recorded in `inertia_adjustments.json`. Fish self-collision is disabled; external collisions use simple proxies.

MuJoCo's ellipsoid fluid model has no wakes or resolved fin-to-fin interaction. Swimming is slow and includes vertical/lateral drift; backward motion requires different caudal strokes, not merely reversing a traveling wave. Hover is an open-loop near-stationary command, not feedback station keeping. Short tuning runs do not establish steady-state speed, efficiency or biological validity. Runtime is measured on the executing machine, not guaranteed. The simplified median fins replace the original ribbon-fin-controller request and substantially reduce simulation cost.
