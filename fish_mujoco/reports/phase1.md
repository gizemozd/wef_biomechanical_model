# Phase 1 — segmentation and seams

14 body segments, 7 whole fin meshes and one chin segment, 22 moving rigid bodies. All output solids are watertight with consistent normals. Exact manifold booleans used on a faceted sphere (4 icosphere subdivisions). Maximum radial overlap 0.350 mm, locally capped at 0.4% of body height; dorsal and anal fins are single meshes. Two original eye holes capped; transparent sclera overlays omitted.

Original textured exterior triangles are retained where the anatomy is unchanged. Ball/socket mode models the chin root with a rounded ball. When configured, a smooth rigid waist is trimmed around the marked joint position, reducing the ball's visible prominence; `chin_root_contour.json` records the local change. UVs on this new exterior are projected radially outward to the source, and shared reference normals keep the root shading consistent. No visual skin is used. Fin bases use configurable, manually inferred external landmarks; these are not recovered bones. Upper (dorsal) and lower (anal) fins are each one complete rigid mesh, neutral by default. The chin is an independent head-attached body with three actuated rotations.

Maximum sampled seam gap / local body height: **2.1109%**. Tests cover rest, ±half and ±full yaw/pitch separately. Four combined yaw/pitch corners per joint are also checked in `body_compound_metrics.json`. Reported penetration includes intentional overlap. Samples run along outer/cap boundary edges; this is a finite numerical test, not a proof for all points, arbitrary compound poses, or fin membranes. Ranges were reduced to preserve body seams; see `assets/segments.json` and `seam_metrics.json`.

Chin limits (seam policy: enforce_socket_coverage): yaw ±45.000°, pitch ±45.000°, roll ±30.000°. These engineering limits are not measured anatomical ROM. Ball/socket mode enforces coverage of the head socket rim across all sampled rotations. Legacy report-only mode permits bent gaps. See `chin_seam_metrics.json` and `chin_compound_metrics.json`. All three rotational DOFs have independent actuators and sensors.

Uniform-density mass after overlap correction: **60.102 g**. Corrected displaced volume: 6.0102462e-05 m³, including any added ball-root volume. Shared overlap volume is counted only once globally via mass scale 0.9898234; local overlap distribution is approximate.

![Final segmentation](segmentation.png)

Body ranges were refreshed on the same meshes. Combined yaw/pitch corner checks are in `body_compound_metrics.json`; see `reference_swim.md` for the regional range revision.
