# Chin cut moved toward the mouth

The previous cut isolated the narrow distal stalk and left too much of its fleshy base on the head. The new landmarks move the pivot **4.09 mm posteriorly**, **0.57 mm dorsally**, and move the most proximal cut point **2.47 mm toward the mouth**. Chin length increases from 14.55 to 17.02 mm at the configured 200 mm fish length.

The spherical radius increases from 0.45 to 0.65 source units to encompass the broader root. The upper selection boundary moves from 2.88 to 2.93 source units. The chin is one closed component; neither selection plane clips its exterior. The mouth and jaw remain on the head. Original exterior geometry and UVs are retained.

The pivot remains a modeling approximation. [Peterson, Evans & Hernandez (2023), Histology of Convergent Probing Appendages in Mormyridae](https://academic.oup.com/iob/article/5/1/obad001/6994526) describes a soft-tissue appendage supported by mucochondroid tissue and moved by muscles, with an attachment to the dentary. It does not establish one rigid joint center in this artistic surface mesh. Moving the pivot into the fleshy root is an inference from the visible mouth and chin landmarks.

This review runs **no swimming parameter search**. Controller, gain, fluid and requested joint settings are preserved. Current angular seam diagnostics are in `chin_seam_metrics.json` and `chin_compound_metrics.json`; with the workspace's report-only large-angle policy, bent seams can exceed the original tolerance. Rest continuity remains checked. The tuning JSON checksum is recorded in `chin_cut_revision.json`.

Reproduce this comparison with `.venv/bin/python review_chin_cut.py`; it rebuilds both diagnostic cuts from the raw mesh. Rebuild geometry and MJCF with `.venv/bin/python segment_mesh.py` and `.venv/bin/python build_model.py`, then render the chin with `.venv/bin/python render_videos.py --only chin`. None of these commands runs tuning.

![Previous and revised cut, side and oblique views](chin_cut_location.png)

## Revision checks

MuJoCo compiles; 10 tests passed (including closed UV-bearing meshes, three chin DOFs, joint limits and stable simulation). Only the head/chin geometry changes above 0.1 micrometre numerical tolerance; other segment geometry and all controller, gain, fluid and requested joint settings are preserved. The saved tuning JSON is byte-identical. The regenerated chin videos are H.264, 1920×1080 at 60 fps. Earlier swimming videos retain their prior geometry; no swimming optimization or rerender was run.

Measured rest seam gap is below 0.000003 mm. At the existing combined 45°/45°/30° range limits the worst sampled gap is 5.299 mm, exceeding the original 3% tolerance; the current report-only policy preserves those angles. See `chin_cut_validation.json` for the recorded checks.
