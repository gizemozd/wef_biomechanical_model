# Figure 1 — model overview

`Figure1_overview.pdf` is the assembled publication figure, 183 × 75 mm. `Figure1_overview.svg` contains the same artwork with named editable groups. `Figure1_overview.png` is a 600 dpi preview. The editable caption is in `Figure1_caption.txt`. The compact layout trims the empty render margins and uses short flow arrows with small heads and thicker lines. In panel c, both pectoral and both pelvic fins are below the body; only the dorsal fin is above it.

## Individual components

- `raw/`: six unlabelled, directly rendered model views, each as lossless PNG and LZW-compressed TIFF, plus the camera and projected geometry data in `projections.json`.
- `panels/`: each labelled panel separately as PDF, SVG and PNG, at the same physical size as in the combined figure.
- `provenance.json`: input asset checksums, model version, font paths, dimensions and resolution.

The PDF embeds Arial Regular, Bold and Italic as TrueType fonts. Text is selectable and editable, not outlined. Arrows, interface curves, scale bars, joint markers and kinematic connections are vector objects. Each shaded 3D view is an independent raster image at approximately 841 pixels/inch at its final placement size; its individual triangles are not editable PDF vectors. To change the rendered geometry, camera or colours, use the figure script and source meshes. The SVG uses live Arial text and embedded images; Arial should be installed when editing it in Illustrator, Affinity Designer or Inkscape. Named SVG groups separate panel artwork and labels.

## Reproduce or edit

From the workspace root:

```sh
fish_mujoco/.venv/bin/python paper/make_figure1.py
```

Change `paper/figure1.yaml` for camera settings, colours, panel titles, font sizes and page dimensions. To reassemble existing raw views after editing only labels, colours of annotations or layout:

```sh
fish_mujoco/.venv/bin/python paper/make_figure1.py --assemble-only
```

Camera or rendered colour changes require the full command. The script reads the current `fish_mujoco/fish.xml` and assets; it writes only under `paper/figures/`. It does not rebuild segmentation, change the model, run swimming optimization or overwrite videos. `--assemble-only` is intended for the existing model and camera configuration. Re-render after model changes. Assertions guard the displayed segment, actuator and joint-range values.

## Figure conventions

White background; RGB colour; 8 pt bold panel letters; 7 pt titles; 5.5–6 pt annotations; and editable vector strokes. These choices follow [Nature's production artwork guidance](https://www.nature.com/nature/for-authors/final-submission) and [figure preparation guide](https://research-figure-guide.nature.com/figures/building-and-exporting-figure-panels/). The final journal may request layout changes.

All fish images are direct renders of the supplied or derived model, with the same lighting and view orientation in panels a–e. Scale bars follow the orthographic camera calibration. The panel-c offsets are for visualization only. Panel-d joint links are the implemented kinematic tree, not anatomical bones. Panel-f joint ranges and attachment location are engineering choices. No micro-CT, thresholded volume or live-animal image is implied. The reference fly figure was used only as a layout reference; none of its image content is included.
