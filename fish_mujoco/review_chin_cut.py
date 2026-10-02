"""Reproduce the chin attachment comparison without running swimming optimization."""
import argparse
import hashlib
import tempfile
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image, ImageDraw
from common import ROOT, config, load_source, solid, dump, report
from segment_mesh import boolean, box
from render_videos import Render, annotate, font, mujoco
import trimesh


def cut(main, scale, center, sg):
    cv = lambda q: (np.asarray(q) - center) * scale
    pivot = cv(sg['chin_pivot_source'])
    radius = sg['chin_radius_source'] * scale
    overlap = min(sg['overlap_m'], .008 * radius)
    roi = box([cv([sg['chin_pivot_source'][0], -3, -2]),
               cv([15, 3, sg['chin_upper_limit_source']])])
    cap = trimesh.creation.icosphere(subdivisions=sg['sphere_subdivisions'], radius=radius)
    cap.apply_translation(pivot)
    socket = trimesh.creation.icosphere(subdivisions=sg['sphere_subdivisions'], radius=radius-overlap)
    socket.apply_translation(pivot)
    chin = boolean(main, boolean(roi, socket, 'sub'), 'and')
    head = boolean(main, boolean(roi, cap, 'sub'), 'sub')
    assert chin.is_watertight and head.is_watertight
    assert len(chin.split()) == 1
    assert chin.bounds[0, 0] > pivot[0] + 1e-6
    assert chin.bounds[1, 2] < cv([0, 0, sg['chin_upper_limit_source']])[2] - 1e-6
    return head, chin, pivot


def project(renderer, point):
    cameras = renderer.r.scene.camera
    pos = np.mean([cam.pos for cam in cameras], axis=0)
    forward = np.mean([cam.forward for cam in cameras], axis=0)
    up = np.mean([cam.up for cam in cameras], axis=0)
    right = np.cross(forward, up)
    rel = point-pos
    depth = rel @ forward
    cam = cameras[0]
    height = (cam.frustum_top-cam.frustum_bottom)*depth/cam.frustum_near
    return (renderer.w/2 + (rel @ right)*renderer.h/height,
            renderer.h/2 - (rel @ up)*renderer.h/height)


def run(c):
    if c['segmentation'].get('chin_attachment')=='ball_socket':
        # The historical comparison below uses the opposite cap/socket layout.
        # Current rigid ball assemblies have their own compiled-model review.
        from check_chin_socket import run as review_socket
        review_socket(c)
        return
    src, _, scale, center = load_source(c)
    clean, _ = solid(src)
    main = max(clean.split(), key=lambda mesh: len(mesh.faces))
    # Fixed historical baseline makes this comparison independent of temporary backups.
    old = dict(c['segmentation'], chin_pivot_source=[9.1, 0, 2.63],
               chin_radius_source=.45, chin_upper_limit_source=2.88)
    states = [cut(main, scale, center, sg) for sg in [old, c['segmentation']]]
    look = (np.array([9.0, 0, 3.0])-center)*scale
    rows = [[], []]
    with tempfile.TemporaryDirectory(prefix='fish_chin_review_') as tmp:
        from pathlib import Path
        for index, (head, chin, pivot) in enumerate(states):
            head.export(Path(tmp)/f'head_{index}.stl'); chin.export(Path(tmp)/f'chin_{index}.stl')
            root = ET.Element('mujoco')
            ET.SubElement(root, 'compiler', meshdir=tmp)
            vis = ET.SubElement(root, 'visual')
            ET.SubElement(vis, 'global', offwidth='960', offheight='620')
            ET.SubElement(vis, 'headlight', ambient='.45 .45 .45', diffuse='.65 .65 .65')
            asset = ET.SubElement(root, 'asset')
            for name in ['head', 'chin']:
                ET.SubElement(asset, 'mesh', name=name, file=f'{name}_{index}.stl', smoothnormal='true')
            world = ET.SubElement(root, 'worldbody')
            ET.SubElement(world, 'light', pos='.1 .1 .2', dir='0 0 -1', diffuse='.7 .7 .7')
            for name, rgba in [('head', '.46 .57 .59 1'), ('chin', '.95 .36 .28 1')]:
                ET.SubElement(world, 'geom', type='mesh', mesh=name, rgba=rgba, contype='0', conaffinity='0')
            model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
            data = mujoco.MjData(model); mujoco.mj_forward(model, data)
            renderer = Render(model, 960, 620)
            for view, (azimuth, elevation) in enumerate([(90, 0), (60, -25)]):
                frame = renderer.frame(data, look=look, distance=.065, azimuth=azimuth, elevation=elevation)
                frame = annotate(frame, ['Previous cut', 'Cut moved toward the mouth'][index],
                                 'Coral = movable chin | yellow dot = internal pivot, projected through skin')
                im = Image.fromarray(frame); draw = ImageDraw.Draw(im)
                x, y = project(renderer, pivot)
                draw.ellipse((x-6, y-6, x+6, y+6), fill='#ffe48b', outline='black', width=1)
                draw.text((x-46, y+17), 'pivot', font=font(19), fill='#ffe48b')
                rows[view].append(np.asarray(im))
            renderer.close()
    Image.fromarray(np.concatenate([np.concatenate(row, axis=1) for row in rows], axis=0)).save(ROOT/'reports/chin_cut_location.png')
    old_head, old_chin, old_pivot = states[0]
    new_head, new_chin, new_pivot = states[1]
    metrics = {
        'previous_landmarks': {k: old[k] for k in ['chin_pivot_source', 'chin_radius_source', 'chin_upper_limit_source']},
        'current_landmarks': {k: c['segmentation'][k] for k in ['chin_pivot_source', 'chin_radius_source', 'chin_upper_limit_source']},
        'pivot_posterior_shift_mm': float((old_pivot[0]-new_pivot[0])*1000),
        'pivot_dorsal_shift_mm': float((new_pivot[2]-old_pivot[2])*1000),
        'proximal_cut_posterior_shift_mm': float((old_chin.bounds[0, 0]-new_chin.bounds[0, 0])*1000),
        'previous_chin_length_mm': float(old_chin.extents[0]*1000),
        'current_chin_length_mm': float(new_chin.extents[0]*1000),
        'current_chin_components': len(new_chin.split()),
        'tuning_sha256': hashlib.sha256((ROOT/'reports/tuning.json').read_bytes()).hexdigest(),
    }
    dump(ROOT/'reports/chin_cut_revision.json', metrics)
    report('chin_cut_revision.md', f'''# Chin cut moved toward the mouth

The previous cut isolated the narrow distal stalk and left too much of its fleshy base on the head. The new landmarks move the pivot **{metrics['pivot_posterior_shift_mm']:.2f} mm posteriorly**, **{metrics['pivot_dorsal_shift_mm']:.2f} mm dorsally**, and move the most proximal cut point **{metrics['proximal_cut_posterior_shift_mm']:.2f} mm toward the mouth**. Chin length increases from {metrics['previous_chin_length_mm']:.2f} to {metrics['current_chin_length_mm']:.2f} mm at the configured 200 mm fish length.

The spherical radius increases from 0.45 to {c['segmentation']['chin_radius_source']} source units to encompass the broader root. The upper selection boundary moves from 2.88 to {c['segmentation']['chin_upper_limit_source']} source units. The chin is one closed component; neither selection plane clips its exterior. The mouth and jaw remain on the head. Original exterior geometry and UVs are retained.

The pivot remains a modeling approximation. [Peterson, Evans & Hernandez (2023), Histology of Convergent Probing Appendages in Mormyridae](https://academic.oup.com/iob/article/5/1/obad001/6994526) describes a soft-tissue appendage supported by mucochondroid tissue and moved by muscles, with an attachment to the dentary. It does not establish one rigid joint center in this artistic surface mesh. Moving the pivot into the fleshy root is an inference from the visible mouth and chin landmarks.

This review runs **no swimming parameter search**. Controller, gain, fluid and requested joint settings are preserved. Current angular seam diagnostics are in `chin_seam_metrics.json` and `chin_compound_metrics.json`; with the workspace's report-only large-angle policy, bent seams can exceed the original tolerance. Rest continuity remains checked. The tuning JSON checksum is recorded in `chin_cut_revision.json`.

Reproduce this comparison with `.venv/bin/python review_chin_cut.py`; it rebuilds both diagnostic cuts from the raw mesh. Rebuild geometry and MJCF with `.venv/bin/python segment_mesh.py` and `.venv/bin/python build_model.py`, then render the chin with `.venv/bin/python render_videos.py --only chin`. None of these commands runs tuning.

![Previous and revised cut, side and oblique views](chin_cut_location.png)
''')
    print(metrics)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('--config')
    run(config(parser.parse_args().config))
