"""Render actual model assets; assemble editable publication artwork.

Run from the workspace: fish_mujoco/.venv/bin/python paper/make_figure1.py
All writes stay in paper/figures. No segmentation, model rebuild or tuning.
"""
from pathlib import Path
import argparse
import hashlib
import json
import sys
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager, colors
from matplotlib.collections import LineCollection
from matplotlib.patches import FancyArrowPatch, Circle
import trimesh

HERE = Path(__file__).resolve().parent
MODEL = HERE.parent / 'fish_mujoco'
OUT = HERE / 'figures'
sys.path.insert(0, str(MODEL))
from common import gl_backend
gl_backend()
import mujoco

INKSCAPE = 'http://www.inkscape.org/namespaces/inkscape'
SVG = 'http://www.w3.org/2000/svg'


def write_json(path, data):
    path.write_text(json.dumps(data, indent=2, default=lambda x: x.tolist()) + '\n')


class Camera:
    def __init__(self, look, height, outward, aspect):
        self.look = np.asarray(look, dtype=float)
        self.out = np.asarray(outward, dtype=float)
        self.out /= np.linalg.norm(self.out)
        self.right = np.cross([0, 0, 1], self.out)
        self.right /= np.linalg.norm(self.right)
        self.up = np.cross(self.out, self.right)
        self.height, self.width = height, height * aspect

    def project(self, points):
        p = np.asarray(points) - self.look
        return np.stack([.5 + p @ self.right / self.width,
                         .5 + p @ self.up / self.height], axis=-1)

    def values(self):
        return {key: getattr(self, key) for key in
                ['look', 'out', 'right', 'up', 'height', 'width']}


def load_model(camera, cfg, source=False):
    """In-memory rendering model; the physical model on disk is untouched."""
    root = ET.parse(MODEL / 'fish.xml').getroot()
    compiler = root.find('compiler')
    compiler.set('meshdir', str(MODEL / 'assets/meshes'))
    compiler.set('texturedir', str(MODEL / 'assets/textures'))
    visual = root.find('visual')
    visual.find('global').set('offwidth', str(cfg['render']['width_px']))
    visual.find('global').set('offheight', str(cfg['render']['height_px']))
    visual.find('quality').set('offsamples', '4')
    visual.find('headlight').attrib.update(ambient='.38 .38 .38', diffuse='.50 .50 .50', specular='.08 .08 .08')
    visual.find('rgba').set('haze', '1 1 1 0')
    asset = root.find('asset')
    for texture in asset.findall('texture'):
        if texture.get('type') == 'skybox':
            texture.set('rgb1', '1 1 1'); texture.set('rgb2', '1 1 1')
    world = root.find('worldbody')
    for light in world.findall('light'):
        light.set('diffuse', '.42 .42 .42')
        light.set('castshadow', 'false')
    if source:
        for tag in ['actuator', 'sensor', 'contact', 'keyframe', 'deformable']:
            item = root.find(tag)
            if item is not None:
                root.remove(item)
        for body in list(world.findall('body')):
            world.remove(body)
        for mesh in list(asset.findall('mesh')):
            asset.remove(mesh)
        ET.SubElement(asset, 'mesh', name='source', file='original.obj', smoothnormal='true')
        ET.SubElement(asset, 'texture', name='source_diffuse', type='2d', file='Elephant_Nose_Fish_Diffuse.png')
        ET.SubElement(asset, 'material', name='source_material', texture='source_diffuse', specular='.12', shininess='.2')
        ET.SubElement(world, 'geom', name='source_visual', mesh='source', type='mesh', material='source_material', group='1', contype='0', conaffinity='0')
    vec = lambda a: ' '.join(map(str, a))
    ET.SubElement(world, 'camera', name='paper_camera', projection='orthographic',
                  fovy=str(camera.height), pos=vec(camera.look + camera.out * .5),
                  xyaxes=vec(np.r_[camera.right, camera.up]))
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))


def geometry_style(model, meta, cfg, style):
    if style == 'textured':
        return
    for record in meta['segments']:
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'visual_' + record['name'])
        model.geom_matid[gid] = -1
        color = np.array([.72, .75, .77])
        if style == 'rig':
            color = np.array([.84, .85, .86])
        elif style == 'regions':
            category = 'body' if record['region'] == 'body' else ('chin' if record['region'] == 'chin' else 'fins')
            color = np.array(colors.to_rgb(cfg['colors'][category]))
            if category == 'body':
                color = color * (.86 if record['body_index'] % 2 else 1.08)
        model.geom_rgba[gid] = [*np.clip(color, 0, 1), 1]


def explode(model, meta):
    offsets = np.zeros((model.nbody, 3))
    for record in meta['segments']:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, record['name'])
        if record['region'] == 'body':
            offsets[bid] = [-record['body_index'] * .0045, 0, 0]
        else:
            offsets[bid] = offsets[model.body_parentid[bid]]
            if record['region'] == 'chin': offsets[bid] += [.020, 0, -.014]
            elif record['region'] == 'dorsal': offsets[bid] += [0, 0, .020]
            elif record['region'] == 'anal': offsets[bid] += [0, 0, -.020]
            elif record['region'] == 'caudal': offsets[bid] += [-.016, 0, 0]
            else:
                sign = 1 if 'L' in record['name'] else -1
                # Both paired fins stay below the body in the exploded view.
                # Stagger along x to distinguish the pair without moving one
                # dorsally. Different z offsets compensate for the oblique view.
                if record['region'] == 'pectoral':
                    offsets[bid] += [.003 if sign > 0 else .029, .015 * sign, -.041 if sign > 0 else -.024]
                else:
                    offsets[bid] += [-.005 if sign > 0 else .018, .010 * sign, -.017 if sign > 0 else -.018]
    for bid in range(1, model.nbody):
        model.body_pos[bid] += offsets[bid] - offsets[model.body_parentid[bid]]


def surface_cut_lines(meta, camera):
    """Actual outer/cap boundary edges, restricted to the facing half."""
    records = {r['name']: r for r in meta['segments']}
    all_edges = []
    for joint in meta['joints']:
        i = joint['body_index']
        name = 'body_00_head' if i == 1 else f'body_{i-1:02d}'
        rec = records[name]
        mesh = trimesh.load(MODEL / 'assets/meshes' / f'{name}.ply', process=False)
        mesh.vertices += np.array(rec['origin'])
        outer = np.load(MODEL / 'assets/meshes' / rec['outer_mask_file'])
        adj = mesh.face_adjacency
        mixed = outer[adj[:, 0]] != outer[adj[:, 1]]
        points = mesh.vertices[mesh.face_adjacency_edges[mixed]]
        pivot = np.array(joint['pivot']); radius = joint['radius']
        mid = points.mean(axis=1)
        on_sphere = np.max(np.abs(np.linalg.norm(points-pivot, axis=2)-radius), axis=1) < .002 * radius + 2e-7
        facing = (mid[:, 1] * camera.out[1]) > 0
        anterior_segment_cap = mid[:, 0] < pivot[0]
        all_edges.extend(points[on_sphere & facing & anterior_segment_cap])
    return np.array(all_edges)


def render_panels(cfg, meta):
    rc = cfg['render']; aspect = rc['width_px'] / rc['height_px']
    pivot = np.array(meta['chin_joint']['pivot'])
    cameras = {
        p['id']: Camera([0, 0, 0], rc['full_view_height_m'], rc['camera_outward'], aspect)
        for p in cfg['panels']
    }
    cameras['c'] = Camera([-.019, 0, -.002], rc['exploded_view_height_m'], rc['camera_outward'], aspect)
    cameras['f'] = Camera(pivot + [.007, 0, .001], rc['chin_view_height_m'], [.27, -1, .23], aspect)
    overlay = {}
    for panel in cfg['panels']:
        key = panel['id']; camera = cameras[key]
        model = load_model(camera, cfg, source=key == 'a')
        if key != 'a':
            geometry_style(model, meta, cfg, {'b': 'gray', 'c': 'regions', 'd': 'rig'}.get(key, 'textured'))
        if key == 'c': explode(model, meta)
        data = mujoco.MjData(model); mujoco.mj_forward(model, data)
        options = mujoco.MjvOption(); options.geomgroup[:] = 0; options.geomgroup[1] = 1
        options.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = False
        renderer = mujoco.Renderer(model, rc['height_px'], rc['width_px'])
        renderer.update_scene(data, camera='paper_camera', scene_option=options)
        rgb = renderer.render().copy(); renderer.close()
        stem = f"Figure1{key}_{panel['name']}"
        im = Image.fromarray(rgb)
        im.save(OUT / 'raw' / f'{stem}.png', dpi=(cfg['figure']['dpi'],) * 2)
        im.save(OUT / 'raw' / f'{stem}.tif', compression='tiff_lzw', dpi=(cfg['figure']['dpi'],) * 2)
        item = {'camera': camera.values(), 'stem': stem}
        if key == 'c':
            item['exploded_fin_positions'] = {
                rec['name']: data.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, rec['name'])].copy()
                for rec in meta['segments'] if rec['region'] not in ('body', 'chin')
            }
        if key == 'b': item['cut_edges'] = surface_cut_lines(meta, camera)
        if key == 'd':
            joints, links = [], []
            for rec in meta['segments']:
                bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, rec['name'])
                if rec['parent'] is None: continue
                parent = model.body_parentid[bid]
                start = data.xipos[parent] if rec['parent'] == 'body_00_head' else data.xpos[parent]
                links.append([start.copy(), data.xpos[bid].copy()]); joints.append(data.xpos[bid].copy())
            item['links'] = np.array(links); item['joints'] = np.array(joints)
        overlay[key] = item
        print('Rendered', stem, flush=True)
    write_json(OUT / 'raw/projections.json', overlay)
    return overlay


def add_scale(ax, camera, length, label, x=.07, y=.09):
    end = x + length / camera.width
    prefix = ax.get_gid()
    ax.plot([x, end], [y, y], color='#17191c', lw=.75, solid_capstyle='butt', gid=prefix+'-scale-bar')
    ax.text((x+end)/2, y+.035, label, ha='center', va='bottom', fontsize=5.5, gid=prefix+'-scale-label')


def overlay_panel(ax, key, item, cfg, meta):
    c = item['camera']
    camera = Camera(c['look'], c['height'], c['out'], c['width']/c['height'])
    col = cfg['colors']
    if key in ('a', 'e'):
        add_scale(ax, camera, .020, '20 mm')
    if key == 'b':
        edges = camera.project(np.array(item['cut_edges']))
        lines = LineCollection(edges, colors=col['cuts'], linewidths=.48, zorder=3)
        lines.set_gid('spherical-interface-edges'); ax.add_collection(lines)
        j = meta['chin_joint']; xy = camera.project(j['pivot'])
        t = np.linspace(0, 2*np.pi, 160)
        ax.plot(xy[0] + j['radius']/camera.width*np.cos(t), xy[1] + j['radius']/camera.height*np.sin(t), color=col['cuts'], lw=.5, gid='chin-cut-sphere')
        ax.annotate('Spherical socket', xy=xy, xytext=(.70, .16), fontsize=5.6,
                    ha='center', arrowprops={'arrowstyle':'-', 'lw':.45, 'color':col['text']}, gid='socket-cut-label')
    if key == 'c':
        for i, (name, label) in enumerate([('body', 'Body'), ('fins', 'Fins'), ('chin', 'Chin')]):
            xx = .23 + i*.23
            ax.plot([xx-.04, xx-.01], [.075, .075], color=col[name], lw=2.1, solid_capstyle='butt')
            ax.text(xx+.01, .075, label, fontsize=5.7, va='center', ha='left')
    if key == 'd':
        links = camera.project(item['links']); joints = camera.project(item['joints'])
        lines = LineCollection(links, colors=col['joints'], linewidths=.65, zorder=3)
        lines.set_gid('kinematic-tree'); ax.add_collection(lines)
        ax.scatter(joints[:, 0], joints[:, 1], s=4, color=col['joints'], edgecolors='white', linewidths=.3, zorder=4, gid='joint-centres')
    if key == 'f':
        j = meta['chin_joint']; xy = camera.project(j['pivot'])
        t = np.linspace(0, 2*np.pi, 160)
        ax.plot(xy[0] + j['radius']/camera.width*np.cos(t), xy[1] + j['radius']/camera.height*np.sin(t), color=col['joints'], lw=.6, gid='chin-joint-outline')
        ax.annotate('Spherical joint', xy=xy+np.array([0, j['radius']/camera.height]),
                    xytext=(.66, .89), fontsize=6, ha='center',
                    arrowprops={'arrowstyle':'-', 'lw':.5, 'color':col['text']}, gid='chin-joint-label')
        add_scale(ax, camera, .005, '5 mm', x=.73, y=.12)
    ax.set_xlim(0, 1); ax.set_ylim(0, 1)


def panel(fig, spec, rect, cfg, meta, data, label_xy, title_xy, detail_xy):
    ax = fig.add_axes(rect, label='panel-' + spec['id'])
    ax.set_gid('panel-' + spec['id'] + '-artwork')
    path = OUT / 'raw' / (data['stem'] + '.png')
    ax.imshow(Image.open(path), extent=(0, 1, 0, 1), aspect='auto', interpolation='none', gid='render-' + spec['id'])
    overlay_panel(ax, spec['id'], data, cfg, meta); ax.axis('off')
    fs = cfg['figure']
    fig.text(*label_xy, spec['id'], fontsize=fs['label_pt'], fontweight='bold', va='top', gid='panel-' + spec['id'] + '-letter')
    fig.text(*title_xy, spec['title'], fontsize=fs['title_pt'], ha='center', va='top', gid='panel-' + spec['id'] + '-title')
    fig.text(*detail_xy, spec['detail'], fontsize=fs['detail_pt'], ha='center', va='top', color='#474c51',
             fontstyle='italic' if spec['id'] == 'a' else 'normal', gid='panel-' + spec['id'] + '-detail')


def svg_layers(path):
    """Keep normal SVG text and name editable top-level groups for Inkscape."""
    ET.register_namespace('', SVG)
    ET.register_namespace('xlink', 'http://www.w3.org/1999/xlink')
    ET.register_namespace('inkscape', INKSCAPE)
    tree = ET.parse(path)
    for group in tree.iter('{' + SVG + '}g'):
        name = group.get('id', '')
        if name.startswith('panel-') or name.startswith('flow-arrow-'):
            group.set('{' + INKSCAPE + '}groupmode', 'layer')
            group.set('{' + INKSCAPE + '}label', name.replace('-', ' '))
    tree.write(path, encoding='utf-8', xml_declaration=True)


def save_artwork(fig, stem, cfg):
    fig.savefig(stem.with_suffix('.pdf'), dpi=cfg['figure']['dpi'],
                metadata={'Title': stem.name, 'Creator': 'paper/make_figure1.py', 'Author': 'SimFish'})
    fig.savefig(stem.with_suffix('.svg'), dpi=cfg['figure']['dpi'])
    svg_layers(stem.with_suffix('.svg'))
    fig.savefig(stem.with_suffix('.png'), dpi=cfg['figure']['dpi'])
    plt.close(fig)


def assemble(cfg, meta, data):
    f = cfg['figure']; w, h = f['width_mm'], f['height_mm']
    layout = cfg['layout']; margin = layout['margin_mm']; gap = layout['column_gap_mm']
    fig = plt.figure(figsize=(w/25.4, h/25.4), facecolor='white')
    colw = (w-2*margin-2*gap)/3
    imageh = colw * cfg['render']['height_px'] / cfg['render']['width_px']
    bottoms = [layout['lower_image_bottom_mm'] + layout['row_pitch_mm'], layout['lower_image_bottom_mm']]
    for i, spec in enumerate(cfg['panels']):
        x = margin + (i % 3)*(colw+gap)
        bottom = bottoms[i//3]
        panel(fig, spec, [x/w, bottom/h, colw/w, imageh/h], cfg, meta, data[spec['id']],
              ((x+.3)/w, (bottom+imageh+3)/h),
              ((x+colw/2)/w, (bottom-1.2)/h),
              ((x+colw/2)/w, (bottom-4.7)/h))
    for bottom in bottoms:
        for i in (0, 1):
            arrow_length = layout.get('arrow_length_mm', gap-1)
            start = margin + colw + i*(colw+gap) + (gap-arrow_length)/2
            fig.add_artist(FancyArrowPatch((start/w, (bottom+imageh/2)/h), ((start+arrow_length)/w, (bottom+imageh/2)/h),
                           transform=fig.transFigure, arrowstyle='-|>,head_length=0.65,head_width=0.34',
                           mutation_scale=layout['arrow_head_pt'], shrinkA=0, shrinkB=0,
                           linewidth=layout['arrow_line_pt'], color='#262a2d', gid=f'flow-arrow-{bottom}-{i}'))
    save_artwork(fig, OUT / 'Figure1_overview', cfg)
    for spec in cfg['panels']:
        pw, ph = colw, imageh+10
        fig = plt.figure(figsize=(pw/25.4, ph/25.4), facecolor='white')
        panel(fig, spec, [0, 7/ph, 1, imageh/ph], cfg, meta, data[spec['id']],
              (.005, (ph-.3)/ph), (.5, 5.8/ph), (.5, 2.3/ph))
        save_artwork(fig, OUT / 'panels' / data[spec['id']]['stem'], cfg)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--assemble-only', action='store_true')
    args = ap.parse_args()
    cfg = yaml.safe_load((HERE / 'figure1.yaml').read_text())
    font = cfg['figure']['font']
    fonts = {style: font_manager.findfont(font_manager.FontProperties(family=font, weight=weight, style=slant), fallback_to_default=False)
             for style, weight, slant in [('regular', 'normal', 'normal'), ('bold', 'bold', 'normal'), ('italic', 'normal', 'italic')]}
    plt.rcParams.update({'font.family':font, 'font.size':6, 'pdf.fonttype':42, 'ps.fonttype':42,
                         'svg.fonttype':'none', 'svg.image_inline':True, 'image.composite_image':False,
                         'text.color':cfg['colors']['text'], 'savefig.facecolor':'white'})
    for folder in (OUT, OUT/'raw', OUT/'panels'): folder.mkdir(parents=True, exist_ok=True)
    meta = json.loads((MODEL / 'assets/segments.json').read_text())
    # Fail on outdated descriptive numbers instead of silently creating a stale figure.
    assert sum(r['region']=='body' for r in meta['segments']) == 14
    assert len(meta['segments']) == 22
    assert [meta['chin_joint'][a+'_deg'] for a in ('yaw', 'pitch', 'roll')] == [45,45,30]
    assert mujoco.MjModel.from_xml_path(str(MODEL/'fish.xml')).nu == 40
    data = json.loads((OUT/'raw/projections.json').read_text()) if args.assemble_only else render_panels(cfg, meta)
    assemble(cfg, meta, data)
    inputs = [MODEL/'fish.xml', MODEL/'config.yaml', MODEL/'assets/segments.json',
              MODEL/'assets/meshes/original.obj', MODEL/'assets/textures/fish_atlas.png',
              MODEL/'assets/textures/Elephant_Nose_Fish_Diffuse.png', HERE/'figure1.yaml', Path(__file__)]
    inputs += [MODEL/'assets/meshes'/r['mesh'] for r in meta['segments']]
    write_json(OUT/'provenance.json', {'figure_size_mm':[cfg['figure']['width_mm'],cfg['figure']['height_mm']],
        'font_files':fonts, 'raster_render_px':[cfg['render']['width_px'],cfg['render']['height_px']],
        'raster_effective_dpi':cfg['render']['width_px']/((cfg['figure']['width_mm']-2*cfg['layout']['margin_mm']-2*cfg['layout']['column_gap_mm'])/3/25.4),
        'pdf_image_export_dpi':cfg['figure']['dpi'], 'mujoco_version':mujoco.__version__,
        'source_type':'User-supplied textured external surface model; no micro-CT or thresholded scan data.',
        'sha256':{str(p.relative_to(HERE.parent)):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}})
    print('Saved', OUT/'Figure1_overview.pdf', flush=True)


if __name__ == '__main__':
    main()
