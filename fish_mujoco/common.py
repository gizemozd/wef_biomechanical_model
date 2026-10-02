"""Shared configuration, geometry and deterministic I/O."""
from pathlib import Path
import json, os, sys
import numpy as np
import yaml
import trimesh
ROOT = Path(__file__).resolve().parent

def config(path=None):
    p = Path(path or os.environ.get('FISH_CONFIG') or ROOT / 'config.yaml').resolve()
    c = yaml.safe_load(p.read_text()); c['_config_dir'] = str(p.parent)
    return c

def source_path(c, key):
    return (Path(c['_config_dir']) / c['source'][key]).resolve()

def dump(path, value):
    def default(x):
        if isinstance(x, np.ndarray): return x.tolist()
        if isinstance(x, np.generic): return x.item()
        raise TypeError(type(x))
    Path(path).write_text(json.dumps(value, indent=2, default=default)+'\n')

def gl_backend():
    os.environ.setdefault('MUJOCO_GL', 'cgl' if sys.platform == 'darwin' else 'egl')

def load_source(c):
    scene = trimesh.load(source_path(c,'mesh'), force='scene', process=False)
    body = max(scene.geometry.values(), key=lambda m:len(m.faces)).copy()
    R = np.array(c['source']['rotation'])
    oriented = body.vertices @ R.T
    scale = c['source']['length_m'] / np.ptp(oriented[:,0])
    center = (oriented.max(0)+oriented.min(0))/2
    for m in scene.geometry.values(): m.vertices = (m.vertices @ R.T-center)*scale
    body = max(scene.geometry.values(), key=lambda m:len(m.faces)).copy()
    return body, scene, scale, center

def solid(mesh):
    m = trimesh.Trimesh(mesh.vertices,mesh.faces,process=True)
    m.update_faces(m.unique_faces()); m.update_faces(m.nondegenerate_faces()); m.remove_unreferenced_vertices()
    edges = m.edges[trimesh.grouping.group_rows(m.edges_sorted,require_count=1)]
    repairs=[]
    if len(edges):
        import networkx as nx
        cycles=nx.cycle_basis(nx.from_edgelist(edges))
        vertices=m.vertices.tolist(); faces=m.faces.tolist()
        for loop in cycles:
            p=m.vertices[loop].mean(0); k=len(vertices); vertices.append(p.tolist())
            faces.extend([[loop[i],loop[(i+1)%len(loop)],k] for i in range(len(loop))])
            repairs.append({'boundary_vertices':len(loop),'center_m':p.tolist()})
        m=trimesh.Trimesh(vertices,faces,process=True)
    m.fix_normals(multibody=True)
    if not m.is_watertight: raise ValueError('Boundary repair did not produce a watertight mesh')
    return m, repairs

def export_obj(path, mesh, uv=None, normals=None):
    """Separate face-corner UV indices preserve seams without unwelding geometry."""
    with open(path,'w') as f:
        f.write('# meters; x anterior, y left, z dorsal\nmtllib fish.mtl\nusemtl fish\n')
        for p in mesh.vertices: f.write('v %.10g %.10g %.10g\n'%tuple(p))
        if uv is None: uv=np.zeros((len(mesh.faces),3,2))
        for p in np.asarray(uv).reshape(-1,2): f.write('vt %.10g %.10g\n'%tuple(p))
        if normals is not None:
            for p in np.asarray(normals).reshape(-1,3): f.write('vn %.10g %.10g %.10g\n'%tuple(p))
        for i,face in enumerate(mesh.faces):
            f.write('f '+' '.join(f'{v+1}/{3*i+j+1}'+(f'/{3*i+j+1}' if normals is not None else '') for j,v in enumerate(face))+'\n')

def report(name,text):
    (ROOT/'reports'/name).write_text(text)
