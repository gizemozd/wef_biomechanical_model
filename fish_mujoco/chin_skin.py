"""Continuous textured head/chin visual skin over the spherical mechanical joint.

MuJoCo's two-bone skin follows the existing head and actuated chin bodies. This
changes visualization only: masses, contacts, fluid proxies, and DOFs stay intact.
"""
import struct
import numpy as np
import trimesh
from common import ROOT, load_source, dump


def smoothstep(t):
    t=np.clip(t,0,1)
    return t*t*t*(10+t*(-15+6*t))


def build(c,meta):
    records={r['name']:r for r in meta['segments']}
    parts=[]
    for name in ['body_00_head','chin_0']:
        rec=records[name]
        obj=trimesh.load(ROOT/'assets/meshes'/rec['mesh'],force='mesh',process=False)
        solid=trimesh.Trimesh(obj.vertices,obj.faces,process=True)
        solid.vertices+=np.asarray(rec['origin']);parts.append(solid)
    # Boolean union removes the internal cap/socket from the visible surface.
    # The original, closed external shape spans both sides of the circular cut.
    surface=trimesh.boolean.union(parts,engine='manifold')
    if not surface.is_watertight or not surface.is_winding_consistent:
        raise ValueError('Head/chin union must be a consistently oriented closed surface')
    for _ in range(c['segmentation']['chin_skin_subdivisions']):
        surface=surface.subdivide()
    source,_,scale,center=load_source(c)
    source.apply_scale(1000)
    _,distance,faceid=trimesh.proximity.closest_point(source,surface.triangles_center*1000)
    source_uv=source.visual.uv[source.faces]
    uv=np.empty((len(surface.faces),3,2))
    for corner in range(3):
        bary=trimesh.triangles.points_to_barycentric(source.triangles[faceid],surface.triangles[:,corner]*1000)
        uv[:,corner]=np.einsum('ij,ijk->ik',bary,source_uv[faceid])
    # Same padded atlas as the rigid visuals, including the head interior swatch.
    uv[:,:,1]=(uv[:,:,1]*4096+128)/4224
    uv[distance>0.0002]=[32/4096,96/4224]
    # Keep UV islands separate while sharing each island's vertices for normals.
    # Position IDs remain available to prove coincident UV copies never separate.
    ids=surface.faces.reshape(-1);uv=uv.reshape(-1,2)
    key=np.column_stack([ids,np.round(uv,6)])
    _,first,inverse=np.unique(key,axis=0,return_index=True,return_inverse=True)
    position_ids=ids[first];vertices=surface.vertices[position_ids]
    texcoord=uv[first].copy();faces=inverse.reshape(-1,3).astype(np.int32)
    # MuJoCo flips OBJ v on import; binary SKN coordinates are already native.
    texcoord[:,1]=1-texcoord[:,1]
    pivot=np.asarray(meta['chin_joint']['pivot'])
    source_points=vertices/scale+center
    lo,hi=c['segmentation']['chin_skin_blend_radii_m']
    if not 0<=lo<hi:raise ValueError('chin_skin_blend_radii_m must increase')
    weight=smoothstep((np.linalg.norm(vertices-pivot,axis=1)-lo)/(hi-lo))
    # Headward vertices and the mouth/jaw must remain attached to the head.
    weight*=smoothstep((source_points[:,0]-c['segmentation']['chin_pivot_source'][0])/.6)
    low_z,high_z=c['segmentation']['chin_skin_mouth_fade_source_z']
    if not low_z<high_z:raise ValueError('chin_skin_mouth_fade_source_z must increase')
    weight*=1-smoothstep((source_points[:,2]-low_z)/(high_z-low_z))
    path=ROOT/'assets/meshes/chin_surface.skn'
    with path.open('wb') as f:
        f.write(struct.pack('<4i',len(vertices),len(vertices),len(faces),2))
        f.write(np.asarray(vertices,dtype='<f4').tobytes())
        f.write(np.asarray(texcoord,dtype='<f4').tobytes())
        f.write(np.asarray(faces,dtype='<i4').tobytes())
        for name,w in [('body_00_head',1-weight),('chin_0',weight)]:
            active=np.flatnonzero(w>0)
            f.write(name.encode().ljust(40,b'\0'))
            f.write(np.asarray(records[name]['origin'],dtype='<f4').tobytes())
            f.write(struct.pack('<4fi',1,0,0,0,len(active)))
            f.write(np.asarray(active,dtype='<i4').tobytes())
            f.write(np.asarray(w[active],dtype='<f4').tobytes())
    np.savez(ROOT/'assets/meshes/chin_surface.npz',vertices=vertices,faces=faces,
             texcoord=texcoord,chin_weight=weight,position_ids=position_ids,
             welded_vertices=surface.vertices,welded_faces=surface.faces)
    info={'file':path.name,'vertices':len(vertices),'triangles':len(faces),
          'watertight_at_rest':bool(surface.is_watertight),'bones':['body_00_head','chin_0'],
          'blend_radii_m':[lo,hi],'partially_weighted_vertices':int(((weight>0)&(weight<1)).sum()),
          'mechanical_cut':'existing concentric spherical cap/socket',
          'physics_changed':False,'note':'Visual linear-blend skin, not a soft-tissue mechanical simulation.'}
    dump(ROOT/'reports/chin_surface_build.json',info)
    return info
