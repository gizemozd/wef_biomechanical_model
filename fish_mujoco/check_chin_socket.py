"""Validate and render the rigid ball/socket attachment in the compiled model."""
import itertools,json
import numpy as np
import trimesh
import mujoco
from common import ROOT,config,dump,report


def measure(model):
    if model.nskin:raise ValueError('Rigid chin model must not have a visual skin')
    samples=np.load(ROOT/'assets/meshes/chin_socket_samples.npz')['rim']
    data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    gid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,'visual_chin_0')
    mid=model.geom_dataid[gid]
    start=model.mesh_vertadr[mid];count=model.mesh_vertnum[mid]
    vertices=model.mesh_vert[start:start+count].astype(float)
    fa=model.mesh_faceadr[mid];fn=model.mesh_facenum[mid]
    mesh=trimesh.Trimesh(vertices*1000,model.mesh_face[fa:fa+fn],process=True)
    if not mesh.is_watertight:raise ValueError('Compiled chin is not a closed solid')
    bid=model.geom_bodyid[gid];hid=model.body_parentid[bid]
    bind=(vertices@data.geom_xmat[gid].reshape(3,3).T+data.geom_xpos[gid]-data.xpos[bid])@data.xmat[bid].reshape(3,3)
    rim_local=(samples-data.xpos[hid])@data.xmat[hid].reshape(3,3)
    adrs=[];limits=[]
    for axis in ['yaw','pitch','roll']:
        j=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,'j_chin_'+axis)
        adrs.append(model.jnt_qposadr[j]);limits.append(model.jnt_range[j,1])
    rows=[]
    for factors in itertools.product([-1,-.5,0,.5,1],repeat=3):
        mujoco.mj_resetDataKeyframe(model,data,0)
        data.qpos[adrs]=np.array(limits)*factors;mujoco.mj_forward(model,data)
        rim=rim_local@data.xmat[hid].reshape(3,3).T+data.xpos[hid]
        query=(rim-data.geom_xpos[gid])@data.geom_xmat[gid].reshape(3,3)*1000
        distances=trimesh.proximity.signed_distance(mesh,query)/1000
        actual=vertices@data.geom_xmat[gid].reshape(3,3).T+data.geom_xpos[gid]
        expected=bind@data.xmat[bid].reshape(3,3).T+data.xpos[bid]
        gap=float(max(0,-distances.min()));rigid_error=float(np.linalg.norm(actual-expected,axis=1).max())
        if gap>1e-7:raise ValueError(f'Socket opens at {factors}: {gap} m')
        if rigid_error>1e-7:raise ValueError('Chin geometry deforms')
        if not np.isfinite(data.qacc).all() or data.warning.number.any():raise ValueError('Invalid ROM pose')
        rows.append({'yaw_pitch_roll_deg':np.rad2deg(data.qpos[adrs]),'socket_rim_gap_m':gap,
                     'max_penetration_m':float(max(0,distances.max())),
                     'rigid_transform_error_m':rigid_error})
    return {'poses_checked':len(rows),'rim_samples':len(samples),'nskin':model.nskin,
            'max_socket_rim_gap_m':max(x['socket_rim_gap_m'] for x in rows),
            'max_rigid_transform_error_m':max(x['rigid_transform_error_m'] for x in rows),
            'note':'Checks coverage of the head socket rim by the rigid chin solid. Exposed ball contour and texture continuity are reviewed in renders; this is not a soft-tissue model.',
            'poses':rows}


def run(c):
    from render_videos import Render,annotate
    from PIL import Image
    model=mujoco.MjModel.from_xml_path(str(ROOT/'fish.xml'))
    result=measure(model);dump(ROOT/'reports/chin_socket_validation.json',result)
    meta=json.loads((ROOT/'assets/segments.json').read_text());j=meta['chin_joint'];pivot=np.array(j['pivot'])
    d=mujoco.MjData(model);r=Render(model,900,600);frames=[]
    for label,angles,az,el in [('Rest',[0,0,0],90,-10),('Pitch +45',[0,45,0],90,-10),
                              ('Yaw +45',[45,0,0],90,-90),('Combined limits',[-45,-45,30],65,-24)]:
        mujoco.mj_resetDataKeyframe(model,d,0)
        for axis,angle in zip(['yaw','pitch','roll'],angles):
            jid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,'j_chin_'+axis)
            d.qpos[model.jnt_qposadr[jid]]=np.deg2rad(angle)
        mujoco.mj_forward(model,d)
        frames.append(annotate(r.frame(d,look=pivot+[.003,0,0],distance=.055,azimuth=az,elevation=el),
                               label,'Rigid spherical attachment | no stretched collar'))
    mujoco.mj_resetDataKeyframe(model,d,0)
    chin_id=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'chin_0')
    position=model.body_pos[chin_id].copy()
    model.body_pos[chin_id]+=np.array([.010,-.004,-.002])
    mujoco.mj_forward(model,d)
    parts=annotate(r.frame(d,look=pivot+[.006,0,0],distance=.07,azimuth=60,elevation=-20),
                   'Rigid joint parts separated','Concave socket on head | full spherical ball on chin')
    Image.fromarray(parts).save(ROOT/'reports/chin_socket_parts.png')
    model.body_pos[chin_id]=position
    r.close()
    Image.fromarray(np.concatenate([np.concatenate(frames[:2],axis=1),np.concatenate(frames[2:],axis=1)],axis=0)).save(ROOT/'reports/chin_socket_preview.png')
    report('chin_socket.md',f'''# Rigid rounded chin attachment

The moving chin now contains a complete spherical ball of radius **{j['radius']*1000:.3f} mm**. The head contains the matching concave socket. Both surfaces share the actual three-axis joint center. A radial overlap of {j['overlap_m']*1000:.4f} mm covers faceting tolerances. The chin is one rigid textured mesh; **no deformable visual skin is loaded**. This removes the stretched, kinked collar from the previous revision.

The center is at source coordinates {c['segmentation']['chin_pivot_source']}, at the mouthward root indicated in the user's marked image. Relative to the previous [9.50, 0, 2.60] center, it moves {(9.50-c['segmentation']['chin_pivot_source'][0])*meta['scale']*1000:.2f} mm toward the mouth and {(c['segmentation']['chin_pivot_source'][2]-2.60)*meta['scale']*1000:.2f} mm dorsally. The source radius is {c['segmentation']['chin_radius_source']}, a {100*(1-c['segmentation']['chin_radius_source']/.265):.2f}% reduction from 0.265.

The nearby exterior is permanently recontoured into a smooth, narrow rigid waist using `chin_root_profile_source`. The head rim and chin root shelter the smaller ball, removing the protruding bead appearance at rest. **The complete exterior stays rigid**, with no flexible cover. The protected upper mouth and distal shaft retain their source geometry. The changed root surface receives outward radial UV projection from the source exterior, avoiding accidental projection onto the inside of the mouth. This is an engineering adjustment to match the marked location and appearance, not a recovered anatomical joint. Inertias and displaced volumes are regenerated from the new solids; swimming tuning is not rerun. `chin_root_contour.json` records the local volume removed.

Full requested limits are retained: yaw ±{j['yaw_deg']:g}°, pitch ±{j['pitch_deg']:g}°, roll ±{j['roll_deg']:g}°. All {result['poses_checked']} combinations of rest, half and full angles were checked against the **compiled MuJoCo mesh**. Maximum sampled socket-rim opening: {result['max_socket_rim_gap_m']*1000:.6g} mm across {result['rim_samples']} rim samples. Maximum departure from a rigid transform: {result['max_rigid_transform_error_m']*1000:.3g} mm. All new head cut faces near the joint are also checked to lie on the socket sphere, excluding hidden planar leftovers.

The metric checks socket coverage, not exact tangency or uninterrupted texture markings between separate rotating parts. A small rounded base remains visible, and extreme upward combinations may intersect nearby head geometry because self-contact is disabled. There is no elastic tissue model or claim of anatomically validated ROM. The old skin reports describe superseded revisions.

Rebuild without swimming optimization:

```sh
.venv/bin/python segment_mesh.py
.venv/bin/python build_model.py
.venv/bin/python render_videos.py --only chin
.venv/bin/python render_videos.py --only exploded
.venv/bin/python -m pytest tests -q
```

![Rest and full-range views](chin_socket_preview.png)

![Rigid joint pieces, separated](chin_socket_parts.png)
''')
    print({k:v for k,v in result.items() if k!='poses'})


if __name__=='__main__':run(config())
