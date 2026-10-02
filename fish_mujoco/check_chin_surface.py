"""Check the actual MuJoCo skin and render an identical-camera before/after."""
import itertools,json
import numpy as np
import mujoco
import trimesh
from common import ROOT,config,dump,report


def measure(model):
    asset=np.load(ROOT/'assets/meshes/chin_surface.npz')
    v=asset['vertices'];faces=asset['faces'];ids=asset['position_ids']
    _,first,inverse=np.unique(ids,return_index=True,return_inverse=True)
    welded_faces=inverse[faces]
    topology=trimesh.Trimesh(v[first],welded_faces,process=False)
    if not topology.is_watertight or not topology.is_winding_consistent:
        raise ValueError('Visible head/chin surface contains open or reversed edges')
    data=mujoco.MjData(model);opt=mujoco.MjvOption();cam=mujoco.MjvCamera()
    scene=mujoco.MjvScene(model,maxgeom=2000)
    sid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SKIN,'chin_surface')
    meta=json.loads((ROOT/'assets/segments.json').read_text())
    chin=next(r for r in meta['segments'] if r['name']=='chin_0')
    bind=np.asarray(chin['origin'])
    # Independent geometric region: exclude only the first 2 mm of the chin
    # mesh. Every remaining shaft vertex must keep its rigid body transform.
    shaft_start=bind[0]+chin['bounds_local'][0][0]+.002
    shaft=v[:,0]>=shaft_start
    if shaft.sum()<30:raise ValueError('Insufficient shaft vertices for rigidity check')
    chin_id=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'chin_0')
    adrs=[];limits=[]
    for axis in ['yaw','pitch','roll']:
        j=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,'j_chin_'+axis)
        adrs.append(model.jnt_qposadr[j]);limits.append(model.jnt_range[j,1])
    rows=[];max_rest_error=0
    poses=list(itertools.product([-1,-.5,0,.5,1],repeat=3))
    for factors in poses:
        mujoco.mj_resetDataKeyframe(model,data,0)
        angles=np.array(limits)*factors;data.qpos[adrs]=angles
        mujoco.mj_forward(model,data)
        mujoco.mjv_updateScene(model,data,opt,None,cam,mujoco.mjtCatBit.mjCAT_ALL,scene)
        start=scene.skinvertadr[sid];n=scene.skinvertnum[sid]
        pts=scene.skinvert.reshape(-1,3)[start:start+n].copy()
        normals=scene.skinnormal.reshape(-1,3)[start:start+n]
        if not np.isfinite(pts).all() or not np.isfinite(normals).all():
            raise ValueError('Nonfinite rendered skin')
        gaps=np.linalg.norm(pts-pts[first][inverse],axis=1)
        # UV islands may duplicate vertices but must never open geometric cracks.
        if gaps.max()>1e-7:raise ValueError('Separated UV boundary in rendered skin')
        if not any(factors):max_rest_error=float(np.max(np.linalg.norm(pts-v,axis=1)))
        expected=(v[shaft]-bind)@data.xmat[chin_id].reshape(3,3).T+data.xpos[chin_id]
        shaft_error=float(np.max(np.linalg.norm(pts[shaft]-expected,axis=1)))
        if shaft_error>1e-7:raise ValueError(f'Chin shaft bends instead of staying rigid: {shaft_error} m')
        posed=trimesh.Trimesh(pts[first],welded_faces,process=False)
        if posed.volume<=0:raise ValueError('Inverted whole skin volume')
        rows.append({'yaw_pitch_roll_deg':np.rad2deg(angles).tolist(),
                     'max_uv_boundary_gap_m':float(gaps.max()),'max_shaft_rigid_transform_error_m':shaft_error,
                     'volume_m3':float(posed.volume)})
    # A moving root must carry the skin rigidly when the chin is neutral.
    from scipy.spatial.transform import Rotation
    rotation=Rotation.from_euler('xyz',[.2,-.3,.4])
    mujoco.mj_resetDataKeyframe(model,data,0)
    data.qpos[:3]=[.01,-.02,.03];data.qpos[3:7]=rotation.as_quat()[[3,0,1,2]]
    mujoco.mj_forward(model,data)
    mujoco.mjv_updateScene(model,data,opt,None,cam,mujoco.mjtCatBit.mjCAT_ALL,scene)
    pts=scene.skinvert.reshape(-1,3)[start:start+n]
    root_error=float(np.max(np.linalg.norm(pts-(rotation.apply(v)+data.qpos[:3]),axis=1)))
    if root_error>1e-7:raise ValueError('Skin does not follow the root transform')
    result={'poses_checked':len(rows),'watertight_position_topology':bool(topology.is_watertight),
            'max_rest_position_error_m':max_rest_error,
            'max_root_transform_error_m':root_error,
            'max_visual_boundary_gap_m':max(r['max_uv_boundary_gap_m'] for r in rows),
            'max_shaft_rigid_transform_error_m':max(r['max_shaft_rigid_transform_error_m'] for r in rows),
            'checked_rigid_shaft_length_m':bind[0]+chin['bounds_local'][1][0]-shaft_start,
            'checked_rigid_shaft_vertices':int(shaft.sum()),
            'note':'Shared visible topology stays continuous. This does not assert volume-preserving tissue deformation or absence of every self-intersection.',
            'poses':rows}
    return result


def run(c):
    from render_videos import Render,annotate
    from PIL import Image
    model=mujoco.MjModel.from_xml_path(str(ROOT/'fish.xml'))
    result=measure(model);dump(ROOT/'reports/chin_surface_validation.json',result)
    data=mujoco.MjData(model);renderer=Render(model,900,600)
    bid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'chin_0')
    sheets=[]
    for label,angles,az,elevation in [
        ('Rest',(0,0,0),90,-10),('Pitch +45',(0,45,0),90,-10),
        ('Yaw +45',(45,0,0),90,-90),('Combined limits',(-45,-45,30),65,-24)]:
        mujoco.mj_resetDataKeyframe(model,data,0)
        for axis,angle in zip(['yaw','pitch','roll'],angles):
            j=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,'j_chin_'+axis)
            data.qpos[model.jnt_qposadr[j]]=np.deg2rad(angle)
        mujoco.mj_forward(model,data);look=data.xpos[bid]+[.005,0,0]
        frames=[]
        for skin in [False,True]:
            renderer.options.geomgroup[4]=not skin
            renderer.options.flags[mujoco.mjtVisFlag.mjVIS_SKIN]=skin
            frames.append(annotate(renderer.frame(data,look=look,distance=.061,azimuth=az,elevation=elevation),
                                   ('Continuous surface' if skin else 'Previous rigid surface')+' | '+label,
                                   'Same pose, camera, lighting, texture and physical joint'))
        sheets.append(np.concatenate(frames,axis=1))
    renderer.close()
    Image.fromarray(np.concatenate(sheets,axis=0)).save(ROOT/'reports/chin_surface_comparison.png')
    Image.fromarray(sheets[1]).save(ROOT/'reports/chin_attachment_comparison.png')
    report('chin_surface.md',f'''# Continuous chin attachment

The mechanical cut was already a concentric sphere. Rotating finite patches of that sphere cannot preserve a nonspherical exterior silhouette. The revised visual surface joins the original head and chin exterior, removes the internal cap/socket faces with a boolean union, and blends its movement smoothly between the existing head and chin bodies. The joint remains spherical underneath. The mouth and proximal head stay fixed to the head; the distal chin follows its three existing actuators.

The local skin uses the original texture coordinates. Its transition is confined to a short collar at the base: the configured radii are {c['segmentation']['chin_skin_blend_radii_m']} m, replacing the former 0.002–0.014 m blend that made the shaft look S-shaped. The mouth fade is confined to source z={c['segmentation']['chin_skin_mouth_fade_source_z']}. The remaining shaft follows the chin rigidly, preserving the source shape rather than introducing extra curvature. `segmentation.chin_skin`, `chin_skin_blend_radii_m`, `chin_skin_mouth_fade_source_z`, and `chin_skin_subdivisions` control it. `build_model.py` regenerates the skin from the current segment meshes; `make all` includes it automatically. Setting `chin_skin: false` restores rigid visuals. The exploded view deliberately displays rigid parts.

**Validation:** {result['poses_checked']} combinations of zero, half and full yaw/pitch/roll limits were checked using MuJoCo's actual updated skin vertices. The visible surface is closed by position topology, with maximum separation between coincident UV-boundary copies **{result['max_visual_boundary_gap_m']*1000:.6g} mm**. Maximum rest-position error versus the union mesh is {result['max_rest_position_error_m']*1000:.6g} mm. Rendered positions and normals remain finite, with positive enclosed volume. See `chin_surface_validation.json` for all sampled poses.

**Straight shaft:** all {result['checked_rigid_shaft_vertices']} rendered shaft vertices beyond the first 2 mm of the chin mesh match the rigid chin body's transform at every sampled pose, to within {result['max_shaft_rigid_transform_error_m']*1000:.6g} mm. This verifies a {result['checked_rigid_shaft_length_m']*1000:.2f} mm length of shaft keeps its shape rather than bending with the visual collar.

The previous rigid-interface gap measurements remain in `chin_seam_metrics.json` and `chin_compound_metrics.json`; they describe hidden rigid parts, not the continuous visible surface. Joint ranges remain ±45° yaw, ±45° pitch and ±30° roll. No swimming tuning, masses, fluid proxies, or actuator gains were changed.

This uses [MuJoCo visual skinning](https://mujoco.readthedocs.io/en/stable/XMLreference.html#deformable-skin), not a mechanical soft-tissue model. It does not add contact geometry or enforce tissue incompressibility. Severe combined bends can compress, stretch, or self-intersect the skin; the tests establish continuity at sampled poses, not anatomical accuracy or freedom from all intersections.

![Identical-camera comparison: rigid left, continuous right](chin_surface_comparison.png)
''')
    print({k:v for k,v in result.items() if k!='poses'})


if __name__=='__main__':
    import argparse
    ap=argparse.ArgumentParser();ap.add_argument('--config');args=ap.parse_args();run(config(args.config))
