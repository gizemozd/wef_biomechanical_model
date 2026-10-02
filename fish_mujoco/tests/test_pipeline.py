"""Deliverable contract and independent simulation checks."""
import json, sys
from pathlib import Path
import numpy as np
import pytest
import trimesh
import mujoco
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from common import ROOT,config
from controllers import Controller,simulate

@pytest.fixture(scope='module')
def cfg():return config()
@pytest.fixture(scope='module')
def meta():return json.loads((ROOT/'assets/segments.json').read_text())
@pytest.fixture(scope='module')
def model():return mujoco.MjModel.from_xml_path(str(ROOT/'fish.xml'))

def test_compiles_and_forward(model):
    d=mujoco.MjData(model);mujoco.mj_forward(model,d)
    assert np.isfinite(d.qacc).all();assert not d.warning.number.any()

def test_every_segment_closed_and_uv(meta):
    for s in meta['segments']:
        p=ROOT/'assets/meshes'/s['mesh']
        m=trimesh.load(p,force='mesh',process=False)
        assert m.visual.uv is not None and np.isfinite(m.visual.uv).all(),s['name']
        # OBJ UV seams split render vertices; welding position-only recovers solid topology.
        solid=trimesh.Trimesh(m.vertices,m.faces,process=True)
        assert solid.is_watertight and solid.is_winding_consistent and solid.volume>0,s['name']

def test_joint_ranges_and_actuators(model,meta,cfg):
    specs=json.loads((ROOT/'assets/joints.json').read_text())
    for j in specs:
        ji=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,j['name']);ai=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_ACTUATOR,'a_'+j['name'])
        np.testing.assert_allclose(model.jnt_range[ji],np.deg2rad(j['range_deg']),rtol=1e-8)
        np.testing.assert_allclose(model.actuator_ctrlrange[ai],model.jnt_range[ji],rtol=1e-8)
        assert model.jnt_limited[ji] and model.actuator_ctrllimited[ai]
    assert len([s for s in meta['segments'] if s['region']=='body'])==cfg['segmentation']['N_body']
    for j in meta['joints']:
        assert j['yaw_deg']<=cfg['joints'][j['region']+'_yaw_deg']
        assert j['pitch_deg']<=cfg['joints']['pitch_deg']

def test_seam_measurements(cfg):
    rows=json.loads((ROOT/'reports/seam_metrics.json').read_text())
    assert len(rows)==(cfg['segmentation']['N_body']-1)*10
    for r in rows:
        assert r['sample_count']>=8
        assert np.isfinite([r['gap_m'],r['penetration_m']]).all()
        threshold=cfg['segmentation']['seam_rest_fraction'] if r['angle_deg']==0 else cfg['segmentation']['seam_bent_fraction']
        assert r['gap_fraction']<=threshold,r

def test_chin_three_dofs_and_whole_median_fins(model,meta,cfg):
    chin=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'chin_0')
    head=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'body_00_head')
    assert model.body_parentid[chin]==head and model.body_jntnum[chin]==3
    for suffix,axis in [('yaw',[0,0,1]),('pitch',[0,1,0]),('roll',[1,0,0])]:
        jid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,'j_chin_'+suffix)
        aid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_ACTUATOR,'a_j_chin_'+suffix)
        assert jid>=0 and aid>=0
        np.testing.assert_allclose(model.jnt_axis[jid],axis)
        assert 0<meta['chin_joint'][suffix+'_deg']<=cfg['joints']['chin_'+suffix+'_deg']
        if not cfg['joints'].get('chin_auto_reduce_for_seams',True):
            np.testing.assert_allclose(np.rad2deg(model.jnt_range[jid]),[-cfg['joints']['chin_'+suffix+'_deg'],cfg['joints']['chin_'+suffix+'_deg']])
    for region in ['dorsal','anal']:
        fins=[s for s in meta['segments'] if s['region']==region]
        assert len(fins)==1 and fins[0]['name']==f'fin_{region}_0'
    assert not any('_ray_' in s['name'] for s in meta['segments'])
    for row in json.loads((ROOT/'reports/chin_seam_metrics.json').read_text())+json.loads((ROOT/'reports/chin_compound_metrics.json').read_text()):
        assert np.isfinite([row['gap_m'],row['penetration_m'],row['gap_fraction']]).all()
        assert row['within_threshold']==(row['gap_fraction']<=row['threshold'])
        enforce=cfg['segmentation'].get('chin_attachment')=='ball_socket' or cfg['joints'].get('chin_auto_reduce_for_seams',True) or row.get('angle_deg')==0
        assert row['enforced']==enforce
        if enforce:assert row['within_threshold']

def test_active_chin_scan_reaches_commanded_range(model,cfg):
    # Verify actual actuated movement relative to the head, not only ctrl limits.
    trace=simulate(model,cfg,'chin_scan',duration=5,record_fps=60)
    steady=trace['time']>1
    for suffix in ['yaw','pitch','roll']:
        jid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,'j_chin_'+suffix)
        q=trace['qpos'][steady,model.jnt_qposadr[jid]]
        amplitude=model.jnt_range[jid,1]*cfg['controller']['chin_amplitude_fraction']
        assert q.max()>.8*amplitude and q.min()<-.8*amplitude,suffix
        assert np.max(np.abs(q))<=model.jnt_range[jid,1]+.002,suffix

def test_chin_surface_continuity_at_combined_limits(model,cfg):
    if cfg['segmentation'].get('chin_attachment')=='ball_socket':
        from check_chin_socket import measure
        result=measure(model)
        assert model.nskin==0
        assert result['poses_checked']==125
        assert result['max_socket_rim_gap_m']<1e-7
        assert result['max_rigid_transform_error_m']<1e-7
        return
    if not cfg['segmentation'].get('chin_skin',False):pytest.skip('Rigid visual comparison mode')
    from check_chin_surface import measure
    assert model.nskin==1
    result=measure(model)
    assert result['poses_checked']==125
    assert result['watertight_position_topology']
    assert result['max_visual_boundary_gap_m']<1e-7
    assert result['max_rest_position_error_m']<1e-7
    assert result['checked_rigid_shaft_vertices']>30
    assert result['max_shaft_rigid_transform_error_m']<1e-7
    for name in ['body_00_head','chin_0']:
        gid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,'visual_'+name)
        assert model.geom_group[gid]==4  # avoid rigid fragments protruding through skin

def test_five_second_simulation(model,cfg):
    tr=simulate(model,cfg,duration=5)
    assert np.isfinite(tr['qpos']).all()

def test_rigid_root_contour_preserves_mouth_and_distal_shaft(cfg):
    if not cfg['segmentation'].get('chin_root_profile_source'):
        pytest.skip('No local rigid root contour')
    from common import load_source
    src,_,scale,center=load_source(cfg)
    reference=trimesh.load(ROOT/'assets/meshes/chin_root_reference.ply',force='mesh')
    reference.apply_scale(1000)
    source_points=src.vertices/scale+center
    pivot=cfg['segmentation']['chin_pivot_source'][0]
    profile=np.array(cfg['segmentation']['chin_root_profile_source'])
    protected=(source_points[:,2]>cfg['segmentation']['chin_upper_limit_source']+1e-4)
    protected|=source_points[:,0]<pivot+profile[0,0]-1e-4
    protected|=source_points[:,0]>pivot+profile[-1,0]+1e-4
    _,distance,_=trimesh.proximity.closest_point(reference,src.vertices[protected]*1000)
    assert distance.max()<.0002  # 0.2 micrometre tolerance on untouched exterior.

def test_neutral_mass_and_passive(model,meta,cfg):
    displacement=sum(s['displaced_volume_m3'] for s in meta['segments'])
    target=cfg['fluid']['density']*displacement
    assert abs(model.body_mass.sum()/target-1)<.01
    d=mujoco.MjData(model)
    for _ in range(round(5/model.opt.timestep)):mujoco.mj_step(model,d)
    assert np.linalg.norm(d.qpos[:3])<1e-6
    assert np.linalg.norm(d.qvel)<1e-6
    assert not d.warning.number.any()

def test_controller_limits_all_behaviors(model,cfg):
    for behavior in ['forward','backward','hover','turning','body_undulation','pitch','braking','chin_scan']:
        ctl=Controller(model,cfg,behavior)
        for t in np.linspace(0,5,101):
            q=ctl(t);assert np.all(q>=ctl.ranges[:,0]) and np.all(q<=ctl.ranges[:,1])
            if behavior=='chin_scan':assert np.all(q[ctl.kind!='chin']==0)

def test_physical_joint_limit_response(model):
    # An unactuated body hinge initialized beyond ROM must be restored by the constraint.
    d=mujoco.MjData(model);j=1;adr=model.jnt_qposadr[j];lo,hi=model.jnt_range[j]
    d.qpos[adr]=hi+.05
    for _ in range(1000):mujoco.mj_step(model,d)
    assert d.qpos[adr]<hi+.002 and d.qpos[adr]>lo-.002

def test_both_fin_groups_propel_forward(model,cfg):
    # Independent free-root runs: moving the tail must add actual forward force,
    # and disabling either set of strokes must reduce anterior speed.
    from evaluate_behaviors import evaluate
    _,both=evaluate(cfg,model,'forward')
    assert both['forces']['caudal']['mean_axial_mN']>0
    assert both['forces']['pectoral']['mean_axial_mN']>0
    thrust=sum(both['forces'][g]['mean_axial_mN'] for g in ['caudal','pectoral'])
    assert both['forces']['caudal']['mean_axial_mN']/thrust>.2
    for group in ['caudal','pectoral']:
        _,without=evaluate(cfg,model,'forward',{'disabled_groups':[group]})
        assert both['signed_speed_bl_s']>without['signed_speed_bl_s']+.0002

def test_both_fin_groups_active_in_four_behaviors(model,cfg):
    for name in ['forward','backward','hover','turning']:
        ctl=Controller(model,cfg,name)
        controls=np.array([ctl(t) for t in np.linspace(1,2,201)])
        assert np.all(controls[:,ctl.kind=='chin']==0),name
        for group in ['caudal','pectoral']:
            assert np.ptp(controls[:,ctl.kind==group],axis=0).max()>np.deg2rad(2),name
        # For straight strokes, sagittal reflection keeps axial y rotation's sign.
        if name!='turning':
            left=ctl.names.index('j_fin_pecL_0_rotate');right=ctl.names.index('j_fin_pecR_0_rotate')
            np.testing.assert_allclose(controls[:,left],controls[:,right])
