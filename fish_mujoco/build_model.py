"""Generate MJCF from segmented, textured assets. Never hand-edit fish.xml."""
import argparse, json, xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from common import *

def fmt(x):
    if np.isscalar(x):return f'{x:.10g}' if not isinstance(x,str) else x
    return ' '.join(f'{v:.10g}' for v in x)

def node(parent,tag,**kw):return ET.SubElement(parent,tag,{k:fmt(v) for k,v in kw.items()})

def run(c):
    meta=json.loads((ROOT/'assets/segments.json').read_text()); records=meta['segments'];jc=c['joints'];fl=c['fluid'];rc=c['render']
    continuous_chin=c['segmentation'].get('chin_skin',False)
    if continuous_chin and c['segmentation'].get('chin_attachment')=='ball_socket':
        raise ValueError('Rigid ball/socket attachment requires chin_skin: false')
    if continuous_chin:
        from chin_skin import build
        skin=build(c,meta)
    root=ET.Element('mujoco',model='Gnathonemus_petersii')
    node(root,'compiler',angle='degree',meshdir='assets/meshes',texturedir='assets/textures',inertiafromgeom='false',autolimits='true')
    node(root,'option',timestep=c['simulation']['timestep'],gravity=[0,0,-9.81 if fl['gravity'] else 0],density=fl['density'],viscosity=fl['viscosity'],integrator='implicitfast',solver='Newton',iterations=30)
    node(root,'size',nuser_body=1)
    vis=node(root,'visual');node(vis,'global',offwidth=rc['width'],offheight=rc['height'],azimuth=rc['azimuth'],elevation=rc['elevation'])
    node(vis,'quality',shadowsize=rc['shadow_size'],offsamples=rc['samples']);node(vis,'headlight',ambient=[.45,.45,.45],diffuse=[.65,.65,.65],specular=[.15,.15,.15])
    node(vis,'rgba',haze=[.05,.16,.19,1]);node(vis,'map',znear=.002,zfar=20,fogstart=1,fogend=5)
    node(root,'statistic',center=[0,0,0],extent=.25,meansize=.008)
    asset=node(root,'asset')
    node(asset,'texture',name='water',type='skybox',builtin='gradient',rgb1=rc['background'],rgb2=[.12,.26,.28],width=512,height=3072)
    node(asset,'texture',name='diffuse',type='2d',file='fish_atlas.png')
    node(asset,'material',name='fish_material',texture='diffuse',specular=.12,shininess=.2,reflectance=.03)
    for rec in records:node(asset,'mesh',name=rec['name'],file=rec['mesh'],smoothnormal='true')
    if continuous_chin:
        node(node(root,'deformable'),'skin',name='chin_surface',file=skin['file'],material='fish_material',group=1)
    world=node(root,'worldbody');node(world,'light',name='key',pos=[.1,.4,.5],dir=[-.2,-.6,-1],diffuse=[.85,.85,.8],castshadow='true')
    node(world,'light',name='fill',pos=[-.2,-.3,.1],dir=[.3,.7,-.4],diffuse=[.5,.6,.65],castshadow='false')
    node(world,'camera',name='fixed_side',pos=[0,.34,.035],xyaxes=[-1,0,0,0,0,1],fovy=40)
    inertia_adjustments=[]
    bodies={};joint_specs=[];act=node(root,'actuator');sensors=node(root,'sensor');contact=node(root,'contact')
    limit_lookup={j['body_index']:j for j in meta['joints']}
    byname={r['name']:r for r in records}
    def add_joint(b,name,axis,limit,kind):
        isbody=kind=='body';kp=jc['body_kp'] if isbody else jc['fin_kp'];kv=jc['body_kv'] if isbody else jc['fin_kv']
        if kind=='chin':kp=jc['chin_kp'];kv=jc['chin_kv']
        node(b,'joint',name=name,type='hinge',axis=axis,range=[-limit,limit],stiffness=jc['stiffness'] if isbody else jc['fin_stiffness'],damping=jc['damping'] if isbody else jc['fin_damping'],armature=jc['armature'],solreflimit=[.003,1],margin=.00001)
        node(act,'position',name='a_'+name,joint=name,kp=kp,kv=kv,ctrlrange=np.deg2rad([-limit,limit]),forcerange=[-.02,.02])
        node(sensors,'jointpos',name=name+'_pos',joint=name);node(sensors,'jointvel',name=name+'_vel',joint=name)
        joint_specs.append({'name':name,'body':b.get('name'),'axis':axis,'range_deg':[-limit,limit],'kind':kind})
    for rec in records:
        name=rec['name'];parent=rec['parent'];origin=np.asarray(rec['origin']);po=np.zeros(3) if parent is None else np.asarray(byname[parent]['origin'])
        b=node(world if parent is None else bodies[parent],'body',name=name,pos=origin-po,gravcomp=fl['density']/fl['fish_density'] if fl['explicit_buoyancy'] else 0,user=rec['displaced_volume_m3'])
        bodies[name]=b
        inertia=np.array(rec['inertia']);vals,vecs=np.linalg.eigh(inertia)
        if np.linalg.det(vecs)<0:vecs[:,0]*=-1
        q=Rotation.from_matrix(vecs).as_quat()[[3,0,1,2]]
        floor=c['simulation']['minimum_inertia_kg_m2']
        clipped=np.maximum(vals,floor)
        if np.any(vals<floor):inertia_adjustments.append({'body':name,'original_eigenvalues':vals,'compiled_eigenvalues':clipped})
        node(b,'inertial',pos=rec['com_local'],mass=rec['mass_kg'],diaginertia=clipped,quat=q)
        if parent is None:
            node(b,'freejoint',name='root_free')
            node(b,'site',name='imu',size=.001,rgba=[0,0,0,0])
            node(b,'camera',name='tracking_side',mode='trackcom',pos=[0,.34,.035],xyaxes=[-1,0,0,0,0,1],fovy=40)
            node(b,'camera',name='tracking_top',mode='trackcom',pos=[0,0,.34],xyaxes=[-1,0,0,0,-1,0],fovy=40)
            node(b,'camera',name='fin_closeup',mode='trackcom',pos=[-.04,.15,.01],xyaxes=[-1,0,0,0,0,1],fovy=35)
        elif rec['region']=='body':
            j=limit_lookup[rec['body_index']]
            for suffix,axis in [('yaw',[0,0,1]),('pitch',[0,1,0])]:
                if j[suffix+'_deg']>0:add_joint(b,f'j_body_{rec["body_index"]:02d}_{suffix}',axis,j[suffix+'_deg'],'body')
        elif rec['region']=='pectoral':
            for suffix,axis,limit in zip(['abduct','protract','rotate'],[[1,0,0],[0,0,1],[0,1,0]],jc['pec_ranges_deg']):add_joint(b,'j_'+name+'_'+suffix,axis,limit,'pectoral')
        elif rec['region']=='chin':
            for suffix,axis in [('yaw',[0,0,1]),('pitch',[0,1,0]),('roll',[1,0,0])]:
                add_joint(b,'j_chin_'+suffix,axis,meta['chin_joint'][suffix+'_deg'],'chin')
        else:
            axis=[0,0,1] if rec['region']=='caudal' else [1,0,0]
            limit=jc['caudal_range_deg'] if rec['region']=='caudal' else (jc['median_fin_range_deg'] if rec['region'] in ('dorsal','anal') else jc['fin_range_deg'])
            add_joint(b,'j_'+name,axis,limit,rec['region'])
        # Keep diagnostic rigid meshes in hidden group 4; normal views use skin.
        group=4 if continuous_chin and name in ('body_00_head','chin_0') else 1
        node(b,'geom',name='visual_'+name,type='mesh',mesh=name,material='fish_material',contype=0,conaffinity=0,group=group,mass=0)
        # Inertia-equivalent ellipsoid aligns to principal axes; thin fins use their extents.
        if rec['region']=='body':
            vals,vecs=np.linalg.eigh(inertia)
            if np.linalg.det(vecs)<0:vecs[:,0]*=-1
            size=np.sqrt(np.maximum(5/(2*rec['mass_kg'])*(vals.sum()-2*vals),1e-10))
            xyzw=Rotation.from_matrix(vecs).as_quat();quat=xyzw[[3,0,1,2]];pos=rec['com_local']
        else:
            bounds=np.array(rec['bounds_local']);size=np.maximum(np.diff(bounds,axis=0)[0]/2,1e-5);pos=bounds.mean(0);quat=[1,0,0,0]
        node(b,'geom',name='fluid_'+name,type='ellipsoid',size=size,pos=pos,quat=quat,mass=0,group=3,contype=2,conaffinity=1,rgba=[.2,.7,.8,.25],fluidshape='ellipsoid',fluidcoef=fl['body_fluidcoef'] if rec['region']=='body' else fl['fin_fluidcoef'])
        if parent:node(contact,'exclude',body1=parent,body2=name)
    node(sensors,'framepos',name='root_position',objtype='body',objname='body_00_head')
    node(sensors,'framequat',name='root_orientation',objtype='body',objname='body_00_head')
    node(sensors,'framelinvel',name='root_velocity',objtype='body',objname='body_00_head')
    node(sensors,'frameangvel',name='root_angular_velocity',objtype='body',objname='body_00_head')
    node(sensors,'accelerometer',name='imu_accelerometer',site='imu');node(sensors,'gyro',name='imu_gyro',site='imu')
    keyframes=node(root,'keyframe');node(keyframes,'key',name='rest',qpos=[0,0,0,1,0,0,0]+[0]*len(joint_specs),ctrl=[0]*len(joint_specs))
    ET.indent(root,space='  ');ET.ElementTree(root).write(ROOT/'fish.xml',encoding='unicode')
    dump(ROOT/'assets/joints.json',joint_specs)
    dump(ROOT/'reports/inertia_adjustments.json',inertia_adjustments)
    import mujoco
    m=mujoco.MjModel.from_xml_path(str(ROOT/'fish.xml'));d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    assert np.isfinite(d.qacc).all()
    com=np.average([np.array(r['origin'])+r['com_local'] for r in records],axis=0,weights=[r['mass_kg'] for r in records])
    report('phase3.md',f'''# Phase 3 — generated MuJoCo model

MuJoCo {mujoco.__version__}; compile and `mj_forward` successful. {m.nbody-1} bodies, {m.njnt-1} hinge joints, {m.nu} position actuators, {m.nsensor} named sensors. Free root; every fin attaches to its anatomical body segment. Body yaw/pitch ranges use measured seam-safe values from the segmentation manifest.

Mass: {m.body_mass.sum()*1000:.4f} g; rest COM: {com.tolist()} m. Explicit principal inertia from closed mesh volume with uniform density and overlap correction. Tiny fin-tip principal inertias are floored at {c['simulation']['minimum_inertia_kg_m2']:.1e} kg·m² for numerical conditioning; {len(inertia_adjustments)} modules adjusted (see inertia_adjustments.json). Fluid/collision proxies have zero additional mass; visuals have no collisions. Ellipsoids fit body principal inertia, fin extents fit thin ellipsoids. Group 1 = textured visuals; group 3 = proxies; group 4 = hidden rigid head/chin comparison meshes when the continuous chin skin is enabled ({continuous_chin}). The skin is visual only and adds no mass, joints, contacts or fluid forces. All fish self-collision is disabled through masks, and adjacent exclusions are also explicit; external contact remains possible with compatible masks.

Gravity: {fl['gravity']}; explicit neutral buoyancy: {fl['explicit_buoyancy']}. MJCF `gravcomp = rho_water/rho_fish` supplies the Archimedean force at each uniform-density segment COM, equivalent to rho_water × corrected displaced volume × g. Thus standalone `fish.xml` is passive-neutral without a Python force callback. This assumes full submersion and coincident buoyancy/mass centroids; native fluid density alone does not provide this force.

Water density {fl['density']} kg/m³ and dynamic viscosity {fl['viscosity']} Pa·s; `implicitfast`, dt={c['simulation']['timestep']} s. The model has no CFD wake, circulation memory or fin-to-fin coupling. Reynolds number is reported from measured behavior speed after tuning.
''')
    print(f'MJCF compiled: nq={m.nq}, nv={m.nv}, nu={m.nu}, mass={m.body_mass.sum():.6f} kg')
    return m
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--config');a=ap.parse_args();run(config(a.config))
