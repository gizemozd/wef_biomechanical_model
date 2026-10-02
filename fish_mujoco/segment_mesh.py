"""Exact manifold booleans, UV transfer, mass properties and seam-limited ROM."""
import argparse
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from scipy.interpolate import UnivariateSpline
from shapely.geometry import Polygon
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from common import *

def boolean(a,b,op):
    f={'and':trimesh.boolean.intersection,'sub':trimesh.boolean.difference,'or':trimesh.boolean.union}[op]
    m=f([a,b],engine='manifold',check_volume=False)
    if len(m.faces):
        m.remove_unreferenced_vertices();m.fix_normals(multibody=True)
        if not m.is_watertight or m.volume<=0:raise ValueError('Boolean output not a positive closed solid')
    return m

def box(bounds):
    b=np.asarray(bounds);return trimesh.creation.box(extents=b[1]-b[0],transform=trimesh.transformations.translation_matrix(b.mean(0)))

def section_shape(m,x):
    path=m.section([1,0,0],[x,0,0])
    if path is None: raise ValueError(f'No body section at {x}')
    polys=[Polygon(q[:,1:3]) for q in path.discrete if len(q)>3]
    p=max((p for p in polys if p.is_valid),key=lambda p:p.area)
    bb=np.array(p.bounds)
    return np.array([x,*p.centroid.coords[0]]),bb[2:]-bb[:2]

def mask_polygon(points, scale, center):
    # Extrusion in x-z, symmetric about y=0.
    p=Polygon(points)
    m=trimesh.creation.extrude_polygon(p,height=40,engine='earcut')
    v=m.vertices.copy();m.vertices=np.column_stack([v[:,0],v[:,2]-20,v[:,1]])
    m.invert(); m.vertices=(m.vertices-center)*scale
    return m

def seam_edges(m,src,tol=2e-7):
    src_mm=src.copy();src_mm.apply_scale(1000)
    _,d,_=trimesh.proximity.closest_point(src_mm,m.triangles_center*1000);d/=1000
    outer=d<tol
    adj=m.face_adjacency
    e=m.face_adjacency_edges[outer[adj[:,0]]!=outer[adj[:,1]]]
    return e,outer

def sample_edges(m,edges,n):
    if not len(edges):return np.empty((0,3))
    a=m.vertices[edges[:,0]];b=m.vertices[edges[:,1]]
    return (a[:,None,:]+np.linspace(0,1,n)[None,:,None]*(b-a)[:,None,:]).reshape(-1,3)

def spherical_interface_points(mesh,src,pivot,radius,n):
    _,outer=seam_edges(mesh,src)
    adj=mesh.face_adjacency
    mix=outer[adj[:,0]]!=outer[adj[:,1]]
    adj=adj[mix];edges=mesh.face_adjacency_edges[mix]
    cap=np.where(outer[adj[:,0]],adj[:,1],adj[:,0])
    rr=np.linalg.norm(mesh.triangles[cap]-pivot,axis=2)
    return sample_edges(mesh,edges[np.max(np.abs(rr-radius),axis=1)<radius*.002+2e-7],n)

def signed_metrics(a,b,pa,pb,pivot,axis,angle,rotation=None):
    rot=Rotation.from_rotvec(np.asarray(axis)*np.deg2rad(angle)).as_matrix() if rotation is None else rotation
    bm=b.copy();bm.vertices=(b.vertices-pivot)@rot.T+pivot
    qb=(pb-pivot)@rot.T+pivot
    # signed_distance: positive inside, negative outside. Nearest triangle, not vertex.
    ds=[]
    for target,pts in [(bm,pa),(a,qb)]:
        if len(pts):
            target_mm=target.copy();target_mm.apply_scale(1000)
            ds.append(trimesh.proximity.signed_distance(target_mm,pts*1000)/1000)
    d=np.concatenate(ds)
    return float(max(0,-d.min())),float(max(0,d.max()))

def measure_chin_rom(c,head,chin,src,chin_p,chin_r,chin_overlap):
    """Measure the rigid attachment; only reduce active ROM when explicitly configured."""
    sg=c['segmentation']
    if sg.get('chin_attachment')=='ball_socket':
        from chin_socket import measure
        return measure(c,head,chin,src,chin_p,chin_r,chin_overlap)
    chin_pa=spherical_interface_points(head,src,chin_p,chin_r,sg['seam_samples_per_edge'])
    chin_pb=spherical_interface_points(chin,src,chin_p,chin_r-chin_overlap,sg['seam_samples_per_edge'])
    if min(len(chin_pa),len(chin_pb))<4:raise ValueError('Insufficient chin seam samples')
    chin_height=float(np.ptp(np.vstack([chin_pa,chin_pb])[:,2]))
    chin_joint={'pivot':chin_p,'radius':chin_r,'overlap_m':chin_overlap,'height_m':chin_height}
    reduce_chin=c['joints'].get('chin_auto_reduce_for_seams',True)
    chin_joint['seam_policy']='reduce_range' if reduce_chin else 'report_only'
    chin_seams=[]
    for label,axis in [('yaw',[0,0,1]),('pitch',[0,1,0]),('roll',[1,0,0])]:
        requested=c['joints']['chin_'+label+'_deg'];actual=requested
        for attempt in range(20):
            tests=[]
            for fraction in [0,-.5,.5,-1,1]:
                angle=fraction*actual;gap,penetration=signed_metrics(head,chin,chin_pa,chin_pb,chin_p,axis,angle)
                threshold=sg['seam_rest_fraction'] if fraction==0 else sg['seam_bent_fraction']/3
                tests.append({'joint':'j_chin_'+label,'axis':label,'angle_deg':angle,'gap_m':gap,'penetration_m':penetration,'gap_fraction':gap/chin_height,'height_m':chin_height,'sample_count':len(chin_pa)+len(chin_pb),'threshold':threshold})
            if not reduce_chin or all(t['gap_fraction']<=t['threshold'] for t in tests):break
            actual*=.65
        else:raise ValueError('Cannot meet chin seam tolerance')
        chin_joint[label+'_deg']=actual;chin_joint['requested_'+label+'_deg']=requested;chin_seams.extend(tests)
        print(f'Chin {label}: {actual:.3f} degrees',flush=True)
    for row in chin_seams:
        row['within_threshold']=row['gap_fraction']<=row['threshold']
        row['enforced']=bool(reduce_chin or row['angle_deg']==0)
        if row['angle_deg']==0 and not row['within_threshold']:
            raise ValueError('Chin rest seam threshold exceeded')
    dump(ROOT/'reports/chin_seam_metrics.json',chin_seams)
    import itertools
    compound=[]
    for signs in itertools.product([-1,1],repeat=3):
        angles=np.array([chin_joint[k+'_deg'] for k in ['yaw','pitch','roll']])*signs
        rot=Rotation.from_euler('ZYX',angles,degrees=True).as_matrix()
        gap,penetration=signed_metrics(head,chin,chin_pa,chin_pb,chin_p,[1,0,0],0,rotation=rot)
        compound.append({'yaw_pitch_roll_deg':angles,'gap_m':gap,'penetration_m':penetration,'gap_fraction':gap/chin_height,'threshold':sg['seam_bent_fraction']})
    for row in compound:
        row['within_threshold']=row['gap_fraction']<=row['threshold']
        row['enforced']=reduce_chin
    if reduce_chin and any(not row['within_threshold'] for row in compound):raise ValueError('Combined chin ROM exceeds seam tolerance')
    dump(ROOT/'reports/chin_compound_metrics.json',compound)
    return chin_joint

def update_chin_rom(c):
    """Refresh only chin limits/diagnostics using existing meshes; no resegmentation."""
    path=ROOT/'assets/segments.json';meta=json.loads(path.read_text())
    old=meta['chin_joint'];records={r['name']:r for r in meta['segments']}
    meshes=[]
    for name in ['body_00_head','chin_0']:
        rec=records[name]
        visual=trimesh.load(ROOT/'assets/meshes'/rec['mesh'],force='mesh',process=False)
        # Weld positions independently of UV islands to restore solid adjacency.
        mesh=trimesh.Trimesh(visual.vertices,visual.faces,process=True)
        mesh.vertices+=np.array(rec['origin']);meshes.append(mesh)
    src,_,_,_=load_source(c)
    if c['segmentation'].get('chin_root_profile_source'):
        src=trimesh.load(ROOT/'assets/meshes/chin_root_reference.ply',force='mesh')
    meta['chin_joint']=measure_chin_rom(c,*meshes,src,np.array(old['pivot']),old['radius'],old['overlap_m'])
    dump(path,meta)
    phase1=ROOT/'reports/phase1.md'
    if phase1.exists():
        import re
        j=meta['chin_joint']
        summary=f"Chin limits (seam policy: {j['seam_policy']}): yaw ±{j['yaw_deg']:.3f}°, pitch ±{j['pitch_deg']:.3f}°, roll ±{j['roll_deg']:.3f}°. Engineering settings, not measured anatomical ROM. Ball/socket mode enforces socket-rim coverage across the full range; legacy report-only mode may exceed bent tolerances. See `chin_seam_metrics.json`, `chin_compound_metrics.json`, and `chin_mobility.md`."
        phase1.write_text(re.sub(r'Chin limits[^\n]*',lambda _:summary,phase1.read_text(),count=1))
    return meta['chin_joint']

def run(c):
    src,scene,scale,center=load_source(c); clean,repairs=solid(src)
    reference_volume=clean.volume
    shells=sorted(clean.split(only_watertight=False),key=lambda m:-len(m.faces))
    remaining=shells[0]; eye_meshes=shells[1:]
    sg=c['segmentation']; fins=[]
    modeled_reference=None
    if sg.get('chin_root_profile_source'):
        if sg.get('chin_attachment')!='ball_socket':
            raise ValueError('Rigid root contour requires the ball/socket attachment')
        from chin_socket import recontour_root
        remaining=recontour_root(c,remaining,scale,center)
        modeled_reference=trimesh.util.concatenate([remaining,*eye_meshes])
        reference_volume=modeled_reference.volume
    previous_manifest=ROOT/'assets/segments.json'
    old_records=json.loads(previous_manifest.read_text())['segments'] if previous_manifest.exists() else []
    # Source-unit landmarks are explicitly configurable and documented.
    def cv(q):return (np.asarray(q)-center)*scale
    # Separate the sensory chin from the lower head using a concentric socket.
    chin_p=cv(sg['chin_pivot_source']);chin_r=sg['chin_radius_source']*scale
    chin_overlap=min(sg['overlap_m'],.008*chin_r)
    chin_roi=box([cv([sg['chin_pivot_source'][0],-3,-2]),cv([15,3,sg['chin_upper_limit_source']])])
    cap=trimesh.creation.icosphere(subdivisions=sg['sphere_subdivisions'],radius=chin_r);cap.apply_translation(chin_p)
    socket=trimesh.creation.icosphere(subdivisions=sg['sphere_subdivisions'],radius=chin_r-chin_overlap);socket.apply_translation(chin_p)
    if sg.get('chin_attachment')=='ball_socket':
        # Reverse the old arrangement: the moving chin owns the entire ball.
        # Its slight radial overlap fills the concave head socket at every angle.
        reference_volume+=boolean(cap,remaining,'sub').volume
        chin=boolean(boolean(remaining,chin_roi,'and'),cap,'or')
        remaining=boolean(remaining,boolean(chin_roi,socket,'or'),'sub')
        if modeled_reference is not None:
            # Coplanar mouth clipping can leave zero-volume slivers. Keep the
            # single positive-volume chin and reject any substantive fragments.
            import warnings
            with warnings.catch_warnings():
                warnings.simplefilter('ignore',RuntimeWarning)
                parts=chin.split()
                parts=[part for part in parts if part.volume>1e-12]
            if len(parts)!=1:raise ValueError('Root contour created multiple chin solids')
            chin=parts[0]
    else:
        chin=boolean(remaining,boolean(chin_roi,socket,'sub'),'and')
        remaining=boolean(remaining,boolean(chin_roi,cap,'sub'),'sub')
    if not len(chin.faces):raise ValueError('Chin landmarks did not isolate an appendage')
    if sg.get('chin_attachment')!='ball_socket' and (chin.bounds[0,0]<=chin_p[0]+1e-6 or chin.bounds[1,2]>=cv([0,0,sg['chin_upper_limit_source']])[2]-1e-6):
        raise ValueError('Chin ROI clipped exterior skin; increase root sphere or refine mouth landmarks')
    fins.append(('chin_0',chin,chin_p,'chin'))
    for side,sign in [('L',1),('R',-1)]:
        lo=[-20,sg['pec_base_y'] if sign>0 else -15,-5]
        hi=[20,15 if sign>0 else -sg['pec_base_y'],15]
        cut=box([cv(lo),cv(hi)])
        part=boolean(remaining,cut,'and');remaining=boolean(remaining,cut,'sub')
        fins.append((f'fin_pec{side}_0',part,cv([3.8,sign*sg['pec_base_y'],3.2]),'pectoral'))
    # Small paired pelvic fins: keep their original surfaces, cap the base.
    for side,sign in [('L',1),('R',-1)]:
        lo=[.05,.35 if sign>0 else -2,-2];hi=[2,2 if sign>0 else -.35,1.55]
        cut=box([cv(lo),cv(hi)])
        part=boolean(remaining,cut,'and');remaining=boolean(remaining,cut,'sub')
        if len(part.faces):fins.append((f'fin_pel{side}_0',part,cv([1,sign*.65,1.55]),'pelvic'))
    for region in ['dorsal','anal']:
        base=np.array(sg[region+'_base']);zout=15 if region=='dorsal' else -5
        points=base.tolist()+[[base[-1,0],zout],[base[0,0],zout]]
        cutter=mask_polygon(points,scale,center)
        fin=boolean(remaining,cutter,'and');remaining=boolean(remaining,cutter,'sub')
        # One complete rigid mesh per median fin, per the revised task.
        dist=np.r_[0,np.cumsum(np.linalg.norm(np.diff(base,axis=0),axis=1))]
        xm=np.interp(dist[-1]/2,dist,base[:,0]);z=np.interp(xm,base[:,0],base[:,1])
        fins.append((f'fin_{region}_0',fin,cv([xm,0,z]),region))
    tail_x=sg['caudal_base_x'];cut=box([cv([-20,-15,-5]),cv([tail_x,15,15])])
    tail=boolean(remaining,cut,'and');remaining=boolean(remaining,cut,'sub')
    tail_p,_=section_shape(remaining,cv([tail_x+.015,0,0])[0])
    fins.append(('fin_caudal_0',tail,tail_p,'caudal'))
    remaining.export(ROOT/'assets/meshes/body_without_fins.ply')
    # Centerline from the extracted body, avoiding chin/eye and caudal-fin cross-sections.
    xs=np.linspace(remaining.bounds[0,0]+1e-5,cv([7,0,0])[0],120)
    centers=[];dims=[]
    for x in xs:
        p,d=section_shape(remaining,x);centers.append(p);dims.append(d)
    centers=np.array(centers);dims=np.array(dims)
    zfun=UnivariateSpline(xs,centers[:,2],s=1e-7)
    centers[:,2]=zfun(xs);centers[:,1]=0
    tangents=np.column_stack([np.ones(len(xs)),np.zeros(len(xs)),zfun.derivative()(xs)])
    tangents/=np.linalg.norm(tangents,axis=1)[:,None]
    np.savez(ROOT/'reports/body_centerline.npz',points=centers,tangents=tangents,width_height=dims)
    px=(np.linspace(sg['pivot_anterior_source'],sg['pivot_posterior_source'],sg['N_body']-1)-center[0])*scale
    pivots=np.column_stack([px,np.zeros(len(px)),zfun(px)])
    radii=sg['cut_radius_factor']*np.maximum(np.interp(px,xs,dims[:,0]),np.interp(px,xs,dims[:,1]))
    bodies=[];rest=remaining;effective=[]
    overlaps=np.minimum(sg["overlap_m"],.004*np.interp(px,xs,dims[:,1]))
    for i,(p,r) in enumerate(zip(pivots,radii)):
        if sg['cut_type']=='sphere':
            sphere=trimesh.creation.icosphere(subdivisions=sg['sphere_subdivisions'],radius=r);sphere.apply_translation(p)
            small=trimesh.creation.icosphere(subdivisions=sg['sphere_subdivisions'],radius=r-overlaps[i]);small.apply_translation(p)
        elif sg['cut_type']=='cylinder':
            sphere=trimesh.creation.cylinder(radius=r,height=.5,sections=128);sphere.apply_translation(p)
            small=trimesh.creation.cylinder(radius=r-overlaps[i],height=.5,sections=128);small.apply_translation(p)
        else:raise ValueError('cut_type must be sphere or cylinder')
        # Keep anterior halfspace plus sphere: a convex posterior cap, concave successor socket.
        half=box([[p[0],-.3,-.3],[.4,.3,.3]])
        cutter=boolean(half,sphere,'or');inner=boolean(half,small,'or')
        segment=boolean(rest,cutter,'and');newrest=boolean(rest,inner,'sub')
        if segment.volume<1e-10 or newrest.volume<1e-11:
            raise ValueError(f'Sphere layout swallowed body region {i}; adjust pivots/radius factor')
        bodies.append(segment);rest=newrest
    bodies.append(rest)
    # Eyes move rigidly with the head. They remain separate watertight visual shells.
    bodies[0]=trimesh.util.concatenate([bodies[0],*eye_meshes])
    records=[];meshes={}; body_origins=[np.zeros(3),*pivots]
    for i,m in enumerate(bodies):
        name='body_00_head' if i==0 else f'body_{i:02d}'
        parent=None if i==0 else ('body_00_head' if i==1 else f'body_{i-1:02d}')
        records.append({'name':name,'parent':parent,'origin':body_origins[i],'region':'body','body_index':i})
        meshes[name]=m
    for name,m,p,region in fins:
        if region in ('pectoral','chin'):parent='body_00_head'
        else:
            # Attach to the body whose actual exterior contains the fin base.
            dd=[trimesh.proximity.closest_point(b,np.array([p]))[1][0] for b in bodies]
            k=int(np.argmin(dd));parent=records[k]['name']
        records.append({'name':name,'parent':parent,'origin':p,'region':region})
        meshes[name]=m
    # Measure seam-boundary displacement against neighboring triangle surfaces.
    seam=[];edge_cache={}
    for i,m in enumerate(bodies):
        edge_cache[i]=seam_edges(m,src)
    for i,p in enumerate(pivots):
        a,b=bodies[i:i+2];r=radii[i];h=float(np.interp(p[0],xs,dims[:,1]))
        def interface_points(mesh, outer, target):
            adj=mesh.face_adjacency
            mix=outer[adj[:,0]]!=outer[adj[:,1]]
            adj=adj[mix];edges=mesh.face_adjacency_edges[mix]
            cap=np.where(outer[adj[:,0]],adj[:,1],adj[:,0])
            vectors=mesh.triangles[cap]-p
            rr=np.linalg.norm(vectors,axis=2) if sg['cut_type']=='sphere' else np.linalg.norm(vectors[:,:,:2],axis=2)
            iscap=np.max(np.abs(rr-target),axis=1)<r*.002+2e-7
            return sample_edges(mesh,edges[iscap],sg['seam_samples_per_edge'])
        aa=interface_points(a,edge_cache[i][1],r)
        bb=interface_points(b,edge_cache[i+1][1],r-overlaps[i])
        def select(points, target_radius):
            rr=np.linalg.norm(points-p,axis=1) if sg['cut_type']=='sphere' else np.linalg.norm((points-p)[:,:2],axis=1)
            return points[(np.abs(rr-target_radius)<r*.002+2e-7)&(points[:,0]<p[0])]
        pa,pb=select(aa,r),select(bb,r-overlaps[i])
        if min(len(pa),len(pb))<4:raise ValueError(f'Insufficient outer seam samples at joint {i+1}')
        source_x=p[0]/scale+center[0]
        region='trunk' if source_x>0 else ('posterior' if source_x>-6.5 else 'peduncle')
        req=c['joints'][region+'_yaw_deg']; reqpitch=0 if sg['cut_type']=='cylinder' else c['joints']['pitch_deg']
        values={}
        for label,axis,limit in [('yaw',[0,0,1],req),('pitch',[0,1,0],reqpitch)]:
            # Conservative displacement bound leaves headroom for combined yaw/pitch.
            actual=min(limit,np.rad2deg(.010*h/r)) if c['joints']['auto_reduce_for_seams'] else limit
            for attempt in range(10):
                tests=[]
                for fraction in [0,-.5,.5,-1,1]:
                    angle=fraction*actual;gap,penetration=signed_metrics(a,b,pa,pb,p,axis,angle)
                    threshold=sg['seam_rest_fraction'] if fraction==0 else sg['seam_bent_fraction']
                    tests.append({'axis':label,'angle_deg':angle,'gap_m':gap,'penetration_m':penetration,'gap_fraction':gap/h,'threshold':threshold})
                if all(t['gap_fraction']<=t['threshold'] for t in tests):break
                if not c['joints']['auto_reduce_for_seams']:raise ValueError('Seam threshold exceeded')
                actual*=.65
            else:
                a.export(ROOT/'reports/debug_a.ply');b.export(ROOT/'reports/debug_b.ply');np.savez(ROOT/'reports/debug_seam.npz',pa=pa,pb=pb,p=p)
                raise ValueError(f'Cannot meet seam threshold at joint {i+1}: {tests}')
            values[label+'_deg']=float(actual);seam.extend([{'joint':f'j_body_{i+1:02d}_{label}','height_m':h,'sample_count':len(pa)+len(pb),**t} for t in tests])
        effective.append({'body_index':i+1,'pivot':p,'radius':r,'overlap_m':overlaps[i],'height_m':h,'region':region,'requested_yaw_deg':req,**values})
        print(f'Joint {i+1}: yaw {values["yaw_deg"]:.3f} pitch {values["pitch_deg"]:.3f}, seam samples {len(pa)+len(pb)}',flush=True)
    chin_joint=measure_chin_rom(c,bodies[0],chin,src if modeled_reference is None else modeled_reference,chin_p,chin_r,chin_overlap)
    # Transfer each face from its source triangle; disjoint UV islands cannot bleed across seams.
    atlas=Image.open(ROOT/'assets/textures/Elephant_Nose_Fish_Diffuse.png').convert('RGB')
    w,htex=atlas.size;pad=128
    combined=Image.new('RGB',(w,htex+pad));combined.paste(atlas,(0,0));pix=np.array(atlas)
    source_tri=src.triangles*1000;source_uv=src.visual.uv[src.faces]
    src_mm=src.copy();src_mm.apply_scale(1000)
    if modeled_reference is not None:
        modeled_reference.export(ROOT/'assets/meshes/chin_root_reference.ply')
        reference_mm=modeled_reference.copy();reference_mm.apply_scale(1000)
    all_volume=sum(m.volume for m in meshes.values());mass_scale=reference_volume/all_volume
    for k,rec in enumerate(records):
        m=meshes[rec['name']]; centers_face=m.triangles_center
        nearest,dist,faceid=trimesh.proximity.closest_point(src_mm,centers_face*1000);dist/=1000
        uv=np.empty((len(m.faces),3,2));normals=np.empty((len(m.faces),3,3))
        outer=dist<2e-7
        altered_outer=np.zeros(len(m.faces),dtype=bool)
        root_normals=np.zeros(len(m.faces),dtype=bool)
        if modeled_reference is not None and rec['name'] in ('body_00_head','chin_0'):
            _,reference_dist,reference_faces=trimesh.proximity.closest_point(reference_mm,centers_face*1000)
            altered_outer=(reference_dist<.0002)&~outer
            outer|=altered_outer
            profile=np.asarray(sg['chin_root_profile_source'])
            interval=chin_p[0]+profile[[0,-1],0]*scale
            root_normals=outer&(centers_face[:,0]>=interval[0])&(centers_face[:,0]<=interval[1])
        rounded_chin=rec['region']=='chin' and sg.get('chin_attachment')=='ball_socket'
        for j in range(3):
            bary=trimesh.triangles.points_to_barycentric(source_tri[faceid],m.triangles[:,j,:]*1000)
            uv[:,j]=np.einsum('ij,ijk->ik',bary,source_uv[faceid])
            normals[:,j]=np.einsum('ij,ijk->ik',bary,src.vertex_normals[src.faces[faceid]])
            if altered_outer.any():
                projected=np.maximum(bary[altered_outer],0)
                projected/=projected.sum(axis=1,keepdims=True)
                uv[altered_outer,j]=np.einsum('ij,ijk->ik',projected,source_uv[faceid[altered_outer]])
            if root_normals.any():
                rf=reference_faces[root_normals]
                rb=trimesh.triangles.points_to_barycentric(reference_mm.triangles[rf],m.triangles[root_normals,j,:]*1000)
                normals[root_normals,j]=np.einsum('ij,ijk->ik',rb,reference_mm.vertex_normals[reference_mm.faces[rf]])
            if rounded_chin:
                projected=np.maximum(bary,0);projected/=projected.sum(axis=1,keepdims=True)
                uv[~outer,j]=np.einsum('ij,ijk->ik',projected[~outer],source_uv[faceid[~outer]])
            if modeled_reference is not None:
                project=altered_outer|(~outer if rounded_chin else False)
                if project.any():
                    from chin_socket import root_texture_projection
                    uv[project,j]=root_texture_projection(src_mm,m.triangles[project,j,:]*1000,chin_p*1000,uv[project,j])
        nearby_uv=source_uv[faceid[~outer]].mean(axis=1) if (~outer).any() else source_uv[faceid].mean(axis=1)
        tx=np.clip((nearby_uv[:,0]*w).astype(int),0,w-1);ty=np.clip(((1-nearby_uv[:,1])*htex).astype(int),0,htex-1)
        color=tuple(np.mean(pix[ty,tx],axis=0).astype(int))
        sx=(k%64)*64;sy=(k//64)*64
        combined.paste(color,(sx,htex+sy,sx+64,htex+sy+64))
        uv[:,:,1]=(uv[:,:,1]*htex+pad)/(htex+pad)
        if not rounded_chin:uv[~outer]=[(sx+32)/w,(pad-sy-32)/(htex+pad)]
        normals[~outer]=m.face_normals[~outer,None,:]
        if rounded_chin:normals[~outer]=m.vertex_normals[m.faces[~outer]]
        normals/=np.maximum(np.linalg.norm(normals,axis=2,keepdims=True),1e-12)
        origin=np.asarray(rec['origin']);local=m.copy();local.vertices-=origin
        export_obj(ROOT/'assets/meshes'/f'{rec["name"]}.obj',local,uv,normals)
        local.export(ROOT/'assets/meshes'/f'{rec["name"]}.ply')
        density=c['fluid']['fish_density']*mass_scale
        m.density=density
        mp=m.mass_properties
        rec.update({'mesh':f'{rec["name"]}.obj','volume_m3':m.volume,'displaced_volume_m3':m.volume*mass_scale,'mass_kg':mp.mass,'com_local':mp.center_mass-origin,'inertia':mp.inertia,'bounds_local':local.bounds,'watertight':m.is_watertight,'winding_consistent':m.is_winding_consistent,'faces':len(m.faces),'outer_faces':int(outer.sum()),'interior_color':color,'outer_mask_file':rec['name']+'_outer.npy'})
        np.save(ROOT/'assets/meshes'/rec['outer_mask_file'],outer)
    combined.save(ROOT/'assets/textures/fish_atlas.png')
    (ROOT/'assets/meshes/fish.mtl').write_text('newmtl fish\nKd 1 1 1\nmap_Kd ../textures/fish_atlas.png\n')
    dump(ROOT/'assets/segments.json',{'segments':records,'joints':effective,'chin_joint':chin_joint,'source_volume_m3':clean.volume,'assembly_volume_m3':reference_volume,'overlap_mass_scale':mass_scale,'scale':scale,'center':center,'body_centers':centers})
    active_names={r['name'] for r in records}
    for old in old_records:
        if old['name'] not in active_names:
            for suffix in ['.obj','.ply','_outer.npy']:
                (ROOT/'assets/meshes'/(old['name']+suffix)).unlink(missing_ok=True)
    dump(ROOT/'reports/seam_metrics.json',seam)
    fig,ax=plt.subplots(figsize=(14,5),layout='constrained')
    colors=plt.cm.turbo(np.linspace(.05,.95,len(bodies)))
    from matplotlib.collections import PolyCollection
    for i,m in enumerate(bodies):ax.add_collection(PolyCollection(m.triangles[:,:,[0,2]],facecolors=colors[i],edgecolors='none',alpha=.85))
    for _,m,_,region in fins:ax.add_collection(PolyCollection(m.triangles[:,:,[0,2]],facecolors='#729fa5',edgecolors='none',alpha=.6))
    ax.plot(centers[:,0],centers[:,2],color='black',lw=1)
    for p,r in zip(pivots,radii):ax.add_patch(Circle((p[0],p[2]),r,fill=False,lw=.5,color='#b57918'))
    ax.autoscale();ax.set_aspect('equal');ax.set_xlabel('x (m)');ax.set_ylabel('z (m)');ax.set_title('Final exact-boolean segmentation and body-only cut spheres')
    fig.savefig(ROOT/'reports/segmentation.png',dpi=160);plt.close(fig)
    maxgap=max(t['gap_fraction'] for t in seam)
    total=sum(s['mass_kg'] for s in records)
    report('phase1.md',f'''# Phase 1 — segmentation and seams

{len(bodies)} body segments, {len(fins)-1} whole fin meshes and one chin segment, {len(records)} moving rigid bodies. All output solids are watertight with consistent normals. Exact manifold booleans used on a faceted sphere ({sg['sphere_subdivisions']} icosphere subdivisions). Maximum radial overlap {sg['overlap_m']*1000:.3f} mm, locally capped at 0.4% of body height; dorsal and anal fins are single meshes. Two original eye holes capped; transparent sclera overlays omitted.

Original textured exterior triangles are retained where the anatomy is unchanged. Ball/socket mode models the chin root with a rounded ball. When configured, a smooth rigid waist is trimmed around the marked joint position, reducing the ball's visible prominence; `chin_root_contour.json` records the local change. UVs on this new exterior are projected radially outward to the source, and shared reference normals keep the root shading consistent. No visual skin is used. Fin bases use configurable, manually inferred external landmarks; these are not recovered bones. Upper (dorsal) and lower (anal) fins are each one complete rigid mesh, neutral by default. The chin is an independent head-attached body with three actuated rotations.

Maximum sampled seam gap / local body height: **{maxgap:.4%}**. Tests cover rest, ±half and ±full yaw/pitch separately. Reported penetration includes intentional overlap. Samples run along outer/cap boundary edges; this is a finite numerical test, not a proof for all points, compound poses, or fin membranes. Ranges were reduced to preserve body seams; see `assets/segments.json` and `seam_metrics.json`.

Chin limits (seam policy: {chin_joint["seam_policy"]}): yaw ±{chin_joint['yaw_deg']:.3f}°, pitch ±{chin_joint['pitch_deg']:.3f}°, roll ±{chin_joint['roll_deg']:.3f}°. These engineering limits are not measured anatomical ROM. Ball/socket mode enforces coverage of the head socket rim across all sampled rotations. Legacy report-only mode permits bent gaps. See `chin_seam_metrics.json` and `chin_compound_metrics.json`. All three rotational DOFs have independent actuators and sensors.

Uniform-density mass after overlap correction: **{total*1000:.3f} g**. Corrected displaced volume: {reference_volume:.9g} m³, including any added ball-root volume. Shared overlap volume is counted only once globally via mass scale {mass_scale:.7f}; local overlap distribution is approximate.

![Final segmentation](segmentation.png)
''')
    report('phase2.md',f'''# Phase 2 — textures

Source diffuse atlas: 4096×4096; derived atlas: 4096×4224 with a separate 128-pixel strip for flat interior-face colors. Original image pixels are unchanged. UV v coordinates receive an affine padding adjustment; all exterior face corners are transferred by source-triangle barycentric interpolation. Per-face-corner `vt` indices preserve UV islands independently of shared geometric vertices. Each segment's interior swatch is the average of nearby sampled diffuse colors. Newly modeled exterior ball faces instead receive nearest-triangle barycentric UV projection; their texture stays rigid during movement.

OBJ and MTL assets are self-contained under `assets/`. MuJoCo uses one diffuse material on all segment visuals. Original normal, specular, roughness, glossiness and opacity maps are archived; classic MuJoCo does not reproduce the full PBR material or alpha-cutout fins. Textured rest-pose comparison is generated during rendering as `texture_continuity.png`.
''')
    print(f'Segmentation complete: {len(records)} solids, mass {total:.6f} kg, max gap {maxgap:.4%}')
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--config');ap.add_argument('--chin-rom-only',action='store_true',help='Update chin ranges and seam diagnostics from existing meshes only');a=ap.parse_args()
    (update_chin_rom if a.chin_rom_only else run)(config(a.config))
