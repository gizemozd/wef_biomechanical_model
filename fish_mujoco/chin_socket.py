"""Coverage checks for a rigid, full spherical chin ball inside a head socket."""
import itertools
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from common import ROOT,dump


def recontour_root(c,mesh,scale,center):
    """Trim only the short rigid chin root to a smooth socket-sized waist.

    The upper mouth stays outside the trimming volume. The resulting reference
    surface carries the modeled exterior, distinct from internal socket faces.
    No vertex deformation or skinning occurs during simulation.
    """
    from scipy.interpolate import PchipInterpolator
    from segment_mesh import boolean,box
    sg=c['segmentation'];profile=np.asarray(sg['chin_root_profile_source'],float)
    pivot=np.asarray(sg['chin_pivot_source'])
    xs=profile[:,0]+pivot[0]
    if not np.all(np.diff(xs)>0) or np.any(profile[:,1]<=0):
        raise ValueError('Chin root profile must have increasing offsets and positive radii')
    samples=np.linspace(xs[0],xs[-1],sg['chin_root_axial_samples'])
    radii=PchipInterpolator(xs,profile[:,1])(samples)
    curve=np.vstack([[0,samples[0]],np.column_stack([radii,samples]),[0,samples[-1]]])
    envelope=trimesh.creation.revolve(curve,sections=sg['chin_root_radial_sections'])
    v=envelope.vertices.copy()
    envelope.vertices=(np.column_stack([v[:,2],v[:,0]+pivot[1],v[:,1]+pivot[2]])-center)*scale
    envelope.fix_normals()
    cv=lambda p:(np.asarray(p)-center)*scale
    roi=box([cv([xs[0],-3,-2]),cv([xs[-1],3,sg['chin_upper_limit_source']])])
    result=boolean(mesh,boolean(roi,envelope,'sub'),'sub')
    if len(result.split())!=1:raise ValueError('Root contour disconnected the source body')
    dump(ROOT/'reports/chin_root_contour.json',{
        'pivot_source':pivot,'profile_source':profile,
        'removed_volume_m3':mesh.volume-result.volume,
        'axial_interval_source':[xs[0],xs[-1]],
        'note':'Permanent local rigid contour change below the mouth; no flexible cover.'})
    return result


def root_texture_projection(source_mm,points_mm,pivot_mm,fallback_uv):
    """Project outward from the chin axis, avoiding nearby inner-mouth UVs."""
    origins=np.asarray(points_mm).copy()
    origins[:,1:]=np.asarray(pivot_mm)[1:]
    directions=points_mm-origins
    directions/=np.maximum(np.linalg.norm(directions,axis=1,keepdims=True),1e-12)
    location,ray_id,face_id=source_mm.ray.intersects_location(origins,directions,multiple_hits=False)
    uv=fallback_uv.copy()
    if len(ray_id):
        bary=trimesh.triangles.points_to_barycentric(source_mm.triangles[face_id],location)
        uv[ray_id]=np.einsum('ij,ijk->ik',bary,source_mm.visual.uv[source_mm.faces[face_id]])
    return uv


def measure(c,head,chin,src,pivot,radius,overlap):
    from segment_mesh import spherical_interface_points
    points=spherical_interface_points(head,src,pivot,radius-overlap,c['segmentation']['seam_samples_per_edge'])
    if len(points)<8:raise ValueError('Insufficient socket rim samples')
    height=float(np.ptp(points[:,2]))
    # A circular sample set alone must not conceal leftover planar ROI cuts.
    near=np.linalg.norm(head.triangles_center-pivot,axis=1)<2*radius
    source_mm=src.copy();source_mm.apply_scale(1000)
    _,distance,_=trimesh.proximity.closest_point(source_mm,head.triangles_center[near]*1000)
    interior=head.triangles[near][distance>0.0002]
    if len(interior):
        cutter=trimesh.creation.icosphere(subdivisions=c['segmentation']['sphere_subdivisions'],radius=(radius-overlap)*1000)
        _,off_cutter,_=trimesh.proximity.closest_point(cutter,((interior-pivot)*1000).reshape(-1,3))
        if off_cutter.max()>0.0002:
            raise ValueError('Non-spherical cut faces remain at the chin socket')
    # A full ball covers the socket rim for all orientations. The front
    # ball/shaft junction is intentionally exposed and is not a mating seam.
    local=chin.copy();local.vertices=(local.vertices-pivot)*1000
    def pose(angles):
        rotation=Rotation.from_euler('ZYX',angles,degrees=True).as_matrix()
        query=(points-pivot)@rotation*1000
        distance=trimesh.proximity.signed_distance(local,query)/1000
        gap=float(max(0,-distance.min()));penetration=float(max(0,distance.max()))
        threshold=c['segmentation']['seam_bent_fraction'] if any(angles) else c['segmentation']['seam_rest_fraction']
        result={'gap_m':gap,'penetration_m':penetration,'gap_fraction':gap/height,
                'threshold':threshold,'height_m':height,'sample_count':len(points),
                'enforced':True,'within_threshold':gap/height<=threshold,
                'metric':'head socket rim covered by rigid chin ball'}
        if not result['within_threshold']:raise ValueError(f'Uncovered ball/socket rim at {angles}: {result}')
        return result
    names=['yaw','pitch','roll'];limits=np.array([c['joints']['chin_'+n+'_deg'] for n in names])
    rows=[]
    for i,name in enumerate(names):
        for factor in [0,-.5,.5,-1,1]:
            angles=np.zeros(3);angles[i]=factor*limits[i]
            rows.append({'joint':'j_chin_'+name,'axis':name,'angle_deg':float(angles[i]),**pose(angles)})
    combined=[]
    for factors in itertools.product([-1,-.5,0,.5,1],repeat=3):
        angles=limits*factors
        combined.append({'yaw_pitch_roll_deg':angles,**pose(angles)})
    dump(ROOT/'reports/chin_seam_metrics.json',rows)
    dump(ROOT/'reports/chin_compound_metrics.json',combined)
    np.savez(ROOT/'assets/meshes/chin_socket_samples.npz',rim=points,pivot=pivot)
    return {'attachment':'ball_socket','pivot':pivot,'radius':radius,'overlap_m':overlap,
            'height_m':height,'seam_policy':'enforce_socket_coverage',
            'socket_faces_verified_spherical':len(interior),
            **{n+'_deg':float(limits[i]) for i,n in enumerate(names)},
            **{'requested_'+n+'_deg':float(limits[i]) for i,n in enumerate(names)}}
