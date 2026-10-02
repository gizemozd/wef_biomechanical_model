"""Phase 0: inspect, normalize, repair, slice, and propose spherical cuts."""
import argparse, shutil
import numpy as np
import trimesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from PIL import Image
from scipy.interpolate import UnivariateSpline
from common import *

def run(c):
    for d in ['reports','assets/meshes','assets/textures','videos','tests']: (ROOT/d).mkdir(parents=True,exist_ok=True)
    shutil.copy2(ROOT/'references/anatomy_notes.md',ROOT/'reports/anatomy_notes.md')
    body, scene, scale, center=load_source(c)
    clean, repairs=solid(body)
    parts=sorted(clean.split(only_watertight=False), key=lambda m:-len(m.faces))
    main=parts[0]
    main.export(ROOT/'assets/meshes/repaired_source.ply')
    export_obj(ROOT/'assets/meshes/original.obj',body,body.visual.uv[body.faces],body.vertex_normals[body.faces])
    for p in source_path(c,'texture_dir').glob('*.png'): shutil.copy2(p,ROOT/'assets/textures'/p.name)
    (ROOT/'assets/meshes/fish.mtl').write_text('newmtl fish\nKd 1 1 1\nmap_Kd ../textures/Elephant_Nose_Fish_Diffuse.png\n')
    texture=Image.open(ROOT/'assets/textures/Elephant_Nose_Fish_Diffuse.png')
    texture.thumbnail((1024,1024));texture.save(ROOT/'reports/texture_atlas.png')
    gl_backend()
    import mujoco
    xml=f"""<mujoco><visual><global offwidth="1200" offheight="700"/><headlight ambient="0.45 0.45 0.45" diffuse="0.65 0.65 0.65"/></visual><asset><texture name="t" type="2d" file="{ROOT}/assets/textures/Elephant_Nose_Fish_Diffuse.png"/><texture name="water" type="skybox" builtin="gradient" rgb1="0.025 0.1 0.13" rgb2="0.12 0.26 0.28" width="64" height="384"/><material name="m" texture="t" specular="0.12"/><mesh name="fish" file="{ROOT}/assets/meshes/original.obj"/></asset><worldbody><light pos="0 1 1"/><geom type="mesh" mesh="fish" material="m" contype="0" conaffinity="0"/></worldbody></mujoco>"""
    model=mujoco.MjModel.from_xml_string(xml);data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    cam=mujoco.MjvCamera();cam.lookat[:]=0;cam.distance=.28;cam.azimuth=90;cam.elevation=-8
    with mujoco.Renderer(model,700,1200) as renderer:
        renderer.update_scene(data,cam);Image.fromarray(renderer.render()).save(ROOT/'reports/original_render.png')

    xs=np.linspace(main.bounds[0,0]+.002,main.bounds[1,0]-.012,110)
    rows=[]
    for x in xs:
        section=main.section([1,0,0],[x,0,0])
        if section is None:continue
        loops=section.discrete
        # Largest central closed loop; centroid from polygon area, not vertex density.
        candidates=[]
        from shapely.geometry import Polygon
        for q in loops:
            if len(q)<4:continue
            p=Polygon(q[:,1:3])
            if p.is_valid and p.area>1e-9:candidates.append(p)
        if not candidates:continue
        p=max(candidates,key=lambda p:p.area); yy,zz=p.centroid.coords[0]
        b=p.bounds; rows.append([x,yy,zz,b[2]-b[0],b[3]-b[1]])
    rows=np.array(rows)
    # Width outliers are pectoral fins. Median fin extent is excluded later after fin extraction.
    rows[:,1]=0
    for j in [2,3,4]: rows[:,j]=UnivariateSpline(rows[:,0],rows[:,j],s=2e-6)(rows[:,0])
    np.savez(ROOT/'reports/centerline.npz',samples=rows,scale=scale,center=center)
    n=c['segmentation']['N_body']
    px=(np.linspace(c['segmentation']['pivot_anterior_source'],c['segmentation']['pivot_posterior_source'],n-1)-center[0])*scale
    pivots=np.column_stack([px,np.zeros(n-1),np.interp(px,rows[:,0],rows[:,2])])
    radii=c['segmentation']['cut_radius_factor']*np.interp(px,rows[:,0],rows[:,4])
    dump(ROOT/'reports/layout.json',{'pivots':pivots,'radii':radii,'scale':scale,'center':center})
    fig,axs=plt.subplots(2,1,figsize=(13,8),layout='constrained')
    for ax,j,label in zip(axs,[2,1],['dorsal z (m)','left y (m)']):
        v=body.vertices;ax.triplot(v[:,0],v[:,j],body.faces,color='#687c89',lw=.15)
        ax.plot(rows[:,0],rows[:,j],color='#c32731',lw=1.8,label='smoothed section centroids')
        for i,(p,r) in enumerate(zip(pivots,radii)):
            ax.add_patch(Circle((p[0],p[j]),r,fill=False,lw=.75,alpha=.65,color='#c47b20'))
            ax.plot(p[0],p[j],'.',color='#c47b20'); ax.text(p[0],p[j],str(i+1),fontsize=7)
        cp=(np.array(c['segmentation']['chin_pivot_source'])-center)*scale
        ax.add_patch(Circle((cp[0],cp[j]),c['segmentation']['chin_radius_source']*scale,fill=False,color='#9b225a',lw=1.5))
        ax.text(cp[0],cp[j]-.008,'chin: 3 DOF',fontsize=8,color='#9b225a')
        ax.set_aspect('equal');ax.set_xlabel('anterior x (m)');ax.set_ylabel(label);ax.grid(alpha=.2)
    axs[0].set_title(f'G. petersii | proposed {n} body regions + 3-DOF chin; sphere centers are pivots')
    fig.savefig(ROOT/'reports/phase0_layout.png',dpi=170);plt.close(fig)
    fig,axs=plt.subplots(2,1,figsize=(9,5),sharex=True,layout='constrained')
    axs[0].plot(rows[:,0],rows[:,2]);axs[0].set_ylabel('centerline z (m)')
    axs[1].plot(rows[:,0],rows[:,3],label='width');axs[1].plot(rows[:,0],rows[:,4],label='height including median fins');axs[1].legend();axs[1].set_xlabel('x (m)')
    fig.savefig(ROOT/'reports/centerline.png',dpi=160);plt.close(fig)
    raw=trimesh.load(source_path(c,'mesh'),force='scene',process=False)
    stats={'format':'OBJ','units':'unspecified source units; calibrated to configured full extent','source_bounds':raw.bounds,'source_vertices':sum(len(m.vertices) for m in raw.geometry.values()),'source_triangles':sum(len(m.faces) for m in raw.geometry.values()),'body_triangles':len(body.faces),'body_components':len(parts),'body_watertight_before':False,'repair':repairs,'scale_m_per_source_unit':scale,'normalized_bounds':body.bounds,'texture_resolution':[4096,4096],'materials':list(raw.geometry),'normalized_volume_m3':clean.volume,'length_m':c['source']['length_m']}
    dump(ROOT/'reports/inspection.json',stats)
    report('phase0.md',f'''# Phase 0 — inspection and proposed layout

Source: `{source_path(c,'mesh').name}`; species established by the existing project README: **Gnathonemus petersii**.

- {stats['source_vertices']:,} OBJ vertices (UV splits included), {stats['source_triangles']:,} triangles, two materials; UVs present.
- Source bounds: {raw.bounds.tolist()}; source units are unspecified.
- Scale: {scale:.8f} m/source unit; full mesh extent **{c['source']['length_m']:.3f} m**, including projecting chin. This is a chosen specimen size, not maximum adult or standard length.
- Coordinates: source -Y → anterior +x, source +X → left +y, source +Z → dorsal +z; origin at bounding-box mid-body.
- Body + two eyes form three shells. Two 14-edge openings near the eyes are fan-capped; normals corrected. Open transparent sclera overlays are omitted; textured eyeballs remain.
- Six 4096² maps: diffuse, opacity, normal, roughness, specular, glossiness. MuJoCo uses the diffuse atlas; full PBR/opacity maps are archived but not reproduced by its classic shader.
- Proposed {n} body regions; head/girdle kept together, trunk mildly flexible, electric-organ peduncle stiff. Dorsal and anal fins are one rigid mesh each, with paired pectorals, paired pelvic fins, and a caudal fin. The chin is a separate segment with yaw, pitch and roll actuators.
- Centerline uses section polygon area centroids and a smoothing spline. Preliminary dimensions include median fins; final sphere dimensions will use the extracted body.
- Spherical interfaces do **not** mathematically ensure continuity of an arbitrary outer surface. Seam tests will limit actual joint ranges, including pitch.

![Original textured render](original_render.png)
![Texture atlas](texture_atlas.png)
![Cut layout](phase0_layout.png)
![Centerline](centerline.png)

See [anatomy notes](anatomy_notes.md) for sources and estimated values.
''')
    print('Phase 0:',stats)
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--config');a=ap.parse_args();run(config(a.config))
