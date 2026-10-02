"""Offscreen MuJoCo videos, H.264, annotated measured trajectories."""
import argparse, json, math, subprocess, xml.etree.ElementTree as ET
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from common import *
gl_backend()
import mujoco

PALETTE=[(224,144,91),(100,184,177),(125,167,215),(207,170,223),(222,205,119)]

def font(size):
    for p in ['/System/Library/Fonts/Helvetica.ttc','/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf']:
        if Path(p).exists():return ImageFont.truetype(p,size)
    return ImageFont.load_default(size=size)

def annotate(im,title,subtitle='',labels=None,scale_px=None):
    out=Image.fromarray(im) if isinstance(im,np.ndarray) else im
    d=ImageDraw.Draw(out);w,h=out.size
    d.rectangle((0,0,w,92),fill=(9,27,35));d.text((28,16),title,font=font(29),fill=(235,243,240));d.text((30,54),subtitle,font=font(19),fill=(155,191,192))
    if labels:
        for text,x,y,color in labels:d.text((x,y),text,font=font(17),fill=color)
    if scale_px:
        x,y=w-180,h-42;d.line((x,y,x+scale_px,y),fill='white',width=3)
        d.text((x,y-27),'20 mm',font=font(17),fill='white')
    return np.array(out)

class Movie:
    def __init__(self,path,c,width=None,height=None):
        import imageio_ffmpeg
        self.path=Path(path);rc=c['render'];w=width or rc['width'];h=height or rc['height']
        self.writer=imageio_ffmpeg.write_frames(str(path),(w,h),fps=rc['fps'],codec='libx264',pix_fmt_out='yuv420p',quality=None,macro_block_size=1,ffmpeg_log_level='error',output_params=['-crf',str(rc['crf']),'-preset','fast','-movflags','+faststart'])
        self.writer.send(None)
    def add(self,frame):self.writer.send(np.ascontiguousarray(frame))
    def close(self):self.writer.close()

class Render:
    def __init__(self,m,w,h,ground=None):
        self.m=m;self.w=w;self.h=h;self.r=mujoco.Renderer(m,h,w);self.options=mujoco.MjvOption();self.options.geomgroup[3:5]=0
        self.ground=ground if ground and ground.get('enabled',True) else None
        self.options.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT]=False
    def frame(self,d,look=None,distance=.33,azimuth=90,elevation=-8):
        cam=mujoco.MjvCamera();cam.lookat[:]=np.zeros(3) if look is None else look;cam.distance=distance;cam.azimuth=azimuth;cam.elevation=elevation
        self.r.update_scene(d,cam,scene_option=self.options)
        if self.ground:self.add_ground()
        return self.r.render().copy()
    def add_ground(self):
        """World-fixed decorative floor/grid; never part of the physics model."""
        g=self.ground;scene=self.r.scene;extent=g['half_extent_m'];z=g['z_m'];spacing=g['spacing_m']
        def geom(kind,size,pos,color):
            if scene.ngeom>=scene.maxgeom:raise RuntimeError('Ground grid exceeds render geometry capacity')
            result=scene.geoms[scene.ngeom];scene.ngeom+=1
            mujoco.mjv_initGeom(result,kind,np.asarray(size,dtype=float),np.asarray(pos,dtype=float),np.eye(3).ravel(),np.asarray(color,dtype=np.float32))
            result.category=mujoco.mjtCatBit.mjCAT_DECOR
            return result
        geom(mujoco.mjtGeom.mjGEOM_PLANE,[extent,extent,.01],[0,0,z],g['floor_rgba'])
        count=int(np.floor(extent/spacing))
        for k in range(-count,count+1):
            style='origin' if k==0 else ('major' if k%g['major_every']==0 else 'minor')
            for axis in [0,1]:
                a=np.array([-extent,k*spacing,z+.00015]);b=np.array([extent,k*spacing,z+.00015])
                if axis==1:a=a[[1,0,2]];b=b[[1,0,2]]
                line=geom(mujoco.mjtGeom.mjGEOM_LINE,[0,0,0],[0,0,0],g[style+'_rgba'])
                mujoco.mjv_connector(line,mujoco.mjtGeom.mjGEOM_LINE,g[style+'_line_px'],a,b)
    def close(self):self.r.close()

def original_model(c):
    tree=ET.parse(ROOT/'fish.xml');r=tree.getroot()
    for tag in ['actuator','sensor','contact','keyframe']:r.remove(r.find(tag))
    if r.find('deformable') is not None:r.remove(r.find('deformable'))
    world=r.find('worldbody')
    for b in list(world.findall('body')):world.remove(b)
    a=r.find('asset')
    for me in list(a.findall('mesh')):a.remove(me)
    ET.SubElement(a,'mesh',name='original',file='original.obj',smoothnormal='true')
    ET.SubElement(a,'texture',name='original_texture',type='2d',file='Elephant_Nose_Fish_Diffuse.png')
    ET.SubElement(a,'material',name='original_material',texture='original_texture',specular='0.12',shininess='0.2',reflectance='0.03')
    ET.SubElement(world,'geom',name='original',type='mesh',mesh='original',material='original_material',group='1',contype='0',conaffinity='0')
    r.find('compiler').set('meshdir',str(ROOT/'assets/meshes'));r.find('compiler').set('texturedir',str(ROOT/'assets/textures'))
    return mujoco.MjModel.from_xml_string(ET.tostring(r,encoding='unicode'))

def stills(c,m):
    rc=c['render'];d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    original=original_model(c);od=mujoco.MjData(original);mujoco.mj_forward(original,od)
    a=Render(original,960,650);b=Render(m,960,650)
    left=a.frame(od,distance=.27);right=b.frame(d,distance=.27)
    comparison=annotate(np.concatenate([left,right],axis=1),'Original unified mesh                                             Segmented rest pose','Identical camera, diffuse texture, lighting and scale | x anterior, z dorsal')
    Image.fromarray(comparison).save(ROOT/'reports/texture_continuity.png')
    # Pixel differences expose seam, shading, UV and repair effects, without hiding them.
    diff=np.abs(left.astype(float)-right.astype(float));mask=np.max(left,axis=2)>70
    dump(ROOT/'reports/texture_comparison.json',{'mean_absolute_rgb_difference_255':diff.mean(),'max_rgb_difference_255':diff.max(),'fraction_pixels_difference_over_15':np.mean(diff.max(axis=2)>15),'note':'Includes per-mesh normal changes and hidden caps. Same camera and material parameters; no image alignment.'})
    Image.fromarray(np.clip(diff*4,0,255).astype('uint8')).save(ROOT/'reports/texture_difference_x4.png')
    meta=json.loads((ROOT/'assets/segments.json').read_text())
    j=meta['joints'][len(meta['joints'])//2];name=f'j_body_{j["body_index"]:02d}_yaw';jid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,name);qadr=m.jnt_qposadr[jid]
    p=np.array(j['pivot']);p[0]-=j['radius']*.85
    frames=[]
    for angle in [0,-j['yaw_deg'],j['yaw_deg']]:
        d.qpos[qadr]=np.deg2rad(angle);mujoco.mj_forward(m,d)
        frames.append(annotate(b.frame(d,look=p,distance=.065,azimuth=90,elevation=-18),f'{name}   {angle:+.3f} deg','Measured body seam; flat interior material'))
    Image.fromarray(np.concatenate(frames,axis=1)).save(ROOT/'reports/seam_closeups.png')
    frames=[]
    cp=np.array(meta['chin_joint']['pivot'])+np.array([.008,0,0])
    for axis in ['yaw','pitch','roll']:
        mujoco.mj_resetDataKeyframe(m,d,0)
        jid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,'j_chin_'+axis)
        angle=meta['chin_joint'][axis+'_deg'];d.qpos[m.jnt_qposadr[jid]]=np.deg2rad(angle)
        mujoco.mj_forward(m,d)
        frames.append(annotate(b.frame(d,look=cp,distance=.052,azimuth=65,elevation=-24),f'Chin {axis}  +{angle:.3f} deg','Independent actuator | '+('continuous attachment skin' if m.nskin else 'rigid attachment')))
    Image.fromarray(np.concatenate(frames,axis=1)).save(ROOT/'reports/chin_dofs.png')
    a.close();b.close()

def overview(c,m):
    rc=c['render'];w,h=rc['width'],rc['height'];fps=rc['fps'];d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    om=original_model(c);od=mujoco.MjData(om);mujoco.mj_forward(om,od)
    a=Render(om,w//2,h);b=Render(m,w//2,h);out=Movie(ROOT/'videos/00_original_vs_segmented.mp4',c)
    for i in range(round(rc['overview_duration_s']*fps)):
        angle=90+360*i/(rc['overview_duration_s']*fps)
        frame=np.concatenate([a.frame(od,distance=.39,azimuth=angle,elevation=-12),b.frame(d,distance=.39,azimuth=angle,elevation=-12)],axis=1)
        out.add(annotate(frame,'Original unified mesh                                      Segmented and textured','Same lighting and camera | repaired eye openings; original silhouette retained at rest'))
    out.close();a.close();b.close()

def exploded(c,m):
    rc=c['render'];w,h=rc['width'],rc['height'];d=mujoco.MjData(m);r=Render(m,w,h);out=Movie(ROOT/'videos/01_exploded_segments.mp4',c)
    r.options.geomgroup[4]=1;r.options.flags[mujoco.mjtVisFlag.mjVIS_SKIN]=False
    positions=m.body_pos.copy();rgba=m.geom_rgba.copy();materials=m.geom_matid.copy();offsets=np.zeros((m.nbody,3))
    meta=json.loads((ROOT/'assets/segments.json').read_text())
    for rec in meta['segments']:
        bid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,rec['name']);gid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'visual_'+rec['name'])
        if rec['region']=='body':
            k=rec['body_index'];offsets[bid]=[-k*.007,0,0];col=PALETTE[k%len(PALETTE)]
        else:
            parent=m.body_parentid[bid];offsets[bid]=offsets[parent]
            if rec['region']=='dorsal':offsets[bid,2]+=.033;col=(135,181,226)
            elif rec['region']=='anal':offsets[bid,2]-=.033;col=(238,181,101)
            elif rec['region']=='caudal':offsets[bid,0]-=.026;col=(195,135,216)
            elif rec['region']=='chin':offsets[bid,0]+=.025;offsets[bid,2]-=.012;col=(230,120,136)
            else:offsets[bid,1]+=.03*(1 if 'L' in rec['name'] else -1);col=(114,205,162)
        m.geom_matid[gid]=-1;m.geom_rgba[gid]=[*np.array(col)/255,1]
    for i in range(round(rc['exploded_duration_s']*rc['fps'])):
        u=i/max(1,round(rc['exploded_duration_s']*rc['fps'])-1);factor=np.sin(np.pi*u)**2
        for bid in range(1,m.nbody):m.body_pos[bid]=positions[bid]+factor*(offsets[bid]-offsets[m.body_parentid[bid]])
        mujoco.mj_forward(m,d)
        im=r.frame(d,look=[-.04,0,0],distance=.48,azimuth=65,elevation=-22)
        labels=[('Head / girdle',35,115,PALETTE[0]),(f'body_01 ... body_{c["segmentation"]["N_body"]-1:02d}',35,142,PALETTE[1]),('Dorsal: one mesh',35,169,(135,181,226)),('Anal: one mesh',35,196,(238,181,101)),('Paired pectoral + pelvic fins',35,223,(114,205,162)),('Caudal fin',35,250,(195,135,216)),('Chin: yaw / pitch / roll',35,277,(230,120,136))]
        out.add(annotate(im,'Anatomical regions | exploded assembly',f'{c["segmentation"]["N_body"]} body regions + 7 fins + movable chin | colors identify rigid parts',labels))
    out.close();r.close();m.body_pos[:]=positions;m.geom_rgba[:]=rgba;m.geom_matid[:]=materials

def rom(c,m,chin_only=False):
    rc=c['render'];w,h=rc['width'],rc['height'];r=Render(m,w//2,h);d=mujoco.MjData(m);out=Movie(ROOT/'videos'/('08_chin_3dof.mp4' if chin_only else '02_joint_rom.mp4'),c)
    specs=json.loads((ROOT/'assets/joints.json').read_text());segs={s['name']:s for s in json.loads((ROOT/'assets/segments.json').read_text())['segments']}
    if chin_only:specs=[j for j in specs if j['kind']=='chin']
    steps=max(8,round((3 if chin_only else rc['rom_seconds_per_joint'])*rc['fps']))
    for n,j in enumerate(specs):
        jid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,j['name']);qa=m.jnt_qposadr[jid];origin=np.array(segs[j['body']]['origin'])
        if j['kind']=='body':
            bj=json.loads((ROOT/'assets/segments.json').read_text())['joints'][segs[j['body']]['body_index']-1];origin[0]-=.8*bj['radius']
        for k in range(steps):
            mujoco.mj_resetDataKeyframe(m,d,0);angle=j['range_deg'][1]*np.sin(2*np.pi*k/(steps-1));d.qpos[qa]=np.deg2rad(angle);mujoco.mj_forward(m,d)
            full=r.frame(d,distance=.40,azimuth=70,elevation=-15);close=r.frame(d,look=origin+([.007,0,0] if j['kind']=='chin' else np.zeros(3)),distance=.048 if j['kind']=='chin' else .085,azimuth=70,elevation=-25)
            note='rigid spherical attachment' if chin_only and c['segmentation'].get('chin_attachment')=='ball_socket' else 'body ROM reduced by seam constraints'
            out.add(annotate(np.concatenate([full,close],axis=1),f'{j["name"]}   {angle:+.2f} deg',f'Joint {n+1}/{len(specs)} | one joint at a time | {note}'))
    out.close();r.close()

def chin_scan(c,m):
    """Actuated three-axis exploration, with all other joint targets held at rest."""
    from controllers import simulate
    continuous=bool(m.nskin)
    ball_socket=c['segmentation'].get('chin_attachment')=='ball_socket'
    attachment_label='Rigid ball and socket | straight chin' if ball_socket else ('Continuous skin over spherical chin joint' if continuous else 'Rigid attachment: visible gaps at large bends')
    rc=c['render'];w,h=rc['width'],rc['height'];fps=rc['fps']
    trace=simulate(m,c,'chin_scan',duration=6,record_fps=fps)
    np.savez(ROOT/'reports/trajectory_chin_scan.npz',**trace)
    d=mujoco.MjData(m);wide=Render(m,w//2,h);close=Render(m,w//2,h//2)
    out=Movie(ROOT/'videos/09_chin_scan.mp4',c)
    bid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'chin_0')
    head=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'body_00_head')
    adrs=[m.jnt_qposadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,'j_chin_'+axis)] for axis in ['yaw','pitch','roll']]
    snapshots=[]
    for i in range(round(6*fps)):
        t=i/fps
        d.qpos[:]=[np.interp(t,trace['time'],trace['qpos'][:,k]) for k in range(m.nq)]
        d.qpos[3:7]/=np.linalg.norm(d.qpos[3:7]);mujoco.mj_forward(m,d)
        look=d.xpos[bid]+d.xmat[head].reshape(3,3)@np.array([.006,0,0])
        full=wide.frame(d,look=d.subtree_com[head],distance=.4,azimuth=70,elevation=-15)
        side=close.frame(d,look=look,distance=.057,azimuth=90,elevation=-10)
        top=close.frame(d,look=look,distance=.057,azimuth=90,elevation=-90)
        angles=np.rad2deg(d.qpos[adrs])
        subtitle=f'{c["controller"]["chin_frequency_hz"]:.1f} Hz | yaw {angles[0]:+.1f}, pitch {angles[1]:+.1f}, roll {angles[2]:+.1f} deg | t = {t:.2f} s'
        frame=annotate(np.concatenate([full,np.concatenate([side,top],axis=0)],axis=1),'Active chin scan | simulated position actuators',subtitle,
                       [('SIDE CLOSE-UP',w//2+20,110,(180,218,218)),('TOP CLOSE-UP',w//2+20,h//2+15,(180,218,218)),(attachment_label,25,h-40,(180,218,218))])
        out.add(frame)
        if i in [round(fps*t) for t in [1.0,1.25,1.5,1.75]]:
            snapshots.append(annotate(top.copy(),f't = {t:.2f} s',subtitle.split(' | t')[0]))
    out.close();wide.close()
    Image.fromarray(np.concatenate([np.concatenate(snapshots[:2],axis=1),np.concatenate(snapshots[2:],axis=1)],axis=0)).save(ROOT/'reports/chin_scan.png')
    limits={};frames=[]
    for axis,qa in zip(['yaw','pitch','roll'],adrs):
        mujoco.mj_resetDataKeyframe(m,d,0)
        jid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,'j_chin_'+axis)
        d.qpos[qa]=m.jnt_range[jid,1];limits[axis]=float(np.rad2deg(d.qpos[qa]))
        mujoco.mj_forward(m,d)
        frames.append(annotate(close.frame(d,look=d.xpos[bid]+[.006,0,0],distance=.057,azimuth=65,elevation=-24),
                               f'Chin {axis} +{limits[axis]:.1f} deg',attachment_label))
    Image.fromarray(np.concatenate(frames,axis=1)).save(ROOT/'reports/chin_dofs.png')
    close.close()
    q=np.rad2deg(trace['qpos'][trace['time']>1][:,adrs])
    stats={'duration_s':6,'frequency_hz':c['controller']['chin_frequency_hz'],
           'amplitude_fraction':c['controller']['chin_amplitude_fraction'],
           'measured_range_deg':{axis:[float(q[:,i].min()),float(q[:,i].max())] for i,axis in enumerate(['yaw','pitch','roll'])},
           'root_displacement_m':float(np.linalg.norm(trace['qpos'][-1,:3]-trace['qpos'][0,:3])),
           'note':'Free-root simulation; only chin targets oscillate. Other joint targets remain zero. Engineering motion, not calibrated biological kinematics.'}
    dump(ROOT/'reports/chin_scan.json',stats)
    seams=json.loads((ROOT/'reports/chin_seam_metrics.json').read_text())+json.loads((ROOT/'reports/chin_compound_metrics.json').read_text())
    worst=max(seams,key=lambda row:row['gap_m'])
    measured='; '.join(f'{axis}: {lo:+.2f} to {hi:+.2f} degrees' for axis,(lo,hi) in stats['measured_range_deg'].items())
    if ball_socket:
        from check_chin_socket import run as check_socket
        check_socket(c)
        surface_note='The chin is fully rigid and contains a rounded ball seated in a concave head socket. No visual skin or flexible collar is loaded. Full 3-axis ranges are retained. See [socket geometry and checks](chin_socket.md).'
        metric_note=f"Maximum sampled head-socket rim opening: **{worst['gap_m']*1000:.6g} mm**. This coverage check is enforced across all individual and combined test poses; the exposed ball/shaft junction is not a mating seam. Compiled-model checks are in `chin_socket_validation.json`."
    elif continuous:
        from check_chin_surface import run as check_surface
        check_surface(c)
        surface_note='The visible head/chin surface is now continuous: a textured MuJoCo skin blends across the spherical joint. It is closed by position topology and was checked at 125 individual/combined poses. See [attachment validation and before/after renders](chin_surface.md). The rigid-interface gap below measures hidden mechanical parts, not visible cracks. Skin deformation is visual only; contacts and fluid forces still use the existing rigid proxies.'
    else:
        surface_note='The rigid attachment exposes gaps at broad deflections. A continuous skin is available with segmentation.chin_skin: true.'
    if not ball_socket:
        metric_note=f"Maximum sampled gap between the underlying rigid segments: **{worst['gap_m']*1000:.3f} mm**, or {worst['gap_fraction']:.1%} of sampled local seam height. The continuous visual skin has its own checks in `chin_surface_validation.json` when enabled."
    report('chin_mobility.md',f'''# Active chin mobility

The three chin hinges share the pivot in `assets/segments.json`. See [the current socket revision](chin_socket.md) for its placement beneath the mouth. Current limits: yaw ±{limits['yaw']:g}°, pitch ±{limits['pitch']:g}°, roll ±{limits['roll']:g}°. Body seam restrictions remain enforced. The ball/socket attachment enforces socket coverage at the full requested chin range. This rendering step uses the generated geometry and runs no swimming parameter optimization.

The scan commands {100*stats['amplitude_fraction']:g}% of each limit at {stats['frequency_hz']:g} Hz. Six seconds of free-root MuJoCo simulation completed without numerical warnings or nonfinite states. Actual ranges after the initial ramp: {measured}. These are joint angles relative to the head, not root movement. Other joint targets stay at zero in this dedicated demonstration; fluid forces and root motion remain active.

{surface_note}

{metric_note}

[Amey-Özel et al. (2015)](https://pubmed.ncbi.nlm.nih.gov/25388854/) documents a highly mobile sensory appendage with motor innervation. [von der Emde et al. (2008)](https://pubmed.ncbi.nlm.nih.gov/18992334/) describes lateral searching and directed probing. The numerical limits and scan frequency here are adjustable engineering assumptions, not values measured in those studies. This is an open-loop scanning demonstration without environmental sensing or target feedback.

`08_chin_3dof.mp4` sweeps each axis kinematically. `09_chin_scan.mp4` shows actual actuation at {w}×{h}, {fps} fps. Earlier swimming videos retain their prior chin motion until explicitly regenerated; their existing tuning parameters are unchanged by the focused update.

![Actuated scan, top views](chin_scan.png)

![Individual range limits](chin_dofs.png)
''')

def behavior(c,m,name,index):
    rc=c['render'];w,h=rc['width'],rc['height'];fps=rc['fps'];d=mujoco.MjData(m)
    from evaluate_behaviors import signature,run as evaluate
    provenance=ROOT/'reports'/f'trajectory_{name}.json'
    if not provenance.exists() or json.loads(provenance.read_text()).get('signature')!=signature(c):
        evaluate(c,[name],ablations=False)
    trace=np.load(ROOT/'reports'/f'trajectory_{name}.npz');stats=json.loads((ROOT/'reports/behaviors.json').read_text())[name];params=stats['controller']
    ground=rc.get('swimming_ground') if index in range(3,7) else None
    side=Render(m,w,h//2,ground);small=Render(m,w//2,h//2,ground);out=Movie(ROOT/'videos'/f'{index:02d}_{name if name not in ("forward","backward") else name+"_swim"}.mp4',c)
    preview=[];nframes=round(c['simulation']['duration_s']*fps);rootid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'body_00_head')
    title=name.replace('_',' ').title();title+=' command' if name=='hover' else ''
    if params.get('motion_label'):title+=' | '+params['motion_label']
    elif params['profile']=='carangiform':title+=' | Carangiform-style body wave'
    cadence=f'body/tail {params["frequency_hz"]:g} Hz | pectorals {params.get("pec_frequency_hz",params["frequency_hz"]):g} Hz'
    tailid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'fin_caudal_0')
    tailqa=m.jnt_qposadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,'j_fin_caudal_0')]
    pecqa=m.jnt_qposadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,'j_fin_pecL_0_abduct')]
    start_position=trace['position'][0].copy()
    fixed_top=rc.get('swimming_fixed_top',True) and index in range(3,7)
    for i in range(nframes):
        t=i/fps
        for k in range(m.nq):d.qpos[k]=np.interp(t,trace['time'],trace['qpos'][:,k])
        d.qpos[3:7]/=np.linalg.norm(d.qpos[3:7]);mujoco.mj_forward(m,d)
        look=d.subtree_com[rootid].copy()
        im_side=side.frame(d,look=look,distance=.29,azimuth=90,elevation=rc.get('swimming_side_elevation_deg',-5) if ground else -5)
        im_top=small.frame(d,look=start_position if fixed_top else look,distance=.30,azimuth=90,elevation=-90)
        finlook=d.xpos[tailid]+d.xmat[tailid].reshape(3,3)@np.array([-.015,0,-.002])
        im_fin=small.frame(d,look=finlook,distance=.10,azimuth=70,elevation=-55)
        frame=np.concatenate([im_side,np.concatenate([im_top,im_fin],axis=1)],axis=0)
        # Quiet label bands keep grid lines from competing with the readouts.
        frame[h//2:h//2+78,:,:]=[9,27,35]
        frame[h-65:,:,:]=[9,27,35]
        speed=np.interp(t,trace['time'],trace['axial_velocity'])/c['source']['length_m']
        labels=[('TRACKING SIDE',28,110,(160,199,201)),('FIXED TOP VIEW' if fixed_top else 'TRACKING TOP VIEW',28,h//2+18,(160,199,201)),('CAUDAL FIN / REAR BODY',w//2+28,h//2+18,(160,199,201)),
                (f'Tail {np.rad2deg(d.qpos[tailqa]):+.1f} deg | left pectoral {np.rad2deg(d.qpos[pecqa]):+.1f} deg',w//2+28,h-42,(215,230,230))]
        if ground and ground.get('enabled',True):
            delta=(look-start_position)*1000
            labels.extend([(f'World-fixed grid: {ground["spacing_m"]*1000:g} mm | gold lines: world origin',28,140,(218,195,149)),
                           (f'Displacement: x {delta[0]:+.1f} mm | y {delta[1]:+.1f} mm | z {delta[2]:+.1f} mm',28,h//2+46,(215,230,230))])
        # Perspective scale at look-at depth in the top panel, 45-degree vertical FOV.
        scale_px=int(.02*(h/2)/(2*.30*np.tan(np.deg2rad(45/2))))
        frame=annotate(frame,title,f'Tail + paired pectoral propulsion | {cadence} | axial speed = {speed:+.4f} BL/s | t = {t:.2f} s',labels)
        draw=Image.fromarray(frame);dd=ImageDraw.Draw(draw);sx,sy=40,h-35;dd.line((sx,sy,sx+scale_px,sy),fill='white',width=3);dd.text((sx,sy-24),'20 mm at center depth',font=font(15),fill='white');frame=np.array(draw)
        out.add(frame)
        if name=='forward' and i%max(1,fps//12)==0 and i<fps*3:
            im=Image.fromarray(frame);im.thumbnail((640,360));preview.append(im)
        if i==nframes//2:Image.fromarray(frame).save(ROOT/'reports'/f'behavior_{name}.png')
    out.close();side.close();small.close()
    if preview:preview[0].save(ROOT/'videos/preview.gif',save_all=True,append_images=preview[1:],duration=83,loop=0)

def verify_ground(c,m):
    """Confirm decorative ground coordinates remain fixed as the fish and camera move."""
    g=c['render']['swimming_ground'];d=mujoco.MjData(m);r=Render(m,320,240,g)
    original=m.geom_pos.copy();count=m.ngeom
    def coordinates():
        return np.array([np.r_[geom.pos,geom.mat.ravel(),geom.size]
                         for geom in r.r.scene.geoms[:r.r.scene.ngeom]
                         if geom.category==mujoco.mjtCatBit.mjCAT_DECOR])
    try:
        mujoco.mj_forward(m,d);r.frame(d);before=coordinates()
        d.qpos[:3]=[.02,.015,.01];mujoco.mj_forward(m,d)
        r.frame(d,look=d.qpos[:3],azimuth=115,elevation=-65)
        np.testing.assert_array_equal(before,coordinates())
        np.testing.assert_array_equal(original,m.geom_pos);assert count==m.ngeom
        dump(ROOT/'reports/ground_world_reference.json',{'decorative_geoms':len(before),
            'grid_lines':max(0,len(before)-1),'unchanged_across_root_and_camera_changes':True,
            'world_z_m':g['z_m'],'spacing_m':g['spacing_m'],'model_geoms_unchanged':True})
    finally:r.close()

def run(c,only=None):
    m=mujoco.MjModel.from_xml_path(str(ROOT/'fish.xml'))
    if only in (None,'stills'):stills(c,m)
    if only in (None,'overview'):overview(c,m);print('Rendered 00',flush=True)
    if only in (None,'exploded'):exploded(c,m);print('Rendered 01',flush=True)
    if only in (None,'rom'):rom(c,m);print('Rendered 02',flush=True)
    if only in (None,'chin'):rom(c,m,chin_only=True);chin_scan(c,m);print('Rendered 08 chin ROM and 09 actuated scan',flush=True)
    for index,name in enumerate(['forward','backward','hover','turning','body_undulation'],start=3):
        if only in (None,name) or (only=='swimming' and index<=6):behavior(c,m,name,index);print('Rendered',name,flush=True)
    if only in (None,'swimming'):
        verify_ground(c,m)
        ground=c['render']['swimming_ground'];rows=[]
        for name in ['forward','backward','hover','turning']:
            tr=np.load(ROOT/'reports'/f'trajectory_{name}.npz')
            adrs=[m.jnt_qposadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,'j_chin_'+axis)] for axis in ['yaw','pitch','roll']]
            peak=np.rad2deg(np.abs(tr['qpos'][:,adrs]).max())
            displacement=(tr['position'][-1]-tr['position'][0])*1000
            rows.append(f'| {name} | {peak:.4f} | {displacement[0]:+.2f} | {displacement[1]:+.2f} | {displacement[2]:+.2f} |')
        report('swimming_ground.md',f'''# Stationary chin and ground reference: videos 03–06

All three chin position targets are held at zero for forward, backward, hover and turning. Tiny passive actuator compliance is measured below; there is no commanded scan. The separate chin demonstration retains its three active DOFs. Both tail and pectoral actuation remain active; trajectories use the current per-gait configuration. See `reference_swim.md` for the current reference-inspired posterior-body wave and measured contributions. `forward_improvement.md` and `larger_tail.md` preserve the earlier controller comparisons.

The ground grid is fixed in world coordinates at z = {ground['z_m']*1000:g} mm, with {ground['spacing_m']*1000:g} mm spacing and a heavier line every {ground['major_every']} cells. Gold lines mark x = 0 and y = 0. It is decorative render geometry and adds no contact, fluid force or mass. Its lines are never translated or rotated with the camera or fish.

The top panel has a stationary camera centered on the initial whole-fish COM; the side and tail views track the fish. The overlays give x/y/z COM displacement relative to the initial pose. These fixed references make the measured forward, backward and lateral movement visible even in tracking views.

| Behavior | Peak absolute chin angle (degrees) | Final Δx (mm) | Final Δy (mm) | Final Δz (mm) |
|---|---:|---:|---:|---:|
'''+ '\n'.join(rows)+f'''

Four H.264 videos rendered at {c['render']['width']}×{c['render']['height']}, {c['render']['fps']} fps. Configure `controller.gaits.<behavior>.chin_amplitude_fraction`, `render.swimming_ground`, `render.swimming_fixed_top` and `render.swimming_side_elevation_deg` in config.yaml. Regenerate with `.venv/bin/python render_videos.py --only swimming`.

![Verified video frames](swimming_videos_preview.png)
''')
    if only is not None:return
    report('phase5.md',f'''# Phase 5 — rendering

Offscreen `mujoco.Renderer`, {c['render']['width']}×{c['render']['height']}, {c['render']['fps']} fps, H.264/yuv420p MP4. Backend: `{os.environ.get('MUJOCO_GL')}`. Linux launcher selects EGL and retries OSMesa if rendering initialization fails; this macOS build uses CGL.

Videos 00–07 show original/segmented rotation, progressive exploded assembly, each hinge ROM separately, then forward, backward, hover command, turning and body-undulation comparison. Videos 03–06 hold the chin neutral, use a world-fixed ground grid, tracking side/caudal-fin views and a fixed top camera. They show measured signed COM velocity along the head's anterior axis and displacement from the start. The root is freely simulated. Cached trajectories carry a model/controller signature and are regenerated when stale. The ROM/exploded clips are explicitly kinematic demonstrations. A 20 mm perspective scale is drawn at the initial COM depth in the top view; ground grid spacing is independently configured. No tank or misleading bubbles/current visualization is added.

`texture_continuity.png` compares the same camera/lighting at rest; `texture_difference_x4.png` exposes differences from normal splitting, caps and repairs. `seam_closeups.png` shows a representative body interface at rest and both ROM limits. `chin_dofs.png` and `08_chin_3dof.mp4` show independent yaw, pitch and roll. `09_chin_scan.mp4` demonstrates simulated three-axis actuation; `chin_mobility.md` records achieved motion and visible rigid-attachment gaps. Dorsal and anal fins are whole rigid meshes. Their long bases follow one body segment each, so body bending can change attachment alignment; no flexible skin is claimed.
''')
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--config');ap.add_argument('--only',choices=['stills','overview','exploded','rom','chin','swimming','forward','backward','hover','turning','body_undulation']);a=ap.parse_args();run(config(a.config),a.only)
