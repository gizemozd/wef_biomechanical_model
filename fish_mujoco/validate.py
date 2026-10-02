"""Independent final checks, runtime benchmark and artifact inventory."""
import json, subprocess, time, hashlib, tempfile
import numpy as np
import mujoco
from common import *

def swimming_videos(c,m,videos):
    """Decode the deliverables and verify that displayed motion has current provenance."""
    from PIL import Image
    from evaluate_behaviors import signature
    expected_signature=signature(c);items=[];chin_angles={};frames=[]
    chin=[m.jnt_qposadr[m.joint('j_chin_'+axis).id] for axis in ['yaw','pitch','roll']]
    controls=[m.actuator('a_j_chin_'+axis).id for axis in ['yaw','pitch','roll']]
    with tempfile.TemporaryDirectory(prefix='fish_video_check_') as directory:
        for index,name in enumerate(['forward','backward','hover','turning'],3):
            filename=f'{index:02d}_{name+"_swim" if name in ("forward","backward") else name}.mp4'
            path=ROOT/'videos'/filename
            subprocess.run(['ffmpeg','-v','error','-xerror','-i',str(path),'-f','null','-'],check=True)
            trace=np.load(ROOT/'reports'/f'trajectory_{name}.npz')
            provenance=json.loads((ROOT/'reports'/f'trajectory_{name}.json').read_text())
            assert provenance['signature']==expected_signature,f'Stale {name} trajectory'
            assert np.all(trace['ctrl'][:,controls]==0),f'Chin commanded during {name}'
            chin_angles[name]=float(np.rad2deg(np.abs(trace['qpos'][:,chin]).max()))
            assert chin_angles[name]<.1,f'Chin compliance is visible during {name}'
            seconds=c['simulation']['duration_s'];frame_time=seconds/2
            image_path=Path(directory)/f'{name}.png'
            subprocess.run(['ffmpeg','-v','error','-y','-ss',str(frame_time),'-i',str(path),'-frames:v','1',str(image_path)],check=True)
            with Image.open(image_path) as im:
                im.save(ROOT/'reports'/f'{path.stem}_verified_frame.png')
                frames.append(im.resize((960,540)))
            item=next(row for row in videos if row['file']==filename)
            assert int(item['nb_frames'])==round(seconds*c['render']['fps'])
            items.append({**item,'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
        canvas=Image.new('RGB',(1920,1080))
        for i,frame in enumerate(frames):canvas.paste(frame,((i%2)*960,(i//2)*540))
        canvas.save(ROOT/'reports/swimming_videos_preview.png')
    dump(ROOT/'reports/swimming_video_validation.json',{
        'trajectory_signature':expected_signature,'videos':items,'decoded_all_frames':True,
        'all_chin_controls_zero':True,'max_absolute_chin_angle_deg':chin_angles,
        'ground':json.loads((ROOT/'reports/ground_world_reference.json').read_text()),
        'tests':(ROOT/'reports/pytest.txt').read_text().strip(),
        'forward_profile':c['controller']['gaits']['forward']['profile'],
        'verified_frame_time_s':c['simulation']['duration_s']/2,
        'regenerated_behavior_videos':['03','04','05','06','07']})

def run(c):
    m=mujoco.MjModel.from_xml_path(str(ROOT/'fish.xml'));d=mujoco.MjData(m)
    start=time.perf_counter()
    for _ in range(round(5/m.opt.timestep)):mujoco.mj_step(m,d)
    wall=time.perf_counter()-start
    passive={'sim_seconds':5,'wall_seconds':wall,'realtime_factor':5/wall,'root_displacement_m':float(np.linalg.norm(d.qpos[:3])),'max_speed':float(np.max(np.abs(d.qvel))),'warnings':d.warning.number.tolist()}
    videos=[]
    expected=['00_original_vs_segmented','01_exploded_segments','02_joint_rom','03_forward_swim','04_backward_swim','05_hover','06_turning','07_body_undulation','08_chin_3dof','09_chin_scan']
    for stem in expected:
        path=ROOT/'videos'/f'{stem}.mp4'
        if not path.exists():raise FileNotFoundError(path)
        p=subprocess.run(['ffprobe','-v','error','-select_streams','v:0','-show_entries','stream=codec_name,width,height,r_frame_rate,nb_frames,duration','-of','json',str(path)],capture_output=True,text=True,check=True)
        stream=json.loads(p.stdout)['streams'][0]
        assert stream['codec_name']=='h264'
        assert (stream['width'],stream['height'])==(c['render']['width'],c['render']['height'])
        assert float(stream['duration'])>0
        num,den=map(float,stream['r_frame_rate'].split('/'));assert num/den==c['render']['fps']
        videos.append({'file':path.name,'bytes':path.stat().st_size,**stream})
    swimming_videos(c,m,videos)
    src=source_path(c,'mesh');checksum=hashlib.sha256(src.read_bytes()).hexdigest()
    seams=json.loads((ROOT/'reports/seam_metrics.json').read_text())+json.loads((ROOT/'reports/body_compound_metrics.json').read_text());segments=json.loads((ROOT/'assets/segments.json').read_text())['segments']
    summary={'source_sha256':checksum,'segments':len(segments),'body_count':sum(s['region']=='body' for s in segments),'mass_kg':float(m.body_mass.sum()),'max_sampled_gap_fraction':max(r['gap_fraction'] for r in seams),'max_penetration_m':max(r['penetration_m'] for r in seams),'passive':passive,'videos':videos}
    dump(ROOT/'reports/validation.json',summary)
    report('phase6.md',f'''# Phase 6 — verification and deliverables

- Model compiles; explicit mass and inertia are finite. {len(segments)} closed segment solids retain OBJ face-corner UV coordinates.
- Independent tests: see `pytest.txt` for the complete result. Tests cover compile/forward, topology/UVs, joint and actuator ranges, seam thresholds, a 5 s driven simulation, 5 s passive buoyancy, controller clipping, actual joint-limit response, three independent chin DOFs and single-mesh median fins.
- Total mass **{m.body_mass.sum()*1000:.3f} g**. Passive root drift after 5 s: **{passive['root_displacement_m']:.3g} m**; max speed {passive['max_speed']:.3g}; simulation rate **{5/wall:.1f}× real time** (rendering excluded).
- Maximum sampled body seam gap, including combined yaw/pitch corners: **{summary['max_sampled_gap_fraction']:.3%} of local height**. Maximum measured penetration: {summary['max_penetration_m']*1000:.4f} mm, including designed overlap. Rest and full-ROM thresholds: 1% and 3%.
- Ten H.264 videos checked with ffprobe at {c['render']['width']}×{c['render']['height']}, {c['render']['fps']} fps. Videos 03–06 were fully decoded, matched to current trajectory signatures and checked for zero chin commands. `videos/preview.gif` is a small behavior preview. Per-video durations and frame counts are in `validation.json`.
- Raw OBJ SHA256: `{checksum}`. Installed package versions are pinned in `requirements.txt`; all derived assets are generated from the raw OBJ, textures and config.

The model is a tested mechanical approximation, not a validated biological digital twin. Body seams are checked at finitely sampled individual-joint poses and combined yaw/pitch corners; deforming fin attachments, arbitrary compound bends, CFD thrust, electrical fields and empirical muscle parameters are not established. Backward/hover labels name control commands; read actual measured displacement in `behaviors.json`.
''')
    print(json.dumps(summary,indent=2))
if __name__=='__main__':run(config())
