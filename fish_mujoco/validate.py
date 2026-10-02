"""Independent final checks, runtime benchmark and artifact inventory."""
import json, subprocess, time, hashlib
import numpy as np
import mujoco
from common import *

def run(c):
    m=mujoco.MjModel.from_xml_path(str(ROOT/'fish.xml'));d=mujoco.MjData(m)
    start=time.perf_counter()
    for _ in range(round(5/m.opt.timestep)):mujoco.mj_step(m,d)
    wall=time.perf_counter()-start
    passive={'sim_seconds':5,'wall_seconds':wall,'realtime_factor':5/wall,'root_displacement_m':float(np.linalg.norm(d.qpos[:3])),'max_speed':float(np.max(np.abs(d.qvel))),'warnings':d.warning.number.tolist()}
    videos=[]
    expected=['00_original_vs_segmented','01_exploded_segments','02_joint_rom','03_forward_swim','04_backward_swim','05_hover','06_turning','07_body_undulation','08_chin_3dof']
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
    src=source_path(c,'mesh');checksum=hashlib.sha256(src.read_bytes()).hexdigest()
    seams=json.loads((ROOT/'reports/seam_metrics.json').read_text());segments=json.loads((ROOT/'assets/segments.json').read_text())['segments']
    summary={'source_sha256':checksum,'segments':len(segments),'body_count':sum(s['region']=='body' for s in segments),'mass_kg':float(m.body_mass.sum()),'max_sampled_gap_fraction':max(r['gap_fraction'] for r in seams),'max_penetration_m':max(r['penetration_m'] for r in seams),'passive':passive,'videos':videos}
    dump(ROOT/'reports/validation.json',summary)
    report('phase6.md',f'''# Phase 6 — verification and deliverables

- Model compiles; explicit mass and inertia are finite. {len(segments)} closed segment solids retain OBJ face-corner UV coordinates.
- Independent tests: see `pytest.txt` for the complete result. Tests cover compile/forward, topology/UVs, joint and actuator ranges, seam thresholds, a 5 s driven simulation, 5 s passive buoyancy, controller clipping, actual joint-limit response, three independent chin DOFs and single-mesh median fins.
- Total mass **{m.body_mass.sum()*1000:.3f} g**. Passive root drift after 5 s: **{passive['root_displacement_m']:.3g} m**; max speed {passive['max_speed']:.3g}; simulation rate **{5/wall:.1f}× real time** (rendering excluded).
- Maximum sampled body seam gap: **{summary['max_sampled_gap_fraction']:.3%} of local height**. Maximum measured penetration: {summary['max_penetration_m']*1000:.4f} mm, including designed overlap. Rest and full-ROM thresholds: 1% and 3%.
- Nine H.264 videos checked with ffprobe at {c['render']['width']}×{c['render']['height']}, {c['render']['fps']} fps. `videos/preview.gif` is a small behavior preview. Per-video durations and frame counts are in `validation.json`.
- Raw OBJ SHA256: `{checksum}`. Installed package versions are pinned in `requirements.txt`; all derived assets are generated from the raw OBJ, textures and config.

The model is a tested mechanical approximation, not a validated biological digital twin. Body seams are checked at finitely sampled individual-joint poses; deforming fin attachments, arbitrary compound bends, CFD thrust, electrical fields and empirical muscle parameters are not established. Backward/hover labels name control commands; read actual measured displacement in `behaviors.json`.
''')
    print(json.dumps(summary,indent=2))
if __name__=='__main__':run(config())
