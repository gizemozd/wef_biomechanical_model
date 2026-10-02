"""Rebuild from raw OBJ + config; phase reports are regenerated in order."""
import argparse, os, subprocess, sys, time
from pathlib import Path
from common import ROOT,config,dump,report

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--config',default=str(ROOT/'config.yaml'));ap.add_argument('--from-phase',type=int,default=0);ap.add_argument('--through-phase',type=int,default=6);ap.add_argument('--quick',action='store_true',help='Explicit 640x360/15fps smoke build; does not represent final video quality')
    a=ap.parse_args();c=config(a.config);cfg=Path(a.config).resolve()
    if a.quick:
        import yaml
        c['render'].update(width=640,height=360,fps=15,overview_duration_s=1,exploded_duration_s=1,rom_seconds_per_joint=.1)
        c['simulation']['duration_s']=2;c['controller'].update(tuning_frequencies=[1,2],tuning_amplitudes=[20],tuning_wavelengths=[.22],tuning_duration_s=2)
        c.pop('_config_dir');c['source']['mesh']=str((cfg.parent/c['source']['mesh']).resolve());c['source']['texture_dir']=str((cfg.parent/c['source']['texture_dir']).resolve())
        cfg=ROOT/'reports/quick_config.yaml';cfg.write_text(yaml.safe_dump(c));c=config(cfg)
    started=time.perf_counter()
    phases=[(0,'inspect_mesh.py'),(1,'segment_mesh.py'),(3,'build_model.py'),(4,'evaluate_behaviors.py'),(5,'render_videos.py'),(5,'review_chin_cut.py')]
    for phase,script in phases:
        if a.from_phase<=phase<=a.through_phase:
            print(f'Phase {phase}: {script}',flush=True)
            command=[sys.executable,str(ROOT/script),'--config',str(cfg)]
            result=subprocess.run(command)
            if result.returncode and phase==5 and sys.platform.startswith('linux') and os.environ.get('MUJOCO_GL','egl')=='egl':
                env={**os.environ,'MUJOCO_GL':'osmesa'};result=subprocess.run(command,env=env)
            if result.returncode:raise SystemExit(result.returncode)
    if a.from_phase<=6<=a.through_phase:
        result=subprocess.run([sys.executable,'-m','pytest',str(ROOT/'tests'),'-q'],capture_output=True,text=True,env={**os.environ,'FISH_CONFIG':str(cfg)})
        (ROOT/'reports/pytest.txt').write_text(result.stdout+result.stderr);print(result.stdout,flush=True)
        if result.returncode:raise SystemExit(result.returncode)
        import validate
        validate.run(c)
    dump(ROOT/'reports/run.json',{'config':str(cfg),'elapsed_s':time.perf_counter()-started,'from_phase':a.from_phase,'through_phase':a.through_phase,'quick':a.quick})
if __name__=='__main__':main()
