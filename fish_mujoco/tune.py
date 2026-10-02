"""Optional CPG experiment; never overwrite configured-gait trajectories."""
import argparse, itertools, time
import numpy as np
import mujoco
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from common import *
from controllers import simulate

def run(c):
    m=mujoco.MjModel.from_xml_path(str(ROOT/'fish.xml'));p=c['controller'];L=c['source']['length_m'];rows=[]
    started=time.perf_counter()
    for f,a,w in itertools.product(p['tuning_frequencies'],p['tuning_amplitudes'],p['tuning_wavelengths']):
        trial={'frequency_hz':f,'amplitude_deg':a,'wavelength_bl':w}
        tr=simulate(m,c,duration=p['tuning_duration_s'],overrides=trial)
        idx=tr['time']>=p['tuning_duration_s']/2
        speed=float(np.polyfit(tr['time'][idx],tr['position'][idx,0],1)[0]/L)
        rows.append({**trial,'speed_bl_s':speed,'final_position_m':tr['position'][-1]})
        print('tune',f,a,w,'speed',round(speed,6),flush=True)
    best=max(rows,key=lambda t:t['speed_bl_s']);bestp={k:best[k] for k in ['frequency_hz','amplitude_deg','wavelength_bl']}
    dump(ROOT/'reports/tuning.json',{'trials':rows,'best':bestp,'wall_seconds':time.perf_counter()-started})
    fig,ax=plt.subplots(figsize=(9,5),layout='constrained')
    for a,w in itertools.product(p['tuning_amplitudes'],p['tuning_wavelengths']):
        group=[r for r in rows if r['amplitude_deg']==a and r['wavelength_bl']==w]
        ax.plot([r['frequency_hz'] for r in group],[r['speed_bl_s'] for r in group],'-o',label=f'A={a:g}°, wavelength={w:g} BL')
    ax.axhline(0,color='gray',lw=.5);ax.set_xlabel('Frequency (Hz)');ax.set_ylabel('Signed anterior speed (BL/s)');ax.legend();ax.grid(alpha=.2);ax.set_title('Free-root CPG grid search | stateless ellipsoid hydrodynamics')
    fig.savefig(ROOT/'reports/tuning.png',dpi=160);plt.close(fig)
    report('tuning_experiment.md',f'''# Optional CPG search

{len(rows)} combinations of frequency, CPG amplitude scale and body wavelength; each simulated {p['tuning_duration_s']} s. Objective: signed whole-fish COM +x displacement slope over the final half, in full-mesh body lengths/s. Chosen parameters: `{bestp}`. Best trial speed: **{best['speed_bl_s']:.6f} BL/s**. A short transient grid is not a steady-state efficiency or biological validation.

This experiment leaves config.yaml, the configured-gait trajectories and phase4.md unchanged. To adopt a candidate, explicitly update the controller configuration and run evaluate_behaviors.py, which checks tail/pectoral forces and independent control ablations. A speed-only winner need not retain useful contributions from both fin groups. Native stateless ellipsoid forces are an approximation, not calibrated biological hydrodynamics.

![Tuning](tuning.png)
''')
    print('Candidate parameters (not automatically adopted):',bestp)
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--config');a=ap.parse_args();run(config(a.config))
