"""Re-simulate configured gaits and audit native fin forces; no parameter search."""
import argparse
import hashlib
import json
import time
import numpy as np
import mujoco
from common import ROOT, config, dump, report
from controllers import Controller, simulate

BEHAVIORS = ['forward','backward','hover','turning','body_undulation']

def signature(c):
    h=hashlib.sha256()
    for name in ['fish.xml','controllers.py','assets/joints.json']:
        h.update((ROOT/name).read_bytes())
    h.update(json.dumps({k:c[k] for k in ['controller','simulation','source']},sort_keys=True).encode())
    return h.hexdigest()

def force_contributions(m,trace):
    """Subtract each group's forces at identical states, without changing the run.

    A tiny positive interaction value prevents MuJoCo's inertia-box fallback when
    a body's only ellipsoid is disabled. This diagnostic never steps the model.
    """
    d=mujoco.MjData(m)
    groups={
        'caudal':[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'fluid_fin_caudal_0')],
        'pectoral':[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'fluid_fin_pec'+s+'_0') for s in ['L','R']],
    }
    original=m.geom_fluid[:,0].copy()
    result={name:[] for name in [*groups,'other','total']}
    axial_velocity=[]
    try:
        for q,v in zip(trace['qpos'],trace['qvel']):
            d.qpos[:]=q;d.qvel[:]=v;mujoco.mj_forward(m,d);mujoco.mj_subtreeVel(m,d)
            axes=d.xmat[1].reshape(3,3)
            axial_velocity.append(d.subtree_linvel[1]@axes[:,0])
            base=d.qfrc_fluid[:3].copy();subtotal=np.zeros(3)
            for name,ids in groups.items():
                m.geom_fluid[ids,0]=1e-30;mujoco.mj_passive(m,d)
                force=base-d.qfrc_fluid[:3];subtotal+=force
                result[name].append(force@axes)
                m.geom_fluid[ids,0]=original[ids]
            result['other'].append((base-subtotal)@axes)
            result['total'].append(base@axes)
    finally:
        m.geom_fluid[:,0]=original
    return {k:np.asarray(v) for k,v in result.items()},np.asarray(axial_velocity)

def summarize(c,m,trace,forces,axial_velocity):
    L=c['source']['length_m'];steady=trace['time']>=trace['time'][-1]/2
    mean_speed=float(np.mean(axial_velocity[steady])/L)
    d=mujoco.MjData(m);yaw=[]
    for q in trace['qpos']:
        d.qpos[:]=q;mujoco.mj_forward(m,d)
        axis=d.xmat[1].reshape(3,3)[:,0];yaw.append(np.arctan2(axis[1],axis[0]))
    motion={}
    specs=json.loads((ROOT/'assets/joints.json').read_text())
    for j in specs:
        if j['kind'] not in ('caudal','pectoral','chin'):continue
        jid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,j['name'])
        q=np.rad2deg(trace['qpos'][steady,m.jnt_qposadr[jid]])
        motion[j['name']]={'min_deg':float(q.min()),'max_deg':float(q.max()),'peak_to_peak_deg':float(np.ptp(q))}
    return {
        'signed_speed_bl_s':mean_speed,
        'world_x_speed_bl_s':float(np.polyfit(trace['time'][steady],trace['position'][steady,0],1)[0]/L),
        'displacement_m':trace['position'][-1]-trace['position'][0],
        'reynolds':abs(mean_speed)*L*L*c['fluid']['density']/c['fluid']['viscosity'],
        'yaw_change_deg':float(np.rad2deg(np.unwrap(yaw)[-1]-yaw[0])),
        'forces':{k:{'mean_axial_mN':float(v[steady,0].mean()*1000),
                     'rms_force_mN':float(np.sqrt(np.mean(np.sum(v[steady]**2,axis=1)))*1000)} for k,v in forces.items()},
        'joint_motion':motion,
    }

def evaluate(c,m,name,overrides=None):
    start=time.perf_counter()
    trace=simulate(m,c,name,overrides=overrides,record_fps=100)
    forces,axial=force_contributions(m,trace)
    stats=summarize(c,m,trace,forces,axial)
    stats['sim_seconds_per_wall_second']=c['simulation']['duration_s']/(time.perf_counter()-start)
    stats['controller']=Controller(m,c,name,overrides).p
    trace['axial_velocity']=axial
    for group,force in forces.items():trace['force_'+group]=force
    return trace,stats

def run(c,names=None,ablations=True):
    m=mujoco.MjModel.from_xml_path(str(ROOT/'fish.xml'))
    names=names or BEHAVIORS
    path=ROOT/'reports/behaviors.json'
    stats=json.loads(path.read_text()) if path.exists() else {}
    audit={}
    for name in names:
        trace,s=evaluate(c,m,name)
        np.savez_compressed(ROOT/'reports'/f'trajectory_{name}.npz',**trace)
        dump(ROOT/'reports'/f'trajectory_{name}.json',{'signature':signature(c),'behavior':name,'duration_s':c['simulation']['duration_s'],'controller':s['controller']})
        stats[name]=s;audit[name]={'both':s}
        print(name,'speed BL/s',round(s['signed_speed_bl_s'],6),'forces mN', {k:round(s['forces'][k]['mean_axial_mN'],6) for k in ['caudal','pectoral']},flush=True)
        if ablations and name!='body_undulation':
            for label,group in [('caudal_held_neutral','caudal'),('pectorals_held_neutral','pectoral')]:
                _,audit[name][label]=evaluate(c,m,name,{'disabled_groups':[group]})
                print(' ',label,round(audit[name][label]['signed_speed_bl_s'],6),flush=True)
    dump(path,stats)
    if ablations:
        dump(ROOT/'reports/propulsion_audit.json',{'signature':signature(c),'behaviors':audit,'method':'Same-state native MuJoCo ellipsoid force subtraction plus independent free-root simulations with selected position targets held neutral. Passive forces and every mesh remain present. Mean statistics use the final half of each run.'})
        plot(c,audit)
        write_report(c,audit)
    return audit

def plot(c,audit):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    names=[n for n in audit if n!='body_undulation']
    fig,axes=plt.subplots(1,2,figsize=(12,4),layout='constrained')
    x=np.arange(len(names));colors=['#208d9b','#cf7c43']
    for i,group in enumerate(['caudal','pectoral']):
        axes[0].bar(x+(i-.5)*.3,[audit[n]['both']['forces'][group]['mean_axial_mN'] for n in names],.3,label=group,color=colors[i])
    axes[0].set(ylabel='Mean fin force along head axis (mN)',title='Native fluid forces | + anterior, − posterior')
    for i,(key,label) in enumerate([('both','Both active'),('caudal_held_neutral','Tail held neutral'),('pectorals_held_neutral','Pectorals held neutral')]):
        axes[1].bar(x+(i-1)*.24,[audit[n][key]['signed_speed_bl_s'] for n in names],.24,label=label)
    axes[1].set(ylabel='Mean signed COM velocity (BL/s)',title='Independent free-root control ablations')
    for ax in axes:
        ax.set_xticks(x,names);ax.axhline(0,color='gray',lw=.6);ax.legend(fontsize=8);ax.grid(axis='y',alpha=.2)
    fig.savefig(ROOT/'reports/propulsion_contributions.png',dpi=180);plt.close(fig)

def write_report(c,audit):
    rows=[]
    for name,entry in audit.items():
        if name=='body_undulation':continue
        s=entry['both'];f=s['forces']
        rows.append(f"| {name} | {f['caudal']['mean_axial_mN']:+.5f} | {f['pectoral']['mean_axial_mN']:+.5f} | {s['signed_speed_bl_s']:+.5f} | {entry['caudal_held_neutral']['signed_speed_bl_s']:+.5f} | {entry['pectorals_held_neutral']['signed_speed_bl_s']:+.5f} |")
    text='''# Tail and pectoral contribution revision

The previous forward controller commanded only 1° at the caudal hinge and disabled the tail during hover. Independent caudal pitching generated backward force in this approximation. The revised forward/turning gait counter-rotates the caudal fin against the summed body-yaw wave. This lets rear-body translation and caudal rotation cooperate. The body amplitude envelope increases posteriorly within the existing seam-safe ROM. The caudal fin and every other exterior mesh remain rigid. The dorsal and anal fins remain single neutral meshes.

In videos 03–06 the chin's three position targets now stay at zero; active sensory scanning is reserved for the dedicated chin demonstration. The current force/trajectory measurements are re-simulated with this neutral chin command.

Pectoral rotation about the span (global local-body y axis) now uses the same sign on left and right, as required by sagittal reflection of an axial vector. Its sign reverses for backward strokes. Previously that component was mirrored incorrectly; it did not reliably reverse pectoral thrust. Backward uses an independent lateral tail stroke together with reverse-feathered pectorals. Hover keeps both groups active with approximately opposing mean forces; it is open-loop and drifts. Turning retains coordinated tail strokes plus body/tail bias and unequal pectoral amplitudes.

No joint range, mesh, actuator gain, inertia, fluid proxy or fluid coefficient was changed for these controller revisions. Native MuJoCo ellipsoid forces drive a free root; no trajectory prescription or added thrust is used. The base gaits use 3 Hz and 1.2 BL wavelength. The forward gait uses its own frequency, amplitude and caudal phase overrides from config.yaml. `carangiform.md` describes its current posterior-body envelope and improved propulsion. `forward_improvement.md` and `larger_tail.md` preserve the earlier speed and tail-amplitude comparisons. Historical `tuning.json`/`tuning.png` are retained and are no longer implicit overrides of config.yaml.

## Measured contributions

Native fluid forces are measured by subtracting each fin group's contribution at identical simulated positions and velocities. Statistics average the final half of a six-second run. Force is projected onto the head's anterior axis. Positive means forward; negative means backward. The other bodies, fins and chin also interact with the fluid, so these are fin-force comparisons, not percentages of swimming speed or biological efficiency.

Independent ablations hold the selected group's position targets at neutral; their passive geometry and fluid forces remain present. The other controller targets are unchanged. These experiments measure the effect of active strokes, rather than removing anatomy.

| Gait | Tail force (mN) | Pectoral force (mN) | Both active (BL/s) | Tail neutral (BL/s) | Pectorals neutral (BL/s) |
|---|---:|---:|---:|---:|---:|
'''+ '\n'.join(rows)+'''

![Native fin forces and control ablations](propulsion_contributions.png)

Complete traces, achieved joint ranges, displacement, yaw change, Reynolds number and per-group RMS forces are in `propulsion_audit.json`, `behaviors.json`, and `trajectory_*.npz`. The JSON beside each trajectory fingerprints the current MJCF, controller, joint definitions and simulation config. Rendering automatically re-simulates stale trajectories without tuning. Videos 03–06 use side, top and caudal-close-up views, with instantaneous measured axial COM velocity and actual joint angles.

## Evidence and limits

[Lannoo & Lannoo (1993), pp. 163–164](https://www.ikhebeenvraag.be/mediastorage/FSDocument/56/Lannoo-157.pdf) discusses carangiform movements in Gnathonemus, a semi-stiff body/peduncle, and lateral tail strokes during backward probing. This supports including active body/caudal motion. It does not establish the amplitudes, phase relationship or thrust shares used here; those remain engineering settings.

[MuJoCo's fluid documentation](https://mujoco.readthedocs.io/en/stable/computation/fluid.html) describes stateless ellipsoid approximations. This model lacks resolved wakes, fin flexibility and fluid coupling between separate surfaces. Speeds are low, and lateral/vertical drift remains. A positive measured tail force and a successful ablation establish contribution in this simulation, not quantitatively validated fish biomechanics. Hover is approximate balance, not feedback station keeping. Historical tuning results describe the previous controller.

Reproduce without optimization:

```sh
.venv/bin/python evaluate_behaviors.py
.venv/bin/python render_videos.py --only swimming
.venv/bin/python -m pytest tests -q
```

`run_pipeline.py` / `make all` now evaluates the configured gaits in phase 4 rather than replacing them with a new search winner.
'''
    report('propulsion_revision.md',text)
    report('phase4.md',text)

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--config');ap.add_argument('--only',nargs='+',choices=BEHAVIORS);ap.add_argument('--no-ablations',action='store_true');a=ap.parse_args()
    run(config(a.config),a.only,not a.no_ablations)
