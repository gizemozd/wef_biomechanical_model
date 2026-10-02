"""Document the reference-inspired gait from actual simulated motion, without tuning."""
import argparse
import json
import numpy as np
import mujoco
from common import ROOT, config, dump, report

SOURCE = 'https://www.youtube.com/shorts/Vf1ye8vfjK0'

def motion(model, trace, body_count):
    d=mujoco.MjData(model);head=model.body('body_00_head').id;tail=model.body('fin_caudal_0').id
    local=[]
    for q in trace['qpos']:
        d.qpos[:]=q;mujoco.mj_forward(model,d)
        local.append((d.xpos[tail]-d.xpos[head])@d.xmat[head].reshape(3,3))
    body=[model.jnt_qposadr[model.joint(f'j_body_{i:02d}_yaw').id] for i in range(1,body_count)]
    caudal=model.jnt_qposadr[model.joint('j_fin_caudal_0').id]
    return {'time':trace['time'],'position':trace['position'],
            'tail_base_head_m':np.asarray(local),
            'tail_angle_deg':np.rad2deg(trace['qpos'][:,caudal]),
            'body_yaw_deg':np.rad2deg(trace['qpos'][:,body])}

def run(c):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    m=mujoco.MjModel.from_xml_path(str(ROOT/'fish.xml'))
    baseline=np.load(ROOT/'references/pre_reference_forward_motion.npz')
    current=motion(m,np.load(ROOT/'reports/trajectory_forward.npz'),c['segmentation']['N_body'])
    old=json.loads((ROOT/'reports/tail_increment.json').read_text())['increased']
    audit=json.loads((ROOT/'reports/propulsion_audit.json').read_text())
    stats=json.loads((ROOT/'reports/behaviors.json').read_text())['forward']
    params=stats['controller'];steady=current['time']>=current['time'][-1]/2
    sweep=float(np.ptp(current['tail_base_head_m'][steady,1])*1000)
    tail_sweep=float(np.ptp(current['tail_angle_deg'][steady]))
    force=stats['forces'];share=force['caudal']['mean_axial_mN']/sum(force[g]['mean_axial_mN'] for g in ['caudal','pectoral'])
    single=json.loads((ROOT/'reports/seam_metrics.json').read_text())
    compound=json.loads((ROOT/'reports/body_compound_metrics.json').read_text())
    max_gap=max(r['gap_fraction'] for r in single+compound)
    comparison={'source_url':SOURCE,'baseline_commit':'d38c5e6c203832fa6b8e739825f6e4954f987dda',
                'baseline':old,'current':stats,'current_tail_base_lateral_sweep_mm':sweep,
                'tail_and_pectoral_force_share':share,'max_body_seam_gap_fraction':max_gap,
                'source_observation':'Qualitative frame inspection; no calibrated 3D reconstruction or measured source speed.'}
    dump(ROOT/'reports/reference_swim_comparison.json',comparison)
    np.savez_compressed(ROOT/'reports/reference_swim_motion.npz',**current)
    fig,axes=plt.subplots(2,2,figsize=(11.5,7),layout='constrained')
    for data,label,color in [(baseline,'Previous: 6 Hz','#507698'),(current,f'Reference-inspired: {params["frequency_hz"]:g} Hz','#bf682e')]:
        t=data['time'];late=t>=t[-1]/2;window=(t>=3)&(t<=4)
        axes[0,0].plot(np.arange(1,data['body_yaw_deg'].shape[1]+1),np.ptp(data['body_yaw_deg'][late],axis=0),'o-',lw=2,color=color,label=label)
        axes[0,1].plot(t[window],data['tail_angle_deg'][window],color=color,lw=1.7)
        axes[1,0].plot(t[window],data['tail_base_head_m'][window,1]*1000,color=color,lw=1.7)
        axes[1,1].plot(t,(data['position'][:,0]-data['position'][0,0])*1000,color=color,lw=2)
    axes[0,0].set(title='More bending in the posterior trunk',xlabel='Body joint, anterior to posterior',ylabel='Achieved yaw peak-to-peak (degrees)')
    axes[0,0].axvspan(6.5,11.5,color='#bf682e',alpha=.07);axes[0,0].legend(fontsize=9)
    axes[0,1].set(title='Slower, broader caudal strokes',xlabel='Time (s)',ylabel='Achieved caudal hinge angle (degrees)')
    axes[1,0].set(title='Greater lateral translation of the tail base',xlabel='Time (s)',ylabel='Lateral position relative to head (mm)')
    axes[1,1].set(title='Free-root forward travel',xlabel='Time (s)',ylabel='COM displacement along world x (mm)')
    for ax in axes.flat:ax.grid(alpha=.2);ax.spines[['top','right']].set_visible(False)
    fig.suptitle('Forward swimming: qualitative motion approximation, not a calibrated reconstruction',fontsize=12)
    fig.savefig(ROOT/'reports/reference_swim_comparison.png',dpi=170);plt.close(fig)
    ablation=audit['behaviors']['forward']
    rows=[]
    for name,item in [('Previous fast gait',old),('Reference-inspired gait',stats)]:
        rows.append(f'| {name} | {item["controller"]["frequency_hz"]:g} | {item["joint_motion"]["j_fin_caudal_0"]["peak_to_peak_deg"]:.2f} | {item["displacement_m"][0]*1000:.2f} | {item["signed_speed_bl_s"]:.5f} |')
    report('reference_swim.md',f'''# Forward swimming from the video reference

The requested priority is body and tail motion during forward swimming. The model now combines slower caudal strokes, stronger posterior-trunk bending and a separate pectoral cadence. The root moves under native MuJoCo fluid forces. No swimming path or extra thrust is prescribed.

## Reference and interpretation

Reference: [AquaVerse, “Elephant Nose Fish: The Mysterious Trunked Swimmer!”]({SOURCE}), uploaded 2024-10-31, inspected 2026-10-02. The inspected video is 720×1280 at 30 fps, duration 21.667 s. Frame inspection covered the whole clip, with denser sampling of 0–3.7 s and 4.2–7.6 s. The former provides a front-quarter view of caudal strokes; the latter shows a side view before a nose-down transition. Later bottom probing is excluded from the requested behavior. The source clip and extracted frames are not redistributed in this repository.

| Visible feature | Model approximation |
|---|---|
| Tail sweeps are individually visible; the front of the body stays relatively quiet | {params['frequency_hz']:g} Hz posterior body wave, low anterior command envelope |
| Rear body translates as the caudal fin rotates | Wider posterior yaw ROM with coordinated caudal counter-bending |
| Pectorals remain active without an obvious fixed phase relationship to the tail | Independent {params.get('pec_frequency_hz',params['frequency_hz']):g} Hz pectoral cycle |
| Stroke strength varies through the footage | Smooth {100*params.get('body_wave_modulation_depth',0):g}% body-wave amplitude modulation at {params.get('body_wave_modulation_hz',0):g} Hz |

The frequencies, modulation, phase and angular amplitudes are engineering choices guided by qualitative inspection. Perspective, partial occlusion and edits prevent a reliable calibrated 3D pose or source swimming-speed estimate. This is not a frame-by-frame reconstruction, a fitted tailbeat measurement, or a claim that the source swims at the simulated speed.

## Geometry and actuation

The existing five posterior joints (`j_body_07_yaw` through `j_body_11_yaw`) now permit ±1.432° instead of ±0.716°. Anterior and peduncle yaw remain ±0.716°, and pitch remains ±0.5°. These effective ranges are below the requested regional caps and come from `joints.seam_displacement_fraction_by_region`. Changing ranges uses the same rigid meshes, textures, chin attachment, mass, fluid proxies and actuator gains.

There are {len(single)} individual-axis seam poses and {len(compound)} combined yaw/pitch corners. The largest sampled gap is **{max_gap:.3%} of local body height**, below the unchanged 3% bent limit. The largest rest gap is {max(r['gap_fraction'] for r in single if r['angle_deg']==0):.3%}, below 1%. These finite checks do not prove arbitrary surface tangency. No flexible cover or skin is introduced. Whole dorsal/anal fins remain neutral; their long rigid bases cannot follow a deforming body perfectly. Chin commands remain zero in videos 03–06; the fixed ground grid remains visible.

The selected forward gait uses wavelength {params['wavelength_bl']:g} BL, body fraction {params['body_amplitude_fraction']:g}, caudal counter-bending gain {params['caudal_counterbend_gain']:g}, phase {params['caudal_phase_deg']:g}°, and command cap ±{params['caudal_amplitude_deg']:g}°. Pectoral amplitude is {params['pec_amplitude_deg']:g}° before the shared {params['amplitude_deg']:g}/22 scaling, with feathering fraction {params['pec_pitch_fraction']:g}. Defaults for the other behaviors retain their previous cadences; their trajectories and videos are regenerated because the physical posterior ranges changed.

## Measured outcome

| Gait | Body/tail Hz | Caudal sweep, peak-to-peak (°) | Forward travel in 6 s (mm) | Mean axial speed, final 3 s (BL/s) |
|---|---:|---:|---:|---:|
'''+ '\n'.join(rows)+f'''

Tail-base lateral sweep relative to the head increases from **{old['tail_base_lateral_sweep_mm']:.2f} to {sweep:.2f} mm**. Actual caudal hinge sweep is **{tail_sweep:.2f}°**. Broader, slower strokes approximate the visible motion better, but do not outperform the previous fast gait in forward travel. The forward distance changes by {100*(stats['displacement_m'][0]/old['displacement_m'][0]-1):+.1f}%.

The tail contributes **{force['caudal']['mean_axial_mN']:+.4f} mN** and paired pectorals **{force['pectoral']['mean_axial_mN']:+.4f} mN** mean anterior force over the final half of the run. The tail supplies {100*share:.1f}% of those two groups' combined axial force, not a percentage of all forces or biological efficiency. Holding the caudal target neutral reduces speed to **{ablation['caudal_held_neutral']['signed_speed_bl_s']:.5f} BL/s**; holding the pectorals neutral gives **{ablation['pectorals_held_neutral']['signed_speed_bl_s']:.5f} BL/s**. Both groups therefore contribute in independent free-root runs. Reynolds number based on body length and mean axial speed is approximately {stats['reynolds']:.0f}.

![Achieved body/tail motion and free-root displacement](reference_swim_comparison.png)

![Forward swimming render](behavior_forward.png)

## Focused trials and limits

`reference_swim_trials.json` records 13 focused controller trials, separating the old and expanded posterior ranges. Slowing the earlier rigid-tail stroke without enough tail-base translation produced drag. Expanded posterior ranges let the tail translate farther while counter-rotating, producing positive forward force at the slower cadence. The selected gain avoids the slight caudal-limit overshoots of larger-gain trials. No general tuning grid, mesh recutting, fin-proxy resizing or fluid-coefficient search was run.

[MuJoCo's ellipsoid fluid model](https://mujoco.readthedocs.io/en/stable/computation/fluid.html) uses local stateless force approximations. It does not resolve wakes, flexible fins or hydrodynamic interaction between surfaces. Realistic-looking kinematics do not establish realistic thrust or efficiency. The model retains passive forebody movement, modest upward/lateral drift and open-loop control. Exact 3D agreement with the reference would require calibrated multi-view measurements and a richer body/fin model.

## Reproduce without optimization

```sh
.venv/bin/python segment_mesh.py --body-rom-only
.venv/bin/python build_model.py
.venv/bin/python evaluate_behaviors.py
.venv/bin/python render_videos.py --only swimming
.venv/bin/python render_videos.py --only body_undulation
.venv/bin/python render_videos.py --only rom
.venv/bin/python -m pytest tests -q
```

`make all` applies the same regional range checks during segmentation and reproduces the configured motion and videos from raw assets. `reference_swim.py` regenerates this report and comparison from the archived pre-reference measurements and current simulated trajectory; it is called after the phase-4 audit. Previous gait reports remain historical records.
''')

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--config');a=ap.parse_args();run(config(a.config))
