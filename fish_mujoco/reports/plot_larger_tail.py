"""Replot the saved larger-tail comparison, without running any optimization."""
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
trace = np.load(ROOT / 'reports/larger_tail_comparison_traces.npz')
model = mujoco.MjModel.from_xml_path(str(ROOT / 'fish.xml'))
joint = model.joint('j_fin_caudal_0')
address = int(model.jnt_qposadr[joint.id])
time = trace['time']
window = (time >= 3) & (time <= 3.5)
fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.0), layout='constrained')
for key, label, color in [('previous', 'Previous stroke', '#426b9b'),
                          ('larger', 'Larger stroke (current)', '#c26c25')]:
    axes[0].plot(time[window], np.rad2deg(trace[key + '_qpos'][window, address]),
                 label=label, color=color, lw=2)
    axes[1].plot(time[window], trace[key + '_tip_y_mm'][window], color=color, lw=2)
    position = trace[key + '_position']
    axes[2].plot(time, (position[:, 0] - position[0, 0]) * 1000, color=color, lw=2)
axes[0].set(title='Caudal hinge: 32% wider sweep', ylabel='Actual angle (degrees)')
axes[1].set(title='Tail tip: about twice the excursion', ylabel='Lateral position relative to head (mm)')
axes[2].set(title='Forward travel: 4.6% lower', ylabel='COM displacement along world x (mm)')
axes[0].legend(loc='lower left', fontsize=8)
for ax in axes:
    ax.set_xlabel('Time (s)')
    ax.grid(alpha=.22)
    ax.spines[['top', 'right']].set_visible(False)
fig.suptitle('A larger tail beat remains propulsive, but does not increase forward speed', fontsize=13)
fig.savefig(ROOT / 'reports/larger_tail_comparison.png', dpi=160)
plt.close(fig)
