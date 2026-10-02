"""Replot the saved posterior-body comparison without re-running simulations."""
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
trace = np.load(ROOT / 'reports/carangiform_comparison_traces.npz')
model = mujoco.MjModel.from_xml_path(str(ROOT / 'fish.xml'))
addresses = [model.jnt_qposadr[model.joint(f'j_body_{i:02d}_yaw').id] for i in range(1, 14)]
time = trace['time']
steady = time >= 3
window = (time >= 3) & (time <= 3.5)
fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.1), layout='constrained')
for key, label, color in [('previous', 'Previous larger-tail gait', '#426b9b'),
                          ('carangiform', 'Carangiform-style gait', '#c26c25')]:
    angles = np.rad2deg(trace[key + '_qpos'][steady][:, addresses])
    axes[0].plot(np.arange(1, 14), np.ptp(angles, axis=0), 'o-', label=label, color=color, lw=2)
    axes[1].plot(time[window], trace[key + '_tail_base_head'][window, 1] * 1000, color=color, lw=2)
    position = trace[key + '_position']
    axes[2].plot(time, (position[:, 0] - position[0, 0]) * 1000, color=color, lw=2)
axes[0].set(title='Bending concentrated before the peduncle',
            xlabel='Body joint (anterior to posterior)', ylabel='Actual yaw peak-to-peak (degrees)',
            xticks=[1, 3, 5, 7, 9, 11, 13], ylim=(0, 1.5))
axes[0].axvspan(6.5, 11.5, color='#c26c25', alpha=.06)
axes[0].axvspan(11.5, 13.5, color='#426b9b', alpha=.06)
axes[0].legend(loc='upper left', fontsize=8)
axes[1].set(title='Tail base: 18% wider lateral sweep', xlabel='Time (s)',
            ylabel='Lateral position relative to head (mm)')
axes[2].set(title='Forward travel: 8.3% farther', xlabel='Time (s)',
            ylabel='COM displacement along world x (mm)')
for ax in axes:
    ax.grid(alpha=.22)
    ax.spines[['top', 'right']].set_visible(False)
fig.suptitle('Posterior body actuation improves forward propulsion within the existing joint limits', fontsize=13)
fig.savefig(ROOT / 'reports/carangiform_comparison.png', dpi=160)
plt.close(fig)
