# Active chin mobility

The three chin hinges share the pivot in `assets/segments.json`. See [the current socket revision](chin_socket.md) for its placement beneath the mouth. Current limits: yaw ±45°, pitch ±45°, roll ±30°. Body seam restrictions remain enforced. The ball/socket attachment enforces socket coverage at the full requested chin range. This rendering step uses the generated geometry and runs no swimming parameter optimization.

The scan commands 85% of each limit at 1 Hz. Six seconds of free-root MuJoCo simulation completed without numerical warnings or nonfinite states. Actual ranges after the initial ramp: yaw: -38.19 to +38.22 degrees; pitch: -38.20 to +38.19 degrees; roll: -25.44 to +25.45 degrees. These are joint angles relative to the head, not root movement. Other joint targets stay at zero in this dedicated demonstration; fluid forces and root motion remain active.

The chin is fully rigid and contains a rounded ball seated in a concave head socket. No visual skin or flexible collar is loaded. Full 3-axis ranges are retained. See [socket geometry and checks](chin_socket.md).

Maximum sampled head-socket rim opening: **0 mm**. This coverage check is enforced across all individual and combined test poses; the exposed ball/shaft junction is not a mating seam. Compiled-model checks are in `chin_socket_validation.json`.

[Amey-Özel et al. (2015)](https://pubmed.ncbi.nlm.nih.gov/25388854/) documents a highly mobile sensory appendage with motor innervation. [von der Emde et al. (2008)](https://pubmed.ncbi.nlm.nih.gov/18992334/) describes lateral searching and directed probing. The numerical limits and scan frequency here are adjustable engineering assumptions, not values measured in those studies. This is an open-loop scanning demonstration without environmental sensing or target feedback.

`08_chin_3dof.mp4` sweeps each axis kinematically. `09_chin_scan.mp4` shows actual actuation at 1920×1080, 60 fps. Earlier swimming videos retain their prior chin motion until explicitly regenerated; their existing tuning parameters are unchanged by the focused update.

![Actuated scan, top views](chin_scan.png)

![Individual range limits](chin_dofs.png)
