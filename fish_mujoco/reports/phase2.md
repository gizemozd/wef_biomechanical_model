# Phase 2 — textures

Source diffuse atlas: 4096×4096; derived atlas: 4096×4224 with a separate 128-pixel strip for flat interior-face colors. Original image pixels are unchanged. UV v coordinates receive an affine padding adjustment; all exterior face corners are transferred by source-triangle barycentric interpolation. Per-face-corner `vt` indices preserve UV islands independently of shared geometric vertices. Each segment's interior swatch is the average of nearby sampled diffuse colors. Newly modeled exterior ball faces instead receive nearest-triangle barycentric UV projection; their texture stays rigid during movement.

OBJ and MTL assets are self-contained under `assets/`. MuJoCo uses one diffuse material on all segment visuals. Original normal, specular, roughness, glossiness and opacity maps are archived; classic MuJoCo does not reproduce the full PBR material or alpha-cutout fins. Textured rest-pose comparison is generated during rendering as `texture_continuity.png`.
