> Historical planning document. The implemented pipeline and current instructions are in [fish_mujoco/README.md](../fish_mujoco/README.md).

# Rigging the Elephant Nose Fish for MuJoCo (SimFish)

## Progress checklist

**Setup**
- [ ] Open Blender 4.x; enable **Rigify** and confirm **Import-Export: OBJ / FBX** are on
- [ ] `File → Import → Wavefront (.obj)` → `Elephant_Nose_Fish_obj/Elephant_Nose_Fish.obj`
- [ ] Select mesh → `Ctrl+A → All Transforms`; confirm +Y forward, +Z up (rotate + re-apply if not)
- [ ] In Shading workspace, wire up `Normal.png`, `Roughness.png` (Non-Color), and `Opacity.png` (Alpha Clip/Blend) on `elephant_nose_fish_mat_all`
- [ ] If the head is at -Y in your import, flip `Y_HEAD` / `Y_TAIL` at the top of `blender/setup_rig.py`

**Armature**
- [ ] Scripting workspace → open `blender/setup_rig.py` → Run Script (creates `fish_rig` + `joint_cutters` collection)
- [ ] Edit Mode on `fish_rig`: drag every bone head/tail onto true anatomical positions (spine along midline, pec shoulders at fin bases, schnauzenorgan under the mouth, jaw on lower mouth)
- [ ] Fix bone rolls: select all bones → `Armature → Bone Roll → Recalculate Roll → Global +Z Axis`
- [ ] Pose Mode: sanity-rotate `spine_03`, `jaw`, `schnauz_02`, `pec_L_1` — check axes behave as expected

**Joint spheres**
- [ ] Resize each `jcut_*` sphere so it matches ≈1.1× local body cross-section (spine ~1.05×, pec shoulder / caudal base slightly larger)
- [ ] Reposition any sphere whose bone head was moved in the previous step (keep sphere center on the joint)

**Skinning (visual deformation)**
- [ ] Select mesh → Shift-select `fish_rig` → `Ctrl+P → Armature Deform → With Automatic Weights`
- [ ] Weight-paint cleanup: pec fins (100% to fin bones), jaw (only lower mouth), schnauzenorgan (trunk gradient), spine (smooth)
- [ ] Test in Pose Mode: rotate `spine_03` ±30° → no tearing at head/fins

**Rigid-body meshes (spherical cuts)**
- [ ] Duplicate body mesh once per bone segment, rename `body_<bone_name>`
- [ ] For each duplicate, apply Boolean Difference against the other-segment cutters (half-space cube **minus** `jcut_` sphere) so each segment ends in a spherical cap at every joint boundary
- [ ] `Mesh → Clean Up → Merge by Distance (1e-4)`; recompute normals
- [ ] Seam test: place two adjacent segments together, rotate ±45° — no gap, no interpenetration outside the sphere
- [ ] Export each `body_*` to `mujoco/assets/body_<bone_name>.obj` (origin at bone head)

**MuJoCo export**
- [ ] Run `obj2mjcf` on each per-bone OBJ (or write a Blender → MJCF exporter walking `fish_rig` pose bones + their custom props)
- [ ] Hand-write `mujoco/fish.xml`: nested `<body>` tree mirroring the bone hierarchy, one `<geom mesh="...">` and `<joint>` per bone, freejoint on `root`, actuators per articulated joint
- [ ] Load in viewer: `python -m mujoco.viewer --mjcf mujoco/fish.xml`
- [ ] Passive-drop test → stable fall; actuator sine test on `spine_03` → clean undulation

## Context

`code/3dmodeling/exp3d/` contains a purchased/downloaded **Elephant Nose Fish** (*Gnathonemus petersii*) model distributed in many 3D formats (.obj, .fbx, .3ds, .stl, .c4d, .lwo, .mb, .max) plus a PBR texture set (diffuse, normal, specular, glossiness, roughness, opacity). There is no code here — it is an asset pack.

The goal is to turn the static mesh into an **articulated, MuJoCo-ready fish** for SimFish (physics/biomechanics simulation). Articulated parts required: spine + caudal tail, pectoral fins (pair), chin/Schnauzenorgan (the signature movable "trunk" of this species), and jaw. Dorsal/anal fins stay passive (rigid with the surrounding body segment).

Because MuJoCo uses rigid-body chains with hinge joints, the rig's purpose is two-fold:
1. Define a bone/joint hierarchy that maps cleanly onto MJCF `<body>`/`<joint>` elements.
2. Act as a skinning source so the visual mesh can deform with the physics chain via MuJoCo `<skin>`.

## Inputs / assets to use

- **Mesh**: `Elephant_Nose_Fish_obj/Elephant_Nose_Fish.obj` (clean, universal, already extracted). Use `.fbx` only if the OBJ import misses groups.
- **Materials**: `Elephant_Nose_Fish_obj/Elephant_Nose_Fish.mtl` — defines two materials: `sclera` (eyes, opaque-black/transparent) and `elephant_nose_fish_mat_all` (body, with diffuse + specular maps already wired). **Keep the OBJ + MTL + textures in the same folder so the `map_Kd`/`map_Ks` paths resolve on import.**
- **Textures**: `Elephant_Nose_Fish_textures/` — `Diffuse`, `Normal`, `Specular`, `Glossiness`, `Roughness`, `Opacity` PNGs. MTL only references Diffuse and Specular; hook the rest manually in Blender's shader editor.

## Deliverables

1. A Blender file `fish_rigged.blend` with:
   - Imported mesh, PBR materials fully hooked.
   - Armature `fish_rig` with the bone hierarchy defined below.
   - Weight-painted mesh bound to the armature (for the skinned-visual variant).
   - **Per-segment rigid meshes cut with spheres at each joint** (see "Spherical joint cuts" below).
   - Custom bone properties documenting joint axis and range (consumed later by the MJCF exporter).
2. Exported MuJoCo assets:
   - `fish.xml` (MJCF scene with a `<worldbody>` tree mirroring the bone hierarchy).
   - Per-segment meshes as `.obj` or `.stl` in `assets/`, one file per rigid body, with spherical end-caps at joints.
3. **`README.md` at the project root** — this file. Single source of truth for the plan and the joint table.

## Bone / joint hierarchy (MuJoCo-friendly)

Naming: snake_case, unique, suffix `_L`/`_R` for pairs. Keep bone roll consistent (Y-axis down the bone) so joint axes are predictable.

```
root (freejoint; site at center of mass near gills)
├── spine_01  (hinge, z-axis, ±20°)   — just behind head
├── spine_02  (hinge, z-axis, ±20°)
├── spine_03  (hinge, z-axis, ±25°)
├── spine_04  (hinge, z-axis, ±25°)
├── spine_05  (hinge, z-axis, ±30°)
├── spine_06  (hinge, z-axis, ±30°)   — peduncle
│   └── caudal_01 (hinge, z-axis, ±35°)
│       └── caudal_02 (hinge, z-axis, ±40°)   — tail tip
├── head       (fixed to root; no joint)
│   ├── jaw          (hinge, x-axis, 0°..25°)   — pitch-down opens mouth
│   ├── schnauz_01   (hinge, x-axis, ±30°)      — chin base
│   │   └── schnauz_02 (hinge, x-axis, ±40°)    — chin tip
│   ├── pec_L_1 (ball or 2×hinge, ±45° yaw / ±30° pitch)
│   │   └── pec_L_2 (hinge, ±30°)               — fin tip flex (optional)
│   └── pec_R_1 (ball or 2×hinge, ±45° / ±30°)
│       └── pec_R_2 (hinge, ±30°)
```

### Joint table

| Bone           | Parent     | Joint type | Axis | Range (deg)  | Notes                          |
|----------------|------------|------------|------|--------------|--------------------------------|
| `root`         | world      | free       | —    | —            | 6-DOF base                      |
| `spine_01`     | root       | hinge      | z    | -20 .. +20   | just behind head                |
| `spine_02`     | spine_01   | hinge      | z    | -20 .. +20   |                                 |
| `spine_03`     | spine_02   | hinge      | z    | -25 .. +25   |                                 |
| `spine_04`     | spine_03   | hinge      | z    | -25 .. +25   |                                 |
| `spine_05`     | spine_04   | hinge      | z    | -30 .. +30   |                                 |
| `spine_06`     | spine_05   | hinge      | z    | -30 .. +30   | peduncle                        |
| `caudal_01`    | spine_06   | hinge      | z    | -35 .. +35   | tail base                       |
| `caudal_02`    | caudal_01  | hinge      | z    | -40 .. +40   | tail tip                        |
| `head`         | root       | fixed      | —    | —            | rigidly parented to root        |
| `jaw`          | head       | hinge      | x    | 0 .. +25     | pitch-down opens mouth          |
| `schnauz_01`   | head       | hinge      | x    | -30 .. +30   | chin base (trunk)               |
| `schnauz_02`   | schnauz_01 | hinge      | x    | -40 .. +40   | chin tip                        |
| `pec_L_1`      | head       | ball       | —    | ±45 / ±30    | shoulder-like                   |
| `pec_L_2`      | pec_L_1    | hinge      | y    | -30 .. +30   | fin tip flex (optional)         |
| `pec_R_1`      | head       | ball       | —    | ±45 / ±30    | mirror of L                     |
| `pec_R_2`      | pec_R_1    | hinge      | y    | -30 .. +30   | mirror of L                     |

Rationale:
- Spine yaw (z-axis hinge) produces the body undulation that propels the fish — this is the dominant DOF for swimming.
- Caudal fin segments amplify the tail beat; two segments give enough whip without over-parameterizing.
- Pectoral fins use a ball joint (or two stacked hinges) because real pec fins rotate in a hemisphere.
- Schnauzenorgan is a forward-pointing chin; a pitch hinge (x-axis) matches its dominant foraging motion.
- Jaw is a single pitch hinge.

Ranges above are starting guesses; tune against real fish footage.

## Step-by-step in Blender

**Pre-flight**
1. Blender 4.x (any recent). Enable **Rigify** (not required, but handy) and **Import-Export: FBX** (on by default).
2. Install the **Phobos** addon (for URDF) only if you later want a second export target — not needed for MuJoCo.

**Import + materials**
3. `File → Import → Wavefront (.obj)` → select `Elephant_Nose_Fish_obj/Elephant_Nose_Fish.obj`. Expect two mesh objects (body + sclera).
4. In **Shading** workspace, on `elephant_nose_fish_mat_all`: plug `Normal.png` into a Normal Map node → Principled BSDF Normal; plug `Roughness.png` into Roughness (set colorspace to **Non-Color**); plug `Opacity.png` into Alpha (Non-Color) and set material Blend Mode to **Alpha Clip** or **Alpha Blend** for the fins.
5. Apply scale/rotation: select mesh → `Ctrl+A → All Transforms`. Confirm +Y forward, +Z up. If not, rotate and re-apply.

**Armature**

A starter Blender Python script at `blender/setup_rig.py` builds the armature, custom properties, and placeholder joint spheres from the joint table above. Run it via `Scripting` workspace → `Open` → Run Script. Then refine positions in Edit Mode.

6. (Or manually) `Shift+A → Armature → Single Bone` at the fish center. Enter Edit Mode (`Tab`). Set display to **Stick** or **B-Bone** for clarity.
7. Build the chain above. Fastest path: extrude (`E`) along the body from head to tail tip for spine + caudal. Separately add bones for head, jaw, schnauz chain, and the two pec fins (mirror with `Shift+E` or name one side `_L` then use `Armature → Symmetrize`).
8. For each bone, set **Bone Roll** so local Y points tip-ward and local Z is vertical (world up). This makes "hinge on z" the swimming DOF without surprises.
9. In Pose Mode, add **Limit Rotation** constraints matching the ranges in the hierarchy. These are only for Blender-side posing sanity; real limits go into MJCF.

**Custom properties for MJCF export**
10. For each pose bone, add custom props: `joint_type` (`hinge`/`ball`/`free`/`fixed`), `joint_axis` (`x`/`y`/`z`), `range_lo`, `range_hi` (degrees). The starter script sets these automatically.

**Skinning**
11. Select mesh, then shift-select armature, `Ctrl+P → Armature Deform → With Automatic Weights`.
12. Weight-paint cleanup (mandatory for fins and jaw — automatic weights will bleed):
    - Pectoral fin verts should weight 100% to their fin bones.
    - Jaw: only lower-mouth verts weighted to `jaw`.
    - Schnauzenorgan: the chin trunk verts weighted progressively along `schnauz_01`/`schnauz_02`.
    - Smooth spine weights along the body — no hard seams.
13. Test deformation in Pose Mode. Pose the spine into an S-shape; confirm no collapsing around the caudal peduncle.

**Split-mesh variant with spherical joint cuts (needed for rigid-body MuJoCo)**

The naive way — separate the mesh by vertex groups along flat cross-sections — produces planar cuts. When neighboring segments rotate, those planar edges pivot around a single edge and expose a triangular gap on one side (with interpenetration on the other). To avoid this, cut each joint along a **sphere centered at the joint**: the adjacent segments then have matching spherical end surfaces, and since both surfaces share the same center, they stay in contact through any rotation — ball-and-socket, no visible seam.

14. **Place a joint-sphere at every bone head.** For each bone that has a parent (every bone except `root`/`head`), add a UV sphere (32 segments, 16 rings) at the bone's **head** position. Radius: measure the body's local cross-section at that point and multiply by ≈1.1. Name it `jcut_<bone_name>` and put it on a hidden helper collection. For pec fin shoulders, use the fin base width.
15. **Produce one rigid-body mesh per bone.** Duplicate the full body mesh once per bone segment (`body_<bone_name>`). For each duplicate, apply Boolean modifiers to isolate that bone's region while keeping spherical interfaces:
    - For every joint-sphere *not* on this segment's own boundaries, apply `Boolean: Difference` with that sphere → removes material belonging to the far segment.
    - For every joint-sphere *on* this segment's boundary with a neighbor, the cut is achieved by a combined plane + sphere cutter: use a large flat cube positioned so its face passes through the joint, **Boolean Union** that cube with the joint-sphere to form the cutter, then Boolean Difference against the cutter. The resulting segment ends in a spherical (not planar) surface at every joint boundary.
    - Equivalent intuition: segment A's cutter at joint J = "the far half-space past J" **minus** "the joint-sphere at J". Subtracting this cutter from the mesh keeps the segment's own side plus the full sphere volume on A's side of the joint — which is exactly the spherical cap.
16. **Verify the cuts.** Place two adjacent segment meshes together in pose, rotate one ±45° around the joint center. You should see no gap open up and no interpenetration outside the sphere volume. If a gap opens: the sphere radius is too small; if segments visibly intersect beyond the sphere: the sphere is too large relative to the joint travel.
17. **Export.** Export each `body_<bone_name>` as a separate `.obj` (or `.stl`) into `mujoco/assets/`. Apply all transforms first; origin of each mesh should be at the corresponding bone's head so MJCF `pos`/`quat` math is trivial.
18. **Keep the skinned mesh** too (the original, armature-bound, un-cut mesh) — useful as a visual `<skin>` layer over the rigid bodies, or as a fallback render asset.

## Export to MuJoCo

Two options, pick one:

**A. `obj2mjcf` + hand-written hierarchy (recommended; maximum control).**
- Use Kevin Zakka's `obj2mjcf` to convert each per-bone `.obj` into MuJoCo's mesh format + generate a stub `<asset>` block.
- Hand-write `fish.xml`: mirror the bone hierarchy as nested `<body>` elements; each `<body>` carries a `<geom mesh="..."/>` and a `<joint>` with the axis/range from the custom props.
- Add actuators (`<position>` or `<general>`) keyed to each named joint.

**B. Blender → MJCF exporter script.**
- Write (or reuse) a small Python add-on that walks the armature, reads the custom props, emits MJCF, and exports per-bone meshes. A starting point: iterate `bpy.data.armatures[0].bones` in pre-order, emit `<body pos="..." quat="...">` with `<joint>` and `<geom mesh="...">`.
- Faster to iterate; less flexible for MuJoCo-specific features (contact pairs, tendons, sites).

For either path, add:
- A **freejoint** on the root body (so the fish floats in water).
- A MuJoCo **fluid** setup: `<option density="1000" viscosity="0.001">` in water, or use the `flex`/`fluid` extensions for smoothed-particle-hydro if available.
- Inertial properties: either set `<inertial>` explicitly per body or let MuJoCo infer from mesh + density.
- Actuators: one per articulated joint. For a swimming controller you'll likely want position actuators with low kp for compliant behavior.

## Critical files / paths

- Source mesh: `Elephant_Nose_Fish_obj/Elephant_Nose_Fish.obj`
- Source materials: `Elephant_Nose_Fish_obj/Elephant_Nose_Fish.mtl`
- Textures dir: `Elephant_Nose_Fish_textures/`
- Starter Blender script: `blender/setup_rig.py`
- New outputs (create):
  - `blender/fish_rigged.blend`
  - `mujoco/fish.xml`
  - `mujoco/assets/body_*.obj` — one per rigid body, with spherical end caps.

## Verification

1. **Blender posing test (skinned mesh)**: in Pose Mode, rotate `spine_03` ±30° — body should bend smoothly without mesh tearing at fins or head. Rotate `jaw` — only lower jaw moves. Rotate `schnauz_02` — chin tip curls.
2. **Spherical-cut seam test (rigid meshes)**: in Object Mode, place two adjacent `body_*` segments together at their shared joint, rotate one ±45° around the joint center. No visible gap, no interpenetration outside the sphere. Repeat at every joint.
3. **MJCF load test**: `python -m mujoco.viewer --mjcf mujoco/fish.xml`. The fish should appear rigid-chain-correct (bodies connected, no disjoint parts).
4. **Passive drop test**: give the root body gravity; fish should fall and bend naturally at the hinges without exploding — any explosion means bad inertia or joint frames.
5. **Actuator sanity**: drive `spine_03` actuator with a sine wave (1–2 Hz); the whole posterior half should undulate, and joint seams should stay clean throughout the motion.
6. **Swimming test** (stretch): in water fluid, apply alternating sine to spine joints with a phase offset along the chain; the fish should develop forward velocity — the canonical validation for a fish swimmer.

## Risks / things to watch

- **Axis conventions differ**: Blender is +Z up, +Y forward by default; MuJoCo is also +Z up but the default "forward" can vary. Apply all transforms before export and verify the first imported MJCF visually.
- **Automatic weights underperform at thin fins**: budget real time for weight painting pec and caudal fins (skinned-visual variant only).
- **Sphere radius at joints is a tuning knob.** Too small → gaps on rotation. Too large → bulky "beaded" look along the spine and wasted sphere volume poking through the surface. Start at 1.1× local cross-section and iterate per joint. Spine joints can go slightly smaller (~1.05×) because their travel per segment is modest; the pec shoulder and caudal base usually need larger spheres because their travel is much wider.
- **Boolean artifacts**: Blender booleans produce messy topology on thin/overlapping geometry. After all cuts, run `Mesh → Clean Up → Merge by Distance` (1e-4) and check normals. Booleans may occasionally fail on non-manifold areas — fix manifold issues on the source mesh first.
- **Over-segmenting the spine** gives prettier undulation but a harder-to-control sim *and* more sphere cuts. Start at 6 spine + 2 caudal; increase only if needed.
- **Eyes (`sclera` material)** are a separate mesh object — parent rigidly to `head`, don't skin, don't cut.
- The imported mesh may have non-uniform scale or be oriented tail-forward; confirm orientation **before** rigging or cutting.
