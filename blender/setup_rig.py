"""
SimFish — Elephant Nose Fish rig scaffolding for Blender 4.x.

Run from Blender's Scripting workspace (Text Editor → Open → Run Script),
or headless:
    blender --background --python setup_rig.py

What it does:
  1. Creates an Armature `fish_rig` with the full bone hierarchy
     (spine, caudal, head, jaw, schnauzenorgan, pectoral fins).
  2. Places each bone at a default position derived from the fish's
     bounding box (assumes +Y forward, +Z up). YOU will still need to
     drag bone heads/tails into the actual anatomical locations in
     Edit Mode — the defaults are only a starting skeleton.
  3. Sets custom properties on every pose bone:
     joint_type, joint_axis, range_lo, range_hi.
     These are read by the MJCF exporter.
  4. Adds a UV-sphere "joint cutter" at every non-root bone head on a
     hidden collection `joint_cutters`. Use these for the spherical
     Boolean cuts described in README.md.

Assumes the fish mesh has already been imported (File → Import → OBJ)
and roughly oriented: +Y forward (head), +Z up. If your import differs,
either rotate the mesh or edit AXIS_FORWARD / AXIS_UP below.
"""

import bpy
import math
from mathutils import Vector

# --------------------------------------------------------------------
# Configuration
# --------------------------------------------------------------------

AXIS_FORWARD = "Y"   # which world axis the fish points head-first along
AXIS_UP = "Z"

# Bounding box for the provided Elephant_Nose_Fish.obj (from OBJ verts).
# Used only for default bone placements; safe to override by editing bones.
BBOX = {
    "x": (-4.635, 4.635),
    "y": (-11.233, 13.241),
    "z": (0.0, 7.545),
}
BODY_LEN = BBOX["y"][1] - BBOX["y"][0]     # ~24.47
BODY_WIDTH = BBOX["x"][1] - BBOX["x"][0]   # ~9.27
BODY_HEIGHT = BBOX["z"][1] - BBOX["z"][0]  # ~7.55
CENTER_Z = 0.5 * (BBOX["z"][0] + BBOX["z"][1])  # mid-body height

# The fish's body spans y in [y_tail, y_head]. If your import has the
# head at -Y instead of +Y, swap these two lines.
Y_HEAD = BBOX["y"][1]
Y_TAIL = BBOX["y"][0]


def lerp_y(t: float) -> float:
    """Interpolate from head (t=0) to tail (t=1) along the body axis."""
    return Y_HEAD + t * (Y_TAIL - Y_HEAD)


# --------------------------------------------------------------------
# Bone definitions
# --------------------------------------------------------------------
# Each entry: (name, parent, head_t | head_pos, tail_t | tail_pos, props)
# t-values are fractions along the body length (0 = head end, 1 = tail end).
# Absolute positions (tuples) override the body axis parametrization.
#
# Joint ranges and axes follow README.md's joint table.
#
# Convention: bone head = where the joint is, bone tail = the far end.

BONES = [
    # name,        parent,      head,                                   tail,                                   props
    # ---- spine chain ----
    ("root",       None,        ("pos", (0.0, lerp_y(0.15), CENTER_Z)), ("pos", (0.0, lerp_y(0.20), CENTER_Z)),
        {"joint_type": "free",  "joint_axis": "",  "range_lo":   0.0, "range_hi":   0.0}),
    ("spine_01",   "root",      ("t",   0.20),                          ("t",   0.30),
        {"joint_type": "hinge", "joint_axis": "z", "range_lo": -20.0, "range_hi":  20.0}),
    ("spine_02",   "spine_01",  ("t",   0.30),                          ("t",   0.40),
        {"joint_type": "hinge", "joint_axis": "z", "range_lo": -20.0, "range_hi":  20.0}),
    ("spine_03",   "spine_02",  ("t",   0.40),                          ("t",   0.50),
        {"joint_type": "hinge", "joint_axis": "z", "range_lo": -25.0, "range_hi":  25.0}),
    ("spine_04",   "spine_03",  ("t",   0.50),                          ("t",   0.60),
        {"joint_type": "hinge", "joint_axis": "z", "range_lo": -25.0, "range_hi":  25.0}),
    ("spine_05",   "spine_04",  ("t",   0.60),                          ("t",   0.70),
        {"joint_type": "hinge", "joint_axis": "z", "range_lo": -30.0, "range_hi":  30.0}),
    ("spine_06",   "spine_05",  ("t",   0.70),                          ("t",   0.80),
        {"joint_type": "hinge", "joint_axis": "z", "range_lo": -30.0, "range_hi":  30.0}),
    ("caudal_01",  "spine_06",  ("t",   0.80),                          ("t",   0.90),
        {"joint_type": "hinge", "joint_axis": "z", "range_lo": -35.0, "range_hi":  35.0}),
    ("caudal_02",  "caudal_01", ("t",   0.90),                          ("t",   1.00),
        {"joint_type": "hinge", "joint_axis": "z", "range_lo": -40.0, "range_hi":  40.0}),
    # ---- head + mouth + trunk ----
    ("head",       "root",      ("t",   0.15),                          ("t",   0.05),
        {"joint_type": "fixed", "joint_axis": "",  "range_lo":   0.0, "range_hi":   0.0}),
    ("jaw",        "head",      ("pos", (0.0, lerp_y(0.07), CENTER_Z - 0.25 * BODY_HEIGHT)),
                                ("pos", (0.0, lerp_y(0.02), CENTER_Z - 0.30 * BODY_HEIGHT)),
        {"joint_type": "hinge", "joint_axis": "x", "range_lo":   0.0, "range_hi":  25.0}),
    ("schnauz_01", "head",      ("pos", (0.0, lerp_y(0.05), CENTER_Z - 0.30 * BODY_HEIGHT)),
                                ("pos", (0.0, lerp_y(0.00), CENTER_Z - 0.40 * BODY_HEIGHT)),
        {"joint_type": "hinge", "joint_axis": "x", "range_lo": -30.0, "range_hi":  30.0}),
    ("schnauz_02", "schnauz_01",("pos", (0.0, lerp_y(0.00), CENTER_Z - 0.40 * BODY_HEIGHT)),
                                ("pos", (0.0, Y_HEAD + 0.05 * BODY_LEN, CENTER_Z - 0.50 * BODY_HEIGHT)),
        {"joint_type": "hinge", "joint_axis": "x", "range_lo": -40.0, "range_hi":  40.0}),
    # ---- pectoral fins (mirrored) ----
    ("pec_L_1",    "head",      ("pos", ( 0.30 * BODY_WIDTH, lerp_y(0.18), CENTER_Z)),
                                ("pos", ( 0.55 * BODY_WIDTH, lerp_y(0.24), CENTER_Z)),
        {"joint_type": "ball",  "joint_axis": "", "range_lo": -45.0, "range_hi":  45.0}),
    ("pec_L_2",    "pec_L_1",   ("pos", ( 0.55 * BODY_WIDTH, lerp_y(0.24), CENTER_Z)),
                                ("pos", ( 0.75 * BODY_WIDTH, lerp_y(0.30), CENTER_Z)),
        {"joint_type": "hinge", "joint_axis": "y", "range_lo": -30.0, "range_hi":  30.0}),
    ("pec_R_1",    "head",      ("pos", (-0.30 * BODY_WIDTH, lerp_y(0.18), CENTER_Z)),
                                ("pos", (-0.55 * BODY_WIDTH, lerp_y(0.24), CENTER_Z)),
        {"joint_type": "ball",  "joint_axis": "", "range_lo": -45.0, "range_hi":  45.0}),
    ("pec_R_2",    "pec_R_1",   ("pos", (-0.55 * BODY_WIDTH, lerp_y(0.24), CENTER_Z)),
                                ("pos", (-0.75 * BODY_WIDTH, lerp_y(0.30), CENTER_Z)),
        {"joint_type": "hinge", "joint_axis": "y", "range_lo": -30.0, "range_hi":  30.0}),
]


def resolve(spec) -> Vector:
    """Resolve a ('t', float) or ('pos', (x,y,z)) spec into a Vector."""
    kind, val = spec
    if kind == "t":
        return Vector((0.0, lerp_y(val), CENTER_Z))
    if kind == "pos":
        return Vector(val)
    raise ValueError(f"Unknown spec kind: {kind}")


# --------------------------------------------------------------------
# Build armature
# --------------------------------------------------------------------

def ensure_collection(name: str) -> bpy.types.Collection:
    coll = bpy.data.collections.get(name)
    if coll is None:
        coll = bpy.data.collections.new(name)
        bpy.context.scene.collection.children.link(coll)
    return coll


def build_armature() -> bpy.types.Object:
    # Remove any prior fish_rig so the script is idempotent.
    old = bpy.data.objects.get("fish_rig")
    if old is not None:
        bpy.data.objects.remove(old, do_unlink=True)
    old_data = bpy.data.armatures.get("fish_rig")
    if old_data is not None:
        bpy.data.armatures.remove(old_data)

    arm_data = bpy.data.armatures.new("fish_rig")
    arm_obj = bpy.data.objects.new("fish_rig", arm_data)
    bpy.context.scene.collection.objects.link(arm_obj)

    # Enter edit mode on the new armature to add bones.
    bpy.context.view_layer.objects.active = arm_obj
    bpy.ops.object.mode_set(mode="EDIT")

    edit_bones = arm_data.edit_bones
    for name, parent_name, head_spec, tail_spec, _props in BONES:
        eb = edit_bones.new(name)
        eb.head = resolve(head_spec)
        eb.tail = resolve(tail_spec)
        if parent_name is not None:
            eb.parent = edit_bones[parent_name]
            # Don't auto-connect: joints need independent head positions.
            eb.use_connect = False

    bpy.ops.object.mode_set(mode="OBJECT")

    # Pose bone custom properties.
    for name, _parent, _h, _t, props in BONES:
        pb = arm_obj.pose.bones[name]
        for k, v in props.items():
            pb[k] = v

    return arm_obj


# --------------------------------------------------------------------
# Joint-cutter spheres
# --------------------------------------------------------------------

def estimate_sphere_radius(bone_name: str, head_pos: Vector) -> float:
    """Rough heuristic: body cross-section varies along the fish.
    Spine region: ~0.35 × width. Caudal: taper. Pec shoulders: ~0.25 × width.
    Adjust per joint after visual inspection in Blender.
    """
    if bone_name.startswith("spine_") or bone_name == "root":
        return 0.40 * BODY_WIDTH
    if bone_name.startswith("caudal_"):
        t = (Y_HEAD - head_pos.y) / BODY_LEN
        taper = max(0.15, 1.0 - (t - 0.8) / 0.2)
        return 0.40 * BODY_WIDTH * taper
    if bone_name.startswith("pec_"):
        return 0.18 * BODY_WIDTH
    if bone_name.startswith("schnauz_"):
        return 0.15 * BODY_WIDTH
    if bone_name == "jaw":
        return 0.22 * BODY_WIDTH
    return 0.25 * BODY_WIDTH


def build_joint_spheres(arm_obj: bpy.types.Object) -> None:
    coll = ensure_collection("joint_cutters")
    # Clear any prior cutters from this collection.
    for obj in list(coll.objects):
        bpy.data.objects.remove(obj, do_unlink=True)

    for name, parent, head_spec, _tail_spec, props in BONES:
        # Skip bones that don't define a physical joint interface.
        if props["joint_type"] in ("free", "fixed"):
            continue

        head_pos = resolve(head_spec)
        # Head position in world space = armature position + bone head (identity here).
        world_head = arm_obj.matrix_world @ head_pos
        radius = estimate_sphere_radius(name, head_pos)

        bpy.ops.mesh.primitive_uv_sphere_add(
            segments=32, ring_count=16, radius=radius, location=world_head
        )
        sphere = bpy.context.active_object
        sphere.name = f"jcut_{name}"
        sphere.display_type = "WIRE"
        sphere.hide_render = True

        # Move the sphere into the joint_cutters collection.
        for c in list(sphere.users_collection):
            c.objects.unlink(sphere)
        coll.objects.link(sphere)

    # Hide the collection by default.
    layer_coll = bpy.context.view_layer.layer_collection.children.get("joint_cutters")
    if layer_coll is not None:
        layer_coll.hide_viewport = False  # keep visible while placing


# --------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------

def main() -> None:
    arm = build_armature()
    build_joint_spheres(arm)
    print(f"[setup_rig] Built armature '{arm.name}' with {len(BONES)} bones "
          f"and joint-cutter spheres in collection 'joint_cutters'.")
    print("[setup_rig] Next steps:")
    print("  1. Select 'fish_rig', enter Edit Mode, drag each bone head/tail")
    print("     onto the anatomically correct position.")
    print("  2. Fix bone rolls (Armature → Bone Roll → Recalculate: Global +Z Axis).")
    print("  3. Resize/reposition the jcut_* spheres per joint until they look right.")
    print("  4. Parent the mesh to the armature (Ctrl+P → With Automatic Weights),")
    print("     then weight-paint cleanup on fins/jaw/schnauzenorgan.")
    print("  5. For rigid-body export: duplicate the mesh per bone and apply")
    print("     Boolean cuts using the jcut_ spheres (see README.md step 15).")


if __name__ == "__main__":
    main()
